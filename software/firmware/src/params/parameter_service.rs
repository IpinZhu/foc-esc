use crate::interfaces::ParameterStorageStatus;
use crate::params::parameter_store::{SaveOutcome, StoredProfile};
use crate::params::parameters::{
    ParameterError, ParameterId, ParameterProfileV1, ParameterValue,
};

#[derive(Clone, Copy, Debug)]
pub struct ParameterState {
    working: ParameterProfileV1,
    persisted: Option<ParameterProfileV1>,
    boot_pole_pairs: u8,
    boot_encoder_cpr: u16,
    generation: u32,
    dirty: bool,
    source_defaults: bool,
}

impl ParameterState {
    pub fn from_boot(
        working: ParameterProfileV1,
        stored: Option<StoredProfile>,
    ) -> Self {
        let (persisted, generation, source_defaults) = match stored {
            | Some(stored) => (Some(stored.profile), stored.generation, false),
            | None => (None, 0, true),
        };
        Self {
            working,
            persisted,
            boot_pole_pairs: working.pole_pairs,
            boot_encoder_cpr: working.encoder_cpr,
            generation,
            dirty: false,
            source_defaults,
        }
    }

    pub const fn working(&self) -> ParameterProfileV1 {
        self.working
    }

    pub fn get(&self, id: ParameterId) -> ParameterValue {
        self.working.get(id)
    }

    pub fn set(
        &mut self,
        id: ParameterId,
        value: ParameterValue,
    ) -> Result<(), ParameterError> {
        self.working.set(id, value)?;
        self.source_defaults = false;
        self.refresh_dirty();
        Ok(())
    }

    pub fn replace_working(&mut self, profile: ParameterProfileV1) {
        self.working = profile;
        self.source_defaults = false;
        self.refresh_dirty();
    }

    pub fn use_defaults(&mut self) {
        self.working = ParameterProfileV1::default();
        self.source_defaults = true;
        self.refresh_dirty();
    }

    pub fn load(&mut self, stored: StoredProfile) {
        self.working = stored.profile;
        self.persisted = Some(stored.profile);
        self.generation = stored.generation;
        self.dirty = false;
        self.source_defaults = false;
    }

    pub fn saved(&mut self, outcome: SaveOutcome) {
        self.persisted = Some(self.working);
        self.generation = match outcome {
            | SaveOutcome::Saved { generation, .. }
            | SaveOutcome::Unchanged { generation, .. } => generation,
        };
        self.dirty = false;
        self.source_defaults = false;
    }

    pub const fn status(&self) -> ParameterStorageStatus {
        ParameterStorageStatus {
            persisted_valid: self.persisted.is_some(),
            source_defaults: self.source_defaults,
            generation: self.generation,
            dirty: self.dirty,
            restart_required: self.restart_required(),
        }
    }

    const fn restart_required(&self) -> bool {
        self.working.pole_pairs != self.boot_pole_pairs
            || self.working.encoder_cpr != self.boot_encoder_cpr
    }

    fn refresh_dirty(&mut self) {
        self.dirty = match self.persisted {
            | Some(persisted) => {
                persisted.encode_payload() != self.working.encode_payload()
            },
            | None => self.working != ParameterProfileV1::default(),
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::parameter_store::Slot;

    #[test]
    fn tracks_dirty_and_restart_state() {
        let defaults = ParameterProfileV1::default();
        let mut state = ParameterState::from_boot(defaults, None);
        assert_eq!(
            state.status(),
            ParameterStorageStatus {
                source_defaults: true,
                ..ParameterStorageStatus::default()
            }
        );
        state
            .set(ParameterId::PolePairs, ParameterValue::U8(5))
            .unwrap();
        assert!(state.status().dirty);
        assert!(state.status().restart_required);
        state
            .set(ParameterId::PolePairs, ParameterValue::U8(4))
            .unwrap();
        assert!(!state.status().dirty);
        assert!(!state.status().restart_required);
    }

    #[test]
    fn saved_and_loaded_profiles_are_clean() {
        let defaults = ParameterProfileV1::default();
        let stored = StoredProfile {
            profile: defaults,
            generation: 7,
            slot: Slot::A,
        };
        let mut state = ParameterState::from_boot(defaults, Some(stored));
        state
            .set(ParameterId::ElectricalZero, ParameterValue::F32(0.5))
            .unwrap();
        assert!(state.status().dirty);
        state.saved(SaveOutcome::Saved {
            generation: 8,
            slot: Slot::B,
        });
        assert_eq!(state.status().generation, 8);
        assert!(!state.status().dirty);
        assert_eq!(state.working().electrical_zero, 0.5);
    }
}
