use crate::control::foc_core::FocConfig;
use crate::params::parameter_store::{SlotLayout, StoredRecord};

/// Dual-slot flash layout for the motor commissioning record, located
/// directly below the control-parameter slots.
pub const MOTOR_SLOT_A_OFFSET: u32 = 0x0003_c000;
pub const MOTOR_SLOT_B_OFFSET: u32 = 0x0003_d000;
pub const MOTOR_SLOT_LAYOUT: SlotLayout = SlotLayout {
    slot_a: MOTOR_SLOT_A_OFFSET,
    slot_b: MOTOR_SLOT_B_OFFSET,
    size: crate::params::parameter_store::SLOT_SIZE,
};

/// Marks a commissioning record that completed identification and passed
/// verification. Any other value means the parameters were never
/// commissioned (or were invalidated) and must not be used to start the
/// motor.
pub const MOTOR_PARAM_VALID: u32 = 0xa55a_c33c;

pub const MOTOR_PARAM_VERSION: u32 = 1;
pub const MOTOR_PARAM_PAYLOAD_SIZE: usize = 112;
const CHECKSUM_BYTES: usize = 88;

/// Unified motor-commissioning parameters.
///
/// Every downstream consumer (FOC current/speed loops, and later the
/// sensorless observer, PLL, and HFI) takes its motor-dependent parameters
/// from this struct. PI gains are stored in the continuous domain
/// (`integrator += ki * error * dt`), matching `pid::PidController`.
///
/// Identified values are filled in by the auto-tune procedure; fields the
/// procedure does not measure (pole pairs, speed-loop gains) are seeded
/// from the running configuration and carried through unchanged.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorParam {
    pub rs: f32,
    pub ld: f32,
    pub lq: f32,
    pub flux: f32,

    pub pole_pairs: u16,

    pub id_kp: f32,
    pub id_ki: f32,
    pub iq_kp: f32,
    pub iq_ki: f32,

    pub speed_kp: f32,
    pub speed_ki: f32,

    pub observer_gain: f32,
    pub observer_bandwidth: f32,

    pub pll_kp: f32,
    pub pll_ki: f32,

    pub hfi_freq: f32,
    pub hfi_voltage: f32,

    pub observer_switch_speed: f32,

    pub max_current: f32,
    pub max_voltage: f32,

    pub version: u32,
    pub checksum: u32,
    pub valid_flag: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorParamError {
    OutOfRange,
    InvalidRelationship,
    ReservedBytes,
    Checksum,
    NotCommissioned,
}

impl Default for MotorParam {
    fn default() -> Self {
        Self::from_foc_config(&FocConfig::default())
    }
}

impl MotorParam {
    /// Seeds a parameter set from the running configuration. The result is
    /// valid as a carrier but is not marked commissioned: identification
    /// and verification must complete first.
    pub fn from_foc_config(config: &FocConfig) -> Self {
        let bandwidth = 2.0 * core::f32::consts::PI * 500.0;
        let rs = 0.050_0;
        let inductance = 0.000_2;
        let flux = 0.010_0;
        Self {
            rs,
            ld: inductance,
            lq: inductance,
            flux,
            pole_pairs: u16::from(config.pole_pairs),
            id_kp: inductance * bandwidth,
            id_ki: rs * bandwidth,
            iq_kp: inductance * bandwidth,
            iq_ki: rs * bandwidth,
            speed_kp: config.velocity_pid.kp,
            speed_ki: config.velocity_pid.ki,
            observer_gain: 0.0,
            observer_bandwidth: 0.0,
            pll_kp: 0.0,
            pll_ki: 0.0,
            hfi_freq: 0.0,
            hfi_voltage: 0.0,
            observer_switch_speed: 0.0,
            max_current: 10.0_f32.min(config.current_limit),
            max_voltage: 30.0,
            version: MOTOR_PARAM_VERSION,
            checksum: 0,
            valid_flag: 0,
        }
    }

    pub const fn is_commissioned(&self) -> bool {
        self.valid_flag == MOTOR_PARAM_VALID
    }

    /// Marks the record as a completed, verified commissioning result and
    /// refreshes the payload checksum.
    pub fn mark_commissioned(&mut self) {
        self.version = MOTOR_PARAM_VERSION;
        self.valid_flag = MOTOR_PARAM_VALID;
        self.checksum = payload_checksum(&self.encode_payload());
    }

    /// Applies the motor-dependent subset to a configuration. Overlapping
    /// fields written later by the runtime control profile win over these
    /// values, so this is a baseline, not an override.
    pub fn apply_to_config(&self, config: &mut FocConfig) -> bool {
        if self.validate().is_err() {
            return false;
        }
        config.pole_pairs = self.pole_pairs.min(u16::from(u8::MAX)) as u8;
        config.current_pid.kp = self.iq_kp;
        config.current_pid.ki = self.iq_ki;
        config.velocity_pid.kp = self.speed_kp;
        config.velocity_pid.ki = self.speed_ki;
        config.current_limit =
            self.max_current.min(config.over_current_threshold);
        true
    }

    pub fn validate(&self) -> Result<(), MotorParamError> {
        let finite_in = |value: f32, minimum: f32, maximum: f32| {
            value.is_finite() && value >= minimum && value <= maximum
        };
        let non_negative = |value: f32| value.is_finite() && value >= 0.0;

        if !finite_in(self.rs, 0.000_2, 5.0)
            || !finite_in(self.ld, 1.0e-6, 0.1)
            || !finite_in(self.lq, 1.0e-6, 0.1)
            || !finite_in(self.flux, 1.0e-4, 1.0)
            || !(1..=64).contains(&self.pole_pairs)
            || !non_negative(self.id_kp)
            || !non_negative(self.id_ki)
            || !non_negative(self.iq_kp)
            || !non_negative(self.iq_ki)
            || !non_negative(self.speed_kp)
            || !non_negative(self.speed_ki)
            || !non_negative(self.observer_gain)
            || !finite_in(self.observer_bandwidth, 0.0, 100_000.0)
            || !non_negative(self.pll_kp)
            || !finite_in(self.pll_ki, 0.0, 1.0e8)
            || !finite_in(self.hfi_freq, 0.0, 20_000.0)
            || !finite_in(self.hfi_voltage, 0.0, 50.0)
            || !finite_in(self.observer_switch_speed, 0.0, 10_000.0)
            || !finite_in(self.max_current, 0.01, 500.0)
            || !finite_in(self.max_voltage, 0.1, 1_000.0)
        {
            return Err(MotorParamError::OutOfRange);
        }

        let lq_over_ld = self.lq / self.ld;
        if !(0.5..=2.0).contains(&lq_over_ld)
            || self.id_kp > 1.0e4
            || self.id_ki > 1.0e7
            || self.iq_kp > 1.0e4
            || self.iq_ki > 1.0e7
            || self.speed_kp > 1.0e4
            || self.speed_ki > 1.0e7
        {
            return Err(MotorParamError::InvalidRelationship);
        }
        Ok(())
    }

    pub fn encode_payload(&self) -> [u8; MOTOR_PARAM_PAYLOAD_SIZE] {
        let mut data = [0u8; MOTOR_PARAM_PAYLOAD_SIZE];
        data[0..4].copy_from_slice(&self.rs.to_le_bytes());
        data[4..8].copy_from_slice(&self.ld.to_le_bytes());
        data[8..12].copy_from_slice(&self.lq.to_le_bytes());
        data[12..16].copy_from_slice(&self.flux.to_le_bytes());
        data[16..18].copy_from_slice(&self.pole_pairs.to_le_bytes());
        data[20..24].copy_from_slice(&self.id_kp.to_le_bytes());
        data[24..28].copy_from_slice(&self.id_ki.to_le_bytes());
        data[28..32].copy_from_slice(&self.iq_kp.to_le_bytes());
        data[32..36].copy_from_slice(&self.iq_ki.to_le_bytes());
        data[36..40].copy_from_slice(&self.speed_kp.to_le_bytes());
        data[40..44].copy_from_slice(&self.speed_ki.to_le_bytes());
        data[44..48].copy_from_slice(&self.observer_gain.to_le_bytes());
        data[48..52].copy_from_slice(&self.observer_bandwidth.to_le_bytes());
        data[52..56].copy_from_slice(&self.pll_kp.to_le_bytes());
        data[56..60].copy_from_slice(&self.pll_ki.to_le_bytes());
        data[60..64].copy_from_slice(&self.hfi_freq.to_le_bytes());
        data[64..68].copy_from_slice(&self.hfi_voltage.to_le_bytes());
        data[68..72].copy_from_slice(&self.observer_switch_speed.to_le_bytes());
        data[72..76].copy_from_slice(&self.max_current.to_le_bytes());
        data[76..80].copy_from_slice(&self.max_voltage.to_le_bytes());
        data[80..84].copy_from_slice(&self.version.to_le_bytes());
        data[84..88].copy_from_slice(&self.valid_flag.to_le_bytes());
        let checksum = payload_checksum(&data);
        data[88..92].copy_from_slice(&checksum.to_le_bytes());
        data
    }

    pub fn decode_payload(
        data: &[u8; MOTOR_PARAM_PAYLOAD_SIZE],
    ) -> Result<Self, MotorParamError> {
        if data[18..20] != [0; 2] || data[92..] != [0; 20] {
            return Err(MotorParamError::ReservedBytes);
        }
        let stored_checksum =
            u32::from_le_bytes(data[88..92].try_into().expect("fixed size"));
        if stored_checksum != payload_checksum(data) {
            return Err(MotorParamError::Checksum);
        }
        let next_f32 = |offset: usize| {
            f32::from_le_bytes(
                data[offset..offset + 4].try_into().expect("fixed size"),
            )
        };
        let param = Self {
            rs: next_f32(0),
            ld: next_f32(4),
            lq: next_f32(8),
            flux: next_f32(12),
            pole_pairs: u16::from_le_bytes(
                data[16..18].try_into().expect("fixed size"),
            ),
            id_kp: next_f32(20),
            id_ki: next_f32(24),
            iq_kp: next_f32(28),
            iq_ki: next_f32(32),
            speed_kp: next_f32(36),
            speed_ki: next_f32(40),
            observer_gain: next_f32(44),
            observer_bandwidth: next_f32(48),
            pll_kp: next_f32(52),
            pll_ki: next_f32(56),
            hfi_freq: next_f32(60),
            hfi_voltage: next_f32(64),
            observer_switch_speed: next_f32(68),
            max_current: next_f32(72),
            max_voltage: next_f32(76),
            version: u32::from_le_bytes(
                data[80..84].try_into().expect("fixed size"),
            ),
            checksum: stored_checksum,
            valid_flag: u32::from_le_bytes(
                data[84..88].try_into().expect("fixed size"),
            ),
        };
        param.validate()?;
        Ok(param)
    }
}

impl StoredRecord for MotorParam {
    const MAGIC: [u8; 4] = *b"MTRP";
    const SCHEMA_VERSION: u16 = 1;
    const PAYLOAD_SIZE: usize = MOTOR_PARAM_PAYLOAD_SIZE;
    type Error = MotorParamError;

    fn validate_record(&self) -> Result<(), Self::Error> {
        self.validate()
    }

    fn encode_payload_into(&self, payload: &mut [u8]) -> usize {
        let data = self.encode_payload();
        payload[..MOTOR_PARAM_PAYLOAD_SIZE].copy_from_slice(&data);
        MOTOR_PARAM_PAYLOAD_SIZE
    }

    fn decode_payload(payload: &[u8]) -> Result<Self, Self::Error> {
        let bytes: &[u8; MOTOR_PARAM_PAYLOAD_SIZE] = payload
            .try_into()
            .map_err(|_| MotorParamError::ReservedBytes)?;
        Self::decode_payload(bytes)
    }
}
fn payload_checksum(data: &[u8; MOTOR_PARAM_PAYLOAD_SIZE]) -> u32 {
    let mut crc = 0xffff_ffff;
    for byte in &data[..CHECKSUM_BYTES] {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0xedb8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::params::parameter_store::{
        SaveOutcome, Slot, StoreError, tests::MockFlash,
    };

    #[test]
    fn default_seed_is_valid_but_not_commissioned() {
        let param = MotorParam::default();
        assert_eq!(param.validate(), Ok(()));
        assert!(!param.is_commissioned());
        assert_eq!(param.version, MOTOR_PARAM_VERSION);
    }

    #[test]
    fn payload_round_trip_is_exact() {
        let mut param = MotorParam {
            rs: 0.031,
            ld: 0.000_145,
            lq: 0.000_150,
            flux: 0.012_5,
            ..MotorParam::default()
        };
        param.mark_commissioned();
        let encoded = param.encode_payload();
        assert_eq!(MotorParam::decode_payload(&encoded), Ok(param));
    }

    #[test]
    fn corrupted_payload_is_rejected_by_checksum() {
        let mut encoded = MotorParam::default().encode_payload();
        encoded[0] ^= 0x01;
        assert_eq!(
            MotorParam::decode_payload(&encoded),
            Err(MotorParamError::Checksum)
        );
    }

    #[test]
    fn out_of_range_and_relationship_values_are_rejected() {
        let param = MotorParam {
            rs: 50.0,
            ..MotorParam::default()
        };
        assert_eq!(param.validate(), Err(MotorParamError::OutOfRange));

        let param = MotorParam {
            ld: 0.000_2,
            lq: 0.000_9,
            ..MotorParam::default()
        };
        assert_eq!(param.validate(), Err(MotorParamError::InvalidRelationship));
    }

    #[test]
    fn reserved_bytes_are_rejected() {
        let mut encoded = MotorParam::default().encode_payload();
        encoded[18] = 1;
        assert_eq!(
            MotorParam::decode_payload(&encoded),
            Err(MotorParamError::ReservedBytes)
        );
    }

    #[test]
    fn apply_to_config_transfers_gains_and_limits() {
        let mut param = MotorParam {
            pole_pairs: 7,
            max_current: 12.0,
            id_kp: 0.5,
            iq_ki: 120.0,
            ..MotorParam::default()
        };
        let mut config = FocConfig::default();
        assert!(param.apply_to_config(&mut config));
        assert_eq!(config.pole_pairs, 7);
        assert_eq!(config.current_limit, 12.0);
        assert_eq!(config.current_pid.kp, param.iq_kp);
        assert_eq!(config.current_pid.ki, 120.0);
        assert_eq!(config.velocity_pid.kp, param.speed_kp);

        param.rs = 0.0;
        let mut config = FocConfig::default();
        assert!(!param.apply_to_config(&mut config));
        assert_eq!(config.pole_pairs, FocConfig::default().pole_pairs);
    }

    #[test]
    fn motor_record_and_profile_record_coexist_in_flash() {
        let flash = MockFlash::erased();
        let mut motor_store: crate::params::parameter_store::ParameterStore<
            MockFlash,
            MotorParam,
        > = crate::params::parameter_store::ParameterStore::new_with_layout(
            flash.clone(),
            MOTOR_SLOT_LAYOUT,
        );
        let mut profile_store: crate::params::parameter_store::ParameterStore<
            MockFlash,
            crate::params::parameters::ParameterProfileV1,
        > = crate::params::parameter_store::ParameterStore::new(flash.clone());

        let mut motor = MotorParam {
            rs: 0.021,
            ..MotorParam::default()
        };
        motor.mark_commissioned();
        assert_eq!(
            motor_store.save(&motor),
            Ok(SaveOutcome::Saved {
                generation: 1,
                slot: Slot::A,
            })
        );
        assert_eq!(
            profile_store
                .save(&crate::params::parameters::ParameterProfileV1::default()),
            Ok(SaveOutcome::Saved {
                generation: 1,
                slot: Slot::A,
            })
        );

        assert_eq!(motor_store.load_latest().unwrap().unwrap().profile, motor);
        assert_eq!(
            profile_store.load_latest().unwrap().unwrap().profile,
            crate::params::parameters::ParameterProfileV1::default()
        );

        motor.rs = 0.022;
        motor.mark_commissioned();
        assert_eq!(
            motor_store.save(&motor),
            Ok(SaveOutcome::Saved {
                generation: 2,
                slot: Slot::B,
            })
        );
        assert_eq!(motor_store.load_latest().unwrap().unwrap().slot, Slot::B);
    }

    #[test]
    fn invalid_motor_param_fails_store_save() {
        let param = MotorParam {
            rs: 0.0,
            ..MotorParam::default()
        };
        let mut store: crate::params::parameter_store::ParameterStore<
            MockFlash,
            MotorParam,
        > = crate::params::parameter_store::ParameterStore::new_with_layout(
            MockFlash::erased(),
            MOTOR_SLOT_LAYOUT,
        );
        assert_eq!(store.save(&param), Err(StoreError::Invalid));
        assert_eq!(store.load_latest().unwrap(), None);
    }

    #[test]
    fn commissioned_payload_survives_uncommissioned_comparison() {
        let mut param = MotorParam::default();
        let plain = param.encode_payload();
        param.mark_commissioned();
        assert_ne!(param.encode_payload(), plain);
    }
}
