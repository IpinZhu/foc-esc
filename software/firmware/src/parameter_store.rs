use crate::parameters::{
    PARAMETER_PAYLOAD_SIZE, ParameterError, ParameterProfileV1,
};

pub const SLOT_SIZE: u32 = 4_096;
pub const SLOT_A_OFFSET: u32 = 0x0003_e000;
pub const SLOT_B_OFFSET: u32 = 0x0003_f000;
pub const RECORD_SIZE: usize = 184;
pub const PRE_COMMIT_SIZE: usize = 176;

const MAGIC: [u8; 4] = *b"FOCP";
const SCHEMA_VERSION: u16 = 1;
const COMMIT_MARKER: [u8; 8] = *b"COMMITV1";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Slot {
    A,
    B,
}

impl Slot {
    pub const fn offset(self) -> u32 {
        match self {
            | Self::A => SLOT_A_OFFSET,
            | Self::B => SLOT_B_OFFSET,
        }
    }

    const fn opposite(self) -> Self {
        match self {
            | Self::A => Self::B,
            | Self::B => Self::A,
        }
    }
}

pub trait FlashBackend {
    type Error;

    fn read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), Self::Error>;
    fn erase_page(&mut self, offset: u32) -> Result<(), Self::Error>;
    fn program_double_word(
        &mut self,
        offset: u32,
        bytes: &[u8; 8],
    ) -> Result<(), Self::Error>;
}

#[derive(Debug, PartialEq)]
pub enum StoreError<E> {
    Read(E),
    Erase(E),
    Program(E),
    Verify,
    InvalidProfile(ParameterError),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StoredProfile {
    pub profile: ParameterProfileV1,
    pub generation: u32,
    pub slot: Slot,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SaveOutcome {
    Saved { generation: u32, slot: Slot },
    Unchanged { generation: u32, slot: Slot },
}

pub struct ParameterStore<B> {
    backend: B,
}

type SlotScan<E> = Result<
    (Option<StoredProfile>, Option<StoredProfile>),
    StoreError<E>,
>;

impl<B> ParameterStore<B>
where
    B: FlashBackend,
{
    pub const fn new(backend: B) -> Self {
        Self { backend }
    }

    pub fn into_inner(self) -> B {
        self.backend
    }

    pub fn load_latest(
        &mut self,
    ) -> Result<Option<StoredProfile>, StoreError<B::Error>> {
        let (slot_a, slot_b) = self.scan()?;
        Ok(select_latest(slot_a, slot_b))
    }

    pub fn save(
        &mut self,
        profile: &ParameterProfileV1,
    ) -> Result<SaveOutcome, StoreError<B::Error>> {
        self.save_inner(profile, false)
    }

    pub fn force_save(
        &mut self,
        profile: &ParameterProfileV1,
    ) -> Result<SaveOutcome, StoreError<B::Error>> {
        self.save_inner(profile, true)
    }

    fn save_inner(
        &mut self,
        profile: &ParameterProfileV1,
        force: bool,
    ) -> Result<SaveOutcome, StoreError<B::Error>> {
        profile.validate().map_err(StoreError::InvalidProfile)?;
        let latest = self.load_latest()?;
        let payload = profile.encode_payload();

        if !force
            && let Some(current) = latest
            && current.profile.encode_payload() == payload
        {
            return Ok(SaveOutcome::Unchanged {
                generation: current.generation,
                slot: current.slot,
            });
        }

        let (target, generation) = match latest {
            | Some(current) => {
                (current.slot.opposite(), current.generation.wrapping_add(1))
            },
            | None => (Slot::A, 1),
        };
        let record = encode_record(profile, generation);
        let base = target.offset();

        self.backend.erase_page(base).map_err(StoreError::Erase)?;
        for (index, chunk) in
            record[..PRE_COMMIT_SIZE].chunks_exact(8).enumerate()
        {
            let bytes: &[u8; 8] =
                chunk.try_into().map_err(|_| StoreError::Verify)?;
            self.backend
                .program_double_word(base + (index * 8) as u32, bytes)
                .map_err(StoreError::Program)?;
        }

        let mut pre_commit = [0u8; PRE_COMMIT_SIZE];
        self.backend
            .read(base, &mut pre_commit)
            .map_err(StoreError::Read)?;
        if pre_commit != record[..PRE_COMMIT_SIZE]
            || validate_pre_commit(&pre_commit).is_none()
        {
            return Err(StoreError::Verify);
        }

        self.backend
            .program_double_word(base + PRE_COMMIT_SIZE as u32, &COMMIT_MARKER)
            .map_err(StoreError::Program)?;

        let mut committed = [0u8; RECORD_SIZE];
        self.backend
            .read(base, &mut committed)
            .map_err(StoreError::Read)?;
        let Some(decoded) = decode_record(&committed, target) else {
            return Err(StoreError::Verify);
        };
        if decoded.generation != generation || decoded.profile != *profile {
            return Err(StoreError::Verify);
        }

        Ok(SaveOutcome::Saved {
            generation,
            slot: target,
        })
    }

    fn scan(
        &mut self,
    ) -> Result<
        (Option<StoredProfile>, Option<StoredProfile>),
        StoreError<B::Error>,
    > {
        let slot_a = self.read_slot(Slot::A)?;
        let slot_b = self.read_slot(Slot::B)?;
        Ok((slot_a, slot_b))
    }

    fn read_slot(
        &mut self,
        slot: Slot,
    ) -> Result<Option<StoredProfile>, StoreError<B::Error>> {
        let mut record = [0u8; RECORD_SIZE];
        self.backend
            .read(slot.offset(), &mut record)
            .map_err(StoreError::Read)?;
        Ok(decode_record(&record, slot))
    }
}

fn select_latest(
    slot_a: Option<StoredProfile>,
    slot_b: Option<StoredProfile>,
) -> Option<StoredProfile> {
    match (slot_a, slot_b) {
        | (Some(a), Some(b))
            if generation_is_newer(b.generation, a.generation) =>
        {
            Some(b)
        },
        | (Some(a), Some(_)) => Some(a),
        | (Some(a), None) => Some(a),
        | (None, Some(b)) => Some(b),
        | (None, None) => None,
    }
}

fn generation_is_newer(candidate: u32, reference: u32) -> bool {
    (candidate.wrapping_sub(reference) as i32) > 0
}

fn encode_record(
    profile: &ParameterProfileV1,
    generation: u32,
) -> [u8; RECORD_SIZE] {
    let mut record = [0xff; RECORD_SIZE];
    record[0..4].copy_from_slice(&MAGIC);
    record[4..6].copy_from_slice(&SCHEMA_VERSION.to_le_bytes());
    record[6..8]
        .copy_from_slice(&(PARAMETER_PAYLOAD_SIZE as u16).to_le_bytes());
    record[8..12].copy_from_slice(&generation.to_le_bytes());
    record[16..PRE_COMMIT_SIZE].copy_from_slice(&profile.encode_payload());
    let crc = record_crc(&record[..PRE_COMMIT_SIZE]);
    record[12..16].copy_from_slice(&crc.to_le_bytes());
    record
}

fn validate_pre_commit(
    record: &[u8; PRE_COMMIT_SIZE],
) -> Option<(ParameterProfileV1, u32)> {
    if record[0..4] != MAGIC
        || u16::from_le_bytes(record[4..6].try_into().ok()?) != SCHEMA_VERSION
        || usize::from(u16::from_le_bytes(record[6..8].try_into().ok()?))
            != PARAMETER_PAYLOAD_SIZE
    {
        return None;
    }
    let expected_crc = u32::from_le_bytes(record[12..16].try_into().ok()?);
    if expected_crc != record_crc(record) {
        return None;
    }
    let payload: &[u8; PARAMETER_PAYLOAD_SIZE] =
        record[16..PRE_COMMIT_SIZE].try_into().ok()?;
    let profile = ParameterProfileV1::decode_payload(payload).ok()?;
    let generation = u32::from_le_bytes(record[8..12].try_into().ok()?);
    Some((profile, generation))
}

fn decode_record(
    record: &[u8; RECORD_SIZE],
    slot: Slot,
) -> Option<StoredProfile> {
    if record[PRE_COMMIT_SIZE..RECORD_SIZE] != COMMIT_MARKER {
        return None;
    }
    let pre_commit: &[u8; PRE_COMMIT_SIZE] =
        record[..PRE_COMMIT_SIZE].try_into().ok()?;
    let (profile, generation) = validate_pre_commit(pre_commit)?;
    Some(StoredProfile {
        profile,
        generation,
        slot,
    })
}

fn record_crc(record: &[u8]) -> u32 {
    let mut crc = 0xffff_ffff;
    for byte in record[..12].iter().chain(&record[16..]) {
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
    use crate::parameters::{ParameterId, ParameterValue};

    const STORAGE_START: u32 = SLOT_A_OFFSET;
    const STORAGE_SIZE: usize = (SLOT_SIZE * 2) as usize;

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum MockError {
        OutOfBounds,
        Unaligned,
        NeedsErase,
        Injected,
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Operation {
        Erase(u32),
        Program(u32),
    }

    #[derive(Clone)]
    struct MockFlash {
        bytes: Vec<u8>,
        operations: Vec<Operation>,
        fail_erase: bool,
        fail_program_at: Option<usize>,
        program_count: usize,
    }

    impl MockFlash {
        fn erased() -> Self {
            Self {
                bytes: vec![0xff; STORAGE_SIZE],
                operations: Vec::new(),
                fail_erase: false,
                fail_program_at: None,
                program_count: 0,
            }
        }

        fn index(offset: u32, len: usize) -> Result<usize, MockError> {
            let index = offset
                .checked_sub(STORAGE_START)
                .ok_or(MockError::OutOfBounds)?
                as usize;
            (index + len <= STORAGE_SIZE)
                .then_some(index)
                .ok_or(MockError::OutOfBounds)
        }
    }

    impl FlashBackend for MockFlash {
        type Error = MockError;

        fn read(
            &mut self,
            offset: u32,
            bytes: &mut [u8],
        ) -> Result<(), Self::Error> {
            let index = Self::index(offset, bytes.len())?;
            bytes.copy_from_slice(&self.bytes[index..index + bytes.len()]);
            Ok(())
        }

        fn erase_page(&mut self, offset: u32) -> Result<(), Self::Error> {
            if !matches!(offset, SLOT_A_OFFSET | SLOT_B_OFFSET) {
                return Err(MockError::Unaligned);
            }
            if self.fail_erase {
                return Err(MockError::Injected);
            }
            let index = Self::index(offset, SLOT_SIZE as usize)?;
            self.bytes[index..index + SLOT_SIZE as usize].fill(0xff);
            self.operations.push(Operation::Erase(offset));
            Ok(())
        }

        fn program_double_word(
            &mut self,
            offset: u32,
            bytes: &[u8; 8],
        ) -> Result<(), Self::Error> {
            if offset % 8 != 0 {
                return Err(MockError::Unaligned);
            }
            if self.fail_program_at == Some(self.program_count) {
                return Err(MockError::Injected);
            }
            self.program_count += 1;
            let index = Self::index(offset, bytes.len())?;
            for (current, requested) in
                self.bytes[index..index + 8].iter().zip(bytes)
            {
                if current & requested != *requested {
                    return Err(MockError::NeedsErase);
                }
            }
            for (current, requested) in
                self.bytes[index..index + 8].iter_mut().zip(bytes)
            {
                *current &= *requested;
            }
            self.operations.push(Operation::Program(offset));
            Ok(())
        }
    }

    #[test]
    fn crc_matches_standard_check_vector() {
        let mut crc = 0xffff_ffff;
        for byte in b"123456789" {
            crc ^= u32::from(*byte);
            for _ in 0..8 {
                crc = if crc & 1 != 0 {
                    (crc >> 1) ^ 0xedb8_8320
                } else {
                    crc >> 1
                };
            }
        }
        assert_eq!(!crc, 0xcbf4_3926);
    }

    #[test]
    fn erased_flash_has_no_record_and_boot_does_not_write() {
        let mut store = ParameterStore::new(MockFlash::erased());
        assert_eq!(store.load_latest(), Ok(None));
        assert!(store.into_inner().operations.is_empty());
    }

    #[test]
    fn save_alternates_slots_and_unchanged_save_does_not_write() {
        let mut store = ParameterStore::new(MockFlash::erased());
        let mut profile = ParameterProfileV1::default();
        assert_eq!(
            store.save(&profile),
            Ok(SaveOutcome::Saved {
                generation: 1,
                slot: Slot::A,
            })
        );
        let operation_count = store.backend.operations.len();
        assert_eq!(
            store.save(&profile),
            Ok(SaveOutcome::Unchanged {
                generation: 1,
                slot: Slot::A,
            })
        );
        assert_eq!(store.backend.operations.len(), operation_count);

        profile
            .set(ParameterId::ElectricalZero, ParameterValue::F32(0.25))
            .unwrap();
        assert_eq!(
            store.save(&profile),
            Ok(SaveOutcome::Saved {
                generation: 2,
                slot: Slot::B,
            })
        );
        assert_eq!(store.load_latest().unwrap().unwrap().profile, profile);
    }

    #[test]
    fn commit_marker_is_the_last_program_operation() {
        let mut store = ParameterStore::new(MockFlash::erased());
        store.save(&ParameterProfileV1::default()).unwrap();
        assert_eq!(
            store.backend.operations.last(),
            Some(&Operation::Program(SLOT_A_OFFSET + PRE_COMMIT_SIZE as u32))
        );
    }

    #[test]
    fn interrupted_program_always_preserves_previous_record() {
        let mut initial = ParameterStore::new(MockFlash::erased());
        let baseline = ParameterProfileV1::default();
        initial.save(&baseline).unwrap();
        let persisted = initial.into_inner();

        let mut changed = baseline;
        changed
            .set(ParameterId::ElectricalZero, ParameterValue::F32(0.75))
            .unwrap();
        let program_operations = PRE_COMMIT_SIZE / 8 + 1;
        for failure_at in 0..program_operations {
            let mut flash = persisted.clone();
            flash.fail_program_at = Some(failure_at);
            flash.program_count = 0;
            let mut store = ParameterStore::new(flash);
            assert!(store.save(&changed).is_err());
            let mut rebooted = ParameterStore::new(store.into_inner());
            assert_eq!(
                rebooted.load_latest().unwrap().unwrap().profile,
                baseline
            );
        }
    }

    #[test]
    fn failed_erase_preserves_previous_record() {
        let mut store = ParameterStore::new(MockFlash::erased());
        let baseline = ParameterProfileV1::default();
        store.save(&baseline).unwrap();
        store.backend.fail_erase = true;
        let mut changed = baseline;
        changed
            .set(ParameterId::ElectricalZero, ParameterValue::F32(1.0))
            .unwrap();
        assert_eq!(
            store.save(&changed),
            Err(StoreError::Erase(MockError::Injected))
        );
        store.backend.fail_erase = false;
        assert_eq!(store.load_latest().unwrap().unwrap().profile, baseline);
    }

    #[test]
    fn force_save_writes_defaults_again() {
        let mut store = ParameterStore::new(MockFlash::erased());
        let defaults = ParameterProfileV1::default();
        store.save(&defaults).unwrap();
        assert_eq!(
            store.force_save(&defaults),
            Ok(SaveOutcome::Saved {
                generation: 2,
                slot: Slot::B,
            })
        );
    }

    #[test]
    fn corrupted_new_slot_falls_back_to_old_slot() {
        let mut store = ParameterStore::new(MockFlash::erased());
        let first = ParameterProfileV1::default();
        store.save(&first).unwrap();
        let mut second = first;
        second
            .set(ParameterId::ElectricalZero, ParameterValue::F32(0.5))
            .unwrap();
        store.save(&second).unwrap();
        let crc_byte = MockFlash::index(SLOT_B_OFFSET + 12, 1).unwrap();
        store.backend.bytes[crc_byte] ^= 1;
        assert_eq!(store.load_latest().unwrap().unwrap().profile, first);
    }

    #[test]
    fn generation_wrap_selects_zero_as_newer_than_maximum() {
        let profile = ParameterProfileV1::default();
        let mut flash = MockFlash::erased();
        let mut a = encode_record(&profile, u32::MAX);
        a[PRE_COMMIT_SIZE..].copy_from_slice(&COMMIT_MARKER);
        let mut b = encode_record(&profile, 0);
        b[PRE_COMMIT_SIZE..].copy_from_slice(&COMMIT_MARKER);
        let a_index = MockFlash::index(SLOT_A_OFFSET, RECORD_SIZE).unwrap();
        let b_index = MockFlash::index(SLOT_B_OFFSET, RECORD_SIZE).unwrap();
        flash.bytes[a_index..a_index + RECORD_SIZE].copy_from_slice(&a);
        flash.bytes[b_index..b_index + RECORD_SIZE].copy_from_slice(&b);
        let mut store = ParameterStore::new(flash);
        assert_eq!(store.load_latest().unwrap().unwrap().slot, Slot::B);
    }
}
