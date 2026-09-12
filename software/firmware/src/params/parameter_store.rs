use crate::params::parameters::{PARAMETER_PAYLOAD_SIZE, ParameterProfileV1};

pub const SLOT_SIZE: u32 = 4_096;
pub const SLOT_A_OFFSET: u32 = 0x0003_e000;
pub const SLOT_B_OFFSET: u32 = 0x0003_f000;
pub const RECORD_HEADER_SIZE: usize = 16;
pub const COMMIT_MARKER_SIZE: usize = 8;
/// Largest supported payload; record buffers are sized for this and the
/// unused tail of smaller payloads stays erased (0xFF).
pub const MAX_PAYLOAD_SIZE: usize = PARAMETER_PAYLOAD_SIZE;
pub const PRE_COMMIT_SIZE: usize = RECORD_HEADER_SIZE + MAX_PAYLOAD_SIZE;
pub const RECORD_SIZE: usize = PRE_COMMIT_SIZE + COMMIT_MARKER_SIZE;

const COMMIT_MARKER: [u8; 8] = *b"COMMITV1";

/// A payload type that can be persisted in the dual-slot parameter store.
///
/// The store guarantees power-loss safety around any implementor: records
/// carry a type-specific magic and schema version, are CRC-protected, and
/// are committed with a marker written as the very last double word.
/// `PAYLOAD_SIZE` must be a multiple of 8 and must not exceed
/// `MAX_PAYLOAD_SIZE` so the pre-commit region programs in whole double
/// words.
pub trait StoredRecord: Copy + PartialEq {
    const MAGIC: [u8; 4];
    const SCHEMA_VERSION: u16;
    const PAYLOAD_SIZE: usize;
    type Error;

    fn validate_record(&self) -> Result<(), Self::Error>;
    /// Writes exactly `PAYLOAD_SIZE` bytes into `payload` and returns
    /// `PAYLOAD_SIZE`.
    fn encode_payload_into(&self, payload: &mut [u8]) -> usize;
    /// Decodes from a slice of exactly `PAYLOAD_SIZE` bytes.
    fn decode_payload(payload: &[u8]) -> Result<Self, Self::Error>;
}

impl StoredRecord for ParameterProfileV1 {
    const MAGIC: [u8; 4] = *b"FOCP";
    const SCHEMA_VERSION: u16 = 1;
    const PAYLOAD_SIZE: usize = PARAMETER_PAYLOAD_SIZE;
    type Error = crate::params::parameters::ParameterError;

    fn validate_record(&self) -> Result<(), Self::Error> {
        self.validate()
    }

    fn encode_payload_into(&self, payload: &mut [u8]) -> usize {
        payload[..PARAMETER_PAYLOAD_SIZE]
            .copy_from_slice(&self.encode_payload());
        PARAMETER_PAYLOAD_SIZE
    }

    fn decode_payload(payload: &[u8]) -> Result<Self, Self::Error> {
        let bytes: &[u8; PARAMETER_PAYLOAD_SIZE] =
            payload.try_into().map_err(|_| {
                crate::params::parameters::ParameterError::ReservedBytes
            })?;
        Self::decode_payload(bytes)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Slot {
    A,
    B,
}

impl Slot {
    pub const fn opposite(self) -> Self {
        match self {
            | Self::A => Self::B,
            | Self::B => Self::A,
        }
    }
}

/// Base offsets of the two alternating slots for one record type.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SlotLayout {
    pub slot_a: u32,
    pub slot_b: u32,
    pub size: u32,
}

pub const PROFILE_SLOT_LAYOUT: SlotLayout = SlotLayout {
    slot_a: SLOT_A_OFFSET,
    slot_b: SLOT_B_OFFSET,
    size: SLOT_SIZE,
};

impl SlotLayout {
    pub const fn offset(&self, slot: Slot) -> u32 {
        match slot {
            | Slot::A => self.slot_a,
            | Slot::B => self.slot_b,
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

/// Lets several stores share one flash peripheral through reborrowing:
/// stores are constructed on demand over `&mut Backend`.
impl<B: FlashBackend> FlashBackend for &mut B {
    type Error = B::Error;

    fn read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), Self::Error> {
        (**self).read(offset, bytes)
    }

    fn erase_page(&mut self, offset: u32) -> Result<(), Self::Error> {
        (**self).erase_page(offset)
    }

    fn program_double_word(
        &mut self,
        offset: u32,
        bytes: &[u8; 8],
    ) -> Result<(), Self::Error> {
        (**self).program_double_word(offset, bytes)
    }
}

#[derive(Debug, PartialEq)]
pub enum StoreError<E> {
    Read(E),
    Erase(E),
    Program(E),
    Verify,
    Invalid,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StoredProfile<P = ParameterProfileV1> {
    pub profile: P,
    pub generation: u32,
    pub slot: Slot,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SaveOutcome {
    Saved { generation: u32, slot: Slot },
    Unchanged { generation: u32, slot: Slot },
}

pub struct ParameterStore<B, P = ParameterProfileV1> {
    backend: B,
    layout: SlotLayout,
    _record: core::marker::PhantomData<P>,
}

type SlotScan<P, E> =
    Result<(Option<StoredProfile<P>>, Option<StoredProfile<P>>), StoreError<E>>;

impl<B, P> ParameterStore<B, P>
where
    B: FlashBackend,
    P: StoredRecord,
{
    pub const fn new(backend: B) -> Self {
        Self::new_with_layout(backend, PROFILE_SLOT_LAYOUT)
    }

    pub const fn new_with_layout(backend: B, layout: SlotLayout) -> Self {
        Self {
            backend,
            layout,
            _record: core::marker::PhantomData,
        }
    }

    pub fn into_inner(self) -> B {
        self.backend
    }

    pub fn load_latest(
        &mut self,
    ) -> Result<Option<StoredProfile<P>>, StoreError<B::Error>> {
        let (slot_a, slot_b) = self.scan()?;
        Ok(select_latest(slot_a, slot_b))
    }

    pub fn save(
        &mut self,
        profile: &P,
    ) -> Result<SaveOutcome, StoreError<B::Error>> {
        self.save_inner(profile, false)
    }

    pub fn force_save(
        &mut self,
        profile: &P,
    ) -> Result<SaveOutcome, StoreError<B::Error>> {
        self.save_inner(profile, true)
    }

    fn save_inner(
        &mut self,
        profile: &P,
        force: bool,
    ) -> Result<SaveOutcome, StoreError<B::Error>> {
        if P::PAYLOAD_SIZE > MAX_PAYLOAD_SIZE {
            return Err(StoreError::Invalid);
        }
        profile.validate_record().map_err(|_| StoreError::Invalid)?;
        let latest = self.load_latest()?;
        let payload = encode_payload_buffer(profile);

        if !force
            && let Some(current) = latest
            && encode_payload_buffer(&current.profile) == payload
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
        let base = self.layout.offset(target);
        let pre_commit = RECORD_HEADER_SIZE + P::PAYLOAD_SIZE;

        self.backend.erase_page(base).map_err(StoreError::Erase)?;
        for (index, chunk) in record[..pre_commit].chunks_exact(8).enumerate() {
            let bytes: &[u8; 8] =
                chunk.try_into().map_err(|_| StoreError::Verify)?;
            self.backend
                .program_double_word(base + (index * 8) as u32, bytes)
                .map_err(StoreError::Program)?;
        }

        let mut pre_commit_read = [0u8; PRE_COMMIT_SIZE];
        self.backend
            .read(base, &mut pre_commit_read)
            .map_err(StoreError::Read)?;
        if pre_commit_read[..pre_commit] != record[..pre_commit]
            || validate_pre_commit::<P>(&pre_commit_read).is_none()
        {
            return Err(StoreError::Verify);
        }

        self.backend
            .program_double_word(base + pre_commit as u32, &COMMIT_MARKER)
            .map_err(StoreError::Program)?;

        let mut committed = [0u8; RECORD_SIZE];
        self.backend
            .read(base, &mut committed)
            .map_err(StoreError::Read)?;
        let Some(decoded) = decode_record::<P>(&committed, target) else {
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

    fn scan(&mut self) -> SlotScan<P, B::Error> {
        let slot_a = self.read_slot(Slot::A)?;
        let slot_b = self.read_slot(Slot::B)?;
        Ok((slot_a, slot_b))
    }

    fn read_slot(
        &mut self,
        slot: Slot,
    ) -> Result<Option<StoredProfile<P>>, StoreError<B::Error>> {
        let mut record = [0xff; RECORD_SIZE];
        self.backend
            .read(self.layout.offset(slot), &mut record)
            .map_err(StoreError::Read)?;
        Ok(decode_record(&record, slot))
    }
}

fn select_latest<P: StoredRecord>(
    slot_a: Option<StoredProfile<P>>,
    slot_b: Option<StoredProfile<P>>,
) -> Option<StoredProfile<P>> {
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

fn encode_payload_buffer<P: StoredRecord>(
    profile: &P,
) -> [u8; MAX_PAYLOAD_SIZE] {
    let mut buffer = [0u8; MAX_PAYLOAD_SIZE];
    let written = profile.encode_payload_into(&mut buffer);
    debug_assert_eq!(written, P::PAYLOAD_SIZE);
    buffer
}

fn encode_record<P: StoredRecord>(
    profile: &P,
    generation: u32,
) -> [u8; RECORD_SIZE] {
    let mut record = [0xff; RECORD_SIZE];
    let written =
        profile.encode_payload_into(&mut record[RECORD_HEADER_SIZE..]);
    debug_assert_eq!(written, P::PAYLOAD_SIZE);
    record[0..4].copy_from_slice(&P::MAGIC);
    record[4..6].copy_from_slice(&P::SCHEMA_VERSION.to_le_bytes());
    record[6..8].copy_from_slice(&(P::PAYLOAD_SIZE as u16).to_le_bytes());
    record[8..12].copy_from_slice(&generation.to_le_bytes());
    let crc = record_crc(
        &record[..12],
        &record[RECORD_HEADER_SIZE..RECORD_HEADER_SIZE + P::PAYLOAD_SIZE],
    );
    record[12..16].copy_from_slice(&crc.to_le_bytes());
    record
}

fn validate_pre_commit<P: StoredRecord>(
    record: &[u8; PRE_COMMIT_SIZE],
) -> Option<(P, u32)> {
    if record[0..4] != P::MAGIC
        || u16::from_le_bytes(record[4..6].try_into().ok()?)
            != P::SCHEMA_VERSION
        || usize::from(u16::from_le_bytes(record[6..8].try_into().ok()?))
            != P::PAYLOAD_SIZE
    {
        return None;
    }
    let expected_crc = u32::from_le_bytes(record[12..16].try_into().ok()?);
    if expected_crc
        != record_crc(
            &record[..12],
            &record[RECORD_HEADER_SIZE..RECORD_HEADER_SIZE + P::PAYLOAD_SIZE],
        )
    {
        return None;
    }
    let payload_slice =
        &record[RECORD_HEADER_SIZE..RECORD_HEADER_SIZE + P::PAYLOAD_SIZE];
    let profile = P::decode_payload(payload_slice).ok()?;
    let generation = u32::from_le_bytes(record[8..12].try_into().ok()?);
    Some((profile, generation))
}

fn decode_record<P: StoredRecord>(
    record: &[u8; RECORD_SIZE],
    slot: Slot,
) -> Option<StoredProfile<P>> {
    let pre_commit = RECORD_HEADER_SIZE + P::PAYLOAD_SIZE;
    if record[pre_commit..pre_commit + COMMIT_MARKER_SIZE] != COMMIT_MARKER {
        return None;
    }
    let pre_commit_bytes: &[u8; PRE_COMMIT_SIZE] =
        record[..PRE_COMMIT_SIZE].try_into().ok()?;
    let (profile, generation) = validate_pre_commit::<P>(pre_commit_bytes)?;
    Some(StoredProfile {
        profile,
        generation,
        slot,
    })
}

fn record_crc(head: &[u8], tail: &[u8]) -> u32 {
    let mut crc = 0xffff_ffff;
    for byte in head.iter().chain(tail) {
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
pub(crate) mod tests {
    use super::*;
    use crate::params::parameters::{ParameterId, ParameterValue};

    /// Covers all four parameter pages at the top of flash so tests can
    /// exercise multiple record layouts over one backend.
    pub(crate) const STORAGE_START: u32 = 0x0003_c000;
    pub(crate) const STORAGE_SIZE: usize = (SLOT_SIZE * 4) as usize;
    pub(crate) const VALID_ERASE_OFFSETS: [u32; 4] =
        [0x0003_c000, 0x0003_d000, SLOT_A_OFFSET, SLOT_B_OFFSET];

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) enum MockError {
        OutOfBounds,
        Unaligned,
        NeedsErase,
        Injected,
    }

    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub(crate) enum Operation {
        Erase(u32),
        Program(u32),
    }

    #[derive(Clone)]
    pub(crate) struct MockFlash {
        pub(crate) bytes: Vec<u8>,
        pub(crate) operations: Vec<Operation>,
        pub(crate) fail_erase: bool,
        pub(crate) fail_program_at: Option<usize>,
        pub(crate) program_count: usize,
    }

    impl MockFlash {
        pub(crate) fn erased() -> Self {
            Self {
                bytes: vec![0xff; STORAGE_SIZE],
                operations: Vec::new(),
                fail_erase: false,
                fail_program_at: None,
                program_count: 0,
            }
        }

        pub(crate) fn index(
            offset: u32,
            len: usize,
        ) -> Result<usize, MockError> {
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
            if !VALID_ERASE_OFFSETS.contains(&offset) {
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
            if !offset.is_multiple_of(8) {
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
        let mut store: ParameterStore<MockFlash> =
            ParameterStore::new(MockFlash::erased());
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
            let mut rebooted: ParameterStore<MockFlash> =
                ParameterStore::new(store.into_inner());
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
        let mut store: ParameterStore<MockFlash> = ParameterStore::new(flash);
        assert_eq!(store.load_latest().unwrap().unwrap().slot, Slot::B);
    }
}
