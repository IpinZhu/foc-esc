#![no_std]

pub const VERSION: u8 = 2;
pub const HEADER: usize = 34;
pub const MAX_PAYLOAD: usize = 198;
pub const MAX_FRAME: usize = HEADER + MAX_PAYLOAD + 2;
const _: () = assert!(MAX_FRAME + 16 <= 250);
pub const LEASE_MS: u64 = 500;
pub const PERMIT_MS: u64 = 250;
pub const HELLO: u8 = 1;
pub const IDENTITY: u8 = 2;
pub const REQUEST: u8 = 3;
pub const RESPONSE: u8 = 4;
pub const TELEMETRY: u8 = 5;
pub const BRIDGE_HELLO: u8 = 6;
pub const BRIDGE_INFO: u8 = 7;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    pub kind: u8,
    pub session: u64,
    pub boot: u64,
    pub request: u32,
    pub sequence: u32,
    pub permit: u32,
    pub len: usize,
    pub payload: [u8; MAX_PAYLOAD],
}

impl Frame {
    pub fn new(kind: u8, session: u64, payload: &[u8]) -> Option<Self> {
        if payload.len() > MAX_PAYLOAD || !(HELLO..=BRIDGE_INFO).contains(&kind) {
            return None;
        }
        let mut frame = Self {
            kind,
            session,
            boot: 0,
            request: 0,
            sequence: 0,
            permit: 0,
            len: payload.len(),
            payload: [0; MAX_PAYLOAD],
        };
        frame.payload[..payload.len()].copy_from_slice(payload);
        Some(frame)
    }

    pub fn data(&self) -> &[u8] {
        &self.payload[..self.len]
    }

    pub fn encode(&self, out: &mut [u8; MAX_FRAME]) -> usize {
        if self.len > MAX_PAYLOAD {
            return 0;
        }
        out[..2].copy_from_slice(b"FC");
        out[2] = VERSION;
        out[3] = self.kind;
        out[4..6].copy_from_slice(&(self.len as u16).to_le_bytes());
        out[6..14].copy_from_slice(&self.session.to_le_bytes());
        out[14..22].copy_from_slice(&self.boot.to_le_bytes());
        out[22..26].copy_from_slice(&self.request.to_le_bytes());
        out[26..30].copy_from_slice(&self.sequence.to_le_bytes());
        out[30..34].copy_from_slice(&self.permit.to_le_bytes());
        out[HEADER..HEADER + self.len].copy_from_slice(self.data());
        let end = HEADER + self.len;
        let crc = crc16(&out[..end]);
        out[end..end + 2].copy_from_slice(&crc.to_le_bytes());
        end + 2
    }

    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < HEADER + 2
            || bytes.len() > MAX_FRAME
            || &bytes[..2] != b"FC"
            || bytes[2] != VERSION
        {
            return None;
        }
        let len = u16::from_le_bytes(bytes[4..6].try_into().ok()?) as usize;
        if len > MAX_PAYLOAD || bytes.len() != HEADER + len + 2 {
            return None;
        }
        let end = HEADER + len;
        if crc16(&bytes[..end]) != u16::from_le_bytes(bytes[end..].try_into().ok()?) {
            return None;
        }
        let mut frame = Self::new(
            bytes[3],
            u64::from_le_bytes(bytes[6..14].try_into().ok()?),
            &bytes[HEADER..end],
        )?;
        frame.boot = u64::from_le_bytes(bytes[14..22].try_into().ok()?);
        frame.request = u32::from_le_bytes(bytes[22..26].try_into().ok()?);
        frame.sequence = u32::from_le_bytes(bytes[26..30].try_into().ok()?);
        frame.permit = u32::from_le_bytes(bytes[30..34].try_into().ok()?);
        Some(frame)
    }
}

pub fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = 0xffffu16;
    for byte in bytes {
        crc ^= (*byte as u16) << 8;
        for _ in 0..8 {
            crc = (crc << 1) ^ if crc & 0x8000 != 0 { 0x1021 } else { 0 };
        }
    }
    crc
}

pub struct Decoder {
    bytes: [u8; MAX_FRAME],
    len: usize,
}
impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}
impl Decoder {
    pub const fn new() -> Self {
        Self {
            bytes: [0; MAX_FRAME],
            len: 0,
        }
    }
    pub fn reset(&mut self) {
        self.len = 0;
    }
    pub fn push(&mut self, byte: u8) -> Option<Frame> {
        if self.len == MAX_FRAME {
            self.shift();
        }
        self.bytes[self.len] = byte;
        self.len += 1;
        loop {
            if self.len == 0 {
                return None;
            }
            if self.bytes[0] != b'F' {
                self.shift();
                continue;
            }
            if self.len < 2 {
                return None;
            }
            if self.bytes[1] != b'C' {
                self.shift();
                continue;
            }
            if self.len < 6 {
                return None;
            }
            let len = u16::from_le_bytes([self.bytes[4], self.bytes[5]]) as usize;
            if self.bytes[2] != VERSION
                || !(HELLO..=BRIDGE_INFO).contains(&self.bytes[3])
                || len > MAX_PAYLOAD
            {
                self.shift();
                continue;
            }
            let size = HEADER + len + 2;
            if self.len < size {
                return None;
            }
            if let Some(frame) = Frame::decode(&self.bytes[..size]) {
                self.bytes.copy_within(size..self.len, 0);
                self.len -= size;
                return Some(frame);
            }
            self.shift();
        }
    }
    fn shift(&mut self) {
        self.bytes.copy_within(1..self.len, 0);
        self.len -= 1;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LeaseError {
    Boot,
    Expired,
    Owner,
    Sequence,
    Permit,
}

pub struct Lease {
    pub boot: u64,
    owner: Option<u64>,
    expires: u64,
    sequence: u32,
    remote: bool,
    retired: [Option<(u64, u64)>; 4],
}
impl Lease {
    pub const fn new(boot: u64) -> Self {
        Self {
            boot,
            owner: None,
            expires: 0,
            sequence: 0,
            remote: false,
            retired: [None; 4],
        }
    }
    pub fn owner(&self) -> Option<u64> {
        self.owner
    }
    pub fn remote_mode(&self) -> bool {
        self.remote
    }
    pub fn expire(&mut self, now: u64) -> bool {
        if self.owner.is_some() && now >= self.expires {
            self.release();
            return true;
        }
        false
    }
    fn credential(&self, frame: &Frame, now: u64) -> Result<(), LeaseError> {
        // A permit must never wrap within one boot identity.
        if now > u32::MAX as u64 {
            return Err(LeaseError::Expired);
        }
        if frame.boot != self.boot {
            return Err(LeaseError::Boot);
        }
        if frame.session == 0 {
            return Err(LeaseError::Owner);
        }
        let age = (now as u32).wrapping_sub(frame.permit) as u64;
        if age >= PERMIT_MS {
            return Err(LeaseError::Permit);
        }
        Ok(())
    }
    pub fn claim(&mut self, frame: &Frame, now: u64) -> Result<(), LeaseError> {
        self.credential(frame, now)?;
        if self.owner.is_some() {
            return Err(LeaseError::Owner);
        }
        for entry in &mut self.retired {
            if entry.is_some_and(|(_, until)| now >= until) {
                *entry = None;
            }
        }
        if self
            .retired
            .iter()
            .flatten()
            .any(|(session, _)| *session == frame.session)
            || self.retired.iter().all(Option::is_some)
        {
            return Err(LeaseError::Owner);
        }
        self.owner = Some(frame.session);
        self.expires = now + LEASE_MS;
        self.sequence = frame.sequence;
        self.remote = true;
        Ok(())
    }
    pub fn authorize(&mut self, frame: &Frame, now: u64) -> Result<(), LeaseError> {
        self.credential(frame, now)?;
        if self.owner != Some(frame.session) {
            return Err(LeaseError::Owner);
        }
        if now >= self.expires {
            return Err(LeaseError::Expired);
        }
        let distance = frame.sequence.wrapping_sub(self.sequence);
        if distance == 0 || distance >= 0x8000_0000 {
            return Err(LeaseError::Sequence);
        }
        self.sequence = frame.sequence;
        Ok(())
    }
    pub fn heartbeat(&mut self, frame: &Frame, now: u64) -> Result<(), LeaseError> {
        self.authorize(frame, now)?;
        self.expires = now + LEASE_MS;
        Ok(())
    }
    pub fn release(&mut self) {
        if let Some(owner) = self.owner.take()
            && let Some(entry) = self.retired.iter_mut().find(|entry| entry.is_none())
        {
            *entry = Some((owner, self.expires + PERMIT_MS));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn crc_reference_and_roundtrip() {
        assert_eq!(crc16(b"123456789"), 0x29b1);
        let mut frame = Frame::new(REQUEST, 0x0102030405060708, b"status").unwrap();
        frame.boot = 0x1112131415161718;
        frame.request = 0x21222324;
        frame.sequence = 0x31323334;
        frame.permit = 0x41424344;
        let mut bytes = [0; MAX_FRAME];
        let size = frame.encode(&mut bytes);
        assert_eq!(size, 42);
        assert_eq!(
            &bytes[..size],
            &[
                0x46, 0x43, 0x02, 0x03, 0x06, 0x00, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,
                0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, 0x24, 0x23, 0x22, 0x21, 0x34, 0x33,
                0x32, 0x31, 0x44, 0x43, 0x42, 0x41, 0x73, 0x74, 0x61, 0x74, 0x75, 0x73, 0x12, 0x1d,
            ]
        );
        assert_eq!(Frame::decode(&bytes[..size]), Some(frame));
        bytes[HEADER] ^= 1;
        assert!(Frame::decode(&bytes[..size]).is_none());
    }
    #[test]
    fn stream_handles_noise_corruption_and_concatenation() {
        let frame = Frame::new(HELLO, 42, b"").unwrap();
        let mut bytes = [0; MAX_FRAME];
        let size = frame.encode(&mut bytes);
        let mut decoder = Decoder::new();
        for value in b"noiseFC\x02\xff\xff\xff" {
            assert!(decoder.push(*value).is_none());
        }
        for _ in 0..2 {
            for (i, value) in bytes[..size].iter().enumerate() {
                assert_eq!(
                    decoder.push(*value),
                    if i + 1 == size { Some(frame) } else { None }
                );
            }
        }
    }
    #[test]
    fn limits() {
        assert!(Frame::new(REQUEST, 1, &[0; MAX_PAYLOAD + 1]).is_none());
        let frame = Frame::new(TELEMETRY, 1, &[0; MAX_PAYLOAD]).unwrap();
        let mut bytes = [0; MAX_FRAME];
        assert_eq!(frame.encode(&mut bytes), MAX_FRAME);
        assert_eq!(Frame::decode(&bytes), Some(frame));
    }
    fn request(session: u64, sequence: u32, now: u32) -> Frame {
        let mut f = Frame::new(REQUEST, session, b"heartbeat").unwrap();
        f.boot = 99;
        f.sequence = sequence;
        f.permit = now;
        f
    }
    #[test]
    fn lease_expires_and_does_not_revive_with_old_heartbeat() {
        let mut lease = Lease::new(99);
        lease.claim(&request(7, 1, 0), 0).unwrap();
        lease.heartbeat(&request(7, 2, 100), 100).unwrap();
        assert!(!lease.expire(599));
        assert!(lease.expire(600));
        assert!(lease.heartbeat(&request(7, 3, 600), 600).is_err());
        assert!(lease.remote_mode());
    }
    #[test]
    fn permit_wrap_requires_new_boot_identity() {
        let mut lease = Lease::new(99);
        assert_eq!(
            lease.claim(&request(7, 1, 0), 1u64 << 32),
            Err(LeaseError::Expired)
        );
    }
    #[test]
    fn released_session_cannot_reclaim_with_buffered_packets() {
        let mut lease = Lease::new(99);
        lease.claim(&request(7, 1, 0), 0).unwrap();
        lease.authorize(&request(7, 2, 10), 10).unwrap();
        lease.release();
        assert_eq!(lease.claim(&request(7, 1, 0), 20), Err(LeaseError::Owner));
        lease.claim(&request(8, 3, 20), 20).unwrap();
        assert_eq!(
            lease.authorize(&request(7, 3, 20), 20),
            Err(LeaseError::Owner)
        );
    }
    #[test]
    fn lease_rejects_wrong_owner_boot_replay_and_delays() {
        let mut lease = Lease::new(99);
        lease.claim(&request(7, 1, 0), 0).unwrap();
        assert_eq!(
            lease.authorize(&request(8, 2, 100), 100),
            Err(LeaseError::Owner)
        );
        assert_eq!(
            lease.authorize(&request(7, 1, 100), 100),
            Err(LeaseError::Sequence)
        );
        assert_eq!(
            lease.authorize(&request(7, 2, 0), 250),
            Err(LeaseError::Permit)
        );
        let mut wrong = request(7, 2, 100);
        wrong.boot = 100;
        assert_eq!(lease.authorize(&wrong, 100), Err(LeaseError::Boot));
        assert!(lease.authorize(&request(7, 2, 100), 100).is_ok());
        assert!(lease.claim(&request(8, 3, 100), 100).is_err());
    }
}
