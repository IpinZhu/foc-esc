use foc_wire::{Frame, Lease, LeaseError, PERMIT_MS, REQUEST, RESPONSE};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    ReadOnly,
    Claim,
    Heartbeat,
    Release,
    Disable,
    Mutate,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reject {
    Boot,
    Owner,
    Expired,
    Sequence,
    Permit,
    Busy,
    Malformed,
    DuplicateConflict,
}

pub struct Endpoint {
    lease: Lease,
    cached: Option<(Frame, Frame)>,
}

impl Endpoint {
    pub const fn new(boot: u64) -> Self {
        Self {
            lease: Lease::new(boot),
            cached: None,
        }
    }

    pub fn remote_mode(&self) -> bool {
        self.lease.remote_mode()
    }
    pub fn owner(&self) -> Option<u64> {
        self.lease.owner()
    }
    pub fn boot(&self) -> u64 {
        self.lease.boot
    }
    pub fn expire(&mut self, now: u64) -> bool {
        let expired = self.lease.expire(now);
        if expired {
            self.cached = None;
        }
        expired
    }

    pub fn authorize(
        &mut self,
        frame: &Frame,
        now: u64,
        action: Action,
        can_claim: bool,
    ) -> Result<(), Reject> {
        if frame.kind != REQUEST || frame.boot != self.lease.boot {
            return Err(Reject::Boot);
        }
        if frame.session == 0 {
            return Err(Reject::Owner);
        }
        let age = (now as u32).wrapping_sub(frame.permit) as u64;
        if age >= PERMIT_MS {
            return Err(Reject::Permit);
        }
        match action {
            | Action::ReadOnly | Action::Disable => Ok(()),
            | Action::Claim => {
                if !can_claim {
                    return Err(Reject::Busy);
                }
                self.lease.claim(frame, now).map_err(map_lease)
            },
            | Action::Heartbeat => {
                self.lease.heartbeat(frame, now).map_err(map_lease)
            },
            | Action::Release | Action::Mutate => {
                self.lease.authorize(frame, now).map_err(map_lease)
            },
        }
    }

    pub fn release(&mut self) {
        self.lease.release();
        self.cached = None;
    }

    pub fn replay(&self, request: &Frame) -> Result<Option<Frame>, Reject> {
        if let Some((cached, reply)) = self.cached
            && cached.session == request.session
            && cached.request == request.request
        {
            if cached == *request {
                return Ok(Some(reply));
            }
            return Err(Reject::DuplicateConflict);
        }
        Ok(None)
    }

    pub fn remember(&mut self, request: Frame, reply: Frame) {
        self.cached = Some((request, reply));
    }

    pub fn response(&self, request: &Frame, now: u64, payload: &[u8]) -> Frame {
        let mut reply = Frame::new(RESPONSE, request.session, payload)
            .unwrap_or_else(|| {
                Frame::new(
                    RESPONSE,
                    request.session,
                    b"rejected response-too-long",
                )
                .unwrap()
            });
        reply.boot = self.lease.boot;
        reply.request = request.request;
        reply.sequence = request.sequence;
        reply.permit = now as u32;
        reply
    }
}

fn map_lease(error: LeaseError) -> Reject {
    match error {
        | LeaseError::Boot => Reject::Boot,
        | LeaseError::Owner => Reject::Owner,
        | LeaseError::Expired => Reject::Expired,
        | LeaseError::Sequence => Reject::Sequence,
        | LeaseError::Permit => Reject::Permit,
    }
}

impl Reject {
    pub const fn text(self) -> &'static [u8] {
        match self {
            | Self::Boot => b"rejected boot",
            | Self::Owner => b"rejected owner",
            | Self::Expired => b"rejected expired",
            | Self::Sequence => b"rejected sequence",
            | Self::Permit => b"rejected permit",
            | Self::Busy => b"rejected busy",
            | Self::Malformed => b"rejected malformed",
            | Self::DuplicateConflict => b"rejected request-conflict",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn req(
        boot: u64,
        session: u64,
        request: u32,
        sequence: u32,
        permit: u32,
    ) -> Frame {
        let mut frame = Frame::new(REQUEST, session, b"claim").unwrap();
        frame.boot = boot;
        frame.request = request;
        frame.sequence = sequence;
        frame.permit = permit;
        frame
    }

    #[test]
    fn claim_authorize_and_expire_owner() {
        let mut endpoint = Endpoint::new(42);
        let claim = req(42, 7, 1, 1, 10);
        endpoint.authorize(&claim, 10, Action::Claim, true).unwrap();
        assert_eq!(endpoint.owner(), Some(7));
        assert!(
            endpoint
                .authorize(&req(42, 7, 2, 2, 509), 509, Action::Mutate, false)
                .is_ok()
        );
        assert!(endpoint.expire(1_009));
        assert_eq!(endpoint.owner(), None);
        assert_eq!(
            endpoint.authorize(
                &req(42, 7, 3, 3, 1_010),
                1_010,
                Action::Mutate,
                false
            ),
            Err(Reject::Owner)
        );
    }

    #[test]
    fn disable_is_fresh_credential_safety_operation_without_owner() {
        let mut endpoint = Endpoint::new(42);
        assert_eq!(
            endpoint.authorize(
                &req(42, 99, 1, 1, 20),
                20,
                Action::Disable,
                false
            ),
            Ok(())
        );
        assert_eq!(
            endpoint.authorize(
                &req(42, 99, 2, 2, 20),
                270,
                Action::Disable,
                false
            ),
            Err(Reject::Permit)
        );
    }

    #[test]
    fn released_owner_cannot_replay_cached_claim() {
        let mut endpoint = Endpoint::new(42);
        let claim = req(42, 7, 1, 1, 10);
        endpoint.authorize(&claim, 10, Action::Claim, true).unwrap();
        let reply = endpoint.response(&claim, 11, b"applied");
        endpoint.remember(claim, reply);
        endpoint.release();
        assert_eq!(endpoint.replay(&claim), Ok(None));
        assert_eq!(
            endpoint.authorize(&claim, 12, Action::Claim, true),
            Err(Reject::Owner)
        );
    }

    #[test]
    fn replay_requires_full_identical_request() {
        let mut endpoint = Endpoint::new(42);
        let request = req(42, 7, 1, 1, 10);
        let reply = endpoint.response(&request, 11, b"applied");
        endpoint.remember(request, reply);
        assert_eq!(endpoint.replay(&request), Ok(Some(reply)));
        assert_eq!(
            endpoint.replay(&req(42, 7, 1, 2, 10)),
            Err(Reject::DuplicateConflict)
        );
        assert_eq!(endpoint.replay(&request), Ok(Some(reply)));
    }
}
