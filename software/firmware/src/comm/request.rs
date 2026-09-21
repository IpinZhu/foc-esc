use crate::interfaces::{ParameterRequest, ParameterResponse, ParameterRoute};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct CanRequestFingerprint {
    request_id: u16,
    request_data: [u8; 8],
}

impl CanRequestFingerprint {
    const fn new(request_id: u16, request_data: [u8; 8]) -> Self {
        Self {
            request_id,
            request_data,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParameterRequestEnvelope {
    request: ParameterRequest,
    can_fingerprint: Option<CanRequestFingerprint>,
}

impl ParameterRequestEnvelope {
    #[cfg(any(target_arch = "arm", test))]
    pub(crate) const fn uart(request: ParameterRequest) -> Self {
        Self {
            request,
            can_fingerprint: None,
        }
    }

    pub(crate) const fn can(
        request: ParameterRequest,
        request_id: u16,
        request_data: [u8; 8],
    ) -> Self {
        Self {
            request,
            can_fingerprint: Some(CanRequestFingerprint::new(
                request_id,
                request_data,
            )),
        }
    }

    pub const fn request(self) -> ParameterRequest {
        self.request
    }

    pub const fn transaction(self) -> Option<u8> {
        match self.request.route {
            | ParameterRoute::Can { transaction } => Some(transaction),
            | ParameterRoute::Uart => None,
        }
    }

    const fn is_can(self) -> bool {
        matches!(self.request.route, ParameterRoute::Can { .. })
            && self.can_fingerprint.is_some()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TransactionLookup {
    New,
    Replay(ParameterResponse),
    Conflict,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ParameterTransactionCache {
    cached: Option<(ParameterRequestEnvelope, ParameterResponse)>,
}

impl ParameterTransactionCache {
    pub const fn new() -> Self {
        Self { cached: None }
    }

    pub fn lookup(
        &self,
        request: ParameterRequestEnvelope,
    ) -> TransactionLookup {
        let Some(transaction) = request.transaction() else {
            return TransactionLookup::New;
        };
        match self.cached {
            | Some((cached_request, cached_response))
                if cached_request.transaction() == Some(transaction) =>
            {
                if request == cached_request {
                    TransactionLookup::Replay(cached_response)
                } else {
                    TransactionLookup::Conflict
                }
            },
            | _ => TransactionLookup::New,
        }
    }

    pub fn record(
        &mut self,
        request: ParameterRequestEnvelope,
        response: ParameterResponse,
    ) {
        if request.is_can() {
            self.cached = Some((request, response));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::interfaces::{
        ParameterAction, ParameterResultCode, ParameterStorageStatus,
    };
    use crate::params::parameters::ParameterId;

    fn envelope(
        transaction: u8,
        request_id: u16,
        request_data: [u8; 8],
        action: ParameterAction,
    ) -> ParameterRequestEnvelope {
        ParameterRequestEnvelope::can(
            ParameterRequest {
                route: ParameterRoute::Can { transaction },
                action,
            },
            request_id,
            request_data,
        )
    }

    fn response(request: ParameterRequest) -> ParameterResponse {
        ParameterResponse {
            route: request.route,
            action: request.action,
            result: ParameterResultCode::Success,
            value: None,
            storage: ParameterStorageStatus::default(),
        }
    }

    #[test]
    fn identical_can_request_replays_cached_response() {
        let request =
            envelope(7, 0x106, [7, 1, 0, 0, 0, 0, 0, 0], ParameterAction::Save);
        let cached_response = response(request.request());
        let mut cache = ParameterTransactionCache::new();
        cache.record(request, cached_response);
        assert_eq!(
            cache.lookup(request),
            TransactionLookup::Replay(cached_response)
        );
    }

    #[test]
    fn reused_transaction_with_different_identity_conflicts() {
        let original =
            envelope(7, 0x106, [7, 1, 0, 0, 0, 0, 0, 0], ParameterAction::Save);
        let mut cache = ParameterTransactionCache::new();
        cache.record(original, response(original.request()));

        let different_id =
            envelope(7, 0x107, [7, 1, 0, 0, 0, 0, 0, 0], ParameterAction::Save);
        assert_eq!(cache.lookup(different_id), TransactionLookup::Conflict);

        let different_data =
            envelope(7, 0x106, [7, 2, 0, 0, 0, 0, 0, 0], ParameterAction::Load);
        assert_eq!(cache.lookup(different_data), TransactionLookup::Conflict);
    }

    #[test]
    fn uart_and_new_transactions_are_not_replayed() {
        let original =
            envelope(7, 0x106, [7, 1, 0, 0, 0, 0, 0, 0], ParameterAction::Save);
        let mut cache = ParameterTransactionCache::new();
        cache.record(original, response(original.request()));

        let next =
            envelope(8, 0x106, [8, 1, 0, 0, 0, 0, 0, 0], ParameterAction::Save);
        assert_eq!(cache.lookup(next), TransactionLookup::New);

        let uart = ParameterRequestEnvelope::uart(ParameterRequest {
            route: ParameterRoute::Uart,
            action: ParameterAction::Get(ParameterId::BatteryCells),
        });
        assert_eq!(cache.lookup(uart), TransactionLookup::New);
        cache.record(uart, response(uart.request()));
        assert_eq!(
            cache.lookup(original),
            TransactionLookup::Replay(response(original.request()))
        );
    }
}
