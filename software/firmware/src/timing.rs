pub const TIMER_CLOCK_HZ: u32 = 170_000_000;
pub const PWM_FREQUENCY_HZ: u32 = 100_000;
pub const DEAD_TIME_NS: u32 = 200;
pub const DEAD_TIME_TICKS: u16 = dead_time_ticks(TIMER_CLOCK_HZ, DEAD_TIME_NS);

const NANOS_PER_SECOND: u64 = 1_000_000_000;

const fn dead_time_ticks(timer_clock_hz: u32, dead_time_ns: u32) -> u16 {
    let scaled = timer_clock_hz as u64 * dead_time_ns as u64;
    assert!(scaled.is_multiple_of(NANOS_PER_SECOND));
    let ticks = scaled / NANOS_PER_SECOND;
    assert!(ticks <= u16::MAX as u64);
    ticks as u16
}

const _: () = assert!(TIMER_CLOCK_HZ.is_multiple_of(PWM_FREQUENCY_HZ * 2));
const _: () = assert!(DEAD_TIME_TICKS == 34);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::foc_core::{
        DEFAULT_DEAD_TIME_NS, DEFAULT_PWM_FREQUENCY_HZ, FocConfig,
    };

    #[test]
    fn board_and_control_timing_share_one_source() {
        assert_eq!(FocConfig::default().pwm_frequency_hz, PWM_FREQUENCY_HZ);
        assert_eq!(DEFAULT_PWM_FREQUENCY_HZ, PWM_FREQUENCY_HZ);
        assert_eq!(DEFAULT_DEAD_TIME_NS, DEAD_TIME_NS);
        assert_eq!(DEAD_TIME_TICKS, 34);
    }
}
