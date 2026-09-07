use core::cell::RefCell;
#[cfg(feature = "sensor-hall")]
use core::f32::consts::FRAC_PI_3;
use core::f32::consts::TAU;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_stm32::adc::{
    Adc, AdcChannel as _, Exten, InjectedAdc, InjectedAdcTrigger, SampleTime,
};
use embassy_stm32::can;
use embassy_stm32::gpio::{OutputType, Pull};
use embassy_stm32::interrupt::typelevel::{ADC1_2, Interrupt};
#[cfg(feature = "sensor-hall")]
use embassy_stm32::pac;
use embassy_stm32::pac::adc::Adc as AdcRegisters;
#[cfg(feature = "sensor-hall")]
use embassy_stm32::pac::gpio::vals::Idr;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::complementary_pwm::{
    ComplementaryPwm, ComplementaryPwmPin, Mms2, Ossi,
};
#[cfg(feature = "sensor-hall")]
use embassy_stm32::timer::input_capture::{CaptureInput, InputCapture};
use embassy_stm32::timer::low_level::CountingMode;
#[cfg(feature = "sensor-hall")]
use embassy_stm32::timer::low_level::InputCaptureMode;
use embassy_stm32::timer::qei::{Config as QeiConfig, Qei};
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_stm32::timer::{CaptureCompareInterruptHandler, Channel};
use embassy_stm32::triggers::TIM1_TRGO2;
use embassy_stm32::{
    Peri, bind_interrupts, dma, interrupt, peripherals, usart,
};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use static_cell::StaticCell;

use crate::foc_math::PhaseDuty;
#[cfg(feature = "sensor-hall")]
use crate::foc_math::normalize_angle;
use crate::hardware::{PwmBridge, RotorSensor};
use crate::interfaces::{RawAdcFrame, RotorSample};

pub const TIMER_CLOCK_HZ: u32 = 170_000_000;
pub const PWM_FREQUENCY_HZ: u32 = 100_000;
pub const DEAD_TIME_NS: u32 = 200;
pub const DEAD_TIME_TICKS: u16 = 34;
pub const ENCODER_COUNTS_PER_REVOLUTION: u16 = 4_096;
pub const HALL_TIMER_FREQUENCY_HZ: u32 = 10_000;

bind_interrupts!(pub struct BoardIrqs {
    TIM3 => CaptureCompareInterruptHandler<peripherals::TIM3>;
    USART3 => usart::InterruptHandler<peripherals::USART3>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
    FDCAN1_IT0 => can::IT0InterruptHandler<peripherals::FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<peripherals::FDCAN1>;
});

pub struct Tim1PwmBridge<'d> {
    pwm: ComplementaryPwm<'d, peripherals::TIM1>,
    max_duty: u32,
    enabled: bool,
}

impl<'d> Tim1PwmBridge<'d> {
    pub fn new(
        tim: Peri<'d, peripherals::TIM1>,
        phase_a_high: Peri<'d, peripherals::PA8>,
        phase_a_low: Peri<'d, peripherals::PB13>,
        phase_b_high: Peri<'d, peripherals::PA9>,
        phase_b_low: Peri<'d, peripherals::PB14>,
        phase_c_high: Peri<'d, peripherals::PA10>,
        phase_c_low: Peri<'d, peripherals::PB15>,
    ) -> Self {
        let mut pwm = ComplementaryPwm::new(
            tim,
            Some(PwmPin::new(phase_a_high, OutputType::PushPull)),
            Some(ComplementaryPwmPin::new(phase_a_low, OutputType::PushPull)),
            Some(PwmPin::new(phase_b_high, OutputType::PushPull)),
            Some(ComplementaryPwmPin::new(phase_b_low, OutputType::PushPull)),
            Some(PwmPin::new(phase_c_high, OutputType::PushPull)),
            Some(ComplementaryPwmPin::new(phase_c_low, OutputType::PushPull)),
            None,
            None,
            Hertz::hz(PWM_FREQUENCY_HZ),
            CountingMode::CenterAlignedBothInterrupts,
        );

        pwm.set_master_output_enable(false);
        pwm.set_dead_time(DEAD_TIME_TICKS);
        pwm.set_repetition_counter(1);
        pwm.set_mms2(Mms2::Update);
        pwm.set_automatic_output_enable(false);
        pwm.set_break_enable(false);
        pwm.set_break2_enable(false);
        pwm.set_break_input_pin_enable(false);
        pwm.set_break2_input_pin_enable(false);
        pwm.set_off_state_selection_idle(Ossi::IdleLevel);
        pwm.set_normal_output_idle_state(
            &[Channel::Ch1, Channel::Ch2, Channel::Ch3],
            false,
        );
        pwm.set_complementary_output_idle_state(
            &[Channel::Ch1, Channel::Ch2, Channel::Ch3],
            false,
        );

        let max_duty = pwm.get_max_duty();
        let neutral = max_duty / 2;
        for channel in [Channel::Ch1, Channel::Ch2, Channel::Ch3] {
            pwm.set_duty(channel, neutral);
            pwm.enable(channel);
        }

        Self {
            pwm,
            max_duty,
            enabled: false,
        }
    }

    fn duty_ticks(&self, duty: f32) -> u32 {
        (duty.clamp(0.0, 1.0) * self.max_duty as f32) as u32
    }
}

impl PwmBridge for Tim1PwmBridge<'_> {
    fn set_duty(&mut self, duty: PhaseDuty) {
        self.pwm.set_duty(Channel::Ch1, self.duty_ticks(duty.a));
        self.pwm.set_duty(Channel::Ch2, self.duty_ticks(duty.b));
        self.pwm.set_duty(Channel::Ch3, self.duty_ticks(duty.c));
    }

    fn enable(&mut self) {
        if !self.enabled {
            self.pwm.set_master_output_enable(true);
            self.enabled = true;
        }
    }

    fn disable(&mut self) {
        self.pwm.set_master_output_enable(false);
        self.enabled = false;
    }

    fn is_enabled(&self) -> bool {
        self.enabled
    }
}

static ADC_FRAME_SIGNAL: Signal<CriticalSectionRawMutex, RawAdcFrame> =
    Signal::new();
static ADC_SEQUENCE: AtomicU32 = AtomicU32::new(0);
static ADC1_HANDLE: CriticalSectionMutex<
    RefCell<Option<InjectedAdc<'static, AdcRegisters>>>,
> = CriticalSectionMutex::new(RefCell::new(None));
static ADC2_HANDLE: CriticalSectionMutex<
    RefCell<Option<InjectedAdc<'static, AdcRegisters>>>,
> = CriticalSectionMutex::new(RefCell::new(None));
static ADC3_HANDLE: CriticalSectionMutex<
    RefCell<Option<InjectedAdc<'static, AdcRegisters>>>,
> = CriticalSectionMutex::new(RefCell::new(None));

static PHASE_A_PIN: StaticCell<Peri<'static, peripherals::PA0>> =
    StaticCell::new();
static PHASE_B_PIN: StaticCell<Peri<'static, peripherals::PA1>> =
    StaticCell::new();
static PHASE_C_PIN: StaticCell<Peri<'static, peripherals::PB0>> =
    StaticCell::new();
static BUS_VOLTAGE_PIN: StaticCell<Peri<'static, peripherals::PA2>> =
    StaticCell::new();
static NTC_PIN: StaticCell<Peri<'static, peripherals::PA3>> = StaticCell::new();

pub fn init_injected_adcs(
    adc1: Peri<'static, peripherals::ADC1>,
    adc2: Peri<'static, peripherals::ADC2>,
    adc3: Peri<'static, peripherals::ADC3>,
    phase_a: Peri<'static, peripherals::PA0>,
    phase_b: Peri<'static, peripherals::PA1>,
    phase_c: Peri<'static, peripherals::PB0>,
    bus_voltage: Peri<'static, peripherals::PA2>,
    ntc: Peri<'static, peripherals::PA3>,
) {
    let phase_a = PHASE_A_PIN.init(phase_a).reborrow_adc();
    let phase_b = PHASE_B_PIN.init(phase_b).reborrow_adc();
    let phase_c = PHASE_C_PIN.init(phase_c).reborrow_adc();
    let bus_voltage = BUS_VOLTAGE_PIN.init(bus_voltage).reborrow_adc();
    let ntc = NTC_PIN.init(ntc).reborrow_adc();

    let adc1 = Adc::new(adc1, Default::default()).setup_injected_conversions(
        [
            (phase_a, SampleTime::Cycles125),
            (bus_voltage, SampleTime::Cycles475),
            (ntc, SampleTime::Cycles2475),
        ],
        InjectedAdcTrigger::from(TIM1_TRGO2, Exten::RisingEdge),
        true,
    );
    let adc2 = Adc::new(adc2, Default::default()).setup_injected_conversions(
        [(phase_b, SampleTime::Cycles125)],
        InjectedAdcTrigger::from(TIM1_TRGO2, Exten::RisingEdge),
        false,
    );
    let adc3 = Adc::new(adc3, Default::default()).setup_injected_conversions(
        [(phase_c, SampleTime::Cycles125)],
        InjectedAdcTrigger::from(TIM1_TRGO2, Exten::RisingEdge),
        false,
    );

    critical_section::with(|cs| {
        ADC1_HANDLE.borrow(cs).replace(Some(adc1));
        ADC2_HANDLE.borrow(cs).replace(Some(adc2));
        ADC3_HANDLE.borrow(cs).replace(Some(adc3));
    });

    ADC1_2::unpend();
    unsafe { ADC1_2::enable() };
}

pub async fn wait_for_adc_frame() -> RawAdcFrame {
    ADC_FRAME_SIGNAL.wait().await
}

#[interrupt]
unsafe fn ADC1_2() {
    let samples = critical_section::with(|cs| {
        let mut adc1_handle = ADC1_HANDLE.borrow(cs).borrow_mut();
        let mut adc2_handle = ADC2_HANDLE.borrow(cs).borrow_mut();
        let mut adc3_handle = ADC3_HANDLE.borrow(cs).borrow_mut();
        let (Some(adc1), Some(adc2), Some(adc3)) = (
            adc1_handle.as_mut(),
            adc2_handle.as_mut(),
            adc3_handle.as_mut(),
        ) else {
            return None;
        };

        let mut adc1_samples = [0; 3];
        let mut adc2_samples = [0; 1];
        let mut adc3_samples = [0; 1];
        adc1.read_injected_samples(&mut adc1_samples);
        adc2.read_injected_samples(&mut adc2_samples);
        adc3.read_injected_samples(&mut adc3_samples);
        Some((adc1_samples, adc2_samples[0], adc3_samples[0]))
    });

    if let Some((adc1, phase_b, phase_c)) = samples {
        ADC_FRAME_SIGNAL.signal(RawAdcFrame {
            phase_a: adc1[0],
            phase_b,
            phase_c,
            bus_voltage: adc1[1],
            ntc: adc1[2],
            sequence: ADC_SEQUENCE
                .fetch_add(1, Ordering::Relaxed)
                .wrapping_add(1),
            overrun: ADC_FRAME_SIGNAL.signaled(),
        });
    }
}

#[cfg(feature = "sensor-encoder")]
pub struct EncoderSensor<'d> {
    qei: Qei<'d, peripherals::TIM3>,
    counts_per_revolution: i32,
    previous_count: i32,
}

#[cfg(feature = "sensor-encoder")]
impl<'d> EncoderSensor<'d> {
    pub fn new(
        tim: Peri<'d, peripherals::TIM3>,
        channel_a: Peri<'d, peripherals::PB4>,
        channel_b: Peri<'d, peripherals::PB5>,
        counts_per_revolution: u16,
    ) -> Self {
        let counts_per_revolution = counts_per_revolution.max(2);
        let qei = Qei::new(
            tim,
            channel_a,
            channel_b,
            QeiConfig {
                ch1_pull: Pull::Up,
                ch2_pull: Pull::Up,
                auto_reload: counts_per_revolution - 1,
                ..Default::default()
            },
        );
        let previous_count = qei.count() as i32;
        Self {
            qei,
            counts_per_revolution: counts_per_revolution as i32,
            previous_count,
        }
    }
}

#[cfg(feature = "sensor-encoder")]
impl RotorSensor for EncoderSensor<'_> {
    fn sample(&mut self, dt: f32) -> RotorSample {
        let count = self.qei.count() as i32;
        let mut delta = count - self.previous_count;
        let half_range = self.counts_per_revolution / 2;
        if delta > half_range {
            delta -= self.counts_per_revolution;
        } else if delta < -half_range {
            delta += self.counts_per_revolution;
        }
        self.previous_count = count;

        let radians_per_count = TAU / self.counts_per_revolution as f32;
        RotorSample {
            mechanical_angle: count as f32 * radians_per_count,
            mechanical_velocity: if dt.is_finite() && dt > 0.0 {
                delta as f32 * radians_per_count / dt
            } else {
                0.0
            },
            valid: true,
        }
    }
}

#[cfg(feature = "sensor-hall")]
pub struct HallSensor<'d> {
    capture: InputCapture<'d, peripherals::TIM3>,
    pole_pairs: f32,
    previous_state: u8,
    previous_sector: Option<u8>,
    previous_capture: u16,
    electrical_angle: f32,
    electrical_velocity: f32,
    time_since_transition: f32,
}

#[cfg(feature = "sensor-hall")]
impl<'d> HallSensor<'d> {
    pub fn new(
        tim: Peri<'d, peripherals::TIM3>,
        hall_u: Peri<'d, peripherals::PC6>,
        hall_v: Peri<'d, peripherals::PC7>,
        hall_w: Peri<'d, peripherals::PC8>,
        pole_pairs: u8,
    ) -> Self {
        let ch1 = CaptureInput::from_pin(hall_u, Pull::Up).unwrap();
        let ch2 = CaptureInput::from_pin(hall_v, Pull::Up).unwrap();
        let ch3 = CaptureInput::from_pin(hall_w, Pull::Up).unwrap();
        let mut capture = InputCapture::new(
            tim,
            Some(ch1),
            Some(ch2),
            Some(ch3),
            None,
            BoardIrqs,
            Hertz::hz(HALL_TIMER_FREQUENCY_HZ),
            CountingMode::EdgeAlignedUp,
        );
        for channel in [Channel::Ch1, Channel::Ch2, Channel::Ch3] {
            capture
                .set_input_capture_mode(channel, InputCaptureMode::BothEdges);
            capture.enable(channel);
        }

        Self {
            capture,
            pole_pairs: pole_pairs.max(1) as f32,
            previous_state: 0,
            previous_sector: None,
            previous_capture: 0,
            electrical_angle: 0.0,
            electrical_velocity: 0.0,
            time_since_transition: 0.0,
        }
    }

    fn hall_state() -> u8 {
        let inputs = pac::GPIOC.idr().read();
        let u = (inputs.idr(6) == Idr::High) as u8;
        let v = (inputs.idr(7) == Idr::High) as u8;
        let w = (inputs.idr(8) == Idr::High) as u8;
        u | (v << 1) | (w << 2)
    }

    fn sector(state: u8) -> Option<u8> {
        const SECTORS: [u8; 8] = [u8::MAX, 0, 4, 5, 2, 1, 3, u8::MAX];
        let sector = SECTORS[state as usize];
        (sector != u8::MAX).then_some(sector)
    }

    fn capture_for_transition(&self, changed: u8) -> Option<u16> {
        match changed {
            | 1 => Some(self.capture.get_capture_value(Channel::Ch1)),
            | 2 => Some(self.capture.get_capture_value(Channel::Ch2)),
            | 4 => Some(self.capture.get_capture_value(Channel::Ch3)),
            | _ => None,
        }
    }
}

#[cfg(feature = "sensor-hall")]
impl RotorSensor for HallSensor<'_> {
    fn sample(&mut self, dt: f32) -> RotorSample {
        let state = Self::hall_state();
        let Some(sector) = Self::sector(state) else {
            return RotorSample {
                mechanical_angle: normalize_angle(
                    self.electrical_angle / self.pole_pairs,
                ),
                mechanical_velocity: 0.0,
                valid: false,
            };
        };

        if dt.is_finite() && dt > 0.0 {
            self.time_since_transition += dt;
        }

        if self.previous_sector.is_none() {
            self.previous_sector = Some(sector);
            self.previous_state = state;
            self.electrical_angle = sector as f32 * FRAC_PI_3;
        } else if state != self.previous_state {
            let previous_sector = self.previous_sector.unwrap();
            let step = (sector + 6 - previous_sector) % 6;
            let direction = match step {
                | 1 => 1.0,
                | 5 => -1.0,
                | _ => {
                    self.previous_state = state;
                    self.previous_sector = Some(sector);
                    return RotorSample {
                        mechanical_angle: normalize_angle(
                            self.electrical_angle / self.pole_pairs,
                        ),
                        mechanical_velocity: self.electrical_velocity
                            / self.pole_pairs,
                        valid: false,
                    };
                },
            };

            if let Some(capture) =
                self.capture_for_transition(state ^ self.previous_state)
            {
                let ticks = capture.wrapping_sub(self.previous_capture);
                if self.previous_capture != 0 && ticks != 0 {
                    self.electrical_velocity =
                        direction * FRAC_PI_3 * HALL_TIMER_FREQUENCY_HZ as f32
                            / ticks as f32;
                }
                self.previous_capture = capture;
            }

            self.electrical_angle = sector as f32 * FRAC_PI_3;
            self.previous_state = state;
            self.previous_sector = Some(sector);
            self.time_since_transition = 0.0;
        } else if self.time_since_transition > 0.5 {
            self.electrical_velocity = 0.0;
        }

        RotorSample {
            mechanical_angle: normalize_angle(
                self.electrical_angle / self.pole_pairs,
            ),
            mechanical_velocity: self.electrical_velocity / self.pole_pairs,
            valid: true,
        }
    }
}
