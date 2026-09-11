#![cfg_attr(target_arch = "arm", no_std)]
#![cfg_attr(target_arch = "arm", no_main)]

#[cfg(all(feature = "sensor-encoder", feature = "sensor-hall"))]
compile_error!("select exactly one rotor sensor feature");
#[cfg(not(any(feature = "sensor-encoder", feature = "sensor-hall")))]
compile_error!("select one rotor sensor feature");

#[cfg(target_arch = "arm")]
mod firmware {
    use defmt::{info, warn};
    use embassy_executor::Spawner;
    use embassy_futures::select::{Either, select};
    use embassy_stm32::Config;
    use embassy_stm32::can;
    use embassy_stm32::gpio::{Input, Pull};
    use embassy_stm32::interrupt;
    use embassy_stm32::interrupt::{InterruptExt, Priority};
    use embassy_stm32::usart::{Config as UartConfig, Uart};
    use embassy_time::{Duration, Timer};
    use foc_firmware::bsp_flash::Stm32FlashBackend;
    #[cfg(all(feature = "sensor-encoder", not(feature = "sensor-hall")))]
    use foc_firmware::bsp_hardware::EncoderSensor;
    #[cfg(all(feature = "sensor-hall", not(feature = "sensor-encoder")))]
    use foc_firmware::bsp_hardware::HallSensor;
    use foc_firmware::bsp_hardware::{
        BoardIrqs, InjectedAdcResources, Tim1PwmBridge,
        enable_injected_adc_interrupt, init_injected_adcs, wait_for_adc_frame,
    };
    use foc_firmware::comm_task::{
        can_receive_task, can_telemetry_task, publish_parameter_response,
        publish_telemetry, try_receive_command, try_receive_parameter_request,
        uart_task,
    };
    use foc_firmware::foc_core::{FocConfig, FocController};
    use foc_firmware::foc_math::PhaseDuty;
    use foc_firmware::hardware::{PwmBridge, RotorSensor};
    use foc_firmware::interfaces::{
        Command, ParameterAction, ParameterRequest, ParameterResponse,
        ParameterResultCode, ParameterRoute,
    };
    use foc_firmware::parameter_service::ParameterState;
    use foc_firmware::parameter_store::{
        ParameterStore, SaveOutcome, StoreError,
    };
    use foc_firmware::parameters::ParameterProfileV1;
    use {defmt_rtt as _, panic_probe as _};

    #[cfg(all(feature = "sensor-encoder", not(feature = "sensor-hall")))]
    type SelectedSensor = EncoderSensor<'static>;
    #[cfg(all(feature = "sensor-hall", not(feature = "sensor-encoder")))]
    type SelectedSensor = HallSensor<'static>;

    type BoardParameterStore = ParameterStore<Stm32FlashBackend>;

    fn store_result_code<E>(error: StoreError<E>) -> ParameterResultCode {
        match error {
            | StoreError::Read(_) => ParameterResultCode::FlashRead,
            | StoreError::Erase(_) => ParameterResultCode::FlashErase,
            | StoreError::Program(_) => ParameterResultCode::FlashProgram,
            | StoreError::Verify => ParameterResultCode::Verify,
            | StoreError::InvalidProfile(_) => ParameterResultCode::Invalid,
        }
    }

    fn with_control_sampling_paused<T>(
        pwm: &mut Tim1PwmBridge<'static>,
        controller: &mut FocController,
        operation: impl FnOnce() -> T,
    ) -> T {
        pwm.pause_control_sampling();
        let result = operation();
        pwm.resume_control_sampling();
        controller.resynchronize_control_input();
        result
    }

    fn storage_operation_safe(
        pwm: &Tim1PwmBridge<'static>,
        controller: &FocController,
    ) -> bool {
        controller.storage_safe() && !pwm.is_enabled()
    }

    fn process_parameter_request(
        request: ParameterRequest,
        pwm: &mut Tim1PwmBridge<'static>,
        controller: &mut FocController,
        store: &mut BoardParameterStore,
        parameters: &mut ParameterState,
    ) -> ParameterResponse {
        let mut result = ParameterResultCode::Success;
        let mut value = None;

        match request.action {
            | ParameterAction::Get(parameter) => {
                value = Some(parameters.get(parameter));
            },
            | ParameterAction::Status => {},
            | ParameterAction::Set(parameter, requested) => {
                value = Some(parameters.get(parameter));
                if !storage_operation_safe(pwm, controller) {
                    result = ParameterResultCode::Busy;
                } else {
                    let mut candidate = *parameters;
                    match candidate.set(parameter, requested) {
                        | Err(_) => result = ParameterResultCode::Invalid,
                        | Ok(())
                            if parameter.startup_only()
                                || controller.apply_live_profile(
                                    candidate.working(),
                                ) =>
                        {
                            *parameters = candidate;
                            value = Some(parameters.get(parameter));
                        },
                        | Ok(()) => result = ParameterResultCode::Busy,
                    }
                }
            },
            | ParameterAction::Defaults => {
                if !storage_operation_safe(pwm, controller) {
                    result = ParameterResultCode::Busy;
                } else {
                    let mut candidate = *parameters;
                    candidate.use_defaults();
                    if controller.apply_live_profile(candidate.working()) {
                        *parameters = candidate;
                    } else {
                        result = ParameterResultCode::Busy;
                    }
                }
            },
            | ParameterAction::Load => {
                if !storage_operation_safe(pwm, controller) {
                    result = ParameterResultCode::Busy;
                } else {
                    let loaded =
                        with_control_sampling_paused(pwm, controller, || {
                            store.load_latest()
                        });
                    match loaded {
                        | Ok(Some(stored)) => {
                            let mut candidate = *parameters;
                            candidate.load(stored);
                            if controller
                                .apply_live_profile(candidate.working())
                            {
                                *parameters = candidate;
                            } else {
                                result = ParameterResultCode::Busy;
                            }
                        },
                        | Ok(None) => {
                            result = ParameterResultCode::NoValidRecord
                        },
                        | Err(error) => result = store_result_code(error),
                    }
                }
            },
            | ParameterAction::Save => {
                if !storage_operation_safe(pwm, controller) {
                    result = ParameterResultCode::Busy;
                } else {
                    let outcome =
                        with_control_sampling_paused(pwm, controller, || {
                            store.save(&parameters.working())
                        });
                    match outcome {
                        | Ok(outcome) => {
                            result = if matches!(
                                outcome,
                                SaveOutcome::Unchanged { .. }
                            ) {
                                ParameterResultCode::Unchanged
                            } else {
                                ParameterResultCode::Success
                            };
                            parameters.saved(outcome);
                        },
                        | Err(error) => result = store_result_code(error),
                    }
                }
            },
            | ParameterAction::FactoryReset => {
                if !storage_operation_safe(pwm, controller) {
                    result = ParameterResultCode::Busy;
                } else {
                    let defaults = ParameterProfileV1::default();
                    let outcome =
                        with_control_sampling_paused(pwm, controller, || {
                            store.force_save(&defaults)
                        });
                    match outcome {
                        | Ok(outcome) => {
                            let mut candidate = *parameters;
                            candidate.use_defaults();
                            candidate.saved(outcome);
                            if controller
                                .apply_live_profile(candidate.working())
                            {
                                *parameters = candidate;
                            } else {
                                result = ParameterResultCode::Busy;
                            }
                        },
                        | Err(error) => result = store_result_code(error),
                    }
                }
            },
        }

        ParameterResponse {
            route: request.route,
            action: request.action,
            result,
            value,
            storage: parameters.status(),
        }
    }

    fn synchronize_legacy_parameters(
        command: Command,
        accepted: bool,
        controller: &FocController,
        parameters: &mut ParameterState,
    ) {
        if !accepted
            || !matches!(
                command,
                Command::SetCellCount(_)
                    | Command::SetElectricalZero(_)
                    | Command::SetCurrentPid(_)
                    | Command::SetVelocityPid(_)
            )
        {
            return;
        }

        let previous = parameters.working();
        let mut profile = ParameterProfileV1::from_config(
            &controller.config(),
            previous.encoder_cpr,
        );
        profile.pole_pairs = previous.pole_pairs;
        parameters.replace_working(profile);
    }

    fn can_transaction(route: ParameterRoute) -> Option<u8> {
        match route {
            | ParameterRoute::Can { transaction, .. } => Some(transaction),
            | ParameterRoute::Uart => None,
        }
    }

    #[cfg(any(
        all(feature = "sensor-encoder", not(feature = "sensor-hall")),
        all(feature = "sensor-hall", not(feature = "sensor-encoder"))
    ))]
    #[embassy_executor::task]
    async fn motor_control_task(
        mut pwm: Tim1PwmBridge<'static>,
        mut sensor: SelectedSensor,
        config: FocConfig,
        mut store: BoardParameterStore,
        mut parameters: ParameterState,
    ) {
        enable_injected_adc_interrupt();
        let mut controller = FocController::new(config);
        let control_period = 1.0 / config.pwm_frequency_hz as f32;
        let telemetry_divider = (config.pwm_frequency_hz / 1_000).max(1);
        let mut last_can_response: Option<(
            ParameterRequest,
            ParameterResponse,
        )> = None;

        pwm.disable();
        loop {
            while let Some(command) = try_receive_command() {
                let accepted = controller.handle_command(command);
                synchronize_legacy_parameters(
                    command,
                    accepted,
                    &controller,
                    &mut parameters,
                );
            }

            while let Some(request) = try_receive_parameter_request() {
                let transaction = can_transaction(request.route);
                let response = match (transaction, last_can_response) {
                    | (
                        Some(transaction),
                        Some((cached_request, cached_response)),
                    ) if can_transaction(cached_request.route)
                        == Some(transaction) =>
                    {
                        if request == cached_request {
                            cached_response
                        } else {
                            ParameterResponse {
                                route: request.route,
                                action: request.action,
                                result: ParameterResultCode::Conflict,
                                value: None,
                                storage: parameters.status(),
                            }
                        }
                    },
                    | _ => process_parameter_request(
                        request,
                        &mut pwm,
                        &mut controller,
                        &mut store,
                        &mut parameters,
                    ),
                };

                if transaction.is_some()
                    && !matches!(response.result, ParameterResultCode::Conflict)
                {
                    last_can_response = Some((request, response));
                }
                let _ = publish_parameter_response(response);
            }

            let raw = match select(
                wait_for_adc_frame(),
                Timer::after(Duration::from_micros(100)),
            )
            .await
            {
                | Either::First(frame) => frame,
                | Either::Second(_) => {
                    controller.handle_command(Command::ReportControlOverrun);
                    pwm.disable();
                    continue;
                },
            };

            let rotor = sensor.sample(control_period);
            let output = controller.step(raw, rotor);
            if output.bridge_enabled {
                pwm.set_duty(output.duty);
                pwm.enable();
            } else {
                pwm.disable();
                pwm.set_duty(PhaseDuty::DISABLED);
            }

            if raw.sequence % telemetry_divider == 0 {
                publish_telemetry(output.telemetry);
            }
        }
    }

    #[cfg(any(
        all(feature = "sensor-encoder", not(feature = "sensor-hall")),
        all(feature = "sensor-hall", not(feature = "sensor-encoder"))
    ))]
    #[embassy_executor::main]
    async fn main(spawner: Spawner) {
        let mut config = Config::default();
        {
            use embassy_stm32::rcc::*;
            config.rcc.pll = Some(Pll {
                source: PllSource::Hsi,
                prediv: PllPreDiv::Div4,
                mul: PllMul::Mul85,
                divp: None,
                divq: Some(PllQDiv::Div8),
                divr: Some(PllRDiv::Div2),
            });
            config.rcc.mux.adc12sel = mux::Adcsel::Sys;
            config.rcc.mux.adc345sel = mux::Adcsel::Sys;
            config.rcc.mux.fdcansel = mux::Fdcansel::Pll1Q;
            config.rcc.sys = Sysclk::Pll1R;
        }
        let p = embassy_stm32::init(config);

        let mut parameter_store =
            ParameterStore::new(Stm32FlashBackend::new(p.FLASH));
        let stored_parameters = match parameter_store.load_latest() {
            | Ok(stored) => stored,
            | Err(_) => {
                warn!("parameter Flash read failed; using compiled defaults");
                None
            },
        };
        let working_parameters = stored_parameters
            .map(|stored| stored.profile)
            .unwrap_or_default();
        let parameter_state =
            ParameterState::from_boot(working_parameters, stored_parameters);
        let mut foc_config = FocConfig::default();
        working_parameters.apply_boot(&mut foc_config);

        interrupt::ADC1_2.set_priority(Priority::P1);
        interrupt::TIM3.set_priority(Priority::P4);
        interrupt::USART3.set_priority(Priority::P6);
        interrupt::DMA1_CHANNEL1.set_priority(Priority::P6);
        interrupt::DMA1_CHANNEL2.set_priority(Priority::P6);
        interrupt::FDCAN1_IT0.set_priority(Priority::P6);
        interrupt::FDCAN1_IT1.set_priority(Priority::P6);

        let _software_break_only = Input::new(p.PA6, Pull::Down);
        let pwm = Tim1PwmBridge::new(
            p.TIM1, p.PA8, p.PB13, p.PA9, p.PB14, p.PA10, p.PB15,
        );
        init_injected_adcs(InjectedAdcResources {
            adc1: p.ADC1,
            adc2: p.ADC2,
            adc3: p.ADC3,
            phase_a: p.PA0,
            phase_b: p.PA1,
            phase_c: p.PB0,
            bus_voltage: p.PA2,
            ntc: p.PA3,
        });

        #[cfg(all(feature = "sensor-encoder", not(feature = "sensor-hall")))]
        let sensor = EncoderSensor::new(
            p.TIM3,
            p.PB4,
            p.PB5,
            working_parameters.encoder_cpr,
        );
        #[cfg(all(feature = "sensor-hall", not(feature = "sensor-encoder")))]
        let sensor =
            HallSensor::new(p.TIM3, p.PC6, p.PC7, p.PC8, foc_config.pole_pairs);

        let mut uart_config = UartConfig::default();
        uart_config.baudrate = 115_200;
        let uart = Uart::new(
            p.USART3,
            p.PB11,
            p.PB10,
            p.DMA1_CH1,
            p.DMA1_CH2,
            BoardIrqs,
            uart_config,
        )
        .unwrap();

        let mut can_configurator =
            can::CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, BoardIrqs);
        let can_config = can_configurator
            .config()
            .set_global_filter(can::config::GlobalFilter::reject_all());
        can_configurator.set_config(can_config);
        can_configurator.properties().set_standard_filter(
            can::filter::StandardFilterSlot::_0,
            can::filter::StandardFilter::accept_all_into_fifo0(),
        );
        can_configurator.set_bitrate(500_000);
        let can =
            can_configurator.start(can::OperatingMode::NormalOperationMode);
        let (can_tx, can_rx, _) = can.split();

        spawner.spawn(uart_task(uart).unwrap());
        spawner.spawn(can_receive_task(can_rx).unwrap());
        spawner.spawn(can_telemetry_task(can_tx).unwrap());
        spawner.spawn(
            motor_control_task(
                pwm,
                sensor,
                foc_config,
                parameter_store,
                parameter_state,
            )
            .unwrap(),
        );

        info!("FOC firmware started at 100 kHz PWM with 200 ns dead time");
        core::future::pending::<()>().await;
    }
}

#[cfg(not(target_arch = "arm"))]
fn main() {}
