use foc_sim::plant::{Load, Plant, PlantParams, PlantState, StationaryVoltage};

#[test]
fn locked_d_axis_rl_matches_analytic_step() {
    let params = PlantParams::default();
    let mut plant = Plant::new(params).unwrap();
    for _ in 0..1_000 {
        plant.advance(
            Some(StationaryVoltage {
                alpha: 1.0,
                beta: 0.0,
            }),
            1.0e-6,
        );
    }
    let expected = (1.0 - (-params.resistance / params.ld * 0.001).exp()) / params.resistance;
    assert!((plant.state.id - expected).abs() < 1.0e-6);
    assert!(plant.state.iq.abs() < 1.0e-10);
    assert!(plant.state.mechanical_speed.abs() < 1.0e-10);
}

#[test]
fn torque_sign_and_mechanical_acceleration_follow_iq() {
    for sign in [-1.0, 1.0] {
        let params = PlantParams {
            friction: 0.0,
            ..PlantParams::default()
        };
        let mut plant = Plant::new(params).unwrap();
        plant.state.iq = sign;
        plant.advance(
            Some(StationaryVoltage {
                alpha: 0.0,
                beta: params.resistance * sign,
            }),
            1.0e-6,
        );
        let initial_accel =
            1.5 * f64::from(params.pole_pairs) * params.flux * sign / params.inertia;
        assert!((plant.state.mechanical_speed - initial_accel * 1.0e-6).abs() < 1.0e-7);
        assert_eq!(plant.state.mechanical_speed.signum(), sign);
    }
}

#[test]
fn phase_currents_balance_for_any_angle() {
    let params = PlantParams::default();
    for angle in [0.0, 0.7, 1.5, 4.0, 6.1] {
        let state = PlantState {
            id: 1.0,
            iq: 2.0,
            mechanical_angle: angle,
            mechanical_speed: 0.0,
        };
        assert!(state.phase_currents(params).into_iter().sum::<f64>().abs() < 1.0e-14);
    }
}

#[test]
fn load_changes_speed_without_assigning_it() {
    let params = PlantParams::default();
    let mut free = Plant::new(params).unwrap();
    let mut loaded = Plant::new(params).unwrap();
    free.state.iq = 1.0;
    loaded.state.iq = 1.0;
    loaded.load = Load::Signed(0.03);
    for _ in 0..100 {
        let v = Some(StationaryVoltage {
            alpha: 0.0,
            beta: 1.0,
        });
        free.advance(v, 1.0e-6);
        loaded.advance(v, 1.0e-6);
    }
    assert!(free.state.mechanical_speed > loaded.state.mechanical_speed);
    assert!(loaded.state.mechanical_speed > 0.0);
}

#[test]
fn smaller_physics_step_converges() {
    let params = PlantParams::default();
    let mut coarse = Plant::new(params).unwrap();
    let mut fine = Plant::new(params).unwrap();
    let voltage = Some(StationaryVoltage {
        alpha: 0.0,
        beta: 1.5,
    });
    for _ in 0..10_000 {
        coarse.advance(voltage, 1.0e-6);
    }
    for _ in 0..20_000 {
        fine.advance(voltage, 0.5e-6);
    }
    assert!((coarse.state.mechanical_speed - fine.state.mechanical_speed).abs() < 0.01);
    assert!((coarse.state.id - fine.state.id).abs() < 0.01);
    assert!((coarse.state.iq - fine.state.iq).abs() < 0.01);
}

#[test]
fn electrical_input_balances_magnetic_mechanical_and_copper_energy() {
    let params = PlantParams::default();
    let mut plant = Plant::new(params).unwrap();
    plant.state = PlantState {
        id: 0.3,
        iq: 0.4,
        mechanical_angle: 0.2,
        mechanical_speed: 1.0,
    };
    let voltage = StationaryVoltage {
        alpha: 0.2,
        beta: 0.7,
    };
    let stored = |s: PlantState| {
        0.75 * (params.ld * s.id * s.id + params.lq * s.iq * s.iq)
            + 0.5 * params.inertia * s.mechanical_speed * s.mechanical_speed
    };
    let power = |s: PlantState| {
        let (sin, cos) = s.electrical_angle(params).sin_cos();
        let vd = cos * voltage.alpha + sin * voltage.beta;
        let vq = cos * voltage.beta - sin * voltage.alpha;
        1.5 * (vd * s.id + vq * s.iq)
            - 1.5 * params.resistance * (s.id * s.id + s.iq * s.iq)
            - params.friction * s.mechanical_speed * s.mechanical_speed
    };
    let start_energy = stored(plant.state);
    let mut net = 0.0;
    for _ in 0..1_000 {
        let before = power(plant.state);
        plant.advance(Some(voltage), 1.0e-6);
        let after = power(plant.state);
        net += 0.5 * (before + after) * 1.0e-6;
    }
    assert!((stored(plant.state) - start_energy - net).abs() < 2.0e-8);
}
