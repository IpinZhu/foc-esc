//! Measurement math for the auto-tune procedure.
//!
//! All functions are pure, allocation-free, and host-testable. They run in
//! the slow (1 kHz) auto-tune task, never in the fast injection loop.

/// Least-squares fit of `y = slope * x + intercept` over `points`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FitResult {
    pub slope: f32,
    pub intercept: f32,
    pub residual_rms: f32,
}

pub fn linear_fit(points: &[(f32, f32)]) -> Option<FitResult> {
    let count = points.len();
    if count < 2 {
        return None;
    }
    let count_f = count as f32;
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_xx = 0.0;
    let mut sum_xy = 0.0;
    for (x, y) in points {
        let (x, y) = (*x, *y);
        if !x.is_finite() || !y.is_finite() {
            return None;
        }
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }
    let denominator = count_f * sum_xx - sum_x * sum_x;
    if !denominator.is_finite() || denominator.abs() < f32::EPSILON {
        return None;
    }
    let slope = (count_f * sum_xy - sum_x * sum_y) / denominator;
    let intercept = (sum_y - slope * sum_x) / count_f;
    if !slope.is_finite() || !intercept.is_finite() {
        return None;
    }
    let mut residual_square_sum = 0.0;
    for (x, y) in points {
        let error = *y - (slope * *x + intercept);
        residual_square_sum += error * error;
    }
    Some(FitResult {
        slope,
        intercept,
        residual_rms: libm::sqrtf(residual_square_sum / count_f),
    })
}

/// Resistance fit over measured `(current, voltage)` points using the
/// model `V = Rs * I + V_offset`, which absorbs inverter dead-time and
/// device drops.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RsFit {
    pub rs: f32,
    pub v_offset: f32,
    pub residual_rms: f32,
    pub r_squared: f32,
}

/// Fits `V = Rs * I + V_offset`. One outlier (the point with the largest
/// residual, when it exceeds three times the RMS residual) is dropped and
/// the fit repeated, provided enough points remain. At most the first
/// eight points are considered.
pub fn fit_rs(points: &[(f32, f32)]) -> Option<RsFit> {
    const MAX_POINTS: usize = 8;
    let count = points.len().min(MAX_POINTS);
    if count < 2 {
        return None;
    }
    let points = &points[..count];
    let mut fit = fit_resistance(points)?;

    if count >= 4 {
        let mut worst_index = None;
        let mut worst_residual = 0.0;
        for (index, (current, voltage)) in points.iter().enumerate() {
            let residual = (voltage - (fit.rs * current + fit.v_offset)).abs();
            if residual > worst_residual {
                worst_residual = residual;
                worst_index = Some(index);
            }
        }
        if let Some(index) = worst_index
            && fit.residual_rms > 0.0
            && worst_residual > 3.0 * fit.residual_rms
        {
            let mut retained = [(0.0f32, 0.0f32); MAX_POINTS];
            let mut kept = 0;
            for (position, point) in points.iter().enumerate() {
                if position != index {
                    retained[kept] = *point;
                    kept += 1;
                }
            }
            if let Some(refit) = fit_resistance(&retained[..kept]) {
                fit = refit;
            }
        }
    }
    if !fit.rs.is_finite() || fit.rs <= 0.0 {
        return None;
    }
    Some(fit)
}

fn fit_resistance(points: &[(f32, f32)]) -> Option<RsFit> {
    let fit = linear_fit(points)?;
    let mean =
        points.iter().map(|(_, y)| *y).sum::<f32>() / points.len() as f32;
    let mut total = 0.0;
    let mut residual = 0.0;
    for (x, y) in points {
        total += (y - mean) * (y - mean);
        let error = y - (fit.slope * x + fit.intercept);
        residual += error * error;
    }
    let r_squared = if total > f32::EPSILON {
        1.0 - residual / total
    } else {
        0.0
    };
    Some(RsFit {
        rs: fit.slope,
        v_offset: fit.intercept,
        residual_rms: fit.residual_rms,
        r_squared,
    })
}

pub fn median(values: &mut [f32]) -> Option<f32> {
    if values.is_empty() || values.iter().any(|value| !value.is_finite()) {
        return None;
    }
    sort_ascending(values);
    let middle = values.len() / 2;
    Some(if values.len().is_multiple_of(2) {
        (values[middle - 1] + values[middle]) / 2.0
    } else {
        values[middle]
    })
}

/// Insertion sort; the sampled buffers here hold at most a few dozen
/// entries and `slice::sort` is not available without `alloc`.
fn sort_ascending(values: &mut [f32]) {
    for index in 1..values.len() {
        let key = values[index];
        let mut position = index;
        while position > 0 && values[position - 1] > key {
            values[position] = values[position - 1];
            position -= 1;
        }
        values[position] = key;
    }
}

/// Removes values that deviate from the median by more than three scaled
/// median-absolute-deviations. Returns the retained values in order.
pub fn reject_outliers(values: &[f32]) -> ([f32; 32], usize) {
    let mut buffer = [0.0; 32];
    let count = values.len().min(32);
    if count == 0 {
        return (buffer, 0);
    }
    buffer[..count].copy_from_slice(&values[..count]);
    let Some(center) = median(&mut buffer[..count]) else {
        return (buffer, 0);
    };
    let mut deviations = [0.0; 32];
    for (deviation, value) in deviations[..count].iter_mut().zip(&buffer) {
        *deviation = (*value - center).abs();
    }
    let Some(mad) = median(&mut deviations[..count]) else {
        return (buffer, 0);
    };
    let limit = 3.0 * mad.max(f32::EPSILON);
    let mut kept = 0;
    for index in 0..count {
        if (buffer[index] - center).abs() <= limit {
            buffer[kept] = buffer[index];
            kept += 1;
        }
    }
    (buffer, kept)
}

/// Median of the values that survive three-sigma outlier rejection.
pub fn analyze_rejected_median(values: &[f32]) -> Option<f32> {
    let (buffer, kept) = reject_outliers(values);
    if kept == 0 {
        return None;
    }
    let mut values = [0.0f32; 32];
    values[..kept].copy_from_slice(&buffer[..kept]);
    median(&mut values[..kept])
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepMetrics {
    pub peak: f32,
    pub overshoot: f32,
    pub rise_time: f32,
    pub settling_time: f32,
    pub rms_error: f32,
    pub oscillations: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct StepLimits {
    pub max_overshoot: f32,
    pub max_oscillations: u32,
    pub max_rms_ratio: f32,
}

impl Default for StepLimits {
    fn default() -> Self {
        Self {
            max_overshoot: 0.15,
            max_oscillations: 2,
            max_rms_ratio: 0.4,
        }
    }
}

impl StepMetrics {
    /// Analyzes a recorded step response `(time, response)` towards
    /// `target`. Returns `None` when the record is too short or contains
    /// non-finite samples.
    pub fn analyze(samples: &[(f32, f32)], target: f32) -> Option<StepMetrics> {
        if samples.len() < 4 || !target.is_finite() || target == 0.0 {
            return None;
        }
        if samples
            .iter()
            .any(|(time, value)| !time.is_finite() || !value.is_finite())
        {
            return None;
        }
        let target_abs = target.abs();
        let mut peak = f32::MIN;
        for (_, value) in samples {
            peak = peak.max(*value);
        }
        let overshoot = ((peak - target) / target_abs).max(0.0);

        let threshold_low = 0.1 * target_abs;
        let threshold_high = 0.9 * target_abs;
        let cross = |threshold: f32| {
            samples.iter().find_map(|(time, value)| {
                (value.abs() >= threshold).then_some(*time)
            })
        };
        let rise_time = match (cross(threshold_low), cross(threshold_high)) {
            | (Some(low), Some(high)) => (high - low).max(0.0),
            | _ => 0.0,
        };

        let mut settling_time = 0.0;
        for (time, value) in samples {
            if (value - target).abs() > 0.05 * target_abs {
                settling_time = *time;
            }
        }

        let mut square_sum = 0.0;
        for (_, value) in samples {
            let error = value - target;
            square_sum += error * error;
        }
        let rms_error = libm::sqrtf(square_sum / samples.len() as f32);

        let mut oscillations = 0u32;
        let mut previous_error = samples[0].1 - target;
        for (_, value) in &samples[1..] {
            let error = *value - target;
            if error.signum() != previous_error.signum()
                && error.signum() != 0.0
                && previous_error.signum() != 0.0
                && error.abs() > 0.02 * target_abs
                && previous_error.abs() > 0.02 * target_abs
            {
                oscillations += 1;
            }
            previous_error = error;
        }

        Some(StepMetrics {
            peak,
            overshoot,
            rise_time,
            settling_time,
            rms_error,
            oscillations,
        })
    }

    /// Returns true when the response is well damped and accurate enough:
    /// overshoot and ringing within limits, settled inside the record, and
    /// a total RMS error that indicates the response actually moved.
    pub fn passes(
        &self,
        limits: &StepLimits,
        target: f32,
        window: f32,
    ) -> bool {
        self.overshoot <= limits.max_overshoot
            && self.oscillations <= limits.max_oscillations
            && self.settling_time <= window * 0.9
            && self.rms_error <= limits.max_rms_ratio * target.abs()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linear_fit_recovers_exact_line() {
        let points = [(0.0, 1.0), (1.0, 3.0), (2.0, 5.0), (3.0, 7.0)];
        let fit = linear_fit(&points).unwrap();
        assert!((fit.slope - 2.0).abs() < 1.0e-5);
        assert!((fit.intercept - 1.0).abs() < 1.0e-5);
        assert!(fit.residual_rms < 1.0e-5);
    }

    #[test]
    fn linear_fit_rejects_degenerate_input() {
        assert!(linear_fit(&[]).is_none());
        assert!(linear_fit(&[(1.0, 1.0)]).is_none());
        assert!(linear_fit(&[(1.0, 1.0), (1.0, 2.0)]).is_none());
        assert!(linear_fit(&[(f32::NAN, 1.0), (2.0, 3.0)]).is_none());
    }

    #[test]
    fn fit_rs_recovers_resistance_and_offset() {
        let rs = 0.05;
        let offset = 0.4;
        let points: Vec<(f32, f32)> = [0.5, 1.0, 1.5, 2.0]
            .into_iter()
            .map(|current| (current, rs * current + offset))
            .collect();
        let fit = fit_rs(&points).unwrap();
        assert!((fit.rs - rs).abs() < 1.0e-4);
        assert!((fit.v_offset - offset).abs() < 1.0e-4);
        assert!(fit.r_squared > 0.999);
    }

    #[test]
    fn fit_rs_drops_single_outlier() {
        let mut points: Vec<(f32, f32)> = [0.5, 1.0, 1.5, 2.0, 2.5]
            .into_iter()
            .map(|current| (current, 0.05 * current + 0.4))
            .collect();
        points[2] = (1.5, 5.0);
        let fit = fit_rs(&points).unwrap();
        assert!((fit.rs - 0.05).abs() < 1.0e-3);
    }

    #[test]
    fn median_handles_odd_even_and_non_finite() {
        assert_eq!(median(&mut [3.0, 1.0, 2.0]), Some(2.0));
        assert_eq!(median(&mut [4.0, 1.0, 2.0, 3.0]), Some(2.5));
        assert_eq!(median(&mut [1.0, f32::NAN]), None);
        assert_eq!(median(&mut []), None);
    }

    #[test]
    fn reject_outliers_removes_extreme_values() {
        let values = [10.0, 10.1, 9.9, 10.05, 100.0];
        let (buffer, kept) = reject_outliers(&values);
        assert_eq!(kept, 4);
        let mean: f32 = buffer[..kept].iter().sum::<f32>() / kept as f32;
        assert!((mean - 10.012_5).abs() < 1.0e-3);
    }

    #[test]
    fn step_metrics_on_first_order_response() {
        // i(t) = target * (1 - e^(-t/tau)), tau = 2 ms, sampled at 1 ms.
        let target = 1.0;
        let tau = 0.002;
        let samples: Vec<(f32, f32)> = (0..40)
            .map(|index| {
                let time = index as f32 * 0.001;
                (time, target * (1.0 - (-time / tau).exp()))
            })
            .collect();
        let metrics = StepMetrics::analyze(&samples, target).unwrap();
        assert!(metrics.overshoot < 0.001);
        assert_eq!(metrics.oscillations, 0);
        assert!((metrics.rise_time - tau * 2.197_2).abs() < 0.002);
        assert!(metrics.settling_time <= 0.012);
        assert!(metrics.passes(&StepLimits::default(), target, 0.039));
    }

    #[test]
    fn step_metrics_detect_overshoot_and_oscillation() {
        // Decaying oscillation around the target.
        let samples: Vec<(f32, f32)> = (0..40)
            .map(|index| {
                let time = index as f32 * 0.001;
                let decay = (-time / 0.01).exp();
                (time, 1.0 + decay * (time * 2_000.0).sin())
            })
            .collect();
        let metrics = StepMetrics::analyze(&samples, 1.0).unwrap();
        assert!(metrics.overshoot > 0.5);
        assert!(metrics.oscillations >= 2);
        assert!(!metrics.passes(&StepLimits::default(), 1.0, 0.039));
    }

    #[test]
    fn step_metrics_reject_bad_input() {
        assert!(StepMetrics::analyze(&[], 1.0).is_none());
        assert!(StepMetrics::analyze(&[(0.0, 0.0)], 1.0).is_none());
        assert!(
            StepMetrics::analyze(&[(0.0, 0.0), (0.1, f32::NAN)], 1.0).is_none()
        );
    }
}
