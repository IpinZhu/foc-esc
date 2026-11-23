//! Field-oriented control core: the hardware-independent controller,
//! transforms and modulation, and the fixed-period PID.

mod feedback;
pub mod foc_core;
pub mod foc_math;
pub mod pid;
pub mod sensorless;
