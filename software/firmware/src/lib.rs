#![cfg_attr(not(test), no_std)]

#[cfg(any(
    all(feature = "sensor-encoder", feature = "sensor-hall"),
    all(
        feature = "sensorless",
        any(feature = "sensor-hall", feature = "sensor-encoder")
    )
))]
compile_error!("select exactly one rotor sensor feature");
#[cfg(not(any(
    feature = "sensor-encoder",
    feature = "sensor-hall",
    feature = "sensorless"
)))]
compile_error!("select one rotor sensor feature");

pub mod autotune;
pub mod bsp;
pub mod comm;
pub mod control;
pub mod interfaces;
pub mod params;
pub mod timing;
