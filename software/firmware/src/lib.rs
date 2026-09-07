#![cfg_attr(not(test), no_std)]

#[cfg(target_arch = "arm")]
pub mod bsp_hardware;
pub mod comm_task;
pub mod foc_core;
pub mod foc_math;
pub mod hardware;
pub mod interfaces;
pub mod pid;
