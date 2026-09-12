//! Board-support layer: hardware abstraction traits plus the STM32
//! peripheral implementations (compiled only for the ARM target).

#[cfg(target_arch = "arm")]
pub mod flash;
pub mod hardware;
#[cfg(target_arch = "arm")]
pub mod stm32;
