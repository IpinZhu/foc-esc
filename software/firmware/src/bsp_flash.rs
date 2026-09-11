use embassy_stm32::Peri;
use embassy_stm32::flash::{
    Blocking, Error, FLASH_SIZE, Flash, MAX_ERASE_SIZE, WRITE_SIZE,
};
use embassy_stm32::peripherals;

use crate::parameter_store::{
    FlashBackend, SLOT_A_OFFSET, SLOT_B_OFFSET, SLOT_SIZE,
};

const _: () = assert!(FLASH_SIZE == 0x0004_0000);
const _: () = assert!(MAX_ERASE_SIZE == SLOT_SIZE as usize);
const _: () = assert!(WRITE_SIZE == 8);
const _: () = assert!(SLOT_B_OFFSET + SLOT_SIZE == FLASH_SIZE as u32);
const _: () = assert!(SLOT_A_OFFSET + SLOT_SIZE == SLOT_B_OFFSET);

pub struct Stm32FlashBackend {
    flash: Flash<'static, Blocking>,
}

impl Stm32FlashBackend {
    pub fn new(flash: Peri<'static, peripherals::FLASH>) -> Self {
        Self {
            flash: Flash::new_blocking(flash),
        }
    }
}

impl FlashBackend for Stm32FlashBackend {
    type Error = Error;

    fn read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.flash.blocking_read(offset, bytes)
    }

    fn erase_page(&mut self, offset: u32) -> Result<(), Self::Error> {
        self.flash.blocking_erase(offset, offset + SLOT_SIZE)
    }

    fn program_double_word(
        &mut self,
        offset: u32,
        bytes: &[u8; 8],
    ) -> Result<(), Self::Error> {
        self.flash.blocking_write(offset, bytes)
    }
}
