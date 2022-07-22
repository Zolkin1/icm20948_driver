#![no_std]
#![cfg_attr(test, no_main)]

use icm20948_driver as _; // memory layout + panic handler

#[defmt_test::tests]
mod tests {}
