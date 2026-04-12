#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

#[entry]
fn main() -> ! {
    loop {
        let mut _i: usize = 0;
        for _ in 0..10_000 {
            cortex_m::asm::nop();
            _i += 1;
        }
    }
}
