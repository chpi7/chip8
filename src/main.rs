use std::{thread, time};
use chip8::Chip8Emulator;

fn main() {
    let mut e = Chip8Emulator::create();

    // e.load_rom("./roms/ibm.ch8".to_string()).unwrap();
    // e.load_rom("./roms/test-corax89.ch8".to_string()).unwrap();
    // e.load_rom("./roms/BC_test.ch8".to_string()).unwrap();
    e.load_rom("./roms/games/BLINKY".to_string()).unwrap();

    // instructions per second
    let ips = 700;
    while !e.is_in_infinite_loop {
        // TODO: keyboard input
        e.step();
        e.dump_regs();
        e.display.draw();
        thread::sleep(time::Duration::from_millis(1000 / ips));
    }
    println!("Emulator done");
    e.stop_heartbeat_timers();
}

