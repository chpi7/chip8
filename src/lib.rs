use std::io::Error;
use std::fs;
use std::thread::{spawn, JoinHandle};
use std::sync::{Arc, Mutex};
use std::sync::mpsc::{channel, Sender};

#[derive(Debug)]
pub struct RegisterFile {
    pub pc: u16,
    pub i: u16,
    pub v: [u8; 16],
}

impl RegisterFile {
    fn create() -> RegisterFile {
        RegisterFile { 
            pc: 0,
            i: 0,
            v: [0; 16],
        }
    }
}

#[derive(Debug)]
pub struct Display {
    pub width: usize,
    pub height: usize,
    memory: [u64; 32],
}

impl Display {
    fn create() -> Display {
        let mut d = Display { 
            width: 64,
            height: 32,
            memory: [0u64; 32],
        };
        // Clear will also initialize the vector to the correct size.
        d.clear();
        return d;
    }

    pub fn set_pixel(&mut self, x: usize, y: usize) {
        let mask = 1u64 << (63 - x);
        self.memory[y] |= mask;
    }

    pub fn draw_sprite_row(&mut self, x: usize, y: usize, sprite_row: u8) -> bool {
        // align to left side
        let mut shifted = (sprite_row as u64) << (64 - 8);
        // then shift to correct x coordinate (this will clip the sprite on the right)
        shifted >>= x;
        let old = self.memory[y];
        // a pixel is turned of if it is 1 now and the sprite is also 1
        let any_turned_off = old & shifted;
        self.memory[y] = old ^ shifted;

        any_turned_off != 0
    }

    pub fn draw(&self) {
        for y in 0..self.height {
            let row = self.memory[y];
            for x in 0..self.width {
                let mask = 1u64 << (63 - x);
                let pixel = row & mask;
                print!("{}", if pixel.eq(&0u64) {"  "} else {"██"});
            } 
            println!("");
        } 
    }

    pub fn clear(&mut self) {
        println!("Clearing display");
        for i in 0..32 {
            self.memory[i] = 0;
        }
    }
}

#[derive(Debug)]
pub struct Timer {
    pub value: u8,
}

impl Timer {
    fn create() -> Timer {
        Timer { 
            value: 0,
        }
    }
}

fn get_nibble(opcode: u16, idx: u8) -> u8 {
    let mask = 0x000fu16;
    let shift_nibbles = 3 - idx;
    let shifted_opcode = opcode >> (shift_nibbles * 4);
    (shifted_opcode & mask) as u8
}

fn hex_dump(bytes: &Vec<u8>) {
    let bytes_per_row = 16;
    for i in 0..bytes.len() {
        // row start (end of last)
        if i % bytes_per_row == 0 {
            if i > 0 {
                println!();
            }
            print!("{:04x}: ", i);
        } else {
            // if its not the line end / new line start, output  a seperator
            print!(" ");
        }
        print!("{:02x}", bytes[i]);
    }
    println!();
}

#[derive(Debug)]
pub struct Chip8EmulatorOptions {
    pub load_store_inc_i: bool,
    // TODO other options
}

impl Chip8EmulatorOptions {
    pub fn default() -> Chip8EmulatorOptions {
        Chip8EmulatorOptions {
            load_store_inc_i: false, // modern behaviour preferred
        }
    }
}

#[derive(Debug)]
pub struct Chip8Emulator {
    pub options: Chip8EmulatorOptions,
    pub memory: [u8; 4096],
    pub registers: RegisterFile,
    pub display: Display,
    pub stack: Vec<u16>,
    pub timer_delay: Arc<Mutex<Timer>>,
    pub timer_sound: Arc<Mutex<Timer>>,
    timer_thread: JoinHandle<()>,
    timer_thread_tx: Sender<bool>,
    rnd_byte_state: u32,
    pub keyboard: u16, // kb has 16 keys labelled 0 to F
    keyboard_last_kc: u8,
    keyboard_has_press: bool,
    pub is_in_infinite_loop: bool,
}

impl Chip8Emulator {
    pub fn create() -> Chip8Emulator {
        let (tx, rx) = channel();
        let td = Arc::new(Mutex::new(Timer::create()));
        let ts = Arc::new(Mutex::new(Timer::create()));
        let mut e = Chip8Emulator { 
            options: Chip8EmulatorOptions::default(),
            memory: [0; 4096],
            registers: RegisterFile::create(),
            display: Display::create(),
            stack: Vec::new(),
            timer_sound: Arc::clone(&ts),
            timer_delay: Arc::clone(&td),
            timer_thread: spawn(move ||{
                let timer_sound = Arc::clone(&ts);
                let timer_delay = Arc::clone(&td);
                loop {
                    let should_stop = rx.try_recv().unwrap_or(false);
                    if should_stop {
                        break;
                    }
                    let mut ts = timer_sound.lock().unwrap(); 
                    let mut td = timer_delay.lock().unwrap();
                    // println!("Timer heartbeat");
                    if ts.value > 0 {
                        ts.value -= 1;
                    }
                    if td.value > 0 {
                        td.value -= 1;
                    }
                    let sleep_duration = std::time::Duration::from_millis(1000 / 60);
                    std::thread::sleep(sleep_duration);
                }
            }),
            timer_thread_tx: tx,
            rnd_byte_state: 0,
            keyboard: 0,
            keyboard_last_kc: 0,
            keyboard_has_press: false,
            is_in_infinite_loop: false,
        };
        
        // load font at 0x050 - 0x09f
        let font_data = [
            0xF0u8, 0x90, 0x90, 0x90, 0xF0, // 0
            0x20, 0x60, 0x20, 0x20, 0x70, // 1
            0xF0, 0x10, 0xF0, 0x80, 0xF0, // 2
            0xF0, 0x10, 0xF0, 0x10, 0xF0, // 3
            0x90, 0x90, 0xF0, 0x10, 0x10, // 4
            0xF0, 0x80, 0xF0, 0x10, 0xF0, // 5
            0xF0, 0x80, 0xF0, 0x90, 0xF0, // 6
            0xF0, 0x10, 0x20, 0x40, 0x40, // 7
            0xF0, 0x90, 0xF0, 0x90, 0xF0, // 8
            0xF0, 0x90, 0xF0, 0x10, 0xF0, // 9
            0xF0, 0x90, 0xF0, 0x90, 0x90, // A
            0xE0, 0x90, 0xE0, 0x90, 0xE0, // B
            0xF0, 0x80, 0x80, 0x80, 0xF0, // C
            0xE0, 0x90, 0x90, 0x90, 0xE0, // D
            0xF0, 0x80, 0xF0, 0x80, 0xF0, // E
            0xF0, 0x80, 0xF0, 0x80, 0x80  // F
        ];

        let load_offset = 0x50;
        for idx in 0..16 {
            for row in 0..5 {
                e.memory[load_offset + (5 * idx + row)] = font_data[5 * idx + row];
            }
        }
        
        return e;
    }

    pub fn stop_heartbeat_timers(self) {
        self.timer_thread_tx.send(true).expect("Unable to send timer thead kill signal");
        let _  = self.timer_thread.join();
    }

    fn get_timer_value(&mut self, timer: Arc<Mutex<Timer>>) -> u8 {
        timer.lock().unwrap().value
    }

    fn set_timer_value(&mut self, timer: Arc<Mutex<Timer>>, value: u8) {
        timer.lock().unwrap().value = value;
    }

    pub fn set_kb_key(&mut self, keycode: u8, pressed: bool) {
        let offset = keycode;
        // println!("keyboard = {:016b}, mask = {:016b}, kc = {:x}", self.keyboard, mask, keycode);
        if pressed {
            self.keyboard |= 1 << offset;
            self.keyboard_has_press = true;
            self.keyboard_last_kc = keycode;
        } else {
            self.keyboard &= !(1 << offset);
        }
        // println!("after keyboard = {:016b}", self.keyboard);
    }

    pub fn get_kb_key(&self, keycode: u8) -> bool {
        let offset = keycode;
        let masked = self.keyboard & (1 << offset);
        masked != 0
    }

    pub fn load_rom(&mut self, path: String) -> Result<(), Error> {
       let bytes = fs::read(&path)?;
       let mem_load_offset = 0x200u16;
       println!("Loading {} with length {}", &path, bytes.len());
       hex_dump(&bytes);
       for i in 0..bytes.len() {
            self.memory[mem_load_offset as usize + i] = bytes[i];
       }

       self.registers.pc = mem_load_offset;

       Ok(())
    }

    fn read_u16(&self, address: usize) -> u16 {
        // Chip8 is big endian:
        let upper = self.memory[address] as u16;
        let lower = self.memory[address + 1] as u16;
        // println!("upper={:02x} lower={:02x}", upper, lower);
        let result = (upper << 8) | lower;
        result
    }

    fn next_rnd_byte(&mut self) -> u8 {
        // LCG prng (glibc params)
        let a = 1103515245u32;
        let c = 12345u32;
        let m = 1u32 << 31;
        self.rnd_byte_state = (a.wrapping_mul(self.rnd_byte_state).wrapping_add(c)) % m; 

        // Does this preserve theoretical properties of the lcg? Prob not, but should be fine...
        self.rnd_byte_state as u8
    }

    pub fn dump_regs(&self) {
        print!("[pc={:x}, i={:04x}, ", self.registers.pc, self.registers.i);
        for idx in 0..16 {
            print!("v{:01x}={:02x}", idx, self.registers.v[idx as usize]);
            if idx != 15 {
                print!(", ");
            }
        }
        println!("]");
    }

    // One run loop step (fetch, decode, execute)
    pub fn step(&mut self) {
        // fetch
        println!("-----------------------------------");
        let instruction = self.read_u16(self.registers.pc as usize);
        // println!("Fetch 0x{:x}", self.registers.pc);
        println!("op={:04x}", instruction);
        self.registers.pc += 2;

        // decode
        let op_type = get_nibble(instruction, 0);
        match op_type {
            0x00 => {
                // could also be 0x0NNN which we dont implement
                match instruction {
                    0x00e0 => self.display.clear(),
                    0x00ee => {
                        // return from subroutine call
                        self.registers.pc = self.stack.pop().unwrap();
                    },
                    _ => (),
                }
            },
            0x01 => { // 1NNN
                let addr = instruction & 0x0FFF;
                println!("go to {:04x}", addr);
                if self.registers.pc - 2 == addr {
                    self.is_in_infinite_loop = true;
                }
                self.registers.pc = addr;
            },
            0x02 => { // 2NNN
                let addr = instruction & 0x0FFF;
                self.stack.push(self.registers.pc);
                self.registers.pc = addr;
                println!("call {:04x}", addr);
            },
            0x03 => { // 3XNN skip instruction if vx == nn
                let immediate = (instruction & 0x0FF) as u8;
                let reg = get_nibble(instruction, 1);
                let vx = self.registers.v[reg as usize];
                println!("skip op if vx == nn");
                if vx == immediate {
                    println!("skipping");
                    self.registers.pc += 2;
                }
            },
            0x04 => { // 4XNN skip instruction if vx != nn
                let immediate = (instruction & 0x0FF) as u8;
                let reg = get_nibble(instruction, 1);
                let vx = self.registers.v[reg as usize];
                println!("skip op if vx != nn");
                if vx != immediate {
                    println!("skipping");
                    self.registers.pc += 2;
                }
            },
            0x05 => { // 5XY0 skip instrucion if vx == vx
                let x = get_nibble(instruction, 1);
                let vx = self.registers.v[x as usize];
                let y = get_nibble(instruction, 2);
                let vy = self.registers.v[y as usize];
                println!("skip op if vx == vy");
                if vx == vy {
                    println!("skipping");
                    self.registers.pc += 2;
                }
            },
            0x06 => { // 6XNN
                let value = (instruction & 0x0FF) as u8;
                let reg = get_nibble(instruction, 1);
                assert!(reg < 16u8);
                self.registers.v[reg as usize] = value;
                println!("assign v{:x} = {:02x}", reg, value);
            },
            0x07 => { // 7XNN
                let value = (instruction & 0x0FF) as u8;
                let reg = get_nibble(instruction, 1);
                assert!(reg < 16u8);
                self.registers.v[reg as usize] = self.registers.v[reg as usize].wrapping_add(value);
                println!("assign v{:x} += {:02x}", reg, value);
            },
            0x08 => {
                let op_variant = get_nibble(instruction, 3);
                let x = get_nibble(instruction, 1);
                let y = get_nibble(instruction, 2);
                let vx = self.registers.v[x as usize];
                let vy = self.registers.v[y as usize];
                match op_variant {
                    0 => { // 8XY0
                        println!("set vx := vy");
                        self.registers.v[x as usize] = vy;
                    },
                    1 => { // 8XY1
                        println!("set vx := vx | vy");
                        self.registers.v[x as usize] = vx | vy;
                    },
                    2 => { // 8XY2
                        println!("set vx := vx & vy");
                        self.registers.v[x as usize] = vx & vy;
                    },
                    3 => { // 8XY3
                        println!("set vx := vx ^ vy");
                        self.registers.v[x as usize] = vx ^ vy;
                    },
                    4 => { // 8XY4
                        println!("set vx := vx + vy");
                        self.registers.v[x as usize] = vx.wrapping_add(vy);
                    },
                    5 => { // 8XY5
                        let has_overflow = vy > vx;
                        // if there is an overflow we will take the imaginary 1 out, if not we will
                        // leave if in (set to 0 if overflow, else set to 1)
                        self.registers.v[0xf] = !has_overflow as u8;
                        self.registers.v[x as usize] = vx.wrapping_sub(vy);
                        println!("set vx := vx - vy");
                    },
                    6 => { // 8XY6 shift right
                        let has_right_most_bit = 1u8 & vx > 0;
                        self.registers.v[0xf] = has_right_most_bit as u8;
                        self.registers.v[x as usize] = vx.wrapping_shr(1);
                        println!("shr vx");
                        println!("vx has rmb = {}", has_right_most_bit);
                    },
                    7 => { // 8XY7
                        let has_overflow = vx > vy;
                        // same as for 8XY5, set vf=0 if of, else vf=1
                        self.registers.v[0xf] = !has_overflow as u8;
                        self.registers.v[x as usize] = vy.wrapping_sub(vx);
                        println!("set vx := vy - vx");
                    },
                    0xE => { // 8XYE shift left
                        let has_left_most_bit = 128u8 & vx > 0;
                        self.registers.v[0xf] = has_left_most_bit as u8;
                        self.registers.v[x as usize] = vx.wrapping_shl(1);
                        println!("shl vx");
                        println!("vx has lmb = {}", has_left_most_bit);
                    },
                    _ => panic!("Unhandled instruction {:x}", instruction)

                }
            }
            0x09 => { // 9XY0 skip instrucion if vx != vx
                let x = get_nibble(instruction, 1);
                let vx = self.registers.v[x as usize];
                let y = get_nibble(instruction, 2);
                let vy = self.registers.v[y as usize];

                if vx != vy {
                    println!("skipping");
                    self.registers.pc += 2;
                }
            },
            0x0A => { // ANNN
                let addr = instruction & 0x0FFF;
                self.registers.i = addr;
                println!("vi = {:02x}", addr);
            },
            0xB => { // BNNN jump to NNN + V0
                let addr = instruction & 0x0FFF;
                let v0 = self.registers.v[0];
                self.registers.pc = addr + (v0 as u16);
            },
            0xC => { // CXNN
                let mask = (instruction & 0x00FF) as u8;
                let rnd = self.next_rnd_byte();
                let x = get_nibble(instruction, 1);
                self.registers.v[x as usize] = rnd & mask;
            },
            0xD => { // DXYN
                let x = get_nibble(instruction, 1);
                let y = get_nibble(instruction, 2);
                let n = get_nibble(instruction, 3);

                let vx = self.registers.v[x as usize];
                let vy = self.registers.v[y as usize];
                let sprite_addr = self.registers.i;
                let px = vx % (self.display.width as u8);
                let py = vy % (self.display.height as u8);

                self.registers.v[0xf] = 0;
                for i in 0..n {
                    let sprite_row = self.memory[(sprite_addr + i as u16) as usize];
                    let turned_off = self.display.draw_sprite_row(px as usize, py as usize + i as usize, sprite_row);
                    self.registers.v[0xf] |= turned_off as u8;
                }
            },
            0xE => { // EX9E / EXA1
                let switch = instruction & 0x00ff;
                let x = get_nibble(instruction, 1);
                let vx = self.registers.v[x as usize];
                assert!(vx <= 0xf); // Check if valid keycode.

                match switch {
                    0x9e => if self.get_kb_key(vx) {
                        self.registers.pc += 2;
                    },
                    0xa1 => if ! self.get_kb_key(vx) {
                        self.registers.pc += 2;
                    },
                    _ => panic!("Unhandled instruction {:x}", instruction)
                }
            },
            0xF => { // FX..
                let switch = instruction & 0x00ff;
                let x = get_nibble(instruction, 1);
                let vx = self.registers.v[x as usize];
                println!("vx={:x}", vx);

                match switch {
                    0x07 => { // set vx = delay_timer
                        self.registers.v[x as usize] = self.get_timer_value(self.timer_delay.clone());
                    },
                    0x15 => { // set delay_timer = vx
                        self.set_timer_value(self.timer_delay.clone(), vx);
                    },
                    0x18 => { // set sound_timer = vx
                        self.set_timer_value(self.timer_sound.clone(), vx);
                    },
                    0x1e => { // v_i += vx
                        self.registers.i += vx as u16;
                        // set flag if "overflowing" normal address range
                        self.registers.v[0xf as usize] = (self.registers.i >= 0x1000u16) as u8;
                    },
                    0x0a => {
                        if self.keyboard_has_press {
                            self.registers.v[x as usize] = self.keyboard_last_kc;
                        } else {
                            // go back to this instruction until we hit a press
                            self.registers.pc -= 2;
                        }
                    },
                    0x29 => {
                        assert!(vx <= 0xf);
                        // each character sprite is 5 rows
                        // sprites are loaded at 0x50 to 0x9f
                        let char_addr = 0x50u16 + (5u16 * vx as u16);
                        self.registers.i = char_addr;
                    },
                    0x33 => {
                        let vi = self.registers.i as usize;
                        let mut tmp = vx;
                        for i in 0..3 {
                            let digit = tmp % 10;
                            self.memory[vi + (2 - i)] = digit;
                            tmp = tmp / 10;
                        }
                    },
                    0x55 => { // FX55 store regs -> mem
                        let mem_addr = self.registers.i as usize;
                        for idx in 0..=x {
                            self.memory[mem_addr + idx as usize] = self.registers.v[idx as usize];
                        }
                        if self.options.load_store_inc_i {
                            self.registers.i += vx as u16 + 1;
                        }
                    },
                    0x65 => { // FX65 load mem -> regs
                        let mem_addr = self.registers.i as usize;
                        for idx in 0..=x {
                            self.registers.v[idx as usize] = self.memory[mem_addr + idx as usize];
                        }
                        if self.options.load_store_inc_i {
                            self.registers.i += vx as u16 + 1;
                        }
                    },
                    _ => panic!("Unhandled instruction {:x}", instruction)
                }
            },
            _ => println!("Unhandled instruction {:x}", instruction)
        }

        // after every instruction do:
        self.keyboard_has_press = false; // only valid for one step
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_nibble() {
        assert_eq!(get_nibble(0x1234u16, 0), 1u8);
        assert_eq!(get_nibble(0x1234u16, 1), 2u8);
        assert_eq!(get_nibble(0x1234u16, 2), 3u8);
        assert_eq!(get_nibble(0x1234u16, 3), 4u8);
    }

    #[test]
    fn test_keyboard() {
        let mut e = Chip8Emulator::create();
        for kc in 0..=0xf {
            assert!(e.get_kb_key(kc as u8) == false);
            e.set_kb_key(kc as u8, true);
            assert!(e.get_kb_key(kc as u8) == true);
            e.set_kb_key(kc as u8, false);
            assert!(e.get_kb_key(kc as u8) == false);
        }
        for kc in 0..=0xf {
            e.set_kb_key(kc as u8, true);
        }
        assert!(e.keyboard == 0xffffu16);
        for kc in 0..=0xf {
            e.set_kb_key(kc as u8, false);
        }
        assert!(e.keyboard == 0u16);
    }
}
