use std::io::Error;
use std::{thread, time};
use std::fs;

#[derive(Debug)]
struct RegisterFile {
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
struct Display {
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
struct Timer {
}

impl Timer {
    fn create() -> Timer {
        Timer {  }
    }
}

#[derive(Debug)]
struct Chip8Emulator {
    pub memory: [u8; 4096],
    pub registers: RegisterFile,
    pub display: Display,
    pub stack: Vec<u16>,
    pub timer_delay: Timer,
    pub timer_sound: Timer,
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

impl Chip8Emulator {
    fn create() -> Chip8Emulator {
        Chip8Emulator { 
            memory: [0; 4096],
            registers: RegisterFile::create(),
            display: Display::create(),
            stack: Vec::new(),
            timer_sound: Timer::create(),
            timer_delay: Timer::create(),
        }
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
        println!("upper={:02x} lower={:02x}", upper, lower);
        let result = (upper << 8) | lower;
        result
    }

    // One run loop step (fetch, decode, execute)
    pub fn step(&mut self) {
        // fetch
        println!("-----------------------------------");
        println!("Fetch 0x{:x}", self.registers.pc);
        let instruction = self.read_u16(self.registers.pc as usize);
        self.registers.pc += 2;

        // decode
        let op_type = get_nibble(instruction, 0);
        println!("op_type = {:02x}", op_type);
        match op_type {
            0x00 => self.display.clear(),
            0x01 => { // 1NNN
                let addr = instruction & 0x0FFF;
                self.registers.pc = addr;
                println!("go to {:04x}", addr);
            },
            0x06 => { // 6XNN
                let value = (instruction & 0x0FF) as u8;
                let reg = get_nibble(instruction, 1);
                assert!(reg < 16u8);
                self.registers.v[reg as usize] = value;
                println!("v{:x} = {:02x}", reg, value);
            },
            0x07 => { // 7XNN
                let value = (instruction & 0x0FF) as u8;
                let reg = get_nibble(instruction, 1);
                assert!(reg < 16u8);
                self.registers.v[reg as usize] += value;
                println!("v{:x} += {:02x}", reg, value);
            },
            0x0A => { // ANNN
                let addr = instruction & 0x0FFF;
                self.registers.i = addr;
                println!("vi = {:02x}", addr);
            },
            0x0D => { // DXYN
                let x = get_nibble(instruction, 1);
                let y = get_nibble(instruction, 2);
                let n = get_nibble(instruction, 3);
                println!("Draw x={}, y={}, n={}", x, y, n);
                let vx = self.registers.v[x as usize];
                let vy = self.registers.v[y as usize];
                let sprite_addr = self.registers.i;
                let px = vx % (self.display.width as u8);
                let py = vy % (self.display.height as u8);
                for i in 0..n {
                    let sprite_row = self.memory[(sprite_addr + i as u16) as usize];
                    self.display.draw_sprite_row(px as usize, py as usize + i as usize, sprite_row);
                }
                self.display.draw();
            },
            _ => panic!("Unhandled instruction {:x}", instruction)
        }

        // execute
    }
}

fn main() {
    let mut e = Chip8Emulator::create();

    e.load_rom("./c8-roms/ibm.ch8".to_string()).unwrap();

    // instructions per second
    let ips = 25;
    loop {
        e.step();
        thread::sleep(time::Duration::from_millis(1000 / ips));
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
}
