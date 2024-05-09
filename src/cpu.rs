use crate::opcodes;
use crate::rom::Rom;

#[derive(Debug)]
pub enum AddressingMode {
    Immediate,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    IndirectX,
    IndirectY,
    None,
}

pub trait Memory {
    fn memory_read(&self, address: u16) -> u8;
    fn memory_write(&mut self, address: u16, value: u8);

    fn memory_read_u16(&self, address: u16) -> u16 {
        let low_byte = self.memory_read(address) as u16;
        let high_byte = self.memory_read(address + 1) as u16;
        (high_byte << 8) | low_byte
    }

    fn memory_write_u16(&mut self, address: u16, value: u16) {
        let low_byte = value as u8;
        let high_byte = (value >> 8) as u8;
        self.memory_write(address, low_byte);
        self.memory_write(address + 1, high_byte);
    }
}

#[derive(Debug)]
pub struct Bus {
    cpu_ram: [u8; 2048],
    rom: Rom,
}

impl Bus {
    pub fn new(rom: Rom) -> Self {
        Bus {
            cpu_ram: [0; 2048],
            rom,
        }
    }

    pub fn read_prg_rom(&self, mut address: u16) -> u8 {
        address -= 0x8000;
        if self.rom.prg.len() == 0x4000 && address >= 0x4000 {
            address %= 0x4000;
        }

        self.rom.prg[address as usize]
    }
}

impl Memory for Bus {
    fn memory_read(&self, address: u16) -> u8 {
        match address {
            0x0000..=0x1fff => self.cpu_ram[(address & 0x07ff) as usize],
            0x2000..=0x3fff => {
                todo!("PPU registers");
            }
            0x8000..=0xffff => self.read_prg_rom(address),
            _ => {
                println!("ignoring read from address {:#06x}", address);
                0
            }
        }
    }

    fn memory_write(&mut self, address: u16, value: u8) {
        match address {
            0x0000..=0x1fff => self.cpu_ram[(address & 0x07ff) as usize] = value,
            0x2000..=0x3fff => {
                todo!("PPU registers");
            }
            0x8000..=0xffff => {
                panic!("Cannot write to ROM");
            }
            _ => {
                println!("ignoring write to address {:#06x}", address);
            }
        }
    }
}

#[derive(Debug)]
#[allow(clippy::upper_case_acronyms)]
pub struct CPU {
    pub program_counter: u16,
    pub stack_pointer: u8,
    pub status_register: u8, // N V - B D I Z C
    pub accumulator: u8,
    pub index_register_x: u8,
    pub index_register_y: u8,

    pub bus: Bus,
}

impl Memory for CPU {
    fn memory_read(&self, address: u16) -> u8 {
        self.bus.memory_read(address)
    }

    fn memory_write(&mut self, address: u16, value: u8) {
        self.bus.memory_write(address, value);
    }
}

impl CPU {
    pub fn new(bus: Bus) -> Self {
        CPU {
            program_counter: 0,
            stack_pointer: 0xfd,
            status_register: 0b0010_0100,
            accumulator: 0,
            index_register_x: 0,
            index_register_y: 0,
            bus,
        }
    }

    pub fn load_program(&mut self, address: u16, program: Vec<u8>) {
        for (i, byte) in program.iter().enumerate() {
            self.memory_write(address + i as u16, *byte);
        }
        // self.memory_write_u16(0xfffc, address | 0x8000);
    }

    pub fn reset(&mut self) {
        self.program_counter = self.memory_read_u16(0xfffc);

        self.status_register = 0b0010_0100;
        self.accumulator = 0;
        self.index_register_x = 0;
        self.index_register_y = 0;
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load_program(0x0600, program);
        self.reset();
        self.program_counter = 0x0600;
        self.run();
    }

    pub fn get_operand_address_pc(&self, mode: &AddressingMode, program_counter: u16) -> u16 {
        match mode {
            AddressingMode::Immediate => program_counter,
            AddressingMode::ZeroPage => self.memory_read(program_counter) as u16,
            AddressingMode::ZeroPageX => {
                let pos = self.memory_read(program_counter);
                pos.wrapping_add(self.index_register_x) as u16
            }
            AddressingMode::ZeroPageY => {
                let pos = self.memory_read(program_counter);
                pos.wrapping_add(self.index_register_y) as u16
            }
            AddressingMode::Absolute => self.memory_read_u16(program_counter),
            AddressingMode::AbsoluteX => {
                let base = self.memory_read_u16(program_counter);
                base.wrapping_add(self.index_register_x as u16)
            }
            AddressingMode::AbsoluteY => {
                let base = self.memory_read_u16(program_counter);
                base.wrapping_add(self.index_register_y as u16)
            }
            AddressingMode::IndirectX => {
                let base = self.memory_read(program_counter);
                let ptr: u8 = base.wrapping_add(self.index_register_x);
                let lo = self.memory_read(ptr as u16);
                let hi = self.memory_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddressingMode::IndirectY => {
                let base = self.memory_read(program_counter);
                let lo = self.memory_read(base as u16);
                let hi = self.memory_read(base.wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                deref_base.wrapping_add(self.index_register_y as u16)
            }
            AddressingMode::None => {
                panic!("Invalid addressing mode");
            }
        }
    }

    pub fn get_operand_address(&self, mode: &AddressingMode) -> u16 {
        match mode {
            AddressingMode::Immediate => self.program_counter,
            AddressingMode::ZeroPage => self.memory_read(self.program_counter) as u16,
            AddressingMode::ZeroPageX => {
                let pos = self.memory_read(self.program_counter);
                pos.wrapping_add(self.index_register_x) as u16
            }
            AddressingMode::ZeroPageY => {
                let pos = self.memory_read(self.program_counter);
                pos.wrapping_add(self.index_register_y) as u16
            }
            AddressingMode::Absolute => self.memory_read_u16(self.program_counter),
            AddressingMode::AbsoluteX => {
                let base = self.memory_read_u16(self.program_counter);
                base.wrapping_add(self.index_register_x as u16)
            }
            AddressingMode::AbsoluteY => {
                let base = self.memory_read_u16(self.program_counter);
                base.wrapping_add(self.index_register_y as u16)
            }
            AddressingMode::IndirectX => {
                let base = self.memory_read(self.program_counter);

                let ptr: u8 = base.wrapping_add(self.index_register_x);
                let lo = self.memory_read(ptr as u16);
                let hi = self.memory_read(ptr.wrapping_add(1) as u16);
                (hi as u16) << 8 | (lo as u16)
            }
            AddressingMode::IndirectY => {
                let base = self.memory_read(self.program_counter);

                let lo = self.memory_read(base as u16);
                let hi = self.memory_read(base.wrapping_add(1) as u16);
                let deref_base = (hi as u16) << 8 | (lo as u16);
                deref_base.wrapping_add(self.index_register_y as u16)
            }
            AddressingMode::None => {
                panic!("Invalid addressing mode");
            }
        }
    }

    fn bit_test(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.memory_read(address);

        let result = self.accumulator & value;
        self.update_zero_flag(result);
        self.update_negative_flag(value);
        self.update_overflow_flag(value & 0b0100_0000 == 0b0100_0000);
    }

    fn and(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.memory_read(address);

        self.accumulator &= value;
        self.update_zero_flag(self.accumulator);
        self.update_negative_flag(self.accumulator);
    }

    fn or(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.memory_read(address);

        self.accumulator |= value;
        self.update_zero_flag(self.accumulator);
        self.update_negative_flag(self.accumulator);
    }

    fn xor(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.memory_read(address);

        self.accumulator ^= value;
        self.update_zero_flag(self.accumulator);
        self.update_negative_flag(self.accumulator);
    }

    fn arithmetic_shift_left(&mut self, mut value: u8) -> u8 {
        let left_most_bit: u8 = value & 0b1000_0000;
        value <<= 1;
        self.update_carry_flag(left_most_bit == 0b1000_0000);
        self.update_zero_flag(value);
        self.update_negative_flag(value);
        value
    }

    fn arithmetic_shift_left_a(&mut self) {
        self.accumulator = self.arithmetic_shift_left(self.accumulator);
    }

    fn arithmetic_shift_left_m(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.memory_read(address);

        let result = self.arithmetic_shift_left(value);
        self.memory_write(address, result);
    }

    fn logic_shift_right(&mut self, mut value: u8) -> u8 {
        let right_most_bit: u8 = value & 0b0000_0001;
        value >>= 1;
        self.update_carry_flag(right_most_bit == 0b0000_0001);
        self.update_zero_flag(value);
        self.update_negative_flag(value);
        value
    }

    fn logic_shift_right_a(&mut self) {
        self.accumulator = self.logic_shift_right(self.accumulator);
    }

    fn logic_shift_right_m(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.memory_read(address);

        let result = self.logic_shift_right(value);
        self.memory_write(address, result);
    }

    fn rotate_left(&mut self, mut value: u8) -> u8 {
        let left_most_bit: u8 = value & 0b1000_0000;
        value <<= 1;
        value |= self.status_register & 0b0000_0001;
        self.update_carry_flag(left_most_bit == 0b1000_0000);
        self.update_zero_flag(value);
        self.update_negative_flag(value);
        value
    }

    fn rotate_left_a(&mut self) {
        self.accumulator = self.rotate_left(self.accumulator);
    }

    fn rotate_left_m(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.memory_read(address);

        let result = self.rotate_left(value);
        self.memory_write(address, result);
    }

    fn rotate_right(&mut self, mut value: u8) -> u8 {
        let right_most_bit: u8 = value & 0b0000_0001;
        value >>= 1;
        value |= (self.status_register & 0b0000_0001) << 7;
        self.update_carry_flag(right_most_bit == 0b0000_0001);
        self.update_zero_flag(value);
        self.update_negative_flag(value);
        value
    }

    fn rotate_right_a(&mut self) {
        self.accumulator = self.rotate_right(self.accumulator);
    }

    fn rotate_right_m(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.memory_read(address);

        let result = self.rotate_right(value);
        self.memory_write(address, result);
    }

    fn increment_x(&mut self) {
        self.index_register_x = self.index_register_x.wrapping_add(1);
        self.update_zero_flag(self.index_register_x);
        self.update_negative_flag(self.index_register_x);
    }

    fn increment_y(&mut self) {
        self.index_register_y = self.index_register_y.wrapping_add(1);
        self.update_zero_flag(self.index_register_y);
        self.update_negative_flag(self.index_register_y);
    }

    fn increment_m(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.memory_read(address);

        let result = value.wrapping_add(1u8);
        self.memory_write(address, result);
        self.update_zero_flag(result);
        self.update_negative_flag(result);
    }

    fn decrement_x(&mut self) {
        self.index_register_x = self.index_register_x.wrapping_sub(1);
        self.update_zero_flag(self.index_register_x);
        self.update_negative_flag(self.index_register_x);
    }

    fn decrement_y(&mut self) {
        self.index_register_y = self.index_register_y.wrapping_sub(1);
        self.update_zero_flag(self.index_register_y);
        self.update_negative_flag(self.index_register_y);
    }

    fn decrement_m(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.memory_read(address);

        let result = value.wrapping_sub(1);
        self.memory_write(address, result);
        self.update_zero_flag(result);
        self.update_negative_flag(result);
    }

    fn store(&mut self, mode: &AddressingMode, value: u8) {
        let address = self.get_operand_address(mode);
        self.memory_write(address, value);
    }

    fn load(&mut self, mode: &AddressingMode) -> u8 {
        let address = self.get_operand_address(mode);
        let value = self.memory_read(address);
        self.update_zero_flag(value);
        self.update_negative_flag(value);
        value
    }

    fn load_a(&mut self, mode: &AddressingMode) {
        self.accumulator = self.load(mode);
    }

    fn transfer_a(&mut self, value: u8) {
        self.accumulator = value;
        self.update_zero_flag(self.accumulator);
        self.update_negative_flag(self.accumulator);
    }

    fn load_x(&mut self, mode: &AddressingMode) {
        self.index_register_x = self.load(mode);
    }

    fn transfer_x(&mut self, value: u8) {
        self.index_register_x = value;
        self.update_zero_flag(self.index_register_x);
        self.update_negative_flag(self.index_register_x);
    }

    fn load_y(&mut self, mode: &AddressingMode) {
        self.index_register_y = self.load(mode);
    }

    fn transfer_y(&mut self, value: u8) {
        self.index_register_y = value;
        self.update_zero_flag(self.index_register_y);
        self.update_negative_flag(self.index_register_y);
    }

    fn load_sp(&mut self, value: u8) {
        self.stack_pointer = value;
        self.update_zero_flag(self.stack_pointer);
        self.update_negative_flag(self.stack_pointer);
    }

    fn compare(&mut self, cmp: u8, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.memory_read(address);
        let (result, overflow) = cmp.overflowing_sub(value);
        self.update_carry_flag(!overflow);
        self.update_zero_flag(result);
        self.update_negative_flag(result);
    }

    fn compare_a(&mut self, mode: &AddressingMode) {
        self.compare(self.accumulator, mode);
    }

    fn compare_x(&mut self, mode: &AddressingMode) {
        self.compare(self.index_register_x, mode);
    }

    fn compare_y(&mut self, mode: &AddressingMode) {
        self.compare(self.index_register_y, mode);
    }

    fn add_a(&mut self, value: u8) {
        let carry: u8 = if self.status_register & 0b0000_0001 == 1 {
            1
        } else {
            0
        };

        let (mut unsigned_result, mut carry_flag) = self.accumulator.overflowing_add(value);
        if !carry_flag {
            (unsigned_result, carry_flag) = unsigned_result.overflowing_add(carry);
        } else {
            (unsigned_result, _) = unsigned_result.overflowing_add(carry);
        }

        let (signed_result, mut overflow_flag) =
            (self.accumulator as i8).overflowing_add(value as i8);
        if !overflow_flag {
            (_, overflow_flag) = signed_result.overflowing_add(carry as i8);
        } else {
            (_, _) = signed_result.overflowing_add(carry as i8);
        }

        self.accumulator = unsigned_result;
        self.update_carry_flag(carry_flag);
        self.update_overflow_flag(overflow_flag);
        self.update_zero_flag(unsigned_result);
        self.update_negative_flag(unsigned_result);
    }

    fn add_with_carry(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.memory_read(address);

        self.add_a(value);
    }

    fn sub_with_carry(&mut self, mode: &AddressingMode) {
        let address = self.get_operand_address(mode);
        let value = self.memory_read(address);

        self.add_a((value as i8).wrapping_neg().wrapping_sub(1) as u8);
    }

    fn push(&mut self, value: u8) {
        if self.stack_pointer < 1 {
            panic!("Stack overflow");
        }
        self.memory_write(0x100 | self.stack_pointer as u16, value);
        self.stack_pointer = self.stack_pointer.wrapping_sub(1);
    }

    fn push_u16(&mut self, value: u16) {
        if self.stack_pointer < 1 {
            panic!("Stack overflow");
        }
        let low_byte = value as u8;
        let high_byte = (value >> 8) as u8;
        self.push(high_byte);
        self.push(low_byte);
    }

    fn pop(&mut self) -> u8 {
        if self.stack_pointer == 0xff {
            panic!("Stack underflow");
        }
        self.stack_pointer = self.stack_pointer.wrapping_add(1);
        self.memory_read(0x100_u16 + self.stack_pointer as u16)
    }

    fn pop_u16(&mut self) -> u16 {
        if self.stack_pointer == 0xff {
            panic!("Stack underflow");
        }
        let low_byte = self.pop();
        let high_byte = self.pop();
        (high_byte as u16) << 8 | (low_byte as u16)
    }

    fn pull_a(&mut self) {
        self.accumulator = self.pop();

        self.update_zero_flag(self.accumulator);
        self.update_negative_flag(self.accumulator);
    }

    fn pull_status_register(&mut self) {
        self.status_register = self.pop();
    }

    fn pull_program_counter(&mut self) {
        self.program_counter = self.pop_u16();
    }

    fn jump_subroutine(&mut self) {
        let target = self.get_operand_address(&AddressingMode::Absolute);
        let return_address = self.program_counter + 1;
        self.push_u16(return_address);
        self.program_counter = target;
    }

    fn force_interrupt(&mut self) {
        self.update_interrupt_disable_flag(true);
        let return_address = self.program_counter + 1;
        self.push_u16(return_address);
        self.program_counter = self.memory_read_u16(0xfffe);
    }

    fn branch(&mut self, mask: u8, condition: u8) {
        let address = self.get_operand_address(&AddressingMode::Immediate);
        let relative = self.memory_read(address) as i8;

        self.program_counter = self.program_counter.wrapping_add(1);

        if self.status_register & mask == condition {
            self.program_counter = self.program_counter.wrapping_add(relative as u16);
        }
    }

    fn update_status_register(&mut self, mask: u8, flag: bool) {
        if flag {
            self.status_register |= mask;
        } else {
            self.status_register &= !mask;
        }
    }

    fn update_carry_flag(&mut self, flag: bool) {
        self.update_status_register(0b0000_0001, flag);
    }

    fn update_zero_flag(&mut self, value: u8) {
        let flag = value == 0;
        self.update_status_register(0b0000_0010, flag);
    }

    fn update_interrupt_disable_flag(&mut self, flag: bool) {
        self.update_status_register(0b0000_0100, flag);
    }

    fn update_decimal_mode_flag(&mut self, flag: bool) {
        self.update_status_register(0b0000_1000, flag);
    }

    fn update_break_command_flag(&mut self, flag: bool) {
        self.update_status_register(0b0001_0000, flag);
    }

    fn update_overflow_flag(&mut self, flag: bool) {
        self.update_status_register(0b0100_0000, flag);
    }

    fn update_negative_flag(&mut self, value: u8) {
        let flag = value & 0b1000_0000 == 0b1000_0000;
        self.update_status_register(0b1000_0000, flag);
    }

    pub fn run_with_callback<F>(&mut self, mut callback: F)
    where
        F: FnMut(&mut CPU),
    {
        let opcodes_map = opcodes::opcode_map();
        loop {
            callback(self);

            let code = self.memory_read(self.program_counter);
            self.program_counter += 1;

            let opcode = opcodes_map.get(&code).unwrap();
            match code {
                0x00 => {
                    // self.force_interrupt();
                    break;
                }
                0x69 | 0x65 | 0x75 | 0x6d | 0x7d | 0x79 | 0x61 | 0x71 => {
                    self.add_with_carry(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0x29 | 0x25 | 0x35 | 0x2d | 0x3d | 0x39 | 0x21 | 0x31 => {
                    self.and(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0x0a => {
                    self.arithmetic_shift_left_a();
                }
                0x06 | 0x16 | 0x0e | 0x1e => {
                    self.arithmetic_shift_left_m(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0x90 => {
                    self.branch(0b0000_0001, 0b0000_0000);
                }
                0xb0 => {
                    self.branch(0b0000_0001, 0b0000_0001);
                }
                0xf0 => {
                    self.branch(0b0000_0010, 0b0000_0010);
                }
                0x24 | 0x2c => {
                    self.bit_test(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0x30 => {
                    self.branch(0b1000_0000, 0b1000_0000);
                }
                0xd0 => {
                    self.branch(0b0000_0010, 0b0000_0000);
                }
                0x10 => {
                    self.branch(0b1000_0000, 0b0000_0000);
                }
                0x50 => {
                    self.branch(0b0100_0000, 0b0000_0000);
                }
                0x70 => {
                    self.branch(0b0100_0000, 0b0100_0000);
                }
                0x18 => {
                    self.update_carry_flag(false);
                }
                0xd8 => {
                    self.update_decimal_mode_flag(false);
                }
                0x58 => {
                    self.update_interrupt_disable_flag(false);
                }
                0xb8 => {
                    self.update_overflow_flag(false);
                }
                0xc9 | 0xc5 | 0xd5 | 0xcd | 0xdd | 0xd9 | 0xc1 | 0xd1 => {
                    self.compare_a(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0xe0 | 0xe4 | 0xec => {
                    self.compare_x(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0xc0 | 0xc4 | 0xcc => {
                    self.compare_y(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0xc6 | 0xd6 | 0xce | 0xde => {
                    self.decrement_m(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0xca => {
                    self.decrement_x();
                }
                0x88 => {
                    self.decrement_y();
                }
                0x49 | 0x45 | 0x55 | 0x4d | 0x5d | 0x59 | 0x41 | 0x51 => {
                    self.xor(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0xe6 | 0xf6 | 0xee | 0xfe => {
                    self.increment_m(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0xe8 => {
                    self.increment_x();
                }
                0xc8 => {
                    self.increment_y();
                }
                0x4c => {
                    let address = self.memory_read_u16(self.program_counter);
                    self.program_counter = address;
                }
                0x6c => {
                    let memory_address = self.memory_read_u16(self.program_counter);
                    // 6502 bug with page boundary
                    let address = if memory_address & 0x00ff == 0x00ff {
                        let lo = self.memory_read(memory_address);
                        let hi = self.memory_read(memory_address & 0xff00);
                        (hi as u16) << 8 | (lo as u16)
                    } else {
                        self.memory_read_u16(memory_address)
                    };
                    self.program_counter = address;
                }
                0x20 => {
                    self.jump_subroutine();
                }
                0xa9 | 0xa5 | 0xb5 | 0xad | 0xbd | 0xb9 | 0xa1 | 0xb1 => {
                    self.load_a(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0xa2 | 0xa6 | 0xb6 | 0xae | 0xbe => {
                    self.load_x(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0xa0 | 0xa4 | 0xb4 | 0xac | 0xbc => {
                    self.load_y(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0x4a => {
                    self.logic_shift_right_a();
                }
                0x46 | 0x56 | 0x4e | 0x5e => {
                    self.logic_shift_right_m(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0xea => {}
                0x09 | 0x05 | 0x15 | 0x0d | 0x1d | 0x19 | 0x01 | 0x11 => {
                    self.or(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0x48 => {
                    self.push(self.accumulator);
                }
                0x08 => {
                    self.push(self.status_register);
                }
                0x68 => {
                    self.pull_a();
                }
                0x28 => {
                    self.pull_status_register();
                }
                0x2a => {
                    self.rotate_left_a();
                }
                0x26 | 0x36 | 0x2e | 0x3e => {
                    self.rotate_left_m(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0x6a => {
                    self.rotate_right_a();
                }
                0x66 | 0x76 | 0x6e | 0x7e => {
                    self.rotate_right_m(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0x40 => {
                    self.pull_status_register();
                    self.pull_program_counter();
                }
                0x60 => {
                    self.pull_program_counter();
                    self.program_counter += 1;
                }
                0xe9 | 0xe5 | 0xf5 | 0xed | 0xfd | 0xf9 | 0xe1 | 0xf1 => {
                    self.sub_with_carry(&opcode.mode);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0x38 => {
                    self.update_carry_flag(true);
                }
                0xf8 => {
                    self.update_decimal_mode_flag(true);
                }
                0x78 => {
                    self.update_interrupt_disable_flag(true);
                }
                0x85 | 0x95 | 0x8d | 0x9d | 0x99 | 0x81 | 0x91 => {
                    self.store(&opcode.mode, self.accumulator);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0x86 | 0x96 | 0x8e => {
                    self.store(&opcode.mode, self.index_register_x);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0x84 | 0x94 | 0x8c => {
                    self.store(&opcode.mode, self.index_register_y);
                    self.program_counter += (opcode.bytes - 1) as u16;
                }
                0xaa => {
                    self.transfer_x(self.accumulator);
                }
                0xa8 => {
                    self.transfer_y(self.accumulator);
                }
                0xba => {
                    self.transfer_x(self.stack_pointer);
                }
                0x8a => {
                    self.transfer_a(self.index_register_x);
                }
                0x9a => {
                    self.load_sp(self.index_register_x);
                }
                0x98 => {
                    self.transfer_a(self.index_register_y);
                }
                _ => {
                    println!("Unknown opcode: {:#04x}", code);
                    return;
                }
            }
        }
    }

    pub fn run(&mut self) {
        self.run_with_callback(|_| {});
    }
}

#[cfg(test)]
mod tests {
    use crate::rom::test::test_rom;

    use super::*;

    #[test]
    fn test_add_with_carry_immediate() {
        let bus = Bus::new(test_rom());
        let mut cpu = CPU::new(bus);
        cpu.accumulator = 0x01;
        cpu.load_program(0x0600, vec![0x69, 0x01, 0x00]);
        cpu.program_counter = 0x0600;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x02);
        assert_eq!(cpu.status_register & 0b0000_0001, 0b0000_0000);
    }

    #[test]
    fn test_add_with_carry_immediate_with_carry() {
        let bus = Bus::new(test_rom());
        let mut cpu = CPU::new(bus);
        cpu.accumulator = 0x01;
        cpu.update_carry_flag(true);
        cpu.load_program(0x0600, vec![0x69, 0x01, 0x00]);
        cpu.program_counter = 0x0600;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x03);
        assert_eq!(cpu.status_register & 0b0000_0001, 0b0000_0000);
    }

    #[test]
    fn test_add_with_carry_carry_flag() {
        let bus = Bus::new(test_rom());
        let mut cpu = CPU::new(bus);
        cpu.accumulator = 0xff;
        cpu.load_program(0x0600, vec![0x69, 0x01, 0x00]);
        cpu.program_counter = 0x0600;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x00);
        assert_eq!(cpu.status_register & 0b0000_0001, 0b0000_0001);
        assert_eq!(cpu.status_register & 0b0000_0010, 0b0000_0010);
    }

    #[test]
    fn test_add_with_carry_overflow_flag() {
        let bus = Bus::new(test_rom());
        let mut cpu = CPU::new(bus);
        cpu.accumulator = 0x7f;
        cpu.load_program(0x0600, vec![0x69, 0x01, 0x00]);
        cpu.program_counter = 0x0600;
        cpu.run();
        assert_eq!(cpu.accumulator, 0x80);
        assert_eq!(cpu.status_register & 0b0100_0000, 0b0100_0000);
        assert_eq!(cpu.status_register & 0b1000_0000, 0b1000_0000);
    }

    #[test]
    fn test_add_with_carry_zero_page() {
        let bus = Bus::new(test_rom());
        let mut cpu = CPU::new(bus);
        cpu.accumulator = 0x01;
        cpu.memory_write(0x0000, 0x01);
        cpu.load_program(0x0600, vec![0x65, 0x00, 0x00]);
        cpu.program_counter = 0x0600;
        cpu.run();
        assert_eq!(cpu.memory_read(0x0000), 0x01);
        assert_eq!(cpu.accumulator, 0x02);
    }

    #[test]
    fn test_add_with_carry_zero_page_x() {
        let bus = Bus::new(test_rom());
        let mut cpu = CPU::new(bus);
        cpu.accumulator = 0x01;
        cpu.memory_write(0x007f, 0x01);
        cpu.index_register_x = 0xff;
        cpu.load_program(0x0600, vec![0x75, 0x80, 0x00]);
        cpu.program_counter = 0x0600;
        cpu.run();
        assert_eq!(cpu.memory_read(0x007f), 0x01);
        assert_eq!(cpu.accumulator, 0x02);
    }

    #[test]
    fn test_add_with_carry_absolute() {
        let bus = Bus::new(test_rom());
        let mut cpu = CPU::new(bus);
        cpu.accumulator = 0x01;
        cpu.memory_write(0x0200, 0x01);
        cpu.load_program(0x0600, vec![0x6d, 0x00, 0x02, 0x00]);
        cpu.program_counter = 0x0600;
        cpu.run();
        assert_eq!(cpu.memory_read(0x0200), 0x01);
        assert_eq!(cpu.accumulator, 0x02);
    }

    #[test]
    fn test_add_with_carry_indirect_x() {
        let bus = Bus::new(test_rom());
        let mut cpu = CPU::new(bus);
        cpu.accumulator = 0x01;
        cpu.memory_write_u16(0x0010, 0x0200);
        cpu.memory_write(0x0200, 0x01);
        cpu.index_register_x = 0x00;
        cpu.load_program(0x0600, vec![0x61, 0x10, 0x00]);
        cpu.program_counter = 0x0600;
        cpu.run();
        assert_eq!(cpu.memory_read(0x0200), 0x01);
        assert_eq!(cpu.accumulator, 0x02);
    }

    #[test]
    fn test_add_with_carry_indirect_y() {
        let bus = Bus::new(test_rom());
        let mut cpu = CPU::new(bus);
        cpu.accumulator = 0x01;
        cpu.index_register_y = 0x00;
        cpu.memory_write_u16(0x0010, 0x0200);
        cpu.memory_write(0x0200, 0x01);
        cpu.load_program(0x0600, vec![0x71, 0x10, 0x00]);
        cpu.program_counter = 0x0600;
        cpu.run();
        assert_eq!(cpu.memory_read(0x0200), 0x01);
        assert_eq!(cpu.accumulator, 0x02);
    }
}
