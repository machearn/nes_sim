const INES_TAG: [u8; 4] = [0x4E, 0x45, 0x53, 0x1A];
const ROM_BANK_SIZE: usize = 0x4000; // 16 KB
const CHR_BANK_SIZE: usize = 0x2000; // 8 KB

#[derive(Debug, PartialEq)]
pub enum Mirroring {
    VERTICAL,
    HORIZONTAL,
    FOURSCREEN,
}

#[derive(Debug)]
pub struct Rom {
    pub prg: Vec<u8>,
    pub chr: Vec<u8>,
    /*
     * high to low
     * 7 6 5 4 3 2 1 0
     * | | | | | | | |
     * | | | | | | | +-- Mirroring (0: horizontal, 1: vertical)
     * | | | | | | +---- 1 for battery-backed RAM at $6000-$7FFF
     * | | | | | +------ 1 for a 512-byte trainer at $7000-$71FF
     * | | | | +-------- 1 for a four-screen VRAM layout
     * + + + +---------- Lower bits of mapper number
     */
    pub control_byte_1: u8,
    /*
     * high to low
     * 7 6 5 4 3 2 1 0
     * | | | | | | | |
     * | | | | | | | +-- should be zero for iNES 1.0 format
     * | | | | | | +---- should be zero for iNES 1.0 format
     * | | | | + +------ 10 for iNES 2.0 format, 00 for iNES 1.0 format
     * + + + +---------- Upper bits of mapper number
     */
    pub control_byte_2: u8,
    pub mapper: u8,
    pub screen_mirroring: Mirroring,
}

impl Rom {
    pub fn new(raw: &[u8]) -> Result<Rom, String> {
        if raw[0..4] != INES_TAG {
            return Err("Invalid iNES header".to_string());
        }

        let control_byte_1 = raw[6];
        let control_byte_2 = raw[7];

        let mapper = (control_byte_2 & 0xF0) | (control_byte_1 >> 4);

        let ines_version = (control_byte_2 >> 2) & 0b11;
        if ines_version != 0 {
            return Err("Unsupported iNES version".to_string());
        }

        let screen_code = control_byte_1 & 0b1001;
        let screen_mirroring = match screen_code {
            0b1000 | 0b1001 => Mirroring::FOURSCREEN,
            0b0001 => Mirroring::VERTICAL,
            0b0000 => Mirroring::HORIZONTAL,
            _ => return Err("Invalid screen mirroring".to_string()),
        };

        let prg_rom_size = raw[4] as usize * ROM_BANK_SIZE;
        let chr_rom_size = raw[5] as usize * CHR_BANK_SIZE;

        let trainer = control_byte_1 & 0b100 != 0;

        let prg_start = 16 + if trainer { 512 } else { 0 };
        let chr_start = prg_start + prg_rom_size;

        Ok(Rom {
            prg: raw[prg_start..prg_start + prg_rom_size].to_vec(),
            chr: raw[chr_start..chr_start + chr_rom_size].to_vec(),
            control_byte_1,
            control_byte_2,
            mapper,
            screen_mirroring,
        })
    }
}

pub mod test {
    use super::*;

    struct TestRom {
        header: Vec<u8>,
        trainer: Option<Vec<u8>>,
        prg: Vec<u8>,
        chr: Vec<u8>,
    }

    fn create_raw(test_rom: TestRom) -> Vec<u8> {
        let mut raw = Vec::with_capacity(
            test_rom.header.len()
                + test_rom.prg.len()
                + test_rom.chr.len()
                + test_rom.trainer.as_ref().map_or(0, |t| t.len()),
        );

        raw.extend(&test_rom.header);
        if let Some(t) = test_rom.trainer {
            raw.extend(t);
        }
        raw.extend(&test_rom.prg);
        raw.extend(&test_rom.chr);

        raw
    }

    pub fn test_rom() -> Rom {
        let raw = create_raw(TestRom {
            header: vec![
                0x4E, 0x45, 0x53, 0x1A, 0x02, 0x01, 0x31, 00, 00, 00, 00, 00, 00, 00, 00, 00,
            ],
            trainer: None,
            prg: vec![1; 2 * ROM_BANK_SIZE],
            chr: vec![2; CHR_BANK_SIZE],
        });

        Rom::new(&raw).unwrap()
    }

    #[test]
    fn test() {
        let rom = test_rom();

        assert_eq!(rom.prg, vec![1; 2 * ROM_BANK_SIZE]);
        assert_eq!(rom.chr, vec![2; CHR_BANK_SIZE]);
        assert_eq!(rom.control_byte_1, 0x31);
        assert_eq!(rom.control_byte_2, 0);
        assert_eq!(rom.mapper, 3);
        assert_eq!(rom.screen_mirroring, Mirroring::VERTICAL);
    }
}
