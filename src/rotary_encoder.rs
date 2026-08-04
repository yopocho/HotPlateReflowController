/// Full-step quadrature (Gray code) decoder designed by Ben Buxton

mod gray {
    pub const START: u8 = 0x0;
    pub const CW_FINAL: u8 = 0x1;
    pub const CW_BEGIN: u8 = 0x2;
    pub const CW_NEXT: u8 = 0x3;
    pub const CCW_BEGIN: u8 = 0x4;
    pub const CCW_FINAL: u8 = 0x5;
    pub const CCW_NEXT: u8 = 0x6;
    pub const DIR_CW: u8 = 0x10;
    pub const DIR_CCW: u8 = 0x20;

    pub const TABLE: [[u8; 4]; 7] = [
        /* START    */ [START,    CW_BEGIN,  CCW_BEGIN, START],
        /* CW_FINAL */ [CW_NEXT,  START,     CW_FINAL,  START | DIR_CW],
        /* CW_BEGIN */ [CW_NEXT,  CW_BEGIN,  START,     START],
        /* CW_NEXT  */ [CW_NEXT,  CW_BEGIN,  CW_FINAL,  START],
        /* CCW_BEGIN*/ [CCW_NEXT, START,     CCW_BEGIN, START],
        /* CCW_FINAL*/ [CCW_NEXT, CCW_FINAL, START,     START | DIR_CCW],
        /* CCW_NEXT */ [CCW_NEXT, CCW_FINAL, CCW_BEGIN, START],
    ];
}

pub struct GrayDecoder {
    state: u8,
}

impl GrayDecoder {
    pub const fn new() -> Self {
        Self { state: gray::START }
    }

    pub fn update(&mut self, a: bool, b: bool) -> Option<Direction> {
        let pins: u8 = ((b as u8) << 1) | (a as u8);
        self.state = gray::TABLE[(self.state & 0x0f) as usize][pins as usize];
        match self.state & 0x30 {
            gray::DIR_CW => Some(Direction::Clockwise),
            gray::DIR_CCW => Some(Direction::CounterClockwise),
            _ => None,
        }
    }
}

pub enum Direction {
    Clockwise,
    CounterClockwise,
}