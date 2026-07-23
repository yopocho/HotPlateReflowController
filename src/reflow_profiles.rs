/* Enum containing all supported reflow profiles */
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub enum ReflowProfiles {
    TS391SNL = 0,
    GC10 = 1,
    NoProfile = 2,
}

/* Func to return Reflow */
impl ReflowProfiles {
    pub const fn profile(self) -> &'static ReflowProfile {
        &PROFILES[self as usize]
    }
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct ReflowProfile {
    pub max_temp:       u32, // °C
    pub total_duration: u32, // Seconds
    pub melt_temp:      u32, // °C
    pub preheat_temp:   u32, // °C
    pub soak_temp:      u32, // °C
    pub reflow_temp:    u32, // °C
    pub cool_temp:      u32, // °C
    pub preheat_time:   u32, // Seconds
    pub soak_time:      u32, // Seconds
    pub reflow_time:    u32, // Seconds
    pub cool_time:      u32, // Seconds
}    

const PROFILES: [ReflowProfile; 3] = [
    ReflowProfile { // TS391SNL from http://www.chipquik.com/datasheets/TS391SNL.pdf
        max_temp:       249, // °C
        total_duration: 300, // Seconds
        melt_temp:      219, // °C
        preheat_temp:   150, // °C
        soak_temp:      175, // °C
        reflow_temp:    249, // °C
        cool_temp:      219, // °C
        preheat_time:   90,  // Seconds
        soak_time:      90,  // Seconds
        reflow_time:    60,  // Seconds
        cool_time:      30,  // Seconds
    },
    
    ReflowProfile { // GC10 from https://www.farnell.com/datasheets/1943941.pdf
        max_temp:       254, // °C
        total_duration: 355, // Seconds
        melt_temp:      217, // °C
        preheat_temp:   150, // °C
        soak_temp:      200, // °C
        reflow_temp:    254, // °C
        cool_temp:      217, // °C
        preheat_time:   45,  // Seconds
        soak_time:      195, // Seconds
        reflow_time:    105, // Seconds
        cool_time:      10,  // Seconds
    },
    ReflowProfile { // NoProfile
        max_temp:       0, // °C
        total_duration: 0, // Seconds
        melt_temp:      0, // °C
        preheat_temp:   0, // °C
        soak_temp:      0, // °C
        reflow_temp:    0, // °C
        cool_temp:      0, // °C
        preheat_time:   0, // Seconds
        soak_time:      0, // Seconds
        reflow_time:    0, // Seconds
        cool_time:      0, // Seconds
    },
];