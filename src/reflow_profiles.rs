/* Enum containing all supported reflow profiles, index of the profile in PROFILES-array */
#[derive(Clone, Copy, PartialEq, Debug)]
#[repr(u8)]
pub enum ReflowProfiles {
    TS391SNL = 0,
    GC10 = 1,
    NoProfile = 2,
}

/* Func to return the profile to access parameters */
impl ReflowProfiles {
    pub const fn profile(self) -> &'static ReflowProfile {
        &PROFILES[self as usize]
    }
}

/* Struct for holding reflow profile parameters */
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct ReflowProfile {
    pub name:           &'static str,
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

/* Array holding all reflow profiles */
const PROFILES: [ReflowProfile; 3] = [
    // ReflowProfile { // TS391SNL from http://www.chipquik.com/datasheets/TS391SNL.pdf
    //     name:           "TS391SNL",
    //     max_temp:       249, // °C
    //     total_duration: 300, // Seconds
    //     melt_temp:      219, // °C
    //     preheat_temp:   150, // °C
    //     soak_temp:      175, // °C
    //     reflow_temp:    249, // °C
    //     cool_temp:      219, // °C
    //     preheat_time:   90,  // Seconds
    //     soak_time:      90,  // Seconds
    //     reflow_time:    60,  // Seconds
    //     cool_time:      30,  // Seconds
    // },
    ReflowProfile { // TS391SNL from http://www.chipquik.com/datasheets/TS391SNL.pdf
        name:           "TS391SNL",
        max_temp:       50, // °C
        total_duration: 80, // Seconds
        melt_temp:      45, // °C
        preheat_temp:   30, // °C
        soak_temp:      40, // °C
        reflow_temp:    50, // °C
        cool_temp:      40, // °C
        preheat_time:   20,  // Seconds
        soak_time:      20, // Seconds
        reflow_time:    20, // Seconds
        cool_time:      20,  // Seconds
    },
    
    ReflowProfile { // GC10 from https://www.farnell.com/datasheets/1943941.pdf
        name:           "GC10",
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
        name:           "NoProfile",
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