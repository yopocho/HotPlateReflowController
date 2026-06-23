Schematic:
I2C address notes for INA219's switched around
Thermocouple has 100nF across it instead of 10nF
I2C address for display is actually 0x3C
Need harder pull-ups on i2c lines for 1MHz

Code:
Too many unwraps, such as on INA219 and MAX31855 reads.
Rotary encoder very much in need of debouncing