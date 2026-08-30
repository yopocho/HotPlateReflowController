## Non-exhaustive list of TODO items and issues

### Firmware
1. Setup internal STM32 temperature sensor
2. Map PWM output duty cycle for fan with internal temperature sensor
3. Add start-up self-test procedure (check for any current through heater, fan, thermocouple, basically everything that can be tested)
4. Populate reflow profile list dynamically instead of hard coding it in the display task

### Schematic
1. MOC3163 230V outputs swiched around, preventing the triac from being enabled.
2. I2C address notes for INA219's switched around
3. Thermocouple has 100nF across it instead of 10nF
4. I2C address for display is actually 0x3C