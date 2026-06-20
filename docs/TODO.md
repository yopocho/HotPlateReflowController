## Non-exhaustive list of TODO items

1. Integrate encoder (possibly with hardware TIMs if everything goes well)
2. Setup FSM architecture for orchistrating the different functions of the hotplate
3. ZCD input for output synchronisation
4. PID w/ thermocouple temperature as input and simple cycle banging output to triac
5. Create a way to easily add and parse reflow profiles for use in PID (starting with TS391SNL)
6. Setup internal STM32 temperature sensor
7. Map PWM output duty cycle for fan with internal temperature sensor (maybe) or ~1:1 with output triac duty cycle
8. Add start-up self-test procedure (check for any current through heater, fan, thermocouple, basically everything that can be tested)
