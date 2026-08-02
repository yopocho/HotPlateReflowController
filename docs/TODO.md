## Non-exhaustive list of TODO items

1. Setup internal STM32 temperature sensor
2. Map PWM output duty cycle for fan with internal temperature sensor
3. Add start-up self-test procedure (check for any current through heater, fan, thermocouple, basically everything that can be tested)
4. Add Error state displaying the error, requiring the user to dismiss it (depending on error if even possible)
5. Add grey-coding for rotary encoder
6. Clean up code and modularize it with more (async) functions