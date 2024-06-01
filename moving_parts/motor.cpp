/**
 * This is the main file for the motor. The DC motor has two GPIO pins to control its speed.
 * 
*/

#include "motor.h"

motor::motor(uint8_t pin_1, uint8_t pin_2): gpio_1(pin_1), gpio_2(pin_2) {
    pinMode(pin_1, OUTPUT);
    pinMode(pin_2, OUTPUT);
}

uint8_t motor::get_gpio1() {
    return gpio_1;
}

uint8_t motor::get_gpio2() {
    return gpio_2;
}

void motor::set_vals(int val_1, int val_2) {
    digitalWrite(gpio_1, val_1);
    digitalWrite(gpio_2, val_2);
}