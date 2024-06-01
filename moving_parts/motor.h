/**
 * This is the helper file for the motor. The DC motor has two GPIO pins to control its speed.
 * 
*/

#ifndef _MOTOR_H
#define _MOTOR_H

#include <stdint.h>
#include <wiringPi.h>

class motor {
private:
    uint8_t gpio_1;
    uint8_t gpio_2;
public:
    motor(uint8_t pin_1, uint8_t pin_2); 
    uint8_t get_gpio1();
    uint8_t get_gpio2();
    void set_vals(int val_1, int val_2);


};

#endif // MOTOR_H