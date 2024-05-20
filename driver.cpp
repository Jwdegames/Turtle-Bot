#include <iostream>
#include <chrono>
#include <thread>
#include <wiringPi.h>
#define PIN_LED 99
#define PIN_BUTTON 18
int main (int argc, char **argv)
{
    // printf("%d\n",physPinToGpio(27));
    wiringPiSetupGpio();
    // pinMode(PIN_LED, OUTPUT);
    // pinMode(PIN_BUTTON, INPUT);
    printf("LED and button pins have beens setup.\n");
    
    while (1)
    {   
    
        digitalWrite(PIN_LED, HIGH);
        printf("On\n");
        
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        delay(500);
        digitalWrite(PIN_LED, LOW);
        printf("Off\n");
        delay(500);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
