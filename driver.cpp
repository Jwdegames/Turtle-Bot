#include <iostream>
#include <chrono>
#include <thread>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <cstdlib>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include "BNO055_OPI_5_PLUS/Adafruit_BNO055.h"

#define BNO055_DEVICE_ID 0x28

#define PIN_LED 27
#define PIN_BUTTON 18


void handle_interrupt(int s){
    printf("Caught signal %d\n",s);
    digitalWrite(25, LOW);
    digitalWrite(26, LOW);
    digitalWrite(PIN_LED, LOW);
    exit(1); 

}

void print_event(sensors_event_t* event) {
    // printf("Got event\n");
    double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
    if (event->type == SENSOR_TYPE_ACCELEROMETER) {
        printf("Accl:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_ORIENTATION) {
        printf("Orient:");
        x = event->orientation.x;
        y = event->orientation.y;
        z = event->orientation.z;
    }
    else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
        printf("Mag:");
        x = event->magnetic.x;
        y = event->magnetic.y;
        z = event->magnetic.z;
    }
    else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    printf("Gyro:");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    }
    else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    printf("Rot:");
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    }
    else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    printf("Linear:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else if (event->type == SENSOR_TYPE_GRAVITY) {
        printf("Gravity:");
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    }
    else {
        printf("Unk:");
    }

    printf("\tx= ");
    printf("%f", x);
    printf(" |\ty= ");
    printf("%f", y);
    printf(" |\tz= ");
    printf("%f\n", z);
}

int main (int argc, char **argv)
{
    wiringPiSetup();
    Adafruit_BNO055 bno055(0, "/dev/i2c-2", BNO055_DEVICE_ID);
    if (!bno055.begin()) {
        printf("Unable to begin BNO055!\n");
        return 0;
    }
    // printf("%d\n",physPinToGpio(27));
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno055.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno055.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno055.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno055.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno055.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno055.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
    printf("Printing initial values\n");
    print_event(&orientationData);
    print_event(&angVelocityData);
    print_event(&linearAccelData);
    print_event(&magnetometerData);
    print_event(&accelerometerData);
    print_event(&gravityData);

    int model = -1;

	piBoardId (&model);
	int gpio_num = 28;
	if (-1 == gpio_num) {
		printf("Failed to get the number of GPIO!\n");
    }
    pinMode(PIN_LED, OUTPUT);
    // pinMode(PIN_BUTTON, INPUT);
    printf("LED and button pins have beens setup.\n");

    // Allow for CTRL-C
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = handle_interrupt;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

   // pause();

    while (1)
    {   
    
        digitalWrite(PIN_LED, HIGH);
        digitalWrite(25, HIGH);
        digitalWrite(23, LOW);
        digitalWrite(26, HIGH);
        digitalWrite(24, LOW);

        bno055.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        bno055.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
        bno055.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
        bno055.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
        bno055.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
        bno055.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
        printf("Printing sensor values\n");
        print_event(&orientationData);
        print_event(&angVelocityData);
        print_event(&linearAccelData);
        print_event(&magnetometerData);
        print_event(&accelerometerData);
        print_event(&gravityData);
        printf("On\n");
        
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        delay(500);
        digitalWrite(25, LOW);
        digitalWrite(26, LOW);
        digitalWrite(PIN_LED, LOW);
        printf("Off\n");
        delay(500);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
