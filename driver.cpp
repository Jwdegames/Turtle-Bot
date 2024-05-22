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
    printf("\nCaught signal %d\n",s);
    digitalWrite(21, LOW);
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

void display_bno055_offsets(const adafruit_bno055_offsets_t &calibData)
{
    printf("Accelerometer: ");
    printf("%f", calibData.accel_offset_x); printf(" ");
    printf("%f", calibData.accel_offset_y); printf(" ");
    printf("%f", calibData.accel_offset_z); printf(" ");

    printf("\nGyro: ");
    printf("%f", calibData.gyro_offset_x); printf(" ");
    printf("%f", calibData.gyro_offset_y); printf(" ");
    printf("%f", calibData.gyro_offset_z); printf(" ");

    printf("\nMag: ");
    printf("%f", calibData.mag_offset_x); printf(" ");
    printf("%f", calibData.mag_offset_y); printf(" ");
    printf("%f", calibData.mag_offset_z); printf(" ");

    printf("\nAccel Radius: ");
    printf("%f", calibData.accel_radius);

    printf("\nMag Radius: ");
    printf("%f", calibData.mag_radius);
}

void config_bno055(Adafruit_BNO055 & bno055) {
    bno055.setMode(OPERATION_MODE_NDOF);
    printf("Magnetometer: Perform the figure-eight calibration dance.\n");
    uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
    bno055.getCalibration(&sys, &gyro, &accel, &mag);
    /*
    Calibration Dance Step One: Magnetometer
        Move sensor away from magnetic interference or shields
        Perform the figure-eight until calibrate
    */
    while (mag != (uint8_t)3) {
        printf("Mag Calib Status: %3.0f\n", 100.0 / 3 * mag);
        delay(1000);
        bno055.getCalibration(&sys, &gyro, &accel, &mag);
    }
    printf("CALIBRATED\n");
    delay(1000);

    printf("Accelerometer: Perform the six-step calibration dance.\n");
    /*
    Calibration Dance Step Two: Accelerometer
    Place sensor board into six stable positions for a few seconds each:
        1) x-axis right, y-axis up,    z-axis away
        2) x-axis up,    y-axis left,  z-axis away
        3) x-axis left,  y-axis down,  z-axis away
        4) x-axis down,  y-axis right, z-axis away
        5) x-axis left,  y-axis right, z-axis up
        6) x-axis right, y-axis left,  z-axis down
    Repeat the steps until calibrated
    */
    while (accel != (uint8_t)3) {
        printf("Accel Calib Status: %3.0f\n", 100.0 / 3 * accel);
        delay(1000);
        bno055.getCalibration(&sys, &gyro, &accel, &mag);
    }
    printf("CALIBRATED\n");
    delay(1000);

    /*
    Calibration Dance Step Three: Gyroscope
        Place sensor in any stable position for a few seconds
        (Accelerometer calibration may also calibrate the gyro)
    */
    while (gyro != (uint8_t)3) {
        printf("Gyro Calib Status: %3.0f\n", 100.0 / 3 * gyro);
        delay(1000);
        bno055.getCalibration(&sys, &gyro, &accel, &mag);
    }
    printf("CALIBRATED\n");
    delay(1000);

    while(!bno055.isFullyCalibrated()) {
        printf("Please continue calibrating\n");
        delay(1000);
    }

    adafruit_bno055_offsets_t offsets_type;
    bool status = bno055.getSensorOffsets(offsets_type);
    if (!status) {
        printf("Calibration Failed!\n");
        return;
    }
    printf("\nCALIBRATION COMPLETED\n");
    printf("Insert these preset offset values into project code:\n");
    display_bno055_offsets(offsets_type);


}

int main (int argc, char **argv)
{
    wiringPiSetup();
    Adafruit_BNO055 bno055(0, "/dev/i2c-2", BNO055_DEVICE_ID);
    if (!bno055.begin()) {
        printf("Unable to begin BNO055!\n");
        return 0;
    }
    // config_bno055(bno055);
    /*
    CALIBRATION COMPLETED
    Insert these preset offset values into project code:
    Accelerometer: 0.000000 0.000000 0.000000 
    Gyro: 0.000000 0.000000 0.000000 
    Mag: 0.000000 0.000000 0.000000 
    Accel Radius: 0.000000
    Mag Radius: 0.0000000
    */
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
    for (int i = 20; i < 27 + 1; ++i) {
        pinMode(i, OUTPUT);
    }
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
        digitalWrite(21, HIGH);
        digitalWrite(22, LOW);
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
        digitalWrite(21, LOW);
        digitalWrite(25, LOW);
        digitalWrite(26, LOW);
        digitalWrite(PIN_LED, LOW);
        printf("Off\n");
        delay(500);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
