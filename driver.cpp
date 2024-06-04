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

#include "moving_parts/motor.h"

#include "sensors/ina219.h"

#define BNO055_DEVICE_ID 0x28
#define INA219_DEVICE_ID 0x41

#define PIN_LED 27
#define PIN_BUTTON 18

motor * motor1;
motor * motor2;
motor * motor3;
motor * motor4;

void handle_interrupt(int s){
    printf("\nCaught signal %d\n",s);
    digitalWrite(21, LOW);
    digitalWrite(25, LOW);
    digitalWrite(26, LOW);
    motor1 -> set_vals(0, 0);
    motor2 -> set_vals(0, 0);
    motor3 -> set_vals(0, 0);
    motor4 -> set_vals(0, 0);
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

    // Allow for CTRL-C
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = handle_interrupt;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    motor1 = new motor(19, 20);
    motor2 = new motor(21, 22);
    motor3 = new motor(23, 25);
    motor4 = new motor(24, 26);
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

	piBoardId(&model);
	int gpio_num = 28;
	if (-1 == gpio_num) {
		printf("Failed to get the number of GPIO!\n");
    }
    pinMode(PIN_LED, OUTPUT);
    for (int i = 18; i < 27 + 1; ++i) {
        pinMode(i, OUTPUT);
    }
    // pinMode(PIN_BUTTON, INPUT);
    printf("LED and button pins have beens setup.\n");

    INA219 power_sensor("/dev/i2c-2", INA219_DEVICE_ID);
		power_sensor.begin();
   // pause();
    
    while (1)
    {   
    
        // digitalWrite(PIN_LED, HIGH);
        motor1 -> set_vals(1, 0);
        motor2 -> set_vals(1, 0);
        motor3 -> set_vals(1, 0);
        motor4 -> set_vals(1, 0);

        // bno055.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        // bno055.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
        // bno055.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
        // bno055.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
        // bno055.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
        // bno055.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
        // printf("Printing sensor values\n");
        // print_event(&orientationData);
        // print_event(&angVelocityData);
        // print_event(&linearAccelData);
        // print_event(&magnetometerData);
        // print_event(&accelerometerData);
        // print_event(&gravityData);

        float shuntvoltage1 = 0;
        float busvoltage1 = 0;
        float current_mA1 = 0;
        float loadvoltage1 = 0;
        float power_mW1 = 0;
        
        shuntvoltage1 = power_sensor.getShuntVoltage_mV();
        busvoltage1 = power_sensor.getBusVoltage_V();
        current_mA1 = power_sensor.getCurrent_mA();
        power_mW1 = power_sensor.getPower_mW();
        loadvoltage1 = busvoltage1 + (shuntvoltage1 / 1000);

        printf("Bus Voltage: "); printf("%f ", busvoltage1); printf("V   ");
        printf("Shunt Voltage: "); printf("%f ", shuntvoltage1); printf("mV   ");
        printf("Load Voltage: "); printf("%f ", loadvoltage1); printf("V   ");
        printf("Current: "); printf("%f ", current_mA1); printf("mA   ");
        printf("Power: "); printf("%f ", power_mW1); printf("mW");
		printf("Percent %: "); printf("%f ", (busvoltage1 - 9)/3.6 * 100); printf("%");
        printf("\n");
        printf("\n");

        printf("On\n");
        
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        delay(500);
        motor1 -> set_vals(0, 0);
        motor2 -> set_vals(0, 0);
        motor3 -> set_vals(0, 0);
        motor4 -> set_vals(0, 0);
        digitalWrite(PIN_LED, LOW);
        printf("Off\n");
        delay(500);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
