TARGET = driver
BNO_SENSOR_DIRECTORY = BNO055_OPI_5_PLUS
MOVING_PARTS_DIRECTORY = moving_parts

CC = g++
CFLAGS = -Wall -g 
POST_CFLAGS = -lwiringPi

all: $(TARGET)
 
$(TARGET): $(TARGET).o $(BNO_SENSOR_DIRECTORY)/Adafruit_I2CDevice.o $(BNO_SENSOR_DIRECTORY)/Adafruit_Sensor.o $(BNO_SENSOR_DIRECTORY)/Adafruit_BNO055.o $(MOVING_PARTS_DIRECTORY)/motor.o
			$(CC) $(CFLAGS) -o $(TARGET) $(TARGET).o $(BNO_SENSOR_DIRECTORY)/Adafruit_I2CDevice.o $(BNO_SENSOR_DIRECTORY)/Adafruit_Sensor.o $(BNO_SENSOR_DIRECTORY)/Adafruit_BNO055.o $(MOVING_PARTS_DIRECTORY)/motor.o $(POST_CFLAGS)

$(TARGET).o: $(TARGET).cpp $(BNO_SENSOR_DIRECTORY)/Adafruit_I2CDevice.h $(BNO_SENSOR_DIRECTORY)/Adafruit_Sensor.h $(BNO_SENSOR_DIRECTORY)/Adafruit_BNO055.h $(MOVING_PARTS_DIRECTORY)/motor.h
			$(CC) $(CFLAGS) -c $(TARGET).cpp 

clean:
			$(RM) $(TARGET) $(TARGET).o $(BNO_SENSOR_DIRECTORY)/Adafruit_I2CDevice.o $(BNO_SENSOR_DIRECTORY)/Adafruit_Sensor.o $(BNO_SENSOR_DIRECTORY)/Adafruit_BNO055.o $(MOVING_PARTS_DIRECTORY)/motor.o



$(BNO_SENSOR_DIRECTORY)/Adafruit_BNO055.o: $(BNO_SENSOR_DIRECTORY)/Adafruit_BNO055.h $(BNO_SENSOR_DIRECTORY)/Adafruit_I2CDevice.h $(BNO_SENSOR_DIRECTORY)/Adafruit_Sensor.h $(BNO_SENSOR_DIRECTORY)/utility/imumaths.h $(BNO_SENSOR_DIRECTORY)/utility/matrix.h $(BNO_SENSOR_DIRECTORY)/utility/quaternion.h $(BNO_SENSOR_DIRECTORY)/utility/vector.h

$(BNO_SENSOR_DIRECTORY)/Adafruit_I2CDevice.o: $(BNO_SENSOR_DIRECTORY)/Adafruit_I2CDevice.h

$(BNO_SENSOR_DIRECTORY)/Adafruit_Sensor.o: $(BNO_SENSOR_DIRECTORY)/Adafruit_Sensor.h 

$(MOVING_PARTS_DIRECTORY)/motor.o: $(MOVING_PARTS_DIRECTORY)/motor.h


# run: $(shell sudo ./$(TARGET))