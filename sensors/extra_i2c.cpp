#include "extra_i2c.h"

static inline int i2c_smbus_access (int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args ;

  args.read_write = rw ;
  args.command    = command ;
  args.size       = size ;
  args.data       = data ;
  return ioctl (fd, I2C_SMBUS, &args) ;
}

int wiringPiI2CReadBlockData (int fd, int reg, uint8_t *values, uint8_t size)
{
  union i2c_smbus_data data;

  if (size>I2C_SMBUS_BLOCK_MAX) {
    size = I2C_SMBUS_BLOCK_MAX;
  }
  data.block[0] = size;
  int result = i2c_smbus_access (fd, I2C_SMBUS_READ, reg, I2C_SMBUS_I2C_BLOCK_DATA, &data);
  if (result<0) {
    return result;
  }
  memcpy(values, &data.block[1], size);
  return data.block[0];
}

int wiringPiI2CWriteBlockData (int fd, int reg, const uint8_t *values, uint8_t size)
{
    union i2c_smbus_data data;

    if (size>I2C_SMBUS_BLOCK_MAX) {
      size = I2C_SMBUS_BLOCK_MAX;
    }
    data.block[0] = size;
    memcpy(&data.block[1], values, size);
    return i2c_smbus_access (fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_BLOCK_DATA, &data) ;
}