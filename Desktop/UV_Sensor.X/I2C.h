#ifndef I2C_H
#define I2C_H

#include <xc.h>
#include <stdint.h>

#define AS7331_ADDR 0x74 // AS7331 I2C address

// AS7331 Registers
#define AS7331_OSR   0x00
#define AS7331_CREG1 0x06
#define AS7331_CREG2 0x07
#define AS7331_CREG3 0x08
#define AS7331_TEMP  0x01
#define AS7331_MRES1 0x02
#define AS7331_MRES2 0x03
#define AS7331_MRES3 0x04

void UVSensor_Init(void);
uint16_t UVSensor_Read(uint8_t channel);
void AS7331_write_reg(uint8_t reg, uint8_t value);
uint8_t AS7331_read_reg(uint8_t reg);
uint16_t AS7331_read_measurement(uint8_t reg);
uint8_t read_device_id(void);

#endif
