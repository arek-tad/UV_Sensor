#include <xc.h>
#include "I2C.h"
#include "hardware.h"

#define I2CDELAY 50  // Adjustable delay for timing
#define I2C_TIMEOUT 100000

// Initiates an I²C Start condition
void I2C_Start(void) {
    SDA_TRIS = 1;  // Release SDA
    SCL_TRIS = 1;  // Release SCL
    __delay_us(I2CDELAY);
    SDA_TRIS = 0;  // Pull SDA LOW
    __delay_us(I2CDELAY);
    SCL_TRIS = 0;  // Pull SCL LOW
    __delay_us(I2CDELAY);
}

// Initiates an I²C Stop condition
void I2C_Stop(void) {
    SDA_TRIS = 0;  // Ensure SDA is LOW
    __delay_us(I2CDELAY);
    SCL_TRIS = 1;  // Release SCL
    __delay_us(I2CDELAY);
    SDA_TRIS = 1;  // Release SDA (STOP condition)
    __delay_us(I2CDELAY);
}

// Sends a byte over I²C
uint8_t I2C_Write(uint8_t data) {
    uint8_t bitCount;
    for (bitCount = 0; bitCount < 8; bitCount++) {
        SDA_TRIS = (data & 0x80) ? 1 : 0;  // Send MSB first
        __delay_us(I2CDELAY);
        SCL_TRIS = 1;
        __delay_us(I2CDELAY);
        data <<= 1;
        SCL_TRIS = 0;
        __delay_us(I2CDELAY);
    }

    SDA_TRIS = 1;  // Release SDA for ACK/NACK
    __delay_us(I2CDELAY);
    SCL_TRIS = 1;
    __delay_us(I2CDELAY);

    uint8_t ack = SDA_PORT;  // Read ACK (should be 0)
    SCL_TRIS = 0;
    __delay_us(I2CDELAY);

    return ack;  // Return 0 if ACK received, 1 if NACK received
}

// Reads a byte over I²C
uint8_t I2C_Read(uint8_t ack) {
    uint8_t bitCount, data = 0;
    SDA_TRIS = 1;  // Release SDA for reading

    for (bitCount = 0; bitCount < 8; bitCount++) {
        data <<= 1;
        __delay_us(I2CDELAY);
        SCL_TRIS = 1;
        __delay_us(I2CDELAY);
        if (SDA_PORT) data |= 1;  // Read bit
        SCL_TRIS = 0;
        __delay_us(I2CDELAY);
    }

    SDA_TRIS = (ack) ? 0 : 1;  // Send ACK (0) or NACK (1)
    __delay_us(I2CDELAY);
    SCL_TRIS = 1;
    __delay_us(I2CDELAY);
    SCL_TRIS = 0;
    __delay_us(I2CDELAY);
    return data;
}

// **AS7331 Sensor Communication**
void AS7331_write_reg(uint8_t reg, uint8_t value) {
    I2C_Start();
    I2C_Write((AS7331_ADDR << 1) | 0);
    I2C_Write(reg);
    I2C_Write(value);
    I2C_Stop();

    // Read back to confirm write
    uint8_t verify = AS7331_read_reg(reg);
    printf("Wrote REG 0x%02X: 0x%02X, Readback: 0x%02X\n", reg, value, verify);
}

uint8_t AS7331_read_reg(uint8_t reg) {
    I2C_Start();
    I2C_Write((AS7331_ADDR << 1) | 0);
    I2C_Write(reg);
    I2C_Start();  // Restart for read
    I2C_Write((AS7331_ADDR << 1) | 1);
    uint8_t data = I2C_Read(0);  // Read + send NACK
    I2C_Stop();
    return data;
}

uint16_t AS7331_read_measurement(uint8_t reg) {
    I2C_Start();
    I2C_Write((AS7331_ADDR << 1) | 0);
    I2C_Write(reg);
    I2C_Start();  // Repeated START
    I2C_Write((AS7331_ADDR << 1) | 1);
    uint8_t msb = I2C_Read(1);  // ACK after first byte
    uint8_t lsb = I2C_Read(0);  // NACK after second byte
    I2C_Stop();

    uint16_t result = ((uint16_t)lsb << 8) | msb;
    return result;
}

void UVSensor_Init(void) {
    AS7331_write_reg(0x00, 0x82);  // Enter config mode (OSR_SEL=1, MODE=10)
    __delay_ms(10);
    // Set gain, integration time, etc.
    AS7331_write_reg(6, 0b00000110);  // Max gain
    AS7331_write_reg(7, 0b01000000);  // Integration time
	AS7331_write_reg(8, 0b00000110);
    // Exit config mode and start continuous mode
    AS7331_write_reg(0x00, 0x83);  // OSR_SEL=1, MODE=11 (continuous mode)
}