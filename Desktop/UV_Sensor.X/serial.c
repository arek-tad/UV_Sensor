#include <xc.h>
#include <stdio.h>
#include "serial.h"
#include "hardware.h"

void InitialiseSerial(uint32_t Baud)
{
    uint16_t Divisor = (uint16_t)(64000000 / (4 * Baud)) - 1ul;

    // Peripheral Pin Select for RX and TX (UART1)
    U1RXPPS  = 0x0F; // RX1 PPS to RB7
    RB6PPS = 0x20;   // TX1 PPS to RB6

    // UART1 Configuration
    U1CON0bits.BRGS = 1;        // High Baud Rate Select
    U1BRG = Divisor;            // Set Baud Rate Divisor
    U1CON0bits.MODE = 0;        // Asynchronous 8-bit mode
    U1CON1bits.ON = 1;          // Enable UART1
    U1CON0bits.TXEN = 1;        // Enable Transmitter
    U1CON0bits.RXEN = 1;        // Enable Receiver
}

void InitialiseBTSerial(uint32_t Baud)
{
    uint16_t Divisor = (uint16_t)(64000000 / (4 * Baud)) - 1ul;

    // Peripheral Pin Select for RX and TX (UART1)
	U3RXPPS = 0b00001010;		//Pin RB2 set to EUSART3 RX
	RB3PPS = 0x26;				//RB3 set to EUSART3 (TX/CK) 1

    // UART1 Configuration
    U3CON0bits.BRGS = 1;        // High Baud Rate Select
    U3BRG = Divisor;            // Set Baud Rate Divisor
    U3CON0bits.MODE = 0;        // Asynchronous 8-bit mode
    U3CON1bits.ON = 1;          // Enable UART2
    U3CON0bits.TXEN = 1;        // Enable Transmitter
    U3CON0bits.RXEN = 1;        // Enable Receiver
}

void putch(char c)
{
    while (!PIR4bits.U1TXIF); // Wait until TX buffer is empty
    U1TXB = c;               // Send the character
}

void UART_SendChar(uint8_t byte)
{
	while(!U3ERRIRbits.TXMTIF)
		continue;

	U3TXB = byte;
}