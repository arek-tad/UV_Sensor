#ifndef SERIAL_H
#define	SERIAL_H
#include "hardware.h"
#include <xc.h>

extern void InitialiseSerial(uint32_t Baud);
void putch(char c);
void UART_SendChar(uint8_t byte);
void InitialiseBTSerial(uint32_t Baud);

#endif	/* SERIAL_H */