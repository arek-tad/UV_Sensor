#ifndef HARDWARE_H
#define	HARDWARE_H

#include <xc.h>
#include <stdint.h>
#include <string.h>


#define _XTAL_FREQ      64000000  // Define the oscillator frequency

#define VERSION			1
#define PARTNUMBER		0x127579

//////////////////////////////////////////////////////////

// Port A
#define UART_TX_IND       LATAbits.LATA0
#define BLE_SYSTEM_CFG    LATAbits.LATA1
#define SYNC              LATAbits.LATA2
#define READY             LATAbits.LATA3
#define EN_BATTERIES      LATAbits.LATA4
#define LED               LATAbits.LATA5

// Port B
#define SDA_LATCH         LATBbits.LATB0
#define SDA_TRIS          TRISBbits.TRISB0
#define SDA_PORT          PORTBbits.RB0
#define SCL_LATCH         LATBbits.LATB1
#define SCL_TRIS          TRISBbits.TRISB1
#define SCL_PORT          PORTBbits.RB1

#define BLE_MISO          PORTBbits.RB2       
#define BLE_MOSI          LATBbits.LATB3      
#define BLE_CTS           PORTBbits.RB4
#define BLE_RTS           PORTBbits.RB5

// Port C
#define BLE_RST           LATCbits.LATC7

///////////////////////////////////////////////////////////////////////////////

// I/O states
#define PORT_A_IO_INIT    0b00000001  
#define PORT_B_IO_INIT    0b10110111  
#define PORT_C_IO_INIT    0b00011000  // Configure RC4-RC7 as inputs, others as outputs

// Sleep States
#define PORT_A_IO_SLEEP   0b00000000  // All pins low during sleep
#define PORT_B_IO_SLEEP   0b00000000
#define PORT_C_IO_SLEEP   0b00000000

// Output States
#define LATCH_A_INIT      0b00110010  // Initialise EN_BATTERIES and LED
#define LATCH_B_INIT      0b00000000
#define LATCH_C_INIT      0b10000000

#define LATCH_A_SLEEP     0b00000000
#define LATCH_B_SLEEP     0b00000001
#define LATCH_C_SLEEP     0b00000000

// Analogue Inputs
#define ANSEL_A_INIT      0b00000000  // All pins as digital
#define ANSEL_B_INIT      0b00000000
#define ANSEL_C_INIT      0b00000000

#endif /* HARDWARE_H */
