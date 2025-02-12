#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "hardware.h"
#include "serial.h"
#include "Bluetooth.h"

// We have these values as a bitmaskable, so that we don't have to use
// an expensive modulo operation which can cause UART read delays (trust me)
#define BT_LINES_LEN            0x10
#define BT_LINES_LEN_MASK       0xF
#define BT_LINE_BUF_SIZE        0x200
#define BT_LINE_BUF_SIZE_MASK   0x1FF

typedef struct{
    uint8_t ready;
    uint8_t buf[BT_LINE_BUF_SIZE];
} bt_line_t;

bt_line_t BTLineBuffer[BT_LINES_LEN];
uint16_t BTBufIndex  = 0;
uint16_t BTWriteIndex = 0;
uint16_t BTReadIndex = 0;
uint16_t BTNewLineRX = 0;

#define BT_PRINTF_BUFFER_SIZE 256

void BT_printf(const char *format, ...)
{
    char buffer[BT_PRINTF_BUFFER_SIZE];
    va_list args;
    va_start(args, format);
    
    // Format the string
    vsnprintf(buffer, BT_PRINTF_BUFFER_SIZE, format, args);
    
    // Send via Bluetooth
    BT_SendString(buffer);
    
    va_end(args);
}

void BT_RxBufWrite(uint8_t data)
{
    if(BTNewLineRX)
    {
        if(data == '%')
        {
            uint8_t next_index = BTWriteIndex + 1;
            next_index &= BT_LINES_LEN_MASK;
            
            if(!BTLineBuffer[next_index].ready)
            {
                BTLineBuffer[BTWriteIndex].buf[BTBufIndex] = 0;
                BTLineBuffer[BTWriteIndex].ready = 1;
                BTWriteIndex = next_index;
            }
            
            BTNewLineRX = 0;
            BTBufIndex = 0;
        }
        else
        {
            BTLineBuffer[BTWriteIndex].buf[BTBufIndex] = data;
            BTBufIndex++;
            BTBufIndex &= BT_LINE_BUF_SIZE_MASK;
        }
    }
    else if(data == '%')
    {
        BTBufIndex = 0;
        BTNewLineRX = 1;
    }
    else
    {
        if(data == '\n')
        {
            uint8_t next_index = BTWriteIndex + 1;
            next_index &= BT_LINES_LEN_MASK;
            
            if(!BTLineBuffer[next_index].ready)
            {
                BTLineBuffer[BTWriteIndex].buf[BTBufIndex] = 0;
                BTLineBuffer[BTWriteIndex].ready = 1;
                BTWriteIndex = next_index;
            }
            
            BTBufIndex = 0;
        }
        else
        {
            BTLineBuffer[BTWriteIndex].buf[BTBufIndex] = data;
            BTBufIndex++;
            BTBufIndex &= BT_LINE_BUF_SIZE_MASK;
        }
    }
}

void BT_RxBufClear(void)
{
    memset(BTLineBuffer, 0, sizeof(BTLineBuffer));
    BTBufIndex  = 0;
    BTWriteIndex = 0;
    BTReadIndex = 0;
    BTNewLineRX = 0;
}

// Clear "CMD> " From the buffer
// I hate this module
void BT_ClearCommand(void)
{
//	printf("Blahhh\r\n");
	
    while(memcmp(BTLineBuffer[BTWriteIndex].buf, "CMD> ", 5));
    memset(BTLineBuffer[BTWriteIndex].buf, 0, BT_LINE_BUF_SIZE);
    BTBufIndex = 0;
}

char* BT_GetLine(void)
{
    static char linebuffer[BT_LINE_BUF_SIZE] = {0};
    
    if(BTLineBuffer[BTReadIndex].ready)
    {
        memcpy(linebuffer, BTLineBuffer[BTReadIndex].buf, BT_LINE_BUF_SIZE);
        memset(BTLineBuffer[BTReadIndex].buf, 0, BT_LINE_BUF_SIZE);
        BTLineBuffer[BTReadIndex].ready = 0;
        BTReadIndex = (BTReadIndex + 1) & BT_LINES_LEN_MASK;
        return linebuffer;
    }
    
    return 0;
}

void BT_SendString(const char *String)
{
    while(*String)
    {
        UART_SendChar(*String++);
    }
}

char* BT_Exchange(const char* string)
{
    char* ret;
    BT_SendString(string);
    do
    {
        ret = BT_GetLine();
    }while(!ret);
    BT_ClearCommand(); 
    return ret;
}

void BT_EnterCommandMode(void)
{
    // Command Mode is the only part of the IC that doesn't have a \r terminator, really annoying to work with
    // So we're sending the command mode string and waiting for the correct response for continuing.
    printf("Entering BT Command Mode... ");
    BT_SendString("$$$");
    BT_ClearCommand();
    printf("done.\r\n");
}

void BT_ExitCommandMode(void)
{
    printf("Exiting BT Command Mode... ");
    char* ret;
    BT_SendString("---\r");
    do
    {
        ret = BT_GetLine();
    }while(!ret);
    printf("done.\r\n");
}

void BT_Reboot(void)
{
    BLE_RST = 0;
    __delay_ms(200);
    BLE_RST = 1;
    __delay_ms(200);
    
    while(strcmp(BT_GetLine(), "REBOOT")) continue;
}


void BT_SendBinary(char *Buff, uint16_t Length)
{
	while(Length--)
	{
		UART_SendChar(*Buff++);
	}
}

void BT_Init(void)
{
    BLE_RST = 0;
    __delay_ms(200);
    
    BT_RxBufClear();
    
    BLE_RST = 1;
    __delay_ms(200);
    
    while(strcmp(BT_GetLine(), "REBOOT")) continue;
}

void BT_ConfigureBeacon()
{
    printf("Configuring Bluetooth Beacon...\r\n");
    
    // Enter command mode
    BT_EnterCommandMode();
    
    // Set the device as a beacon
    BT_Exchange("SB,1\r\n");         // Enable beacon mode

    // Configure beacon parameters
    BT_Exchange("SR,00000000\r\n");  // Reset UUID to default
    BT_Exchange("SH,0201061AFF4C000215\r\n");  // Set iBeacon header

    // Set UUID (example UUID - you can modify as needed)
    BT_Exchange("SU,11223344556677889900AABBCCDDEEFF\r\n");

    // Set Major and Minor values (example values)
    BT_Exchange("SM,0001\r\n");      // Set Major value
    BT_Exchange("SN,0001\r\n");      // Set Minor value

    // Set transmission power (adjust as needed: -12 to +8 dBm)
    BT_Exchange("ST,0\r\n");         // Set TX power to 0dBm

    // Set advertising interval (default 100ms = 0x00A0)
    BT_Exchange("SI,00A0\r\n");      // Set interval

    // Save settings and reboot
    BT_SendString("R,1\r\n");          // Reboot to apply settings
	__delay_ms(1);
    
    printf("Beacon configuration complete.\r\n");
	BT_ExitCommandMode();
}

void UpdateBeaconData(float UVI)
{
    char command[32];
    uint16_t uvi_scaled = (uint16_t)(UVI * 100); // Scale UVI by 100 to preserve 2 decimal places
	BT_EnterCommandMode();
    // Properly format the command string
    snprintf(command, sizeof(command), "SN,%04X\r", uvi_scaled);
    BT_Exchange(command);
	BT_ExitCommandMode();
}