#ifndef BLUETOOTH_H
#define	BLUETOOTH_H

void BT_printf(const char *format, ...);
void BT_RxBufWrite(uint8_t data);
void BT_RxBufClear(void);
void BT_SendString(const char *String);

void BT_EnterCommandMode(void);
void BT_ExitCommandMode(void);

char* BT_GetLine(void);
char* BT_Exchange(const char* string);

void BT_SendBinary(char *Buff, uint16_t Length);

void BT_Reboot(void);

void BT_Init(void);

void BT_Setup(void);

void BT_ConfigureBeacon(void);

void UpdateBeaconData(float UVI);

#endif	/* BLUETOOTH_H */

