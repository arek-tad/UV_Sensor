#include <xc.h>
#include "interrupts.h"
#include "Bluetooth.h"
//
//uint64_t pulse_count = 0;
//uint64_t pulse_snapshot = 0;
//uint32_t pulse_timestamp = 0;
//
void Interrupts_Init(void)
{
	PIE9bits.U3IE = 1;
	PIE9bits.U3RXIE = 1;
    
//    PIE0bits.IOCIE = 1;
//    IOCAPbits.IOCAP2 = 1; // Interrupt on positive edge for RA2
//    IOCANbits.IOCAN2 = 0; // no Interrupt on negative edge for RA2
//    IOCEPbits.IOCEP3 = 0; // no Interrupt on positive edge for RA3
//    IOCENbits.IOCEN3 = 1; // Intterupt on positive edge for RA3
    
	INTCON0bits.IPEN = 1;	//1 Enable priority levels on interrupts
	INTCON0bits.GIEL = 1;
	INTCON0bits.GIE = 1;	//Enables all unmasked interrupts and cleared by hardware for high-priority interrupts only
}

void __interrupt(irq(U3RX),base(INT_BASE),high_priority) U3RxInt(void)
{
	BT_RxBufWrite(U3RXB);
	PIR9bits.U3RXIF = 0;
}
//
//void __interrupt(irq(IOC),base(INT_BASE),high_priority) IOCxInt(void)
//{
//    if(IOCAFbits.IOCAF2)
//    {
//        pulse_count++;
//        IOCAFbits.IOCAF2 = 0;
//    }
//    
//    if(IOCEFbits.IOCEF3)
//    {
//        pulse_snapshot = pulse_count;
//        pulse_count = 0;
//        IOCEFbits.IOCEF3 = 0;
//    }
//    
//    PIR0bits.IOCIF = 0;
//}