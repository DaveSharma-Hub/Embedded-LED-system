
#include "TimeDelay.h"
#include "clocks.h"
#include <xc.h>
#ifndef TIMEDELAY_C
#define TIMEDELAY_C

void delay(uint16_t tmr_ticks, uint8_t idle_on)
{
    tmr_ticks=(tmr_ticks*15.625);  //calculate PR2 ticks from millisecond time

    setClocks();                    // call function setClocks in clocks.c
    
    //T2CON prescaler select
    T2CONbits.TCKPS0 = 1;           /* use prescaler of 256*/
    T2CONbits.TCKPS1 = 1;           
    
    
    // Timer 2 interrupt config
    IPC1bits.T2IP = 3;  //7 is highest and 1 is lowest priority
    IEC0bits.T2IE = 1;  //enable timer interrupt
    IFS0bits.T2IF = 0;  // Clear timer 2 flag
    
    PR2 = tmr_ticks;    //PR2 stores the target to trigger T2 interrupt
    TMR2 = 0;           //zero TMR2 register to start
    T2CONbits.TON = 1;  //start timer 2
    
    if(idle_on == 1)
    {
        Idle();         //Enter idle state if idle_on==1
    }
    T2CONbits.TON = 0;  //Stop timer
    TMR2 = 0;           //zero TMR2 register on exit
    return;
}
#endif