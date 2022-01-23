/* 
 * File:   newmain.c
 * Author: Dave Sharma, Jaron Baldazo, Manjot Singh
 * Driver Project 1
 * Created on October 7, 2021, 7:17 PM
 */

//#include <stdio.h>
//#include <stdlib.h>
//#include "xc.h"
//#include <p24F16KA101.h>


/*
 * 
 */
/*
 * 
int main(int argc, char** argv) {

    TRISBbits.TRISB8=0;//set as output
    TRISBbits.TRISB4=1;//set as input
    TRISAbits.TRISA4=1;//set as input
    while(1){
        if(PORTBbits.RB4==0){
            LATBbits.LATB8=1;
            
        }
        else if(PORTAbits.RA4==0){
            LATBbits.LATB8==0;
        }
    }
    return (EXIT_SUCCESS);
}
*/



#include <math.h>
#include <xc.h>

//void delay(uint16_t, uint8_t);
//void gpio_init(void);
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void); //Timer interrupt function prototype
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void); //Change Notification interrupt function prototype  
void IOCheck(void);

//#include "IOs.c"
//#include "TimeDelay.c"

 int CN1flag = 0; //CN flag for 3 sec led flash 
 int CN0flag = 0; // CN flag for 2 sec led flash
 int CN2flag = 0; //CN flag for 1 sec led flash
 int CN3flag = 0; //CN flag for 2 or more buttons pushed
//Function declarations


int main(void){

    gpio_init(); //setup IO for microprocessor function called
    IOCheck();  //LED blinking and interrupt function called
    return 0;
}

void IOCheck(void){
    
    while (1)               //infinite while loop
    {
        delay(500,1);       // delay 0.5 sec to capture if multiple buttons pushed
        if(CN3flag==1){     //if CN3flag is turned on then enter control structure
            
            //while 2 or more buttons are pushed, keep the LED on
            while((PORTBbits.RB4==0&&PORTAbits.RA4==0&&PORTAbits.RA2==0) || (PORTBbits.RB4==0&&PORTAbits.RA4==0&&PORTAbits.RA2==1) || (PORTBbits.RB4==0&&PORTAbits.RA4==1&&PORTAbits.RA2==0) || (PORTBbits.RB4==1&&PORTAbits.RA4==0&&PORTAbits.RA2==0)){
                 IEC1bits.CNIE = 0;              //Disable CN interrupts to avoid debounce issues
                LATBbits.LATB8 = 1;             //Keep LED on
            }

            
            CN3flag=0;                      //once exit loop turn off CN3flag
            CN0flag=0;                      //once exit loop turn off CN0flag
            CN1flag=0;                      //once exit loop turn off CN1flag
            CN2flag=0;                      //once exit loop turn off CN2flag
            IEC1bits.CNIE = 1;              //Enable CN interrupts
        }
         if(CN1flag==1 ){                   //if CN1flag is turned on
             
             
            LATBbits.LATB8 = 1;             //turn on LED when button pressed 
            IEC1bits.CNIE = 1;              //Enable CN interrupts
            uint16_t timer=3000;            //timer is 3000 msec  
              
            delay(timer, 1);                //initiate a delay (one shot)
            LATBbits.LATB8 = 0;             //turn LED on

            delay(timer, 1);                //Delay for 3000msec
            
            if(PORTBbits.RB4==1){           //If RB4 button not pushed
             CN1flag = 0;                    //clear the global flag variable for PB4 is not pushed
            }
            IEC1bits.CNIE = 1;              //Enable CN interrupts
        }
         if (CN0flag == 1 )                    //if CN0flag turned on
        {
             
            LATBbits.LATB8 = 1;             //turn on LED when button pressed 
            IEC1bits.CNIE = 1;              //Enable CN interrupts
            uint16_t timer2=2000;           //timer is 2000 msec
            delay (timer2, 1);              //initiate a delay (one shot)
            LATBbits.LATB8=0;               //turn off led
            delay (timer2, 1);              //delay for 2000 msec
            if(PORTAbits.RA4==1){           //if RA4 not pushed
               CN0flag = 0;                 //clear the global flag variable for PA4
            }
            IEC1bits.CNIE = 1;              //Enable CN interrupts
        }
         if (CN2flag == 1 )                 //if CN2flag turned on
        {
            LATBbits.LATB8 = 1;              //turn on LED when button pressed 
             IEC1bits.CNIE = 1;              //Enable CN interrupts
             uint16_t timer3=1000 ;          //timer= 1000 msec
            delay (timer3, 1);               //initiate a delay (one shot)
            LATBbits.LATB8=0;                 //turn off LED
            delay (timer3, 1);                //delay for 1000 msec
            
            if(PORTAbits.RA2==1){               //if RA2 is not pushed
               CN2flag = 0;                    //clear the global flag variable for PA2
 
            }
            IEC1bits.CNIE = 1;              //Enable CN interrupts
        }
         else                                //if all CN interrupt flags are off, aka nothing pushed then turn off LED
        {
            LATBbits.LATB8 = 0;             //turn off LED as a default state
        }
    }  
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)  //Timer interrupt function
{
     IFS0bits.T2IF = 0;                                         //Clear timer 2 interrupt flag
    
     return;
}
void __attribute__ ((interrupt, no_auto_psv)) _CNInterrupt (void)   //CN interrupt function
{
    if(IFS1bits.CNIF == 1)                                      //if Timer interrupt flag is turned on
    {
        //if 2 or more buttons pushed set CN3flag on
        if((PORTBbits.RB4==0&&PORTAbits.RA4==0&&PORTAbits.RA2==0) || (PORTBbits.RB4==0&&PORTAbits.RA4==0&&PORTAbits.RA2==1) || (PORTBbits.RB4==0&&PORTAbits.RA4==1&&PORTAbits.RA2==0) || (PORTBbits.RB4==1&&PORTAbits.RA4==0&&PORTAbits.RA2==0)){
            CN3flag = 1;     // user defined global variable used as flag
        } 
        else if(PORTBbits.RB4==0 && PORTAbits.RA4==1 &&PORTAbits.RA2==1)//if only RB4 button pushed, set CN1flag on
        {
            CN1flag = 1;    // user defined global variable used as flag 
            
        }
        else if (PORTAbits.RA4 == 0 && PORTBbits.RB4==1 &&PORTAbits.RA2==1)//if only RA4 button pushed , set CN0flag on 
        {
            CN0flag = 1; // user defined global variable used as flag
            
        }
        else if(PORTAbits.RA2==0 && PORTAbits.RA4==1 && PORTBbits.RB4==1){//if only RA2 button pushed , set CN2flag on
            CN2flag = 1; // user defined global variable used as flag
        }
    }
    
    IFS1bits.CNIF = 0;              // clear IF flag
    Nop();                          // wait to next clock cycle to prevent other issues
    return;
}
