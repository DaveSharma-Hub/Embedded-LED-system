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

// PIC24F16KA101 Configuration Bit Settings

// 'C' source line config statements

// FBS
//#pragma config BWRP = OFF               // Table Write Protect Boot (Boot segment may be written)
//#pragma config BSS = OFF                // Boot segment Protect (No boot program Flash segment)
//
//// FGS
//#pragma config GWRP = OFF               // General Segment Code Flash Write Protection bit (General segment may be written)
//#pragma config GCP = OFF                // General Segment Code Flash Code Protection bit (No protection)
//
//// FOSCSEL
//#pragma config FNOSC = FRCDIV           // Oscillator Select (8 MHz FRC oscillator with divide-by-N (FRCDIV))
//#pragma config IESO = ON                // Internal External Switch Over bit (Internal External Switchover mode enabled (Two-Speed Start-up enabled))
//
//// FOSC
//#pragma config POSCMOD = NONE           // Primary Oscillator Configuration bits (Primary oscillator disabled)
//#pragma config OSCIOFNC = OFF           // CLKO Enable Configuration bit (CLKO output signal is active on the OSCO pin)
//#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8 MHz)
//#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary oscillator configured for high-power operation)
//#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Both Clock Switching and Fail-safe Clock Monitor are disabled)
//
//// FWDT
//#pragma config WDTPS = PS32768          // Watchdog Timer Postscale Select bits (1:32,768)
//#pragma config FWPSA = PR128            // WDT Prescaler (WDT prescaler ratio of 1:128)
//#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected; windowed WDT disabled)
//#pragma config FWDTEN = ON              // Watchdog Timer Enable bit (WDT enabled)
//
//// FPOR
//#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
//#pragma config PWRTEN = ON              // Power-up Timer Enable bit (PWRT enabled)
//#pragma config I2C1SEL = PRI            // Alternate I2C1 Pin Mapping bit (Default location for SCL1/SDA1 pins)
//#pragma config BORV = V18               // Brown-out Reset Voltage bits (Brown-out Reset set to lowest voltage (1.8V))
//#pragma config MCLRE = ON               // MCLR Pin Enable bit (MCLR pin enabled; RA5 input pin disabled)
//
//// FICD
//#pragma config ICS = PGx1               // ICD Pin Placement Select bits (PGC1/PGD1 are used for programming and debugging the device)
//
//// FDS
//#pragma config DSWDTPS = DSWDTPSF       // Deep Sleep Watchdog Timer Postscale Select bits (1:2,147,483,648 (25.7 Days))
//#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select bit (DSWDT uses LPRC as reference clock)
//#pragma config RTCOSC = SOSC            // RTCC Reference Clock Select bit (RTCC uses SOSC as reference clock)
//#pragma config DSBOREN = ON             // Deep Sleep Zero-Power BOR Enable bit (Deep Sleep BOR enabled in Deep Sleep)
//#pragma config DSWDTEN = ON             // Deep Sleep Watchdog Timer Enable bit (DSWDT enabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
//#include "configuration.h"
//#include "timer.h"
//#include "timer.c"
//#include "clock.h"
//#include "clock.c"
//#include "gpio.h"
//#include "gpio.c"
//#include "interrupt.h"
//#include "interrupt.c"

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
//void gpio_init(void)
//{
//    AD1PCFG = 0xFFFF;
//    TRISBbits.TRISB4 = 1; //Set B4 as an input (Pushbutton input)
//    TRISAbits.TRISA4 = 1; //Set A4 - RB4 Up (PU pushbutton active low)
//    //CNPU1bits.CN0PUE = 1; // Pull CN1 - RB4 Up (PU pushbutton active low)
//    TRISAbits.TRISA2 = 1;
//    CLKDIV = 0; //change default timing from 2:1 to 1:1
//
//    //Initialize CN Interrupt
//
//    TRISBbits.TRISB8 = 0; //Set B8 as an output (LED output)
//    CNPU1bits.CN1PUE = 1; // Pull CN1 - RB4 Up (PU pushbutton active low)
//    CNPU1bits.CN0PUE = 1; // Pull CN1 - RB4 Up (PU pushbutton active low)
//    CNPU2bits.CN30PUE = 1;
//            
//    CNEN1bits.CN0IE = 1;    //Enable CN0 interrupt
//    CNEN1bits.CN1IE = 1;    //enable CN1 interrupt
//    CNEN2bits.CN30IE = 1;    // enable CN2 interrupt
//    IPC4bits.CNIP = 5;      //set CN interrupt priority
//    IFS1bits.CNIF = 0;      //clears the CN interrupt flag
//    IEC1bits.CNIE = 1;      //enable the CN interrupt (general)
//    
//    return;
//}
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
//void delay(uint16_t tmr_ticks, uint8_t idle_on)
//{
// //uint16_t ticks=tmr_ticks*(1/256)*(1/(0.25*pow(10,-6)))*(1/1000);
//    //T2CON config
//    T2CONbits.TSIDL = 0; //operate in idle mode
//    T2CONbits.T32 = 0; // operate timer 2 as 16 bit timer
//    T2CONbits.TCS = 0; // use internal clock
//    // T2CONbits.TGATE = 0;
//    
//    //T2CON prescaler select
//    T2CONbits.TCKPS0 = 1;
//    T2CONbits.TCKPS1 = 1;
//    
//    
//    // Timer 2 interrupt config
//    IPC1bits.T2IP = 3;  //7 is highest and 1 is lowest priority
//    IEC0bits.T2IE = 1;  //enable timer interrupt
//    IFS0bits.T2IF = 0;  // Clear timer 2 flag
//    
//    PR2 = tmr_ticks;    //PR2 stores the target to trigger T2 interrupt
//    TMR2 = 0;           //zero TMR2 register to start
//    T2CONbits.TON = 1;  //start timer 2
//    
//    if(idle_on == 1)
//    {
//        Idle();         //Enter idle state if 
//    }
//    T2CONbits.TON = 0;  //Stop timer
//    TMR2 = 0;           //zero TMR2 register on exit
//    return;
//}
//    TRISBbits.TRISB4 = 1; //Set B4 as an input (Pushbutton input)
//    TRISAbits.TRISA4 = 1;
//    TRISAbits.TRISA2=1; //
//    TRISBbits.TRISB8 = 0; //Set B8 as an output (LED output)
//    CNPU1bits.CN1PUE = 1; // Pull CN1 - RB4 Up (PU pushbutton active low)
//    
//    CLKDIV = 0; //change default timing from 2:1 to 1:1
//    
//    unsigned int ctr_delay = 1;             //initiate new variable to be used for PR2
//    unsigned int ctr_delay2 = 1;             //initiate new variable to be used for PR2
//    unsigned int ctr_delay3=1;
//    unsigned int ctr_delaySleep=1;
//    unsigned int flag_idle = 1;             //flag to initiate idle() processor state
//    
//    
//    ctr_delay = 46875;                      //value passed into delay function to be PR2
//    ctr_delay2 = 31250;
//    ctr_delay3 = ctr_delay*4;
//    ctr_delaySleep = 5000;

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

///*
// * Main code function - will be a while 1 structure that will execute forever
// 


//int main(void) {
//    TRISBbits.TRISB4 = 1; //Set B4 as an input (Pushbutton input)
//    TRISAbits.TRISA4 = 1;
//    TRISAbits.TRISA2=1;
//    TRISAbits.TRISA2=1; //
//    TRISBbits.TRISB8 = 0; //Set B8 as an output (LED output)
//    CNPU1bits.CN1PUE = 1; // Pull CN1 - RB4 Up (PU pushbutton active low)
//    
//    CLKDIV = 0; //change default timing from 2:1 to 1:1
//    
//    unsigned int ctr_delay = 1;             //initiate new variable to be used for PR2
//    unsigned int ctr_delay2 = 1;             //initiate new variable to be used for PR2
//    unsigned int ctr_delay3=1;
//    unsigned int ctr_delaySleep=1;
//    unsigned int flag_idle = 1;             //flag to initiate idle() processor state
//    
//    
//    ctr_delay = 46875;                      //value passed into delay function to be PR2
//    ctr_delay2 = 31250;
//    ctr_delay3 = ctr_delay*4;
//    ctr_delaySleep = 5000;
//    gpio_init();                            //setup GPIO
//    
//    while (1)
//    {
//    
//        if(CN1flag==1 ){
//        
//              //CN1flag=1;
//           // IEC1bits.CNIE = 0;
//            
//            LATBbits.LATB8 = 1;    //turn on LED when button pressed 
//            IEC1bits.CNIE = 1;              //Enable CN interrupts
//            delay (ctr_delay, flag_idle);   //initiate a delay (one shot)
//            LATBbits.LATB8 = 0;
//            delay(ctr_delay, flag_idle);
//            if(PORTBbits.RB4==1){
//            CN1flag = 0;   //clear the global flag variable for PB
//            //CN3flag=0;
//            }
//            IEC1bits.CNIE = 1;
//        }
//        if (CN0flag == 1)
//        {
//            //IEC1bits.CNIE = 0;              //Disable CN interrupts to avoid debounce issues
//            LATBbits.LATB8 = 1;             //turn on LED when button pressed 
//            delay (ctr_delay2, flag_idle);   //initiate a delay (one shot)
//            LATBbits.LATB8=0;
//            IEC1bits.CNIE = 1;              //Enable CN interrupts
//            delay (ctr_delay2, flag_idle);
//            if(PORTAbits.RA4==1){
//               CN0flag = 0;                    //clear the global flag variable for PB
//              // CN3flag=0;
//            }
//            IEC1bits.CNIE = 1;              //Enable CN interrupts
//        }
//        if (CN2flag == 1)
//        {
//            IEC1bits.CNIE = 0;              //Disable CN interrupts to avoid debounce issues
//            LATBbits.LATB8 = 1;             //turn on LED when button pressed 
//            delay (ctr_delay3, flag_idle);   //initiate a delay (one shot)
//            LATBbits.LATB8=0;
//            delay (ctr_delay3, flag_idle);
//            //CN0flag = 0;                    //clear the global flag variable for PB
//            IEC1bits.CNIE = 1;              //Enable CN interrupts
//        }
//        delay(ctr_delaySleep,flag_idle);
//        if(CN3flag==1){
//            while(PORTBbits.RB4==0&& PORTAbits.RA4==0){
//                //CN3flag=0;
//                 IEC1bits.CNIE = 0;              //Disable CN interrupts to avoid debounce issues
//                LATBbits.LATB8 = 1;
//            }
////            IEC1bits.CNIE = 0;              //Disable CN interrupts to avoid debounce issues
////            LATBbits.LATB8 = 1;             //turn on LED when button pressed 
////            delay (ctr_delay2, flag_idle);   //initiate a delay (one shot)
////            LATBbits.LATB8=0;
////            delay (ctr_delay2, flag_idle);
////            if((PORTBbits.RB4==1 && PORTAbits.RA2==0 && PORTAbits.){
////            CN3flag = 0;                    //clear the global flag variable for PB
////            }
//            
//            CN3flag=0;
//            CN0flag=0;
//            CN1flag=0;
//            IEC1bits.CNIE = 1;              //Enable CN interrupts
//        }
//        else 
//        {
//            LATBbits.LATB8 = 0;             //turn off LED as a default state
//        }
// 
//    }  
// 
//    return (0);
//}
//
////
//// *  Delay Function
//// *  Inputs: tmr_ticks which is an unsigned int to be used for PR2
//// *          idle_on which is a flag used to trigger idle mode
//// *  Return: nothing
//// *
//// 
//void gpio_init(void)
//{
//    TRISBbits.TRISB4 = 1; //Set B4 as an input (Pushbutton input)
//    TRISAbits.TRISA4 = 1; //Set A4 as an input (Pushbutton IP)
//     TRISAbits.TRISA2=1; //
//
//    TRISBbits.TRISB8 = 0; //Set B8 as an output (LED output)
//    CNPU1bits.CN1PUE = 1; // Pull CN1 - RB4 Up (PU pushbutton active low)
//    CNPU1bits.CN0PUE = 1; // Pull CN1 - RB4 Up (PU pushbutton active low)
//    
//    //Initialize CN Interrupt
//    CNEN1bits.CN0IE = 1;    //Enable CN0 interrupt
//    CNEN1bits.CN1IE = 1;    //enable CN1 interrupt
//    IPC4bits.CNIP = 5;      //set CN interrupt priority
//    IFS1bits.CNIF = 0;      //clears the CN interrupt flag
//    IEC1bits.CNIE = 1;      //enable the CN interrupt (general)
//    
//    CLKDIV=0;
//    return;
//}
//
//void delay(uint16_t tmr_ticks, uint8_t idle_on)
//{
// uint16_t ticks=tmr_ticks*(1/256)*(1/(0.25*pow(10,-6)))*(1/1000);
//    //T2CON config
//    T2CONbits.TSIDL = 0; //operate in idle mode
//    T2CONbits.T32 = 0; // operate timer 2 as 16 bit timer
//    T2CONbits.TCS = 0; // use internal clock
//    // T2CONbits.TGATE = 0;
//    
//    //T2CON prescaler select
//    T2CONbits.TCKPS0 = 1;
//    T2CONbits.TCKPS1 = 1;
//    
//    
//    // Timer 2 interrupt config
//    IPC1bits.T2IP = 3;  //7 is highest and 1 is lowest priority
//    IEC0bits.T2IE = 1;  //enable timer interrupt
//    IFS0bits.T2IF = 0;  // Clear timer 2 flag
//    
//    PR2 = ticks;    //PR2 stores the target to trigger T2 interrupt
//    TMR2 = 0;           //zero TMR2 register to start
//    T2CONbits.TON = 1;  //start timer 2
//    
//    if(idle_on == 1)
//    {
//        Idle();         //Enter idle state if 
//    }
//    T2CONbits.TON = 0;  //Stop timer
//    TMR2 = 0;           //zero TMR2 register on exit
//    return;
//}
//
////
//// *  Timer 2 Interrupt Service Routine
//// *  Inputs: nothing
//// *  Return: nothing
//// 
//void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void)
//{
//     IFS0bits.T2IF = 0; //Clear timer 2 interrupt flag
//    
//     return;
//}
//
//void __attribute__ ((interrupt, no_auto_psv)) _CNInterrupt (void)
//{
//    if(IFS1bits.CNIF == 1)
//    {
//        if((PORTBbits.RB4==0&&PORTAbits.RA4==0 && PORTAbits.RA2==0)){
//            CN3flag = 1;
//        } 
//        else if(PORTBbits.RB4==0 && PORTAbits.RA4==1 && PORTAbits.RA2==1)
//        {
//            CN1flag = 1; // user defined global variable used as flag
//        }
//        else if (PORTAbits.RA4 == 0 && PORTBbits.RB4==1 &&PORTAbits.RA2==1)
//        {
//            CN0flag = 1; // user defined global variable used as flag
//        }
//        else if(PORTAbits.RA2==0 && PORTAbits.RA4==1 && PORTBbits.RB4==1){
//            CN2flag = 1;
//        }
////        if((PORTBbits.RB4==0&&PORTAbits.RA2==0) ||(PORTBbits.RB4==0&&PORTAbits.RA4==0) ||(PORTAbits.RA4==0&&PORTAbits.RA2==0) ||(PORTBbits.RB4==0&&PORTAbits.RA2==0 &&PORTAbits.RA4==0)){
////            CN3flag = 1;
////        } 
//        
//    }
//    
//    IFS1bits.CNIF = 0; // clear IF flag
//    Nop();
//    
//    return;
//}


