#include "TimeDelay.h"
#include "IOs.h"
#include <xc.h>
#ifndef IOS_C
#define IOS_C


void gpio_init(void)
{
    AD1PCFG = 0xFFFF;      // Turn all analog pins as digi
    TRISBbits.TRISB4 = 1;   //Set B4 as an input (Pushbutton input)
    TRISAbits.TRISA4 = 1;   //Set A4 as an input (Pushbutton input)
    TRISAbits.TRISA2 = 1;   //set A2 as as input (Pushbutton input)
    CLKDIV = 0;              //set CLKDIV to zero

    //Initialize CN Interrupt

    TRISBbits.TRISB8 = 0; //Set B8 as an output (LED output)
    CNPU1bits.CN1PUE = 1; // Pull CN1 - RB4 Up (PU pushbutton active low)
    CNPU1bits.CN0PUE = 1; // Pull CN1 - RA4 Up (PU pushbutton active low)
    CNPU2bits.CN30PUE = 1; //Pull CN30- RA2 Up (PU pushbutton active low)
            
    CNEN1bits.CN0IE = 1;    //Enable CN0 interrupt
    CNEN1bits.CN1IE = 1;    //enable CN1 interrupt
    CNEN2bits.CN30IE = 1;    // enable CN2 interrupt
    IPC4bits.CNIP = 5;      //set CN interrupt priority
    IFS1bits.CNIF = 0;      //clears the CN interrupt flag
    IEC1bits.CNIE = 1;      //enable the CN interrupt (general)
    
    return;
}

//void IOCheck(void){
//    while (1)
//    {
//        delay(5000,1);
//        if(CN3flag==1){
//            while((PORTBbits.RB4==0&&PORTAbits.RA4==0&&PORTAbits.RA2==0) || (PORTBbits.RB4==0&&PORTAbits.RA4==0&&PORTAbits.RA2==1) || (PORTBbits.RB4==0&&PORTAbits.RA4==1&&PORTAbits.RA2==0) || (PORTBbits.RB4==1&&PORTAbits.RA4==0&&PORTAbits.RA2==0)){
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
//            CN2flag=0;
//            IEC1bits.CNIE = 1;              //Enable CN interrupts
//        }
//         if(CN1flag==1 ){
//        
//              //CN1flag=1;
////           IEC1bits.CNIE = 0;
//
//            LATBbits.LATB8 = 1;    //turn on LED when button pressed 
//              IEC1bits.CNIE = 1;              //Enable CN interrupts
//
//            delay(46875, 1);   //initiate a delay (one shot)
//            LATBbits.LATB8 = 0;
//
//            delay(46875, 1);
//            if(PORTBbits.RB4==1){
//             CN1flag = 0;   //clear the global flag variable for PB
//            //CN3flag=0;
//             
//            }
//            IEC1bits.CNIE = 1;
//        }
//         if (CN0flag == 1 )
//        {
//            //IEC1bits.CNIE = 0;              //Disable CN interrupts to avoid debounce issues
//            LATBbits.LATB8 = 1;             //turn on LED when button pressed 
//            IEC1bits.CNIE = 1;              //Enable CN interrupts
//
//            delay (31250, 1);   //initiate a delay (one shot)
//            LATBbits.LATB8=0;
//            delay (31250, 1);
//            if(PORTAbits.RA4==1){
//               CN0flag = 0;  //clear the global flag variable for PB
//               
//              // CN3flag=0;
//            }
//            IEC1bits.CNIE = 1;              //Enable CN interrupts
//        }
//         if (CN2flag == 1 )
//        {
//            //IEC1bits.CNIE = 0;              //Disable CN interrupts to avoid debounce issues
//            LATBbits.LATB8 = 1;             //turn on LED when button pressed 
//             IEC1bits.CNIE = 1;              //Enable CN interrupts
//            delay (15625, 1);   //initiate a delay (one shot)
//            LATBbits.LATB8=0;
//            delay (15625, 1);
//            //CN0flag = 0; 
//            if(PORTAbits.RA2==1){
//               CN2flag = 0;                    //clear the global flag variable for PB
//               
//              // CN3flag=0;
//            }//clear the global flag variable for PB
//            IEC1bits.CNIE = 1;              //Enable CN interrupts
//        }
//        //delay(5000,1); //delay for 30 msec to capture if both buttons pushed
////        if(CN3flag==1){
////            while((PORTBbits.RB4==0&&PORTAbits.RA4==0&&PORTAbits.RA2==0) || (PORTBbits.RB4==0&&PORTAbits.RA4==0&&PORTAbits.RA2==1) || (PORTBbits.RB4==0&&PORTAbits.RA4==1&&PORTAbits.RA2==0) || (PORTBbits.RB4==1&&PORTAbits.RA4==0&&PORTAbits.RA2==0)){
////                //CN3flag=0;
////                 IEC1bits.CNIE = 0;              //Disable CN interrupts to avoid debounce issues
////                LATBbits.LATB8 = 1;
////            }
//////            IEC1bits.CNIE = 0;              //Disable CN interrupts to avoid debounce issues
//////            LATBbits.LATB8 = 1;             //turn on LED when button pressed 
//////            delay (ctr_delay2, flag_idle);   //initiate a delay (one shot)
//////            LATBbits.LATB8=0;
//////            delay (ctr_delay2, flag_idle);
//////            if((PORTBbits.RB4==1 && PORTAbits.RA2==0 && PORTAbits.){
//////            CN3flag = 0;                    //clear the global flag variable for PB
//////            }
////            
////            CN3flag=0;
//////            CN0flag=0;
//////            CN1flag=0;
//////            CN2flag=0;
////            IEC1bits.CNIE = 1;              //Enable CN interrupts
////        }
//        else 
//        {
//            LATBbits.LATB8 = 0;             //turn off LED as a default state
//        }
//    }  
//    //return;
//}
#endif