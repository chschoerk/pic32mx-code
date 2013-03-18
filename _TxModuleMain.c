/*
 * File:   _TxModuleMain.c
 * Author: abu
 *
 * Created on 18. Dezember 2012, 10:01
 */

#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>

#include "adf7023_mint.h"
#include "si5326.h"
#include "configandmux.h"
#include "timestamping.h"
#include "switching.h"
#include "smbus.h"
#include "measuring.h"
#include "_TxModuleMain.h"


#define tPREAMBEL_LEN       32  //byte
#define tPREAMBEL           0x55
#define tSYNCBYTE0          0x33
#define tSYNCBYTE1          0x33
#define tSYNCBYTE2          0xA6
#define tPAYLOADLEN         32   //240 //byte

#define VCO_FREQ            12288000


volatile BOOL rxDetected;
volatile unsigned int counterValue;
volatile unsigned int counterValueOld;
volatile unsigned int counterOverflow;

volatile unsigned int stallRecover;


int main(void) {

    BOOL bOk = TRUE;
    TyMCR MCRregisters;
    unsigned char dummyDat;
    unsigned int pbclockfreq;
    UINT32 filtOut;
    UINT32 expectedValue;
    int skippedFirst = 0;
    UINT32 pBuf[TMEAS_BUFFER_SIZE];
    UINT32 bufSum=0;
    int outlier=0;
    float pErrorArray[TMEAS_BUFFER_SIZE];
    float fDeviation;
    UINT32 tsData_32;
    UINT32 tmpTsData[10];
    int mn;

    counterOverflow = 0;
    counterValue = 0;
    counterValueOld = 0;
    
    /*---SYSTEM CONFIG---*/
    pbclockfreq = SYSTEMConfig(GetSystemClock(), SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);
    //SYSTEMConfigWaitStatesAndPB()
    DDPCONbits.JTAGEN = 0; //disable JTAG
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

    /*---PINMUXING (SWITCHING)---------------------------------------*/
    SwitchOffSport(); //remove with new HW
    pinMux01();
    SwitchADFSpi2Spi1(); //remove with new HW

    /*---SETUP------------------------------------------------------*/
    setupI2S();                             //I2S (TIMESTAMP OUT)
    //setupSMBus(pbclockfreq);              //I2C (SMBus slave)
    setupPWM(); //TODO 32 bit mode          //PWM (VCXO CONTROL)
    setupEdgeCount();                       //VCXCO EDGE COUNTING
    setupADF();                             //ADF7023
    ADF_MCRRegisterReadBack(&MCRregisters); //read back the MCRRegisters
    setupDetectInterrupt();                 //PREAMBEL DETECTED IRQ

    /*---ENABLE INTERRUPTS------------------------------------------*/
    INTEnableInterrupts(); 


    /*DO IT----------------------------------------------------------*/

    /*ADF: Go to RX state*/
    stallRecover = 0;
    rxDetected = FALSE;
    bOk = bOk && ADF_GoToRxState();

    expectedValue = TX_COUNTER_PERIOD;
    filtOut = expectedValue; //to minimize settling time

    mn = 0;
    while(1){        
        if (rxDetected){
            if (skippedFirst){

                bOk = readTimestampPackage(&tsData_32); //read received data from packet ram
                /*DEBUG<*/
                tmpTsData[mn] = tsData_32;
                mn++;
                if (mn == 10){
                    bOk = TRUE;
                    mn = 0;
                }
                /*>DEBUG*/

                bOk = measureFrequency(counterValue, counterValueOld, counterOverflow, pBuf, &bufSum, &fDeviation);
                if (bOk == FALSE){
                    //outlier!
                    outlier = 1; //--> do something
                }
                /*DEBUG<*/
                //pErrorArray[bfIdx-1] = fDeviation;
                //if (bfIdx == 10){ 
                //    bfIdx = 400;
                //}
                /*>DEBUG*/

                /*Additional Filter (?)*/
                    /* e.g. simple low pass filter*/
                    /*
                    tmpFilt = (filtOut >> FILT_SZ);
                    filtOut -= tmpFilt;
                    filtOut += (counterDiff >> FILT_SZ);
                    */

                    /*or moving average filter
                    if (MAWindowFull){
                        maSum = maSum - TimerVals[tarrInd] + counterDiff;
                        TimerVals[tarrInd] = counterDiff;
                        tarrInd++;
                        tarrInd &= ((0x01<<MA_SIZE)-1); //equals: tarrInd = tarrInd % 2^MA_SIZE
                        maOutput = maSum >> MA_SIZE; //devide by 2^MA_SIZE
                    }else{
                        maSum = maSum + counterDiff; //fill MA window
                        TimerVals[tarrInd] = counterDiff;
                        maOutput = maSum >> MA_SIZE; //devide by 2^MA_SIZE
                        tarrInd++;
                        if (tarrInd == (0x01<<MA_SIZE)){
                            MAWindowFull = TRUE;
                        tarrInd = 0;
                        }
                    }*/

                /*write to PWM register*/
                //SetDCOC1PWM(0x7FFF);

            }
            skippedFirst = 1;

            /*reset counters*/
            counterOverflow = 0;
            counterValueOld = counterValue;
            //rxDetected = FALSE;

            /*Clear ADF8023 Interrupt*/
            dummyDat = 0xFF;
            bOk = bOk & ADF_MMapWrite(MCR_interrupt_source_0_Adr, 0x1, &dummyDat); //clear all interrupts in source 0 by writing 1's

            //debugging
            //RetVal = ADF_MMapRead(MCR_interrupt_source_0_Adr, 0x01, &MCRByte);
            //retBool = ADF_ReadStatus(&ADStat);

            //if (stallRecover==1){
            //    stallRecover = 0;
            //}

            /*Bring ADF back to RxState*/
            rxDetected = FALSE;
            bOk = bOk && ADF_GoToRxState();
                      
        }
    }

    return (EXIT_SUCCESS);
}


void __ISR(_EXTERNAL_1_VECTOR, ipl3) INT1Interrupt()
{
   //read and reset counter value TMR1
   while(T1CON & 0x0800); //check T1CON.TWIP and wait until theres no write to TMR1 in progess
   counterValue = TMR1;
   rxDetected = TRUE;
   //mPORTBToggleBits(BIT_2);
   mINT1ClearIntFlag();

}

void __ISR(_TIMER_1_VECTOR, ipl1) T1Interrupt()
{
   //unsigned char MCRByte;
   //unsigned char RetVal;
   //BOOL retBool;
   //ADFSTA_Reg ADStat;

   counterOverflow++;
   //stallRecover = 0;
   //if (counterOverflow > 5000){
       //RetVal = ADF_MMapRead(MCR_interrupt_source_0_Adr, 0x01, &MCRByte);
       //retBool = ADF_ReadStatus(&ADStat);
       //counterOverflow = 0;
   //    rxDetected = TRUE;
   //    stallRecover = 1;
       //mPORTBToggleBits(BIT_2);
   //}
   mPORTBToggleBits(BIT_2);
   mT1ClearIntFlag();
}


void __ISR(_TIMER_3_VECTOR, ipl2) T3Interrupt()
{
   //mPORTBToggleBits(BIT_13);
   mT3ClearIntFlag();
}
