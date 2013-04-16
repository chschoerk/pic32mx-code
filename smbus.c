#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>

#include "switching.h"
#include "smbus.h"

extern volatile unsigned char sendData;
extern volatile unsigned char recvData;
extern volatile unsigned char smbusCmdReceived;

void setupSMBus(int pbclockfreq)
{
    UINT16 mySlaveAddress = 0x0E;

    I2CSetSlaveAddress(I2C2, mySlaveAddress, 0, I2C_USE_7BIT_ADDRESS);
    OpenI2C2( I2C_EN | I2C_SM_EN | I2C_SLW_DIS | I2C_ACK_EN | I2C_7BIT_ADD, ((pbclockfreq/2/50000)-2) ); //50000
    SetPriorityIntI2C2(I2C_INT_PRI_2 | I2C_INT_SUB_PRI_3);
    EnableIntSI2C2; //enable slave interrupt

}

void __ISR(_I2C_2_VECTOR, ipl2) I2C2Interrupt()
{
    //static unsigned char sendData = 0;
    unsigned char dummy = 0;
    //unsigned char recvData = 0;
    static int ix = 0;

    /*master wants to read*/
    /*every read command from the Beaglebone triggers 2 IRQs.
     *The first one with I2C2STAT = 0x0E, the second one with
     *I2C2STAT = =0x34 - only the first one is important,ignore
     *in the second one*/
    if ( (I2C2ASTATbits.R_W==1) && (I2C2ASTATbits.RBF==1) ){
        //SlaveWriteI2C2(sendData);
        dummy = SlaveReadI2C2();
        SlaveWriteI2C2(sendData);
    }

    /*master wants to write*/
    /*every write commandfrom the Beaglebone triggers 2 IRQs.
     *The first read results in reading the address+RW, the
     *second one is the actual data*/
    if ( I2C2ASTATbits.R_W==0 ){
        recvData = SlaveReadI2C2();
        ix++;
        if (ix == 2){
            smbusCmdReceived = 1;
            ix = 0;
        }  
    }

    mI2C2SClearIntFlag();
}

/*
void setupSMBus(int pbclockfreq)
{
    unsigned int i2c_address;
    int i2c_data;
    int i2c_rcv1;
    int i2c_rcv2;
    unsigned char i2cRcv[2];

    //Enable I2C channel and set the baud rate to BRG_VAL)
    //while(1);

    OpenI2C2( I2C_EN | I2C_SM_EN | I2C_SLW_DIS | I2C_ACK_EN, ((pbclockfreq/2/50000)-2) ); //50000

    i2c_address = 0x16; //charge = 0x12, gauge = 0x16

    IdleI2C2();
    StartI2C2();	        //Send the Start Bit
    IdleI2C2();		//Wait to complete
    MasterWriteI2C2(i2c_address | 0);  //Sends the slave address over the I2C line.  This must happen first so the
                                             //proper slave is selected to receive data.
    IdleI2C2();	        //Wait to complete
    MasterWriteI2C2(0x0d);  //SBS command: reltaveStateOfCharge
    IdleI2C2();		//Wait to complete

    RestartI2C2();									//Restart signal
    //while(I2C2CONbits.RSEN ); 						//Wait till Restart sequence is completed
    //for(i=0;i<1000;i++);
    IdleI2C2();
    MasterWriteI2C2(i2c_address | 1);
    IdleI2C2();

    MastergetsI2C2(2,i2cRcv,1000);
    //i2c_rcv1 = MasterReadI2C2();		//Read in a value
    //IdleI2C2();
    //i2c_rcv2 = MasterReadI2C2();		//Read in a value

    IdleI2C2();
    StopI2C2();	        //Send the Stop condition
    IdleI2C2();	        //Wait to complete
    CloseI2C2();

    //slave
    //
    // http://hades.mech.northwestern.edu/index.php/PIC32MX:_I2C_Communication_between_PIC32s
    //
}*/
