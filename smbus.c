#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>

#include "switching.h"


void setupSMBus()
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

    /*slave*/
    /*
     http://hades.mech.northwestern.edu/index.php/PIC32MX:_I2C_Communication_between_PIC32s
     */
}
