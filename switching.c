#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>

#include "switching.h"


void SwitchADFSpi2Spi1()
{
    mPORTBSetBits(BIT_9);
}

void SwitchADFSpi2SpiAlt()
{
     mPORTBClearBits(BIT_9);
}

void SwitchOnSport()
{
    mPORTCClearBits(BIT_5);
}

void SwitchOffSport()
{
    mPORTCSetBits(BIT_5);
}
