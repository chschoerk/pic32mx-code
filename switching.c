#include <stdio.h>
#include <stdlib.h>
#include <plib.h>
#include <p32xxxx.h>

#include "switching.h"


void switch2ClockBuffer()
{
    mPORTBClearBits(BIT_13); //disable AND
    mPORTBSetBits(BIT_6); //enable buffer
}

void switch2ClockAnd()
{
    mPORTBClearBits(BIT_6); //disable BUFFER
    mPORTBSetBits(BIT_13); //enable AND

}
