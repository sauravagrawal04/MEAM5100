/* Name: main.c
 * Author: Saurav Agrawal
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file

int main(void)
{   
    DDRC |= 0x40; // port C6 as output 
    float duty_cycle;  // initialise duty cycle
    duty_cycle = 50;   // provide a value to duty cycle 
    set(TCCR3B,CS32);    // set prescaler /256
    clear(TCCR3B,CS31);   // set prescaler /256
    clear(TCCR3B,CS30);    // set prescaler /256

    
    set(TCCR3B,WGM32);   // for time mode 5 
    set(TCCR3A,WGM30);    // for time mode 5 
    clear(TCCR3B,WGM33);   // for time mode 5 
    clear(TCCR3A,WGM31);    // for time mode 5 

    int pmm = (duty_cycle*255/100);
    /* Calculate the integer value for counter using duty cycle.
    Since the timer is only 255 counts and duty cycle in percentage the pmm
    = duty cycle *255*0.01 */
    set(TCCR3A,COM3A1);    // to clear at OCR3A and set at rollover
    clear(TCCR3A,COM3A0);  // to clear at OCR3A and set at rollover

    OCR3A = pmm; // set OC3Ra as pmm

 for(;;){ } //endless loop 
 return 0;

}
