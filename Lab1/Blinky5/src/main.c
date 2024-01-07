/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file
# define compval 7  // 
/*
* for 20 hz 1 cycle takes 1/20 th of second and for 62.5 Khz it takes 1/62.5 Khz to find 
* counter value we need to devide 1/20 to 1/62500 to get number of cycles counter. 
* Sice the clock speed is slowed by a factor of 256 Since it is acomplete cycle we need half 
* time on and half time off so we divide the answer got form above by 2. 
*/

int main(void)
{
   

    _clockdivide(8); //set the clock speed to 62.5Khz
    DDRB |= 0x40; // port B6 as output 
    set(TCCR3B,CS32);    // To set pre scaler to 62.5 Khz that is divide by 256 
    clear(TCCR3B,CS31);   // To set pre scaler to 62.5 Khz that is divide by 256
    clear(TCCR3B,CS30);   // To set pre scaler to 62.5 Khz that is divide by 256
    TCNT3 =0;          // initialsie the counter 


    /* insert your hardware initialization here */
    for(;;){
        /* insert your main loop code here */
        if (TCNT3 >compval)  //compare the counter to the compare value defined above 
        {
            toggle(PORTB,6);  // toggle when the TCNT3 value is greater than compvalue 
            TCNT3 =0; //Reset timer to 0
        }

    }
    return 0;   /* never reached */
} 