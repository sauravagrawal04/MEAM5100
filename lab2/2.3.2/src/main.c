/* Name: main.c 2.3.2
 * Author: Saurav Agrawal
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

 #include "MEAM_general.h"
int find_period() 
{
    int pt;    // store time of previous rising edge
    int st = TCNT3;  // store start time

    set(TIFR3, ICF3);  // clear flag

    // wait for rising edge, exit if it takes too long
    while (!bit_is_set(TIFR3, ICF3)) {
       
    }   
    int curr_time = ICR3;              // store time
    int diff = curr_time - pt;  // calculate time between edges
    pt = curr_time;                      // update previous time
    return diff;
}
int main(void) 
{
    _clockdivide(0);     // set clock speed to 16MHz

    set(TCCR3B, CS30);   // set Timer pre-scaler to 1024
    clear(TCCR3B, CS31);   // set Timer  pre-scaler to 1024
    set(TCCR3B, CS32);   // set Timer  pre-scaler to 1024

    clear(DDRC, 7);      // set port C7 as input
    set(DDRB,1);        // set port B1 as output
    set(DDRB,2);        // set port B2 as output

    
    set(TCCR3B, ICES3);  // set input capture to look for rising edge
    TCNT3 = 0;           // initialize 16-bit timer

    // main infinite loop
    for (;;) {
        // match period to a frequency and turn on matching LED
         int period = find_period();
        if (period > 620 && period < 630) {      
            // frequency is 25 Hz
            clear(PORTB, 1);
            set(PORTB, 2);
            
        } 
         else {     
            clear(PORTB, 2);
            clear(PORTB, 1);
        }  
        
        if (period > 15 && period < 25) {
            // frequency is 662 Hz
            clear(PORTB,2);
            set(PORTB,1);
        } else {     
            clear(PORTB, 2);
            clear(PORTB, 1);
        }     
    }
    return 0;
}
