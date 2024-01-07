/* Name: main.c
 * Author: Saurav Agrawal
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file

int main(void)
{
    float duty_cycle; //initialize duty cycle variable
    float time_period; // initialize time period 
    time_period = 1000; //Set time period 
    duty_cycle = 20; // set duty cycle percentage
    
    _clockdivide(0); //set the clock speed to 16Mhz
    set_led(ON);			// turn on the on board LED
    _delay_ms(1000);		// wait 1000 ms when at 16 MHz
    float on_time = (duty_cycle*time_period/100); /* calculate the time LED stays on 
                                                   using duty cycle and time period*/
    float off_time = time_period - on_time; // calculate off time using time period and on time
    /* insert your hardware initialization here */
    DDRB = 0x40; //Set the direction pin to Port B6 
    
    for(;;){
        //set_led(TOGGLE);	// switch the led state
        toggle(PORTB,6);        // Toggle the state of the LED pin
        _delay_ms(on_time);		// keep it on for the on time 
        toggle(PORTB,6);        // Toggle the state of the LED pin
        _delay_ms(off_time);		// delay for time period off
    }
    return 0;   /* never reached */
}
