/* Name: main.c
 * Author: Saurav Agrawal
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file


int main(void)
{   
    DDRC |= 0x40; // port C6 as output set pin as output pin
   clear(TCCR3B,CS32);     // set prescaler by 64
    set(TCCR3B,CS31);       // set prescaler by 64
    set(TCCR3B,CS30);      // set prescaler by 64

    set(TCCR3B,WGM32);   // for time mode 5 
    set(TCCR3A,WGM30);    // for time mode 5 
    clear(TCCR3B,WGM33);   // for time mode 5 
    clear(TCCR3A,WGM31);    // for time mode 5 

    set(TCCR3A,COM3A1);    // to clear at OCR3A and set at rollover
    clear(TCCR3A,COM3A0);  // to clear at OCR3A and set at rollover

 while(1)
 {
    for(int i=0; i<=255;i++) //for loop for increasing brightness
    {
        OCR3A = i;  // counter
        _delay_ms(1.2);
         /*Since we have divided the time of 0.3 s equall so we get 0.3/255
        * for each step. but it is in ms so we multiply by 1000 to get the time
        * to delay in milliseconds*/
    }
    for(int i =255;i>=0;i--) //for loop for decreasing brightness
    {
        OCR3A =i; // counter
        _delay_ms(2.4);
        /*Since we have divided the time of 0.6 s equall so we get 0.6/255
        * for each step. but it is in ms so we multiply by 1000 to get the time
        * to delay in milliseconds*/
    }
 }
 return 0;

}
