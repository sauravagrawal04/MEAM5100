/* Name: main.c
 * Author: Saurav Agrawal
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include "MEAM_general.h"  // includes the resources included in the MEAM_general.h file

void heartbeat(int b) //creating a function hearbeat to keep track of step
{
    for(int i=0; i<=255;i++) // loop for increasing intensity
    {
        OCR3A = i/b;  // decreasing intensity so divided by b
        _delay_ms(0.4); // 0.1sec /255 *1000
        /*Since we have divided the time of 0.1 s equall so we get 0.1/255
        * for each step. but it is in ms so we multiply by 1000 to get the time
        * to delay in milliseconds*/
    }

    for(int i =255;i>=0;i--) // loop for decreasing intensity
    {
        OCR3A =i/b;  // decreasing intensity so divided by b
        _delay_ms(1.6);  // 0.4sec /255 *1000
        /*Since we have divided the time of 0.4 s equall so we get 0.4/255
        * for each step. but it is in ms so we multiply by 1000 to get the time
        * to delay in milliseconds*/
    }
    for(int i=0; i<=127;i++)  // loop for increasing intensity for 50%
    {
        OCR3A = i/b; // decreasing intensity so divided by b
        _delay_ms(0.78); // 0.1sec /127 *100
        /*Since we have divided the time of 0.1 s equall so we get 0.1/127
        * for each step. but it is in ms so we multiply by 1000 to get the time
        * to delay in milliseconds*/
    }
     for(int i =127;i>=0;i--) //loop for decreasing intensity of 50% 
    {
        OCR3A =i/b;  // decreasing intensity so divided by b
        _delay_ms(3.15);  // 0.4 sec/255 *1000
        /*Since we have divided the time of 0.4 s equall so we get 0.4/127
        * for each step. but it is in ms so we multiply by 1000 to get the time
        * to delay in milliseconds*/
    }
    _delay_ms(2000); // delay for 2 seconds
}

int main(void)
{   
    DDRC |= 0x40; // port C6 as output set pin C6 as aoutput pin 
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
    for(int j=1; j<=20;j++) //the beat should fade in 20 steps 
    {
        heartbeat(j); //function calling 
    }
 }
 return 0;

}
