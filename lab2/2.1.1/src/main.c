/* Name: main.c 2.1.1
* Author: Saurav Agrawal
* Copyright: <insert your copyright message here>
* License: <insert your license reference here>
*/
#include "MEAM_general.h"

int main()
{
 clear(DDRC,7); 
 set(DDRB,3);
 
 while(1) 
 {
   if (bit_is_set(PINC, 7))   
     { set(PORTB,3);}      
   else
    {clear(PORTB,3);}
 }
} 

