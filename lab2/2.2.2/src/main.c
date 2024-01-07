#include "MEAM_general.h"

int main()
{
 clear(DDRC,7); 
 set(DDRB,3);
 
 while(1) 
 {
   if (bit_is_set(PINC, 7))   
     { clear(PORTB,3);}      
   else
    {set(PORTB,3);}
 }
} 

