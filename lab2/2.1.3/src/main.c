/* Name: main.c 2.1.3
* Author: Saurav Agrawal
* Copyright: <insert your copyright message here>
* License: <insert your license reference here>
*/
#include "MEAM_general.h"
#include "m_usb.h"
#define PRINTNUM(x) m_usb_tx_uint(x); m_usb_tx_char(10); m_usb_tx_char(13)
int i;
int oldtime;
int tperiod;
// char 10 and 13 are carriage return and line feed
void checkPC7() 
{
while(!bit_is_set(TIFR3,ICF3));
set(TIFR3,ICF3);
// int oldtime = ICR3;
// int tperiod = ICR3-oldtime;
//m_usb_tx_int(ICR3);
}

int main()
{
  m_usb_init();
  set(TCCR3B,CS30);
  set(TCCR3B,CS32);
  
  clear(DDRC,7); 
  // set(DDRB,3);
  while(1) 
  {
   
   checkPC7();
   oldtime = ICR3;
   checkPC7();
   tperiod = ICR3-oldtime;
   m_usb_tx_int(tperiod);
   m_usb_tx_string("\n");
   _delay_ms(5);
   }
}



