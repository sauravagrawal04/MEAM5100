#include "MEAM_general.h"
#include "m_usb.h"
#define PRINTNUM(x) m_usb_tx_uint(x); m_usb_tx_char(10); m_usb_tx_char(13)
int i;
// char 10 and 13 are carriage return and line feed
void checkPC7() 
{
static int oldstate; 
int pinstate = bit_is_set(PINC,7);
if (pinstate != oldstate) 
{
  m_usb_tx_string("\n time - ");
  m_usb_tx_int(TCNT3);
}
oldstate = pinstate;
}

int main()
{
  m_usb_init();
  set(TCCR3B,CS30);
  set(TCCR3B,CS32);
  
  clear(DDRC,7); 
  set(DDRB,3);
  while(1) 
  {
    checkPC7();
   if (bit_is_set(PINC, 7))   
     {
      set(PORTB,3);
      }      
   else
    {
      clear(PORTB,3);}

 
  }
}

