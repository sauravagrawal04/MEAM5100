/* Name: main.c 3.3.2
 * Author: Saurav Agrawal
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

 #include "MEAM_general.h"

void ADC_Init() 
{
    // Set reference voltage to AVCC
    set(ADMUX, REFS0);
    clear(ADMUX,REFS1);
    
    // Set ADC prescaler to 128 for 125KHz ADC clock (for 8MHz clock)
    set(ADCSRA,ADPS0);
    set(ADCSRA,ADPS1);
    set(ADCSRA,ADPS2);

    // // Disable ADC pin digital input
    // set(DIDR0, ADC0);  // 
    // set(DIDR0, ADC1);  // Disable digital input on ADC1 pin  

    // Set up interrupts and triggering (e.g., enable ADC complete interrupt)
    set(ADCSRA, ADIE);  // Enable ADC complete interrupt
    set(ADCSRA, ADATE); // Enable auto-triggering
}

void adc_ch(adcch)
{
    if(adcch == 0)
        {
            set(DIDR0, ADC0);  // Disable digital input on ADC0 pin 

            clear(ADMUX, MUX0);   // pin f0
            clear(ADMUX, MUX1);
            clear(ADMUX, MUX2);
            clear(ADCSRB, MUX5);
            int adc_value;

            set(ADCSRA, ADEN); // Set ADC enable conversions

         for(;;)
          {
        
             //read the ADC
            if (bit_is_set(ADCSRA, ADIF)) 
               { 
                  // if flag is set (conversion complete) update  
                  set(ADCSRA, ADIF); // reset the flag, see page 316 

                  adc_value = ADC;
                  return adc_value;
                  break;                      
                }
              set(ADCSRA, ADSC); //start converting again
            } 
        }
    if(adcch == 1)
        {
            set(DIDR0, ADC4D);  // Disable digital input on ADC0 pin 
    
            set(ADMUX, MUX0); // pin f1
            clear(ADMUX, MUX1);
            clear(ADMUX, MUX2);
            clear(ADCSRB, MUX5);
             set(ADCSRA, ADEN); // Set ADC enable conversions

         for(;;)
          {
        
             //read the ADC
            if (bit_is_set(ADCSRA, ADIF)) 
               { 
                  // if flag is set (conversion complete) update  
                  set(ADCSRA, ADIF); // reset the flag, see page 316 

                  adc_value = ADC;
                  return adc_value;
                  break;                      
                }
              set(ADCSRA, ADSC); //start converting again
            } 
        }

}




int main() 
{
    ADC_Init(); // To initiate ADC 
    m_usb_init(); //To initialize USB communication

    while(1)
    {
    
    
    int adc1 = adc_ch(0);
    m_usb_tx_string("\rADC1 = ");    
    m_usb_tx_uint(adc1);
    m_usb_tx_string("    "); 
    _delay_ms(50);

    
    int adc2 = adc_ch(2);;

    m_usb_tx_string("ADC2 = ");     
    m_usb_tx_uint(adc2);
    m_usb_tx_string("\n"); // 
    _delay_ms(50);
    
   }
    
    return 0;   
    /* never reached */
    
    }