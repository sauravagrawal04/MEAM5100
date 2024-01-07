
/* Name: main.c 3.1.2
 * Author: Saurav Agrawal
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */
#include "MEAM_general.h"
#include "m_usb.h"
void ADC_Init() 
{
    // Set reference voltage to AVCC                                             step 1
    set(ADMUX, REFS0);
    clear(ADMUX,REFS1);
    
    // Set ADC prescaler to 128 for 125KHz ADC clock (for 8MHz clock)           step 2 
    set(ADCSRA,ADPS0);
    set(ADCSRA,ADPS1);
    set(ADCSRA,ADPS2);

  

    // Set up interrupts and triggering (e.g., enable ADC complete interrupt)
//     set(ADCSRA, ADIE);  // Enable ADC complete interrupt
//     set(ADCSRA, ADATE); // Enable auto-triggering
// 
}

void adc_ch(uint8_t adcch)
{
    if (adcch == 0)//if using adc channel 1
    {

      set(DIDR0,ADC0D);//Disable the Digital input on channel 1             step 3
                                     
      clear(ADMUX,MUX0);//set the adc channel register                      step 5 
      clear(ADMUX,MUX1);//set the adc channel register
      clear(ADMUX,MUX2);//set the adc channel register
      clear(ADCSRB,MUX5);//set the adc channel register
    }

    if (adcch == 4)
    {
        set(DIDR0, ADC4D);  // Disable digital input on ADC0 pin

        set(ADMUX,MUX2);
        clear(ADMUX,MUX1);
        clear(ADMUX,MUX0);
        clear(ADCSRB,MUX5);
    }
    if (adcch == 1)
    {
        set(DIDR0, ADC1D);  // Disable digital input on ADC4 pin

        set(ADMUX, MUX0);  // pin f1
        clear(ADMUX, MUX1);
        clear(ADMUX, MUX2);
        clear(ADCSRB, MUX5);
    }
    if (adcch == 5)
    {
      set(DIDR0,ADC5D);
      
      set(ADMUX,MUX2);
      clear(ADMUX,MUX1);
      set(ADMUX,MUX0);
      clear(ADCSRB,MUX5);
    }
    if (adcch == 6)
    {

        set(DIDR0,ADC6D);
        
        clear(ADMUX,MUX0);
        set(ADMUX,MUX1);
        set(ADMUX,MUX2);
        clear(ADCSRB,MUX5);
    }
    if (adcch == 7)
    {

        set(DIDR0,ADC7D);
        
        set(ADMUX,MUX0);
        set(ADMUX,MUX1);
        set(ADMUX,MUX2);
        clear(ADCSRB,MUX5);
    }
    if (adcch == 9)
    {

        set(DIDR2,ADC9D);
        
        set(ADMUX,MUX0);
        clear(ADMUX,MUX1);
        clear(ADMUX,MUX2);
        set(ADCSRB,MUX5);
    }
 
    if (adcch == 12)
    {

        set(DIDR2,ADC12D);
        
        clear(ADMUX,MUX0);
        clear(ADMUX,MUX1);
        set(ADMUX,MUX2);
        set(ADCSRB,MUX5);
    }
    if (adcch == 13)
    {

        set(DIDR2,ADC13D);
        
        set(ADMUX,MUX0);
        clear(ADMUX,MUX1);
        set(ADMUX,MUX2);
        set(ADCSRB,MUX5);
    }

}
int adc_read()
{
    int adc_value;

    set(ADCSRA, ADEN); // Set ADC enable conversions                                 Step 6
    set(ADCSRA, ADSC); // Start conversion                                           step 7  
    
    for(;;)
    {
        
        //read the ADC
        if (bit_is_set(ADCSRA, ADIF))  //Wait for conversion to finish               step 8
        { 
            // if flag is set (conversion complete) update  
             // reset the flag, see page 316 

            adc_value = ADC;
            return adc_value;  // return ADC value which is used to Print in musb    step 9
            set(ADCSRA, ADIF); // clear the conversion flag                          step 10 
            set(ADCSRA, ADSC); //start converting agai                                            
        }
        
    }
}

int main()
{
    ADC_Init(); // To initiate ADC
    m_usb_init(); // To initialize USB communication

    int adc1, adc2;  // Declare adc1 and adc2 here

    while (1)
    {
        adc_ch(4); // calling the channel for the inputs            
        adc1 = adc_read(); // reading the input from the channel
        m_usb_tx_string("\rADC1 = ");  // print statement to print the channel 1
        
        double angle1 = (double)adc1 / 1024.0 * 180.0; // calculate the angle using the double for the motor    
        m_usb_tx_uint(adc1); // print the angle 
        m_usb_tx_string("    ");
        _delay_ms(50); // delay the run for 50 ms 
   // similar things is done for the second inputs 
        adc_ch(1); 
        adc2 = adc_read();

        m_usb_tx_string("ADC2 = ");
        
        double angle2 = (double)adc2 / 1024.0 * 180.0;
        m_usb_tx_uint(adc2);
        m_usb_tx_string("\n");
        _delay_ms(50);
    }

    return 0;
    /* never reached */
}


