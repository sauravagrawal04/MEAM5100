
/* Name: main.c 3.1.2
 * Author: Saurav Agrawal
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */
#include "MEAM_general.h"
#include "m_usb.h"
void ADC_Init() 
{ 
    // Set reference voltage to AVCC                                    Step 1 
    set(ADMUX, REFS0); 
    clear(ADMUX,REFS1);
    
    // Set ADC prescaler to 128 for 125KHz ADC clock                    Step 2 
    set(ADCSRA,ADPS0);
    set(ADCSRA,ADPS1);
    set(ADCSRA,ADPS2);
}

// Function to disable ADC pin digital input
void ADCInput(uint8_t ADCchannel) //pin in form ADC0D,ADC1D
{
  if (ADCchannel == 0)//if using adc channel 1
  {

    set(DIDR0,ADC0D);//Disable the Digital input on channel 1             step 3
    set(ADCSRA,ADATE);//enable triggering                                 step 4
    clear(ADMUX,MUX0);//set the adc channel register                      step 5 
    clear(ADMUX,MUX1);//set the adc channel register
    clear(ADMUX,MUX2);//set the adc channel register
    clear(ADCSRB,MUX5);//set the adc channel register
  }

  if (ADCchannel == 1)//if using adc channel 1
  {

    set(DIDR0,ADC1D);
    set(ADCSRA,ADATE);
    set(ADMUX,MUX0);                  
    clear(ADMUX,MUX1);
    clear(ADMUX,MUX2);
    clear(ADCSRB,MUX5);
  }
  //not doing pin 8,10,11 since we cannot acces these pins.
  if (ADCchannel == 4)
  {

    set(DIDR0,ADC4D); 
    set(ADCSRA,ADATE);
    set(ADMUX,MUX2);
    clear(ADMUX,MUX1);
    clear(ADMUX,MUX0);
    clear(ADCSRB,MUX5);
  }
  if (ADCchannel == 5)
  {

    set(DIDR0,ADC5D);
    set(ADCSRA,ADATE);
    set(ADMUX,MUX2);
    clear(ADMUX,MUX1);
    set(ADMUX,MUX0);
    clear(ADCSRB,MUX5);
  }
  if (ADCchannel == 6)
  {

    set(DIDR0,ADC6D);
    set(ADCSRA,ADATE);
    clear(ADMUX,MUX0);
    set(ADMUX,MUX1);
    set(ADMUX,MUX2);
    clear(ADCSRB,MUX5);
  }
  if (ADCchannel == 7)
  {

    set(DIDR0,ADC7D);
    set(ADCSRA,ADATE);
    set(ADMUX,MUX0);
    set(ADMUX,MUX1);
    set(ADMUX,MUX2);
    clear(ADCSRB,MUX5);
  }
  if (ADCchannel == 9)
  {

    set(DIDR2,ADC9D);
    set(ADCSRA,ADATE);
    set(ADMUX,MUX0);
    clear(ADMUX,MUX1);
    clear(ADMUX,MUX2);
    set(ADCSRB,MUX5);
  }
 
  if (ADCchannel == 12)
  {

    set(DIDR2,ADC12D);
    set(ADCSRA,ADATE);
    clear(ADMUX,MUX0);
    clear(ADMUX,MUX1);
    set(ADMUX,MUX2);
    set(ADCSRB,MUX5);
  }
  if (ADCchannel == 13)
  {

    set(DIDR2,ADC13D);
    set(ADCSRA,ADATE);
    set(ADMUX,MUX0);
    clear(ADMUX,MUX1);
    set(ADMUX,MUX2);
    set(ADCSRB,MUX5);
  }

}

void adc_read()//the subroutine which reads ADC value
{
  set(ADCSRA,ADEN);//enabling ADC
  set(ADCSRA,ADSC);//starting conversion
}




// Function to wait for conversion to finish
void waitForConversion() {
    while (ADCSRA & (1 << ADSC)); // Wait for the ADSC bit to clear
}

// Function to read the ADC result
uint16_t readADC() {
    return ADC;
}

// Function to clear the conversion flag
void clearConversionFlag() {
    ADCSRA |= (1 << ADIF); // Writing 1 to ADIF clears the flag
}