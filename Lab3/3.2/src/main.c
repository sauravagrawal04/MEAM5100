
/* Name: main.c 3.2.2
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

void setPWMchannel(int ch)
{
    if (ch == 4)
    {
        set(DDRC, 6); // set PIN C6
    
        set(TCCR3B, CS30);
        clear(TCCR3B, CS31);
        set(TCCR3B, CS32); // set prescaler to /1024
    
        set(TCCR3A, COM3A1);
    
        clear(TCCR3B, WGM33); // set to mode 5
        set(TCCR3B, WGM32); // set to mode 5
        clear(TCCR3A, WGM31); // set to mode 5
        set(TCCR3A, WGM30); // set to mode 5

    }
    if(ch == 1)
    {
        set(DDRB, 5); // set PIN B5
        
        set(TCCR1B, CS10);
        clear(TCCR1B, CS11);
        set(TCCR1B, CS12); // set prescaler to /1024
    
        set(TCCR1A, COM1A1);
    
        clear(TCCR1B, WGM13); // set to mode 5
        set(TCCR1B, WGM12); // set to mode 5
        clear(TCCR1A, WGM11); // set to mode 5
        set(TCCR1A, WGM10); // set to mode 5   
    }
    if(ch == 5)
    {
        set(DDRB, 6); // set PIN B10
        
        set(TCCR1B, CS10);
        clear(TCCR1B, CS11);
        set(TCCR1B, CS12); // set prescaler to /1024
    
        set(TCCR1A, COM1B1);
    
        clear(TCCR1B, WGM13); // set to mode 5
        set(TCCR1B, WGM12); // set to mode 5
        clear(TCCR1A, WGM11); // set to mode 5
        set(TCCR1A, WGM10); // set to mode 5   
    }
}

void setMotor(int adc_value, int ch)
{
    // min val taken is 0.5 ms and max val taken is 2.5 ms
    //so on counter with 256 count it will take 8 count to reach 0.5 ms 
    // and will take 40 counts to reach 2.5ms 
    // to map the adc values between the range of (8 and 40) we use line equation for linear maping 
    // giving OCRA = ((40-8)/1024)*Adc value + 8 
    if (ch == 4)
    {
        OCR3A = 0.03*adc_value +8; 
    }
    else if (ch == 1)
    {
        OCR1A = 0.03*adc_value +8;
    
    }
    else if (ch == 5)
    {
        OCR1B = 0.03*adc_value +8;
    
    }
}



int main()
{
    ADC_Init(); // To initiate ADC
    m_usb_init(); // To initialize USB communication

    int adc1, adc2,adc3;  // Declare adc1 and adc2 here

    while (1)
    {
        adc_ch(4); // calling the channel for the inputs  
        setPWMchannel(4);          
        adc1 = adc_read(); // reading the input from the channel
        
        m_usb_tx_string("\rADC1 = ");  // print statement to print the channel 1
        double angle1 = (double)adc1 / 1024.0 * 180.0; // calculate the angle using the double for the motor    
        m_usb_tx_uint(angle1); // print the angle 
        m_usb_tx_string("    ");
        setMotor(adc1, 4);
        _delay_ms(50); // delay the run for 50 ms 
   // similar things is done for the second inputs 



       adc_ch(5); // calling the channel for the inputs  
        setPWMchannel(5);          
        adc3 = adc_read(); // reading the input from the channel
        
        m_usb_tx_string("\rADC3 = ");  // print statement to print the channel 1
        double angle3 = (double)adc3 / 1024.0 * 180.0; // calculate the angle using the double for the motor    
        m_usb_tx_uint(angle3); // print the angle 
        m_usb_tx_string("    ");
        setMotor(adc3, 5);
        _delay_ms(50); // delay the run for 50 ms 
   // similar things is done for the second inputs 







        adc_ch(1); 
        setPWMchannel(1); 
        adc2 = adc_read();

        m_usb_tx_string("ADC2 = ");
        
        double angle2 = (double)adc2 / 1024.0 * 180.0;
        m_usb_tx_uint(angle2);
        m_usb_tx_string("\n");
        setMotor(adc2, 1);
        _delay_ms(50);
    }

    return 0;
    /* never reached */
}


