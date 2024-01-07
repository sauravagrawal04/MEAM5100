/* Name: 4.1.3b
 * Author: Saurav Agrawal

 * License: <insert your license reference here>
 */
#include "WebpageLab43b.h"
#include "html510.h"
HTML510Server h(80);

const char* ssid = "SauravLab413b";
const char* password = "";
//LEDC
const int ledPin = 1;
const int ledcChannel = 1;
const int ledcResolution = 14;
int currentDutyCycle = 50; // Default duty cycle
int currentFrequency = 15; // Default frequency
void handleRoot(){
 Serial.println("Handling root...");
 h.sendhtml(body);
}

void handle_DutyCycle() {
  int newDutyCycle = h.getVal();
  Serial.print("DutyCycle print PWM : ");
  Serial.println(newDutyCycle);
  ledcWrite(ledcChannel,16383*0.01*newDutyCycle);
  h.sendhtml("");
}

void handleSlider_Freq() {
  int newFrequency = h.getVal();
  ledcSetup(ledcChannel, newFrequency, ledcResolution);
  ledcAttachPin(ledPin, ledcChannel);
  Serial.print("Frequency print: ");
  Serial.println(newFrequency);    
  h.sendhtml("");
}

void setup() 
{
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);                 //Selecting a baud rate
  Serial.print("Access Point ");Serial.print(ssid);
  WiFi.softAP(ssid,password);

  WiFi.softAPConfig(IPAddress(192, 168, 5, 2), IPAddress(192, 168, 5, 2), IPAddress(255, 255, 255, 0));
  h.begin();
    //Linking to the Web Page
  h.attachHandler("/setDutyCycle?value=",handle_DutyCycle);
  h.attachHandler("/setFrequency?value=",handleSlider_Freq);
  h.attachHandler("/",handleRoot);

}

void loop() {
  h.serve();
  delay(10);
}


