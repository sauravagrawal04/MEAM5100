/* Name: 4.1.2
 * Author: Saurav Agrawal
 * Copyright: <insert your copyright message here>
 */
#define potPin  A4
const int ledPin = 1;
// Define the LEDC channel and frequency range
const int ledcChannel = 1;
const int freq = 3000; 
const int ledcResolution = 12; // 12-bit resolution
void setup() {
  // Initialize ADC
  Serial.begin(115200);
  analogReadResolution(12); // 12-bit ADC resolution

  // Configure LEDC PWM channel
  ledcSetup(ledcChannel, freq, ledcResolution);
  ledcAttachPin(ledPin, ledcChannel);
}
void loop() {
  // Read the potentiometer value
  int potValue = analogRead(potPin);
  int dutyCycle = map(potValue, 0, 4095, 0, 4095);
  ledcWrite(ledcChannel, dutyCycle);
}