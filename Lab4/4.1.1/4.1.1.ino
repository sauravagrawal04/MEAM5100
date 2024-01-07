/* Name: main.c 4.1.1
 * Author: Saurav Agrawal
 * Copyright: <insert your copyright message here>
 */
// Define GPIO pin numbers
int buttonPin = 4; // Connected to the button
int ledPin = 5;   // Connected to the LED
void setup() {
 Serial.begin(115200);
 pinMode(buttonPin, INPUT); // Set the button pin as input with pull-up resistor
 pinMode(ledPin, OUTPUT);      
}

void loop() {
  int buttonState = digitalRead(buttonPin);
  Serial.println(buttonState);
  
  //If the button is pressed (LOW), turn on the LED
  if (buttonState == HIGH) {
    digitalWrite(ledPin, HIGH);
  } 
  else {
    digitalWrite(ledPin, LOW);
  }

 }
