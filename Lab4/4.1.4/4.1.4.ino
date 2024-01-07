/* Name: 4.1.4
 * Author: Saurav Agrawal
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */
#include "WebpageLab4.h"
#include "html510.h"
HTML510Server h(80);

const char* ssid = "SauravLab414";
const char* password = "";

// Motor control pins connected to the SN754410
const int enablePin = 4; // Enable pin (EN)
const int in1PinMotor = 5;    // Input 1 (IN1)
const int in2PinMotor = 6;    // Input 2 (IN2)

//LEDC
const int ledcChannel = 1;
const int ledcResolution = 14;

// Variables for motor control


void handleRoot(){
  Serial.println("Handling root...");
  h.sendhtml(body);
}
void setMotorSpeedHandler() {
  int newSpeed= h.getVal();
  Serial.print("% speed of motor : ");
  Serial.println(newSpeed);
  ledcWrite(ledcChannel,16383*0.01*newSpeed);
  delay(10);
  h.sendhtml("");
}

void setMotorDirectionHandler() {
  String action = h.getText();
  Serial.print("Setting motor direction to: ");
  Serial.println(action);
  if (action == "forward") {
    // Set motor direction to forward (you can adapt this based on your motor driver)
    digitalWrite(in1PinMotor, HIGH);
    digitalWrite(in2PinMotor, LOW);
  } else if (action == "backward") {
    // Set motor direction to backward (you can adapt this based on your motor driver)
    digitalWrite(in1PinMotor, LOW);
    digitalWrite(in2PinMotor, HIGH);
  } else if (action == "stop") {
    // Set motor direction to stop (you can adapt this based on your motor driver)
    digitalWrite(in1PinMotor, LOW);
    digitalWrite(in2PinMotor, LOW);
  } else {
    // Handle invalid action (optional)
    Serial.println("Invalid action");
  }
}
void setup() {
  Serial.begin(115200);
  // Initialize motor control pins
  pinMode(enablePin, OUTPUT);
  pinMode(in1PinMotor, OUTPUT);
  pinMode(in2PinMotor, OUTPUT);
 
  //Set up LEDC for PWM
  ledcSetup(ledcChannel, 100,ledcResolution ); // Timer 0, 100 Hz PWM, 8-bit resolution
  ledcAttachPin(enablePin,ledcChannel ); // Attach PWM channel 0 to enablePin

  // Connect to Wi-Fi
  Serial.begin(115200);                 //Selecting a baud rate
  Serial.print("Access Point ");Serial.print(ssid);
  WiFi.softAP(ssid,password);
  WiFi.softAPConfig(IPAddress(192, 168, 5, 2), IPAddress(192, 168, 5, 2), IPAddress(255, 255, 255, 0));

  h.begin();
  // Attach web interface handler
  h.attachHandler("/", handleRoot);
  // Set up routes for controlling motor speed and direction
  h.attachHandler("/motor?speed=", setMotorSpeedHandler);
  h.attachHandler("/motor?action=", setMotorDirectionHandler);
  // Start the server
  
}

void loop() {
  h.serve();
  delay(10);
}
