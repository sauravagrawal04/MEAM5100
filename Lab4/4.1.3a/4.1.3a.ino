/* Name: 4.1.3a
 * Author: Saurav Agrawal and 
 * Copyright: <insert your copyright message here>
 */
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "BachelorPad2022";
const char* password = "shangchi";

WiFiUDP UDPTestServer;
const IPAddress myIP(192, 168, 1, 101);         // Device A's IP addres
IPAddress TargetIP(192, 168, 1, 102);           

#define potPin A4
const int ledPin = 1;
// Define the LEDC channel and frequency range
const int ledcChannel = 1;
const int freq = 25;
const int ledcResolution = 11; // 11-bit resolution

void setup() {
Serial.begin(115200);
analogReadResolution(12); // 12-bit ADC resolution
  // Configure LEDC PWM channel
ledcSetup(ledcChannel, freq, ledcResolution);
ledcAttachPin(ledPin, ledcChannel);


Serial.print("Connecting to ");
Serial.println(ssid);
WiFi.config(myIP, IPAddress(192, 168, 1, 1),IPAddress(255, 255, 255, 0));
WiFi.begin(ssid, password);
UDPTestServer.begin(3111); // any UDP port# up to 65535

while (WiFi.status() != WL_CONNECTED) {
delay(500);
Serial.print(".");
}
Serial.println("WiFi connected");

}
const int UDP_PACKET_SIZE = 100; 
//byte sendBuffer[UDP_PACKET_SIZE];           // create the sendBuffer array

// allow packets up to 100 bytes
 byte packetBuffer[UDP_PACKET_SIZE]; // can be up to 65535

void fncUdpSend(int i) // send 2 byte int i
{
byte udpBuffer[2];
udpBuffer[0] = i & 0xff; // send 1st (LSB) byte of i
udpBuffer[1] = i>>8; // send 2nd (MSB) byte of i
UDPTestServer.beginPacket(TargetIP, 3111);
UDPTestServer.write(udpBuffer, 2); // send 2 bytes in udpBuffer
UDPTestServer.endPacket();

}
void handleUDPServer() { // receive and print int i
int i, cb = UDPTestServer.parsePacket();
if (cb) {
UDPTestServer.read(packetBuffer, cb);
i = (packetBuffer[0] + (packetBuffer[1]<<8)); // puts 2 bytes into int
Serial.println(i); // prints the number (note no need to convert to asii)

ledcWrite(ledcChannel, i);
if(i<100){
      ledcWrite(ledcChannel, 0);  // write duty to LEDC 
  }
}
}

void loop() {
 
  int potValue = analogRead(potPin); // Read the value from the potentiometer
  int dutyCycle = map(potValue, 0, 4095, 0, 4095); // Map potValue to LEDC duty cycle range
  // Set the LEDC duty cycle
  handleUDPServer();
  delay(10);    
  // Send the dutyCycle value over UDP
  fncUdpSend(dutyCycle);

  delay(10); // Adjust delay as needed for smooth control
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                  