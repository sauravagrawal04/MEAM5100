#include "html510.h"

HTML510Server h(80);

const char* ssid = "TejESP32";

// Define Motor Connections
// Right Front Motor
#define MF_PWMA 4
#define MF_AI1 6
#define MF_AI2 5
 
// Left Front Motor
#define MF_PWMB 7
#define MF_BI1 16
#define MF_BI2 15

// Right Rear Motor
#define MR_PWMA 9
#define MR_AI1 11
#define MR_AI2 10
 
// Left Rear Motor
#define MR_PWMB 12
#define MR_BI1 13
#define MR_BI2 14

int rf_PWM = 200;  // Right Front Motor
int lf_PWM =200;  // Left Front Motor
int rr_PWM =200;  // Right Rear Motor
int lr_PWM=200; // Left Front Motor
 

 
// Define Bytes to represent Mecannum Wheel Modes
// Individual bits define TB6612FNG motor driver module input states
// B7 = MF_AI1, B6 = MF_AI2, B5 = MF_BI1, B4 = MF_BI2, B3 = MR_AI1, B2 = MR_AI2, B1 = MR_BI1, B0 = MR_BI2
 
const byte MEC_FORWARD = B10101010;
const byte MEC_BACKWARD = B01010101;
const byte MEC_RIGHT = B01101001;
const byte MEC_LEFT = B10010110;
const byte MEC_DFR = B00101000;
const byte MEC_DBR = B10000010;
const byte MEC_DBL = B00010100;
const byte MEC_DFL= B01000001;
//const byte MEC_PIVOT_RIGHT_FORWARD = B00100010;
//const byte MEC_PIVOT_RIGHT_BACKWARD = B00010001;
//const byte MEC_PIVOT_LEFT_FORWARD = B10001000;
//const byte MEC_PIVOT_LEFT_BACKWARD = B01000100;
const byte MEC_ROTATE_CLOCKWISE = B01100110;
const byte MEC_ROTATE_COUNTERCLOCKWISE = B10011001;
//const byte MEC_PIVOT_SIDEWAYS_FRONT_RIGHT = B01100000;
//const byte MEC_PIVOT_SIDEWAYS_FRONT_LEFT = B10010000;
//const byte MEC_PIVOT_SIDEWAYS_REAR_RIGHT = B00001001;
//const byte MEC_PIVOT_SIDEWAYS_REAR_LEFT = B00000110;
 
// Variable for test time delay
int timeDelay = 1000;
 
// PWM Parameters for motor control
// PWM Frequency = 1KHz
const int mtrPWMFreq = 1000;
// PWM Resolution
const int mtrPWMResolution = 8;
// Define PWM channels for each motor
const int mtrRFpwmchannel = 4;
const int mtrLFpwmchannel = 5;
const int mtrRRpwmchannel = 6;
const int mtrLRpwmchannel = 7;


const char body[] PROGMEM = R"===(
<!DOCTYPE html>  
<html>
<body>        
  <h1>Motor Control</h1>
  <p>Motor Speed Control:
    <input type="range" min="0" max="255" value="127" id="speedSlider" oninput="updateSpeed(this.value)">
    <span id="speedValue">50</span>
  </p>
  <button onmousedown="startMove('forward')" onmouseup="stopCar()">Forward</button>
  <button onmousedown="startMove('backward')" onmouseup="stopCar()">Backward</button>
  <button onmousedown="startMove('left')" onmouseup="stopCar()">Left</button>
  <button onmousedown="startMove('right')" onmouseup="stopCar()">Right</button>
  <button onmousedown="startMove('diagonal-front-left')" onmouseup="stopCar()">Diagonal Front Left</button>
  <button onmousedown="startMove('diagonal-front-right')" onmouseup="stopCar()">Diagonal Front Right</button>
  <button onmousedown="startMove('diagonal-back-left')" onmouseup="stopCar()">Diagonal Back Left</button>
  <button onmousedown="startMove('diagonal-back-right')" onmouseup="stopCar()">Diagonal Back Right</button>
  <button onmousedown="startMove('rotate-left')" onmouseup="stopCar()">Rotate Left</button>
  <button onmousedown="startMove('rotate-right')" onmouseup="stopCar()">Rotate Right</button>

  <script>
    var moveInterval;

    function updateSpeed(val) {
      document.getElementById('speedValue').innerText = val;
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/speed?value=" + val, true);
      xhr.send();
    }

    function startMove(direction) {
      moveInterval = setInterval(function() {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/" + direction, true);
        xhr.send();
      }, 100); // Adjust the interval based on your requirements
    }

    function stopCar() {
      clearInterval(moveInterval);
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/stop", true);
      xhr.send();
    }
  </script>
</body>
</html>
)===";




void moveMotors(int speedRF, int speedLF, int speedRR, int speedLR, byte dircontrol) {
 
  // Moves all 4 motors
  // Directions specified in direction byte
 
  // Right Front Motor
  digitalWrite(MF_AI1, bitRead(dircontrol, 7));
  digitalWrite(MF_AI2, bitRead(dircontrol, 6));
  ledcWrite(mtrRFpwmchannel, abs(speedRF));
 
  // Left Front Motor
  digitalWrite(MF_BI1, bitRead(dircontrol, 5));
  digitalWrite(MF_BI2, bitRead(dircontrol, 4));
  ledcWrite(mtrLFpwmchannel, abs(speedLF));
 
  // Right Rear Motor
  digitalWrite(MR_AI1, bitRead(dircontrol, 3));
  digitalWrite(MR_AI2, bitRead(dircontrol, 2));
  ledcWrite(mtrRRpwmchannel, abs(speedRR));
 
  // Left Rear Motor
  digitalWrite(MR_BI1, bitRead(dircontrol, 1));
  digitalWrite(MR_BI2, bitRead(dircontrol, 0));
  ledcWrite(mtrLRpwmchannel, abs(speedLR));
}
 
void stopMotors() {
 
  // Stops all motors and motor controllers
  ledcWrite(mtrRFpwmchannel, 0);
  ledcWrite(mtrLFpwmchannel, 0);
  ledcWrite(mtrRRpwmchannel, 0);
  ledcWrite(mtrLRpwmchannel, 0);
 
  digitalWrite(MF_AI1, 0);
  digitalWrite(MF_AI2, 0);
  digitalWrite(MF_BI1, 0);
  digitalWrite(MF_BI2, 0);
  digitalWrite(MR_AI1, 0);
  digitalWrite(MR_AI2, 0);
  digitalWrite(MR_BI1, 0);
  digitalWrite(MR_BI2, 0);
  Serial.println("Stop");
}

void handleRoot() {
  h.sendhtml(body);
}

void handleSpeed() {
  int SpeedValue = h.getVal(); // Retrieve the brightness value from the request
  int rf_PWM = SpeedValue;  // Right Front Motor
  int lf_PWM = SpeedValue;  // Left Front Motor
  int rr_PWM = SpeedValue;  // Right Rear Motor
  int lr_PWM = SpeedValue;  // Left Front Motor
  h.sendhtml(body);
}

void handleForward(){
  moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_FORWARD);
  Serial.println(rf_PWM);
  h.sendhtml(body);
}
void handleBackward(){
  moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_BACKWARD);
  h.sendhtml(body);
}
void handleLeft(){
  moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_LEFT);
  h.sendhtml(body);
}
void handleRight(){
  moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_RIGHT);
  h.sendhtml(body);
}
void handleDFR(){
  moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_DFR);
  h.sendhtml(body);
}
void handleDFL(){
  moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_DFL);
  h.sendhtml(body);
}
void handleDBR(){
  moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_DBR);
  h.sendhtml(body);
}
void handleDBL(){
  moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_DBL);
  h.sendhtml(body);
}
void handleRC(){
  moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_ROTATE_CLOCKWISE);
  h.sendhtml(body);
}
void handleRAC(){
  moveMotors(rf_PWM, lf_PWM, rr_PWM, lr_PWM, MEC_ROTATE_COUNTERCLOCKWISE);
  h.sendhtml(body);
}


 
void setup() {
 
  // Set up Serial Monitor
  Serial.begin(115200);
  WiFi.softAP(ssid, ""); // Set up the access point
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
 
  // Set all connections as outputs
  pinMode(MF_PWMA, OUTPUT);
  pinMode(MF_AI1, OUTPUT);
  pinMode(MF_AI2, OUTPUT);
  pinMode(MF_PWMB, OUTPUT);
  pinMode(MF_BI1, OUTPUT);
  pinMode(MF_BI2, OUTPUT);
  pinMode(MR_PWMA, OUTPUT);
  pinMode(MR_AI1, OUTPUT);
  pinMode(MR_AI2, OUTPUT);
  pinMode(MR_PWMB, OUTPUT);
  pinMode(MR_BI1, OUTPUT);
  pinMode(MR_BI2, OUTPUT);
 
  //Set up PWM channels with frequency and resolution
  ledcSetup(mtrRFpwmchannel, mtrPWMFreq, mtrPWMResolution);
  ledcSetup(mtrLFpwmchannel, mtrPWMFreq, mtrPWMResolution);
  ledcSetup(mtrRRpwmchannel, mtrPWMFreq, mtrPWMResolution);
  ledcSetup(mtrLRpwmchannel, mtrPWMFreq, mtrPWMResolution);
 
  // Attach channels to PWM output pins
  ledcAttachPin(MF_PWMA, mtrRFpwmchannel);
  ledcAttachPin(MF_PWMB, mtrLFpwmchannel);
  ledcAttachPin(MR_PWMA, mtrRRpwmchannel);
  ledcAttachPin(MR_PWMB, mtrLRpwmchannel);

 
  h.begin();
  h.attachHandler("/", handleRoot);
  h.attachHandler("/speed?value=", handleSpeed); // Handle brightness changes
  h.attachHandler("/forward",handleForward);
  h.attachHandler("/backward",handleBackward);
  h.attachHandler("/left",handleLeft);
  h.attachHandler("/right",handleRight);
  h.attachHandler("/diagonal-front-right",handleDFR);
  h.attachHandler("/diagonal-front-left",handleDFL);
  h.attachHandler("/diagonal-back-right",handleDBR);
  h.attachHandler("/diagonal-back-left",handleDBL);
  h.attachHandler("/rotate-right",handleRC);
  h.attachHandler("/rotate-left",handleRAC);
  h.attachHandler("/stop",stopMotors);
  h.attachHandler("/", handleRoot);
}
 
void loop() {
 
  // Cycle through some Mecanum Wheel modes
  h.serve();
  delay(10);  
 

}
