#include <Servo.h>
#include "PinChangeInterrupt.h"

#define SERVO_PIN1 9
#define SERVO_PIN2 10
#define SERVO_PIN3 11

#define ROTARY_1_PINA 12
#define ROTARY_1_PINB 13

#define ROTARY_2_PINA 6
#define ROTARY_2_PINB 7

Servo servo1;  // create servo object to control a servo
Servo servo2;
// twelve servo objects can be created on most boards

int pos1 = 90;    // variable to store the servo position
int pos2 = 90;
int pos3 = 90;

int rotary1_state = 0;
int rotary2_state = 0; 
//0 for A.
//1 for AB
//2 for .B
//3 for ..

String inputBuffer = "";  // Buffer for incoming serial data
boolean stringComplete = false;  // Flag to indicate complete message


void setup() {
  servo1.attach(SERVO_PIN1, 500, 2500);  // attaches the servo on pin 9 to the servo object
  servo1.write(pos1);

  servo2.attach(SERVO_PIN2, 250, 2500);  // attaches the servo on pin 10 to the servo object
  servo2.write(pos2);

  pinMode(ROTARY_1_PINA, INPUT);
  pinMode(ROTARY_1_PINB, INPUT);
  attachPCINT(digitalPinToPCINT(ROTARY_1_PINA), ISR_ROTARY_1A, CHANGE);
  attachPCINT(digitalPinToPCINT(ROTARY_1_PINB), ISR_ROTARY_1B, CHANGE);

  pinMode(ROTARY_2_PINA, INPUT);
  pinMode(ROTARY_2_PINB, INPUT);
  attachPCINT(digitalPinToPCINT(ROTARY_2_PINA), ISR_ROTARY_2A, CHANGE);
  attachPCINT(digitalPinToPCINT(ROTARY_2_PINB), ISR_ROTARY_2B, CHANGE);

  Serial.begin(115200);  // Changed to match Python transmitter
  inputBuffer.reserve(200);  // Reserve space for input buffer
}

void ISR_ROTARY_1A(void)
{
  switch (rotary1_state){
  case 0:
    pos1 += 1;
    rotary1_state = 3;
    break;
  case 1:
    pos1 -= 1;
    rotary1_state = 2;
    break;
  case 2:
    rotary1_state = 1;
    break;
  case 3:
    rotary1_state = 0;
    break;    
  }
  if (pos1 >= 180){
    pos1 = 180;
  }
  if (pos1 <= 0){
    pos1 = 0;
  }
  servo1.write(pos1);
  Serial.println(pos1);
}

void ISR_ROTARY_1B(void)
{
  switch (rotary1_state){
  case 0:
    pos1 -= 1;
    rotary1_state = 1;
    break;
  case 1:
    pos1 += 1;
    rotary1_state = 0;
    break;
  case 2:
    rotary1_state = 3;
    break;
  case 3:
    rotary1_state = 2;
    break;    
  }
  if (pos1 >= 180){
    pos1 = 180;
  }
  if (pos1 <= 0){
    pos1 = 0;
  }
  servo1.write(pos1);
  Serial.println(pos1);
}

void ISR_ROTARY_2A(void)
{
  switch (rotary2_state){
  case 0:
    pos2 -= 1;
    rotary2_state = 3;
    break;
  case 1:
    pos2 += 1;
    rotary2_state = 2;
    break;
  case 2:
    rotary2_state = 1;
    break;
  case 3:
    rotary2_state = 0;
    break;    
  }
  if (pos2 >= 180){
    pos2 = 180;
  }
  if (pos2 <= 0){
    pos2 = 0;
  }
  servo2.write(pos2);
  Serial.println(pos2);
}

void ISR_ROTARY_2B(void)
{
  switch (rotary2_state){
  case 0:
    pos2 += 1;
    rotary2_state = 1;
    break;
  case 1:
    pos2 -= 1;
    rotary2_state = 0;
    break;
  case 2:
    rotary2_state = 3;
    break;
  case 3:
    rotary2_state = 2;
    break;    
  }
  if (pos2 >= 180){
    pos2 = 180;
  }
  if (pos2 <= 0){
    pos2 = 0;
  }
  servo2.write(pos2);
  Serial.println(pos2);
}

void loop() {
  // Read serial data directly in loop for reliability
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n') {
      stringComplete = true;
      break;  // Process the command
    } else {
      inputBuffer += inChar;
    }
  }
  
  // Process received serial commands
  if (stringComplete) {
    processSerialCommand(inputBuffer);
    inputBuffer = "";
    stringComplete = false;
  }
}

// Note: Removed serialEvent() and moved serial reading to loop() for better reliability

void processSerialCommand(String command) {
  command.trim();  // Remove whitespace
  
  // Check if it's an angle command (format: "A:angle0,angle1,angle2")
  if (command.startsWith("A:")) {
    command.remove(0, 2);  // Remove "A:" prefix
    
    // Parse angles (can be negative for relative joints)
    float angles[3] = {90.0, 0.0, 0.0};  // Default values
    int angleIndex = 0;
    int startIndex = 0;
    
    for (int i = 0; i <= command.length(); i++) {
      if (i == command.length() || command.charAt(i) == ',') {
        if (angleIndex < 3) {
          String angleStr = command.substring(startIndex, i);
          angles[angleIndex] = angleStr.toFloat();
          angleIndex++;
        }
        startIndex = i + 1;
      }
    }
    
    // Map angles to servo range (0-180)
    // Joint 0: absolute angle, map directly
    pos1 = constrain((int)angles[0], 0, 180);
    
    // Joint 1: relative angle, map -90 to 90 -> 0 to 180
    // Map: -90° -> 0, 0° -> 90, +90° -> 180
    pos2 = constrain((int)(angles[1] + 90), 0, 180);
    
    // Joint 2: relative angle, same mapping
    pos3 = constrain((int)(angles[2] + 90), 0, 180);
    
    servo1.write(pos1);
    servo2.write(pos2);
    // servo3.write(pos3);  // Uncomment if you have a third servo
    
    // Send confirmation with original angles received
    Serial.print("OK: ");
    Serial.print(angles[0], 1);
    Serial.print(",");
    Serial.print(angles[1], 1);
    Serial.print(",");
    Serial.print(angles[2], 1);
    Serial.print(" -> Servo: ");
    Serial.print(pos1);
    Serial.print(",");
    Serial.print(pos2);
    Serial.print(",");
    Serial.println(pos3);
  }
}