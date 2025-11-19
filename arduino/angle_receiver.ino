/*
 * Arduino Serial Receiver for FABRIK Chain Angles
 * 
 * This sketch receives joint angle data from the Python simulator
 * and can be used to control servo motors or display the angles.
 * 
 * Protocol:
 * - Receives messages in format: "A:<angle0>,<angle1>,<angle2>,...\n"
 * - Angles are in degrees
 * - Example: "A:45.50,90.00,-30.25\n"
 */

const int MAX_JOINTS = 6;
float jointAngles[MAX_JOINTS];
int numJoints = 0;

// Serial communication settings
const long BAUD_RATE = 115200;
const int BUFFER_SIZE = 128;
char receiveBuffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  
  // Initialize joint angles to zero
  for (int i = 0; i < MAX_JOINTS; i++) {
    jointAngles[i] = 0.0;
  }
  
  Serial.println("Arduino Ready - Waiting for angle data...");
}

void loop() {
  // Read incoming serial data
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    
    if (inChar == '\n') {
      // End of message - process it
      receiveBuffer[bufferIndex] = '\0';
      processMessage(receiveBuffer);
      bufferIndex = 0;
    } else if (bufferIndex < BUFFER_SIZE - 1) {
      receiveBuffer[bufferIndex++] = inChar;
    }
  }
  
  // Your control code here (e.g., move servos based on jointAngles)
  // Example:
  // servo0.write(map(jointAngles[0], -180, 180, 0, 180));
  // servo1.write(map(jointAngles[1], -180, 180, 0, 180));
}

void processMessage(char* message) {
  // Check if message starts with "A:"
  if (message[0] != 'A' || message[1] != ':') {
    Serial.println("ERROR: Invalid message format");
    return;
  }
  
  // Parse angles
  numJoints = 0;
  char* token = strtok(message + 2, ",");
  
  while (token != NULL && numJoints < MAX_JOINTS) {
    jointAngles[numJoints] = atof(token);
    numJoints++;
    token = strtok(NULL, ",");
  }
  
  // Print received angles for debugging
  Serial.print("Received ");
  Serial.print(numJoints);
  Serial.print(" angles: ");
  for (int i = 0; i < numJoints; i++) {
    Serial.print(jointAngles[i], 2);
    if (i < numJoints - 1) Serial.print(", ");
  }
  Serial.println();
  
  // Optional: Send acknowledgment
  // Serial.println("OK");
}

/* 
 * Example Servo Control Code:
 * 
 * #include <Servo.h>
 * 
 * Servo servo0, servo1, servo2;
 * 
 * void setup() {
 *   Serial.begin(115200);
 *   servo0.attach(9);
 *   servo1.attach(10);
 *   servo2.attach(11);
 * }
 * 
 * void updateServos() {
 *   // Convert angles to servo positions (0-180)
 *   // Adjust mapping based on your servo setup
 *   if (numJoints >= 1) servo0.write(map(jointAngles[0], 0, 180, 0, 180));
 *   if (numJoints >= 2) servo1.write(map(jointAngles[1], -90, 90, 0, 180));
 *   if (numJoints >= 3) servo2.write(map(jointAngles[2], -90, 90, 0, 180));
 * }
 */
