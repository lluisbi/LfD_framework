#include <Servo.h>

// Define a servo object to control a servo
Servo motor0; // Thumb
Servo motor1; // Index
Servo motor2; // Long
Servo motor3; // Ring
Servo motor4; // Small

String incomingData = "";  // String to hold the received data

int  pos[5]     = {0, 0, 0, 0, 0};
int  aPos[5]    = {0, 0, 0, 0, 0};
bool display[5] = {0, 0, 0, 0, 0};

void setup() {
  // Initialize serial communication
  Serial.begin(9600);  

  // Attach the servos on pin
  motor0.attach(8);
  motor1.attach(9);
  motor2.attach(10);
  motor3.attach(11);
  motor4.attach(12);

  // Initial movements
  motor0.write(0);
  motor1.write(0);
  motor2.write(0);
  motor3.write(0);
  motor4.write(0);
  delay(500);
  motor0.write(20);
  motor1.write(20);
  motor2.write(20);
  motor3.write(20);
  motor4.write(20);
  delay(500);
  motor0.write(0);
  motor1.write(0);
  motor2.write(0);
  motor3.write(0);
  motor4.write(0);

}

void loop() {
  if (Serial.available()) {

    // Read the data from the Serial Monitor into the inputString
    incomingData = Serial.readStringUntil('\n');

    // Extract motor positions from incoming data
    pos[0] = incomingData.substring(0, 3).toInt();
    pos[1] = incomingData.substring(3, 6).toInt();
    pos[2] = incomingData.substring(6, 9).toInt();
    pos[3] = incomingData.substring(9, 12).toInt();
    pos[4] = incomingData.substring(12, 16).toInt();

    // Clear the inputString for the next iteration
    incomingData = "";
  }

  // Send motors position
  if(pos[0] > 0 && pos[0] < 140){
    motor0.write(pos[0]);
  }
  if(pos[1] > 0 && pos[1] < 180){
    motor1.write(pos[1]);
  }
  if(pos[2] > 0 && pos[2] < 180){
    motor2.write(pos[2]);
  }
  if(pos[3] > 0 && pos[3] < 180){
    motor3.write(pos[3]);
  }
  if(pos[4] > 0 && pos[4] < 180){
    motor4.write(pos[4]);
  }

}