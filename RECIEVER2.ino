#include <HCPCA9685.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#define in1 2
#define in2 4
#define camPin 10
#define in3 5
#define in4 7
#define light 12
#define enA 3
#define enB 6

/* I2C slave address for the servodriver device/module. For the HCMODU0097 the default I2C address
   is 0x40 */
#define  I2CAdd 0x40

/* Create an instance of the servodriver library */
HCPCA9685 HCPCA9685(I2CAdd);
Servo camServo;

/* Create an instance of the RF library */
RF24 radio(8, 9); // CE, CSN

const byte address[6] = "00001";

//String holding received data
char recieved[41] = "";
String recievedData, substringLeft, substringRight;


int  xAxis, yAxis;
int leftArm = A0;
int rightArm = A1;
int readyPin = A2;

//Wheel Speeds
int motorSpeedA = 0;
int motorSpeedB = 0;

//initial parking position of the motor

const int servo_joint_3_parking_pos = 150;
const int cam_parking_pos = 100;

//Degree of robot servo sensitivity - Intervals

int servo_joint_3_pos_increment = 5;
int cam_pos_increment = 5;

//Keep track of the current value of the motor positions
int servo_joint_3_parking_pos_i = servo_joint_3_parking_pos;
int cam_parking_pos_i = cam_parking_pos;

//Minimum and maximum angle of servo motor
int servo_joint_3_min_pos = 100;
int servo_joint_3_max_pos = 350;

int cam_min_pos = 100;
int cam_max_pos = 200;

int cam_pos;
char statae0, state1, state2, state3, state4, state10, state11, camStatus;
int response_time_5 = 5;
int response_time = 10;


void setup() {

  // Serial.begin(9600);// Initialise default communication rate of the NRF module

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(light, OUTPUT);
  pinMode(camPin, OUTPUT);
  pinMode(leftArm, OUTPUT);
  pinMode(rightArm, OUTPUT);
  pinMode(readyPin, OUTPUT);

  digitalWrite(light, HIGH);

  /* Wake the root arms up */
  HCPCA9685.Init(SERVO_MODE);
  HCPCA9685.Sleep(false);
  delay(3000);
  camServo.attach(camPin);

  /* initialize radio */
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

}
void loop() {
  if (radio.available()) {   // If the NRF240L01 module recieved data
    //  Serial.println("YEAH");
    delay(50);
    radio.read(&recieved, sizeof(recieved));
    recievedData = String(recieved);
    //  Serial.println(recievedData);

    //Check First Byte
    if (recievedData.charAt(0) == '0') {
      joystickSubroutine();
    }
    else if (recievedData.charAt(0) == '1') {
      gloveSubroutine();
    }
  }
  else {
    //  Serial.println("NOPE");
    //Stop the Wheels in case of a loss in transmission
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);

    analogWrite(enA, 0);
    analogWrite(enB, 0);

    //Write a 0 to the readyPin to indicate that no signals are going to the NANO_SERVODRIVER
    analogWrite(readyPin, 0);
  }
  delay(20);
}

void joystickSubroutine() {
  //Check Second Byte
  //Wheel Command
  wheelSubroutine();
  //Arms Command
  armSubroutine();
  //Camera Command
  cameraSubroutine();
}

void gloveSubroutine() {
  state1 = recievedData.charAt(1);
  state2 = recievedData.charAt(2);
  state3 = recievedData.charAt(3);
  state4 = recievedData.charAt(4);


  //Move (Base Rotation) Left for left arm
  if (state3 == 'A') {
    baseRotateLeft(6);
    baseRotateLeft(7);
  }

  //Move (Base Rotation) Right for left arm
  else if (state3 == 'B') {
    baseRotateRight(6);
    baseRotateRight(7);
  }

  //Write 8 to the ready Pin to show that command is coming from Arduino Uno Master
  if (state4 == 'E') {//Down
    analogWrite(readyPin, 8);
    analogWrite(leftArm, 350);
    analogWrite(rightArm,350);
  }

  else if (state4 == 'F') {//Up
    analogWrite(readyPin, 8);
    analogWrite(leftArm, 90);
    analogWrite(rightArm,90);
  }

  if (state2 == 'T') {// Serial.println("FRONT");
    // Set Motor A forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // Convert the increasing Y-axis readings for going forward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = 220;
    motorSpeedB = 220;

  }
  else if (state2 == 't') {  //Serial.println("BACK");
    // Set Motor A backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B backward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    // Convert the declining x-axis readings for going backward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = 220;
    motorSpeedB = 220;
  }

  else if (state1 == 'I') {
    //rIGHT
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B forward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    // Convert the increasing Y-axis readings for going forward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = 220;
    motorSpeedB = 220;

  }

  else if (state1 == 'i') {//lEFT
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // Convert the increasing Y-axis readings for going forward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = 220;
    motorSpeedB = 220;

  }

  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);

    motorSpeedA = 0;
    motorSpeedB = 0;
  }

  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);

  delay(50);
}

void wheelSubroutine() {
  //Both xaxis and y axis are 2 bytes long, default is between 490 and 540
  xAxis = recievedData.substring(2, 6).toInt(); // Convert the data from the character array (recieved X value) into integer
  yAxis = recievedData.substring(6, 10).toInt();


  // X-axis used for forward and backward control
  if (xAxis > 600) {  //Serial.println("BACK");
    // Set Motor A backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B backward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    // Convert the declining x-axis readings for going backward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(xAxis, 550, 1023, 0, 255);
    motorSpeedB = map(xAxis, 550, 1023, 0, 255);

  }
  else if (xAxis < 490) { //Serial.println("FRONT");
    // Set Motor A forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // Convert the increasing Y-axis readings for going forward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(xAxis, 490, 0, 0, 255);
    motorSpeedB = map(xAxis, 490, 0, 0, 255);

  }

  // Y-axis used for left and right control
  else if (yAxis < 490) {
    // Go Right
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    // Convert the declining y-axis readings for going backward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(yAxis, 490, 0, 0, 255);
    motorSpeedB = map(yAxis, 490, 0, 0, 255);

  }
  else if (yAxis > 600) {
    // Go Left
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    // Convert the declining x-axis readings for going backward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(yAxis, 550, 1023, 0, 255);
    motorSpeedB = map(yAxis, 550, 1023, 0, 255);

  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);

    motorSpeedA = 0;
    motorSpeedB = 0;

  }

  if (motorSpeedA < 70) {
    motorSpeedA = 0;
  }
  if (motorSpeedB < 70) {
    motorSpeedB = 0;
  }

  analogWrite(enA, motorSpeedA);
  analogWrite(enB, motorSpeedB);

  delay(20);
}

void armSubroutine() {
  //This data will be * to deselect
  state10 = recievedData.charAt(10);
  state11 = recievedData.charAt(11);
  substringLeft = recievedData.substring(12, 16) + recievedData.substring(20, 24) + recievedData.substring(28, 32);
  substringRight = recievedData.substring(16, 20) + recievedData.substring(24, 28) + recievedData.substring(32, 36);

  //Move (Base Rotation) Left for left arm
  if (state10 == 'A') {
    baseRotateLeft(6);
  }

  //Move (Base Rotation) Right for left arm
  else if (state10 == 'B') {
    baseRotateRight(6);
  }

  //Move (Base Rotation) Left for right arm
  if (state11 == 'a') {
    baseRotateLeft(7);
  }

  //Move (Base Rotation) Right for right arm
  else if (state11 == 'b') {
    baseRotateRight(7);
  }

  //Write 9 to the ready Pin to show that command is coming from Arduino Uno Master
  analogWrite(readyPin, 9);
  analogWrite(leftArm, substringLeft.toInt());
  analogWrite(rightArm, substringRight.toInt());

}

void cameraSubroutine() {
  camStatus = recievedData.charAt(1);
  cam_pos = recievedData.substring(36, 40).toInt();

  cam_pos = map(cam_pos, 0, 1023, 0, 360);     // converts reading from potentiometer to an output value in degrees of rotation that the servo can understand
  camServo.write(cam_pos);                  // sets the servo position according to the input from the potentiometer

  if (camStatus == '0')digitalWrite(light, LOW); //switch on LED
  else if (camStatus == '1')digitalWrite(light, HIGH); //switch off LED
  delay(10);
}

//Boiler plate function - These functions move the servo motors in a specific direction for a duration.

//servoNum = 6 for left base and 7 for right base
void baseRotateLeft(int servoNum) {

  if (servo_joint_3_parking_pos_i < servo_joint_3_max_pos) {
    HCPCA9685.Servo(servoNum, servo_joint_3_parking_pos_i);
    delay(response_time);

    servo_joint_3_parking_pos_i = servo_joint_3_parking_pos_i + servo_joint_3_pos_increment;

  }

}
void baseRotateRight(int servoNum) {

  if (servo_joint_3_parking_pos_i > servo_joint_3_min_pos) {
    HCPCA9685.Servo(servoNum, servo_joint_3_parking_pos_i);
    delay(response_time);

    servo_joint_3_parking_pos_i = servo_joint_3_parking_pos_i - servo_joint_3_pos_increment;

  }
}
