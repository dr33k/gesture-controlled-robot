#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7,8); // CE, CSN
const byte address[6] = "00001";
String sentData;

Adafruit_MPU6050 mpu;

//Create thumb Sensors
int index, thumb = 0; //Index thumb

int index_Data = A0;
int thumb_Data = A1;

/*
  2 Flex sensors used. Thumb, and Index
*/
int thumb_high = 0;
int thumb_low = 0;
int index_high = 0;
int index_low = 0;
int x,y,z = 0;


//Stop Caliberating the Flex Sensor when complete
bool bool_caliberate = false;

//How often to send values to the Robotic Arm
int response_time = 10;

void setup() {
  
 Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
 
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-16G
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);

}
void loop() {
  sentData = "1";
/* Get the accelerometer values */
  smoothen();
  /* Get the flex values */
  index = analogRead(index_Data);
  thumb = analogRead(thumb_Data);

  //Calibrate to find upper and lower limit of the Flex Sensor
  if (bool_caliberate == false ) {
    
    thumb_high = (thumb * 1.05);
    thumb_low = (thumb * 0.98);

    index_high = (index * 1.03);
    index_low = (index * 0.97);

    bool_caliberate = true;
  }
  
//LOGIC
 // index  (Gripper)
  if (index >= index_high) {
    sentData+="I";
  }

  else if (index <= index_low) {
    sentData+="i";
  }
  else sentData+="*";
  

  // thumb  (DC mOTORS)
  if (thumb >= thumb_high) {
    sentData+="T";
  }

  else if (thumb <= thumb_low) {
    sentData+="t";
  }
  else sentData+="*";
  
//Waist left
if ( y <= -1.6) {
    sentData+="A";
  }

  //Waist Right
  else if (  y >= 1.6) {
    sentData+="B";
  }
  else sentData+="*";


//Elbow Up
if ( x >= 1.6) {
    sentData+="E";
  }

  //Elbow down
  else if (  x <= -1.6) {
    sentData+="F";
  }
  else sentData+="*";

    delay(10);
//END OF LOGIC

  Serial.println(sentData); // Send the array data (X value) to the other NRF24L01 modile
  const char sentDataC[23] = "";
  sentData.toCharArray(sentDataC,sizeof(sentDataC));
  radio.write(&sentDataC, sizeof(sentDataC));
  delay(10);
  
}

void smoothen(){
  sensors_event_t a, g, temp;
  for(int i = 0; i< 6;i++){
    mpu.getEvent(&a, &g, &temp);
  delay(response_time);

  x += a.acceleration.x;
  y += a.acceleration.y;
  z += a.acceleration.z;
  }
  x/=6;
  y/=6;
  z/=6;
  }
