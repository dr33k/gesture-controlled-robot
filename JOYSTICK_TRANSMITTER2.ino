/*Arduino JOYSTICK CONTROLLED CAR (TRANSMITTER)
          
YOU HAVE TO INSTALL THE RF24 LIBRARY BEFORE UPLOADING THE CODE
   https://github.com/tmrh20/RF24/      
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7,8); // CE, CSN
const byte address[6] = "00001";
String sentData;

#define cameraPin A11
#define camStatPin 26
#define rGripPin A15
#define lGripPin A5
#define wheelXPin A6
#define wheelYPin A7
#define lWSXPin A13
#define lWSYPin A14
#define rWSXPin A8
#define rWSYPin A9
#define lElbXPin A12
#define rElbXPin A10

String camera, camStat, rGrip,lGrip, wheelX,wheelY,lWSX, lWSY, rWSX, rWSY, lElbX, lElbY, rElbX, rElbY; 
 
void setup() {   
  
  digitalWrite(camStatPin, HIGH);
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

}
void loop() {
  //Clear the string at the beginning of each iteration
  sentData = String();
  
  camera = analogRead(cameraPin);
  rGrip = analogRead(rGripPin);
  lGrip = analogRead(lGripPin);
  wheelX = analogRead(wheelXPin);
  wheelY = analogRead(wheelYPin);
  lWSX = analogRead(lWSXPin);
  lWSY = analogRead(lWSYPin);
  rWSX = analogRead(rWSXPin);
  rWSY = analogRead(rWSYPin);
  lElbX = analogRead(lElbXPin);
  rElbX = analogRead(rElbXPin);
  

  //Build 40 character string index by index
  sentData += "0";   //Index 0

  sentData += String(digitalRead(camStatPin));   //Index 1
  
  // wheelX value     //Indices 2 3 4 5
  sentData += leadingZeros(wheelX, 4);
  // wheelY value     //Indices 6 7 8 9
  sentData += leadingZeros(wheelY, 4);

  //Left Waist/Base Rotation    Index 10
  if(lWSY.toInt()<450)  sentData += "B";//right
  else if (lWSY.toInt()>600)  sentData += "A"; //left
  else sentData += "*";

  //Right Waist/Base Rotation       Index 11
  if(rWSY.toInt()<450)  sentData += "b";//right
  else if (rWSY.toInt()>600)  sentData += "a";//left
  else sentData += "*";

  //Left Shoulder Up/Down     Index 12 13 14 15
  sentData += leadingZeros(lWSX,4);
  
  //Right Shoulder Up/Down        Index 16 17 18 19
  sentData += leadingZeros(rWSX,4);

  //Left Elbow Up/Down          Index 20 21 22 23
  sentData += leadingZeros(lElbX,4);

  //Right Elbow Up/Down         Index 24 25 26 27
   sentData += leadingZeros(rElbX,4);

    //Left Gripper Close/Open          Index 28 29 30 31
    sentData += leadingZeros(lGrip,4);

  //Right Gripper Close/Open         Index 32 33 34 35
    sentData += leadingZeros(rGrip,4);

  // camera value       Indices 36 37 38 39
  sentData += leadingZeros(camera, 4);

  delay(10);
  Serial.print(sentData); // Send the array data (X value) to the other NRF24L01 modile
  Serial.print("\n");
  const char sentDataC[41] = "";
  sentData.toCharArray(sentDataC,sizeof(sentDataC));
  radio.write(&sentDataC, sizeof(sentDataC));
  delay(10);
}

String leadingZeros(String x, int lim){
  while(x.length()<lim)
    {x="0"+x;}
     return x;
  }

 int anRead(int pin){
  int sum = 0;
  for(int i = 0;i<5; i++){ sum += analogRead(pin);}
  return int(sum/5);
  }
