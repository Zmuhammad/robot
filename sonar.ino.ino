/*
  HC-SR04 in 3-Wire Mode 
  Demonstrates enhancements of HC-SR04 Ultrasonic Range Finder
  Displays results on Serial Monitor


*/

// Dependant upon Adafruit_Sensors Library

// Include NewPing Library
#include "NewPing.h"

// Define Constants

//sensor 1
#define TRIGGER_PIN_1  2  // Trigger and Echo both on pin 2
#define ECHO_PIN_1     2

//sensor 2
#define TRIGGER_PIN_2  3  // Trigger and Echo both on pin 3
#define ECHO_PIN_2     3

//sensor 3
#define TRIGGER_PIN_3  4  // Trigger and Echo both on pin 4
#define ECHO_PIN_3     4

//sensor 4
#define TRIGGER_PIN_4  5  // Trigger and Echo both on pin 5
#define ECHO_PIN_4     5

//sensor 5
#define TRIGGER_PIN_5  6  // Trigger and Echo both on pin 6
#define ECHO_PIN_5     6



//sensor 6
#define TRIGGER_PIN_6  7  // Trigger and Echo both on pin 7
#define ECHO_PIN_6     7

//sensor 7
#define TRIGGER_PIN_7  8  // Trigger and Echo both on pin 8
#define ECHO_PIN_7     8

//sensor 8
#define TRIGGER_PIN_8  9  // Trigger and Echo both on pin 9
#define ECHO_PIN_8     9




#define MAX_DISTANCE 50  //max distance range in cm



NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
NewPing sonar3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE);
NewPing sonar4(TRIGGER_PIN_4, ECHO_PIN_4, MAX_DISTANCE);
NewPing sonar5(TRIGGER_PIN_5, ECHO_PIN_5, MAX_DISTANCE);


NewPing sonar6(TRIGGER_PIN_6, ECHO_PIN_6, MAX_DISTANCE);
NewPing sonar7(TRIGGER_PIN_7, ECHO_PIN_7, MAX_DISTANCE);
NewPing sonar8(TRIGGER_PIN_8, ECHO_PIN_8, MAX_DISTANCE);





void setup() 
{
  Serial.begin (9600);

}

void loop()
{
  char s[9]={'\0'};
  ultrasonic (MAX_DISTANCE,s);
  Serial.println(s);
  delay(100);
}


void ultrasonic( int DISTANCE, char sensors[] )

  {

  unsigned int distance1 = sonar1.ping_cm();
  unsigned int distance2 = sonar2.ping_cm();
  unsigned int distance3 = sonar3.ping_cm();
  unsigned int distance4 = sonar4.ping_cm();
  unsigned int distance5 = sonar5.ping_cm();


  unsigned int distance6 = sonar6.ping_cm();
  unsigned int distance7 = sonar7.ping_cm();
  unsigned int distance8 = sonar8.ping_cm();


   
  
  // Send results to Serial Monitor


    if (distance1 >=  DISTANCE || distance1 <= 2) 
      sensors[0]='O';
    else
      sensors[0]='F';    

    if (distance2 >= DISTANCE || distance2 <= 2) 
      sensors[1]='O';
    else
      sensors[1]='F';

    if (distance3 >= DISTANCE || distance3 <= 2)
      sensors[2]='O';
    else
      sensors[2]='F';

    if (distance4 >= DISTANCE || distance4 <= 2) 
      sensors[3]='O';
    else
      sensors[3]='F';

    if (distance5 >= DISTANCE || distance5 <= 2) 
      sensors[4]='O';
    else
      sensors[4]='F';

    if (distance6 >= DISTANCE || distance6 <= 2) 
      sensors[5]='O';
    else
      sensors[5]='F';

    if (distance7 >= DISTANCE || distance7 <= 2) 
     sensors[6]='O';
    else
      sensors[6]='F';

    if (distance8 >= DISTANCE || distance8 <= 2) 
     sensors[7]='O';
    else
      sensors[7]='F';

  }




  