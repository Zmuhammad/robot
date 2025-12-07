// Include NewPing Library #### Sonars communication #############
#include "NewPing.h"
#include <arduino-timer.h>
#include <SoftwareSerial.h>
#include <Wire.h>
// QMC5883L Compass Library
#include <QMC5883LCompass.h>
SoftwareSerial BTSerial(10, 11);  // اتصال ماژول بلوتوث به پین‌های 10 و 11 آردوینو
QMC5883LCompass compass;
//####################### Robot Action Parameters #########################################
#define Def_run_speed 55
#define Def_rotate_speed 65
#define Max_rotate_speed 100
#define Max_Velocity 30
float run_speed = Def_run_speed;
float stop_speed = Def_run_speed * 2;
float rotation_speed = Def_rotate_speed;
int movement_status = 0;
int dist[8] = { 0 };
int Angle = 0;
int Old_Angle = 0;
int diff_angle = 0;
int Obs = 40;
float Robot_dist = 0, Old_Robot_dist = 0, Velocity = 0;
volatile long counter = 0;
//^^^^^^^^^^^^^^^^^^^ Remote Control ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
int VT, D0, D1, D2, D3, ESTOP = 1;
int FD0 = 0;
int FD1 = 0;
int FD2 = 0;
int FD3 = 0;
#define VTpin A7
#define D0pin A0  //Control Push A
#define D1pin A1  //Control Push C
#define D2pin A2  //Control Push B
#define D3pin A3  //Control Push D

//$$$$$$$$$$$$$$$$$$$ PWM Throttle control $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
#define left_front_speed 4
#define right_front_speed 5
#define left_back_speed 6
#define right_back_speed 7

//################### Direction & Others relay Control ######################
#define left_front_direction 23
#define right_front_direction 24
#define left_back_direction 26
#define right_back_direction 25
#define Free_relay6 22
#define Free_relay7 27
#define Free_relay8 29
#define Alarm_relay 28

//################## Sonars Signals ################################
//sensor 1
#define sonar_RRF_TRIGGER 45
#define sonar_RRF_ECHO 45
//sensor 2
#define sonar_RF_TRIGGER 44
#define sonar_RF_ECHO 44
//sensor 3
#define sonar_CF_TRIGGER 43
#define sonar_CF_ECHO 43
//sensor 4
#define sonar_LF_TRIGGER 42
#define sonar_LF_ECHO 42
//sensor 5
#define sonar_LLF_TRIGGER 41
#define sonar_LLF_ECHO 41
//sensor 6
#define sonar_LB_TRIGGER 38
#define sonar_LB_ECHO 38
//sensor 7
#define sonar_CB_TRIGGER 39
#define sonar_CB_ECHO 39
//sensor 8
#define sonar_RB_TRIGGER 40
#define sonar_RB_ECHO 40
//  INITIAL SONARS
#define SONAR_MAX_DISTANCE 1500

NewPing sonar_RRF(sonar_RRF_TRIGGER, sonar_RRF_ECHO, SONAR_MAX_DISTANCE);
NewPing sonar_RF(sonar_RF_TRIGGER, sonar_RF_ECHO, SONAR_MAX_DISTANCE);
NewPing sonar_CF(sonar_CF_TRIGGER, sonar_CF_ECHO, SONAR_MAX_DISTANCE);
NewPing sonar_LF(sonar_LF_TRIGGER, sonar_LF_ECHO, SONAR_MAX_DISTANCE);
NewPing sonar_LLF(sonar_LLF_TRIGGER, sonar_LLF_ECHO, SONAR_MAX_DISTANCE);

NewPing sonar_LB(sonar_LB_TRIGGER, sonar_LB_ECHO, SONAR_MAX_DISTANCE);
NewPing sonar_CB(sonar_CB_TRIGGER, sonar_CB_ECHO, SONAR_MAX_DISTANCE);
NewPing sonar_RB(sonar_RB_TRIGGER, sonar_RB_ECHO, SONAR_MAX_DISTANCE);
////////////////////////////////////////////////////////////////////
auto Sonars = timer_create_default();
bool ultrasonic() {
  dist[0] = sonar_RRF.ping_cm() + 1;
  dist[1] = sonar_RF.ping_cm() + 1;
  dist[2] = sonar_CF.ping_cm() + 1;
  dist[3] = sonar_LF.ping_cm() + 1;
  dist[4] = sonar_LLF.ping_cm() + 1;
  dist[5] = sonar_LB.ping_cm() + 1;
  dist[6] = sonar_CB.ping_cm() + 1;
  dist[7] = sonar_RB.ping_cm() + 1;
  return true;  // keep timer active? true
}
//&&&&&&&&&&&&&&&& Encodel &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void A() {
  if (digitalRead(3) == LOW) {
    counter++;
  } else {
    counter--;
  }
}

void B() {
  if (digitalRead(2) == LOW) {
    counter--;
  } else {
    counter++;
  }
}
//////////////////////////////////////////////////////////////////
void stop() {
  if (movement_status == 1)  //forward stop
  {
    digitalWrite(left_front_direction, LOW);
    digitalWrite(right_front_direction, LOW);
    digitalWrite(left_back_direction, LOW);
    digitalWrite(right_back_direction, LOW);
    delay(100);
    analogWrite(left_front_speed, 0);
    analogWrite(right_front_speed, 0);
    analogWrite(left_back_speed, run_speed);
    analogWrite(right_back_speed, run_speed);
    delay(500);
    analogWrite(left_back_speed, 0);
    analogWrite(right_back_speed, 0);
  }

  if (movement_status == 2)  // backward stop
  {
    digitalWrite(left_front_direction, LOW);
    digitalWrite(right_front_direction, LOW);
    digitalWrite(left_back_direction, LOW);
    digitalWrite(right_back_direction, LOW);
    delay(100);
    analogWrite(left_front_speed, run_speed);
    analogWrite(right_front_speed, run_speed);
    analogWrite(left_back_speed, 0);
    analogWrite(right_back_speed, 0);
    delay(500);
    analogWrite(left_front_speed, 0);
    analogWrite(right_front_speed, 0);
  }
  if (movement_status == 3 or movement_status == 4)  // rotation stop
  {
    digitalWrite(left_front_direction, LOW);
    digitalWrite(right_front_direction, LOW);
    digitalWrite(left_back_direction, LOW);
    digitalWrite(right_back_direction, LOW);
    delay(100);
    //analogWrite(left_front_speed, stop_speed);
    //analogWrite(right_front_speed, stop_speed);
    //analogWrite(left_back_speed, stop_speed);
    //analogWrite(right_back_speed, stop_speed);
    //delay(500);
    analogWrite(left_front_speed, 0);
    analogWrite(right_front_speed, 0);
    analogWrite(left_back_speed, 0);
    analogWrite(right_back_speed, 0);
  }
  movement_status = 0;
  diff_angle = 0;
  run_speed = Def_run_speed;
  rotation_speed = Def_rotate_speed;
  //counter=0;
}
///////////////////////////////////////////////////////////////////
void move_forward(int Speed) {
  //movement_status = 1;
  //stop();
  digitalWrite(left_front_direction, LOW);
  digitalWrite(right_front_direction, LOW);
  //digitalWrite(left_back_direction,  LOW);
  //digitalWrite(right_back_direction, HIGH);

  delay(100);
  analogWrite(left_front_speed, Speed);
  analogWrite(right_front_speed, Speed);
  //analogWrite(left_back_speed, Speed);
  //analogWrite(right_back_speed, Speed);
}
////////////////////////////////////////////////////////////////////
void move_backward(int Speed) {
  //movement_status = 2;
  //stop();
  //digitalWrite(left_front_direction, HIGH);
  //digitalWrite(right_front_direction, HIGH);
  digitalWrite(left_back_direction, LOW);
  digitalWrite(right_back_direction, LOW);
  delay(100);
  //analogWrite(left_front_speed, Speed);
  //analogWrite(right_front_speed, Speed);
  analogWrite(left_back_speed, Speed);
  analogWrite(right_back_speed, Speed);
}
//////////////////////////////////////////////////////////////////
void rotation_right(int Speed) {
  //movement_status = 3;
  //stop();
  digitalWrite(left_front_direction, LOW);
  digitalWrite(right_front_direction, HIGH);
  digitalWrite(left_back_direction, HIGH);
  digitalWrite(right_back_direction, LOW);
  delay(100);
  analogWrite(left_front_speed, rotation_speed);
  analogWrite(right_front_speed, rotation_speed);
  analogWrite(left_back_speed, rotation_speed);
  analogWrite(right_back_speed, rotation_speed );
}
//////////////////////////////////////////////////////////////////
void rotation_left(int Speed) {
  //movement_status = 4;
  //stop();
  digitalWrite(left_front_direction, HIGH);
  digitalWrite(right_front_direction, LOW);
  digitalWrite(left_back_direction, LOW);
  digitalWrite(right_back_direction, HIGH);
  delay(100);
  analogWrite(left_front_speed, rotation_speed);
  analogWrite(right_front_speed, rotation_speed);
  analogWrite(left_back_speed, rotation_speed );
  analogWrite(right_back_speed, rotation_speed);
}
//***************************************************************
auto MONITOR_Data = timer_create_default();  // create a timer with default settings
bool Monitor_Data(void *) {
  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // toggle the LED
  //RRF     RF    CF    LF  LLF   RB    CB     LB
  String str;
  if (abs(Velocity) > 0) {
    str = "RRF:" + String(dist[0]) + " RF:" + String(dist[1]) + " CF:" + String(dist[2]);
    str += " LF:" + String(dist[3]) + " LLF:" + String(dist[4]) + " RB:" + String(dist[5]) + " CB:" + String(dist[6]) + " LB:" + String(dist[7]);
    str += " Angle:" + String(Angle);
    switch (movement_status) {
      case 0:
        str += " NOP";
        break;
      case 1:
        str += " Move Forw";
        break;
      case 2:
        str += " Move Back";
        break;
      case 3:
        str += " rot R";
        break;
      case 4:
        str += " rot L";
        break;
    }
    str = str + " D_Ang:" + String(diff_angle) + " speed=" + String(run_speed) + " rot_s= " + String(rotation_speed);
    str = str + "Robot_dist= " + String(Robot_dist) + " Velocity=" + String(Velocity);
    BTSerial.println(str);
  }
  return true;  // keep timer active? true
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
int Sub_abs_Angle(int a, int b) {
  int r = 0;
  r = b - a;
  r = (r + 180) % 360 - 180;
  if(abs(r)>180)
    r=360-abs(r);
  return abs(r);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
auto COMPASS = timer_create_default();
bool Compass(void *) {
  int a;

  // Read compass values
  compass.read();
  Angle = compass.getAzimuth();
  return true;  // keep timer active? true
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
auto ENCODER = timer_create_default();
bool Encoder(void *) {
  Robot_dist = 2 * 11 * (3.1415 / 180.0) * counter / 10;
  Velocity = (Robot_dist - Old_Robot_dist) / 0.250;
  Old_Robot_dist = Robot_dist;
  return true;  // keep timer active? true
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
auto Move_Action = timer_create_default();
bool Move_Act(void *) {
  switch (movement_status) {
    case 1:
      move_forward(run_speed);
      break;
    case 2:
      move_backward(run_speed);
      break;
    case 3:
      rotation_right(rotation_speed);
      break;
    case 4:
      rotation_left(rotation_speed);
      break;
    case 0:
      stop();
      break;
  }
  Serial.println(movement_status);
  return true;  // keep timer active? true
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void ReadDataIsr() {
  //delay(300);
  D0 = 0;
  D1 = 0;
  D2 = 0;
  D3 = 0;
  //LD_Open = LD_Close = RD_Close = RD_Open = 1;
  VT = digitalRead(VTpin);
  D0 = digitalRead(D0pin);
  D1 = digitalRead(D1pin);
  D2 = digitalRead(D2pin);
  D3 = digitalRead(D3pin);
  if (VT == 1)
    Serial.println((String)D0 + ", " + (String)D1 + ", " + (String)D2 + ", " + (String)D3);
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(left_front_direction, OUTPUT);
  digitalWrite(left_front_direction, LOW);
  pinMode(right_front_direction, OUTPUT);
  digitalWrite(right_front_direction, LOW);
  pinMode(left_back_direction, OUTPUT);
  digitalWrite(left_back_direction, LOW);
  pinMode(right_back_direction, OUTPUT);
  digitalWrite(right_back_direction, LOW);
  pinMode(Alarm_relay, OUTPUT);
  digitalWrite(Alarm_relay, LOW);
  pinMode(Free_relay6, OUTPUT);
  digitalWrite(Free_relay6, LOW);
  pinMode(Free_relay7, OUTPUT);
  digitalWrite(Free_relay7, LOW);
  pinMode(Free_relay8, OUTPUT);
  digitalWrite(Free_relay8, LOW);

  pinMode(left_front_speed, OUTPUT);
  analogWrite(left_front_speed, 0);
  pinMode(left_back_speed, OUTPUT);
  analogWrite(left_back_speed, 0);
  pinMode(right_front_speed, OUTPUT);
  analogWrite(right_front_speed, 0);
  pinMode(right_back_speed, OUTPUT);
  analogWrite(right_back_speed, 0);
  //~~~~~~~~~~~~~~~~~~~Encoder  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~````
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), A, RISING);
  attachInterrupt(digitalPinToInterrupt(3), B, RISING);
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  attachInterrupt(digitalPinToInterrupt(A0), ReadDataIsr, RISING);
  pinMode(VTpin, INPUT);
  pinMode(D0pin, INPUT);
  pinMode(D1pin, INPUT);
  pinMode(D2pin, INPUT);
  pinMode(D3pin, INPUT);
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //stop();
  BTSerial.begin(9600);  // سرعت بیت بر ثانیه را با سرعت ماژول بلوتوث هماهنگ کن
  // call the toggle_led function every 1000 millis (1 second)
  MONITOR_Data.every(500, Monitor_Data);  // sent data by blutooth
  COMPASS.every(100, Compass);            // read angle by compass
  ENCODER.every(250, Encoder);
  Sonars.every(300, ultrasonic);     // read Sonars
  Move_Action.every(200, Move_Act);  // Apply run Action Param to wheels

  Wire.begin();
  // Initialize the Compass.
  compass.init();
  Serial.begin(9600);
}

//////////////////////////////////////////////
void Obstacles() {
  stop();
  Serial.println("Obstacels   OOOOOOOOO");
  digitalWrite(Alarm_relay, HIGH);
  delay(500);
  digitalWrite(Alarm_relay, LOW);
}
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
void Stop_Movement() {
  if (movement_status == 1) {
    if ((dist[0] > 5 and dist[0] < Obs) or (dist[1] > 5 and dist[1] < Obs) or (dist[2] > 5 and dist[2] < Obs) or (dist[3] > 5 and dist[3] < Obs) or (dist[4] > 5 and dist[4] < Obs))
      Obstacles();
  } else if (movement_status == 2) {
    if ((dist[5] > 5 and dist[5] < Obs) or (dist[6] > 5 and dist[6] < Obs) or (dist[7] > 5 and dist[7] < Obs))
      Obstacles();
  } else if (movement_status == 3 or movement_status == 4) {
    if ((dist[0] > 5 and dist[0] < Obs) or (dist[1] > 5 and dist[1] < Obs) or (dist[2] > 5 and dist[2] < Obs) or (dist[3] > 5 and dist[3] < Obs) or (dist[4] > 5 and dist[4] < Obs))
      // Angle=Angle;
      //or(dist[5]>0 and dist[5]<Obs)or(dist[6]>0 and dist[6]<Obs)or(dist[7]>0 and dist[7]<Obs))
      Obstacles();
  }
}
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

int Auto = 0;
int flag=0;
/////////////////////////////////////////////
void loop() {
  delay(200);
  MONITOR_Data.tick();  // monitoring
  COMPASS.tick();       // compass (Angle)
  ENCODER.tick();       // Robot dist
  Sonars.tick();        // Sonars
  Move_Action.tick();   // Apply Run Parameters
  ReadDataIsr();
  
  /*
  if(flag==0)
    {
      Old_Angle=Angle;
      flag=1;
    }
  if (Angle != Old_Angle) 
        diff_angle = Sub_abs_Angle(Angle, Old_Angle);
  */
  if (D3 == 1 and D0 == 1)  // A and B
  {
    stop();
    Auto = 1;
    Serial.println("Autonomus RUN");
  }
  if (D1 == 1 and D2 == 1)  // C and D
  {
    stop();
    Auto = 0;
    Serial.println("MANUAL RUN");
    //delay(200);
  }

  if (Auto == 0)  // MANUAL RUN
  {
    Stop_Movement();
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    if (VT == 1) {
      if (D0 == 0 and D1 == 0 and D2 == 0 and D3 == 1 and FD3 == 0)  // and FD1 == 0 and FD2 == 0)     //A
      {
        movement_status = 1;
        FD3 = 1;
        Serial.println("FORWARD");
      } else if (D3 == 1 and FD3 == 1) {
        stop();
        FD3 = 0;
        Serial.println("FORWARD OFF");
      }
      if (D0 == 1 and D1 == 0 and D2 == 0 and D3 == 0 and FD0 == 0)  //B
      {
        movement_status = 2;
        FD0 = 1;
        Serial.println("BACKWARD");
      } else if (D0 == 1 and FD0 == 1) {
        stop();
        FD0 = 0;
        Serial.println("BACKWARD OFF");
      }

      if (D0 == 0 and D1 == 1 and D2 == 0 and D3 == 0 and FD1 == 0)  //C
      {
        Old_Angle = Angle;
        movement_status = 3;
        FD1 = 1;
        Serial.println("rotation LEFT");
      } else if (D1 == 1 and FD1 == 1) {
        stop();
        FD1 = 0;
        Serial.println("rotation LEFT OFF");
      }
      if (D0 == 0 and D1 == 0 and D2 == 1 and D3 == 0 and FD2 == 0)  //D
      {
        movement_status = 4;
        Old_Angle = Angle;
        FD2 = 1;
        Serial.println("rotation RIGHT");
      } else if (D2 == 1 and FD2 == 1) {
        stop();
        FD2 = 0;
        Serial.println("rotation RIGHT OFF");
      }
    }
    if (movement_status == 3 or movement_status == 4) {
      if (Angle != Old_Angle) {
        diff_angle = Sub_abs_Angle(Angle, Old_Angle);
        if (rotation_speed < 120 and diff_angle < 15)
          rotation_speed += 4;
        if(diff_angle>15 and diff_angle<90)
        {
          if(rotation_speed>45 and rotation_speed>120)
            rotation_speed-=30;
          else
            rotation_speed-=20;
        }
      }
      //Serial.print(" Old_Angle: ");   Serial.print(Old_Angle); Serial.print(" Diffff="); Serial.println(diff_angle);
      if (diff_angle > 65) {
        diff_angle = 0;
        rotation_speed = Def_rotate_speed;
        FD1 = FD2 = 0;
        stop();
      }
    }
    if (movement_status == 1 or movement_status == 2) {
      if (abs(Velocity) < Max_Velocity)
        run_speed += 3;
      else
        run_speed -= 3;
    }
  }

  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
  else if (Auto == 1) {
    if ((dist[0] > 0 and dist[0] < Obs) or (dist[1] > 0 and dist[1] < Obs) or (dist[2] > 0 and dist[2] < Obs)) {
      //stop();
      digitalWrite(Alarm_relay, LOW);
      rotation_left(rotation_speed);
      Serial.println("Auto rotation LEFT");

    } else if ((dist[3] > 0 and dist[3] < Obs) or (dist[4] > 0 and dist[4] < Obs)) {
      //stop();
      digitalWrite(Alarm_relay, LOW);
      rotation_right(rotation_speed);
      Serial.println("Auto rotation RIGHT");
    } else {
      //stop();
      digitalWrite(Alarm_relay, HIGH);
      move_forward(run_speed);
      Serial.println("Auto FORWARD ");
    }
  }

  //delay(300);
}

