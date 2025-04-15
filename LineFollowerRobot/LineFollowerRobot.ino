// Options
#include "Config.h"

int DEBUG_PRINT_SENTINEL=0;

// Speeds
#define MOTOR_SPEED 100 // Reverse sign as needed.

// Pin Numbers
#define IR_SENSOR_RIGHT 11
#define IR_SENSOR_LEFT 12

//Right motor
int enableRightMotor=6;
int rightMotorPin1=7;
int rightMotorPin2=8;

//Left motor
int enableLeftMotor=5;
int leftMotorPin1=9;
int leftMotorPin2=10;

// Remote (RC code is based on the following video: https://youtu.be/q-Clw0m3E18)
int receiver_pins[] = {A0, A1, A2, A3, A4, A5};
int receiver_values[] = {0, 0, 0, 0, 0, 0};
int res_min = 950;
int res_max = 2020;
int rp = 0;
int working_range = 255; // motor driver range
int mode = 0;
//-1 = transmitter not connected or out of range
// 0 = transmitter connected and ready
// 1 = slow speed mode
// 2 = high speed mode



void setup()
{
  if (DEBUG_PRINT) {
    Serial.begin(9600); 
  }
  //The problem with TT gear motors is that, at very low pwm value it does not even rotate.
  //If we increase the PWM value then it rotates faster and our robot is not controlled in that speed and goes out of line.
  //For that we need to increase the frequency of analogWrite.
  //Below line is important to change the frequency of PWM signal on pin D5 and D6
  //Because of this, motor runs in controlled manner (lower speed) at high PWM value.
  //This sets frequency as 7812.5 hz.
  //TCCR0B = TCCR0B & B11111000 | B00000010; // Magic line; we had better results with it disabled.

  // put your setup code here, to run once:
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  rotateMotors(0,0);   
}

void receive() {
  receiver_values[rp] = map(pulseIn (receiver_pins[rp], HIGH), res_min, res_max, -1 * working_range, working_range);
  rp++;
  if (rp == 6){
    rp = 0;
  }
  boolean activevalues = true;
  for (int i = 0; i < 6; i++) {
    if (DEBUG_PRINT) {
      Serial.print("CH");
      Serial.print(i);
      Serial.print(" : ");
      Serial.print(receiver_values[i]);
      Serial.print(",\t");
    }
    if (receiver_values[i] < -500) {
      activevalues = false;
    }
  }
  mode = 0;
  if (!activevalues) {
    mode = -1;
  } else if (receiver_values[4] > -100) {
    mode = 2;
  } else if (receiver_values[5] > -100) {
    mode = 1;
  }
  if (DEBUG_PRINT) {
    Serial.println("");
  }
}

void loop()
{

  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  if (DEBUG_PRINT) {
    if (DEBUG_PRINT_SENTINEL == 0) {
      Serial.print("rightIRSensorValue = ");
      Serial.println(rightIRSensorValue, DEC);
      Serial.print("leftIRSensorValue = ");
      Serial.print(leftIRSensorValue, DEC);
      Serial.println();
    }

    if (DEBUG_PRINT_SENTINEL >= DEBUG_PRINT_INTERVAL) { // This is carefully constructed so that all users of the sentinel can simply check for equality to zero and do not need to consider incrementation/reset and the cycle will begin on the first run of the loop instead of skipping until after the first interval.
      DEBUG_PRINT_SENTINEL = 0;
    } else {
      DEBUG_PRINT_SENTINEL += 1;
    }
  }

  if (DEBUG_MOTORS) {
    rotateMotors(MOTOR_SPEED, MOTOR_SPEED);
    return;
  }

  if (REMOTE_MODE) {
    receive();

    int m1 = 0;
    int m2 = 0;

    int rot = receiver_values[0];

    if (mode == 1) {
      m1 = receiver_values[1] / 2 + (rot)/1.5;
      m2 = receiver_values[1] / 2 - (rot)/1.5;
    } else if (mode == 2) {
      m1 = receiver_values[1] + rot / 1.75;
      m2 = receiver_values[1] - rot / 1.75;
    }
    rotateMotors(m1, m2);
  } else {
    //If none of the sensors detects black line, then go straight
    if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
    {
      rotateMotors(MOTOR_SPEED, MOTOR_SPEED);
    }
    //If right sensor detects black line, then turn right
    else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
    {
      rotateMotors(-MOTOR_SPEED, MOTOR_SPEED); 
    }
    //If left sensor detects black line, then turn left  
    else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
    {
      rotateMotors(MOTOR_SPEED, -MOTOR_SPEED); 
    } 
    //If both the sensors detect black line, then stop 
    else 
    {
      rotateMotors(0, 0);
    }
  }
}


void rotateMotors(int rightMotorSpeed, int leftMotorSpeed)
{
  rotateMotor(rightMotorSpeed, rightMotorPin1, rightMotorPin2, enableRightMotor);
  rotateMotor(leftMotorSpeed, leftMotorPin1, leftMotorPin2, enableLeftMotor);
}

void rotateMotor(int speed, int pin1, int pin2, int enable)
{
  if (DEBUG_PRINT && DEBUG_PRINT_SENTINEL == 0) {
    Serial.print("Turning motor with pin1 = ");
    Serial.print(pin1, DEC);
    Serial.print(" , pin2 = ");
    Serial.print(pin2, DEC);
    Serial.print(" , and enable pin = ");
    Serial.print(enable, DEC);
    //Serial.print(" (" + (enable == enableLeftMotor) ? "left" : (enable == enableRightMotor) ? "right" : "unknown" + ")"); // Concatenation/Type-Strictness issues, my beloved.
    Serial.print(" with speed = ");
    Serial.print(speed, DEC);
    Serial.println();
  }

  if (speed < 0)
  {
    digitalWrite(pin1,LOW);
    digitalWrite(pin2,HIGH);    
  }
  else if (speed > 0)
  {
    digitalWrite(pin1,HIGH);
    digitalWrite(pin2,LOW);      
  }
  else
  {
    digitalWrite(pin1,LOW);
    digitalWrite(pin2,LOW);      
  }

  analogWrite(enable, abs(speed));
}

