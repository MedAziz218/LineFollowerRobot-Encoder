// number of encoder pulses in one full rotation
#define ENC_REV_COUNT 220
#define WHEEL_RADIUS 4.5
#include "main.h"
#include "Arduino.h"
#include "PIDController.h"
#include "MotorController.h"
#include "SoftTimer.h"
// encoderPinA is  /* YELLOW */
// encoderPinB is  /* GREEN */

// Controller Objects
MotorController rightMotor(25,26,35,34);
MotorController leftMotor (32,33,36,39);

PIDController rightMotorSpeedPID , leftMotorSpeedPID;

// setup
void setup(){
  Serial.begin(115200);Serial.println("Booting MAN...");
  
  /*===========================* rightMotor setup *==================================*/
  
  rightMotor.set_PWM_channels(0,1);
  rightMotor.setup();

  rightMotorSpeedPID.begin();
  rightMotorSpeedPID.tune(10,3,0);
  rightMotorSpeedPID.setpoint(110);
  rightMotorSpeedPID.limit(-150,150);
  /*===========================* leftMotor setup *==================================*/
  // pinMode(leftMotor.hbridgeA,OUTPUT); pinMode(leftMotor.hbridgeB,OUTPUT);
  // pinMode(leftMotor.encoderPinA,INPUT);pinMode(leftMotor.encoderPinB,INPUT);
  // ledcSetup(2, PWM_Freq, PWM_Res); ledcAttachPin(leftMotor.hbridgeA, 2) ;
  // ledcSetup(3, PWM_Freq, PWM_Res); ledcAttachPin(leftMotor.hbridgeB, 3) ;
  leftMotor.set_PWM_channels(2,3);
  leftMotor.setup();

  leftMotorSpeedPID.begin();
  leftMotorSpeedPID.tune(10,3.2,0);
  leftMotorSpeedPID.setpoint(110);
  leftMotorSpeedPID.limit(-150,150);
  /*==========================================================================*/

  attachInterrupt(digitalPinToInterrupt(rightMotor.encoderPinA),read_rightEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(leftMotor.encoderPinA),read_leftEncoder,RISING);

  Serial.println("Setup Complete ...");
  vDelay(1000);
}


SoftTimer timer1;

void loop(){
  timer1.update();

  int leftMotor_pos= leftMotor.getPos() *360/ENC_REV_COUNT; // angle in degrees
  double leftMotor_rpm= leftMotor.getSpeed() *60.0/ENC_REV_COUNT ; // speed in rpm
  
  int rightMotor_pos= rightMotor.getPos()*360/ENC_REV_COUNT; // angle in degrees
  double rightMotor_rpm= rightMotor.getSpeed() *60.0/ENC_REV_COUNT; // speed in rpm

  double target = degrees(20 * radians(90) / WHEEL_RADIUS );

  if (timer1.is_stopped() && leftMotor_pos <target ) {
    int pwr;

    pwr = (int) leftMotorSpeedPID.compute(leftMotor_rpm);
    leftMotor.setMotor(pwr);
    Serial.print(String(110)+" "+ String(leftMotor_rpm)+ " "+ String(pwr));

    // Serial.print(" --- ");

    pwr = (int) rightMotorSpeedPID.compute(rightMotor_rpm);
    // rightMotor.setMotor(pwr);
    // Serial.print(String(110)+" "+ String(rightMotor_rpm) + " "+ String(pwr));
    
    Serial.print("\n");
    timer1.start(1);
  }
  if (leftMotor_pos >= target ){
    leftMotor.setMotor(0);
  Serial.print(String(leftMotor_pos)+" "+String(target) + "\n");
  }
  // Serial.print(String(leftMotor_pos) + " ");

  // Serial.print(String(leftMotor_rpm) + " ");
  // Serial.print(String(rightMotor_rpm) + "\n");

}

void read_rightEncoder(){
  int b = digitalRead(rightMotor.encoderPinB);
  if (b>0) {rightMotor.increment_pos();}
  else {rightMotor.decrement_pos();}
  rightMotor.last_elapsed_time = micros() - rightMotor.last_interrupt_time;
  rightMotor.last_interrupt_time = micros();
}

void read_leftEncoder(){
  int b = digitalRead(leftMotor.encoderPinB);
  if (b>0) {leftMotor.increment_pos();}
  else {leftMotor.decrement_pos();}
  leftMotor.last_elapsed_time = micros() - leftMotor.last_interrupt_time;
  leftMotor.last_interrupt_time = micros();
}


