// pulse width modulation constants
#define PWM_Res      8
#define PWM_Freq  1200

// number of encoder pulses in one full rotation
#define ENC_REV_COUNT 220

// if this amount of time passes without any encoder pulses 
// then the motor has stopped
#define ENC_STOP_TIME_THRESHOLD 10000 // 10000 micro second

#include "main.h"
#include <Arduino.h>
#include <PIDController.h>

// encoderPinA is  /* YELLOW */
// encoderPinB is  /* GREEN */
class MotorController{
  public:
    // control data
    int hbridgeA, hbridgeB, encoderPinA, encoderPinB;
    int pwm_channel1 , pwm_channel2;
    // sensor data
    volatile unsigned long last_elapsed_time = 0;
    volatile unsigned long last_interrupt_time = 0;
    volatile long int pos = 0;  
    volatile int  movement_direction = 0;

    MotorController(int hbridgeA,int hbridgeB,int encoderPinA,int encoderPinB,long pos=0):
        hbridgeA(hbridgeA),hbridgeB(hbridgeB),encoderPinA(encoderPinA),encoderPinB(encoderPinB),
        pos(pos)
        {};

    // returns the position in units of encoder counts 
    long int getPos(){
      long int pos_returned=0; noInterrupts(); pos_returned = pos; interrupts();
      return pos_returned;
    }

    // returns the speed in units of encoder counts per second
    double getSpeed(){ 
      double speed; unsigned long stop_indicator;
      noInterrupts(); 
      speed = 1/(last_elapsed_time/1.0e6) * movement_direction;
      stop_indicator = micros() - last_interrupt_time;
      interrupts();
      if (stop_indicator>ENC_STOP_TIME_THRESHOLD){ speed = 0; }
      return speed;
    }
    // set motor direction and voltage applied to the motor
    void setMotor(int direction, int pwm_val){
      if(direction == 1){
        ledcWrite(pwm_channel1,pwm_val);
        ledcWrite(pwm_channel2,LOW);
      }
      else if (direction == -1){
        ledcWrite(pwm_channel1,LOW);
        ledcWrite(pwm_channel2,pwm_val);
      }
      else {
        ledcWrite(pwm_channel1,LOW);
        ledcWrite(pwm_channel2,LOW);
      }
    }
    // set the pwm channels dedicated for the motor 
    // (needed only in ESP32)
    void set_PWM_channels(int channel1,int channel2){
      pwm_channel1 = channel1; pwm_channel2 = channel2;
    }
    void increment_pos(){
      pos ++; movement_direction = 1;
    }
    void decrement_pos(){
      pos --; movement_direction = -1;
    }
       
};

// Controller Objects
MotorController rightMotor(25,26,34,35);
MotorController leftMotor (32,33,36,39);

PIDController rightMotorPid , leftMotorPid;

// setup
void setup(){
  Serial.begin(115200);Serial.println("Booting MAN...");
  
  /*===========================* rightMotor setup *==================================*/
  pinMode(rightMotor.hbridgeA,OUTPUT); pinMode(rightMotor.hbridgeB,OUTPUT);
  pinMode(rightMotor.encoderPinA,INPUT);pinMode(rightMotor.encoderPinB,INPUT);
  ledcSetup(0, PWM_Freq, PWM_Res); ledcAttachPin(rightMotor.hbridgeA, 0) ;
  ledcSetup(1, PWM_Freq, PWM_Res); ledcAttachPin(rightMotor.hbridgeB, 1) ;
  rightMotor.set_PWM_channels(0,1);
  
  rightMotorPid.begin();
  rightMotorPid.tune(10,0,0);
  rightMotorPid.setpoint(100);
  rightMotorPid.limit(-150,150);
  /*===========================* leftMotor setup *==================================*/
  pinMode(leftMotor.hbridgeA,OUTPUT); pinMode(leftMotor.hbridgeB,OUTPUT);
  pinMode(leftMotor.encoderPinA,INPUT);pinMode(leftMotor.encoderPinB,INPUT);
  ledcSetup(2, PWM_Freq, PWM_Res); ledcAttachPin(leftMotor.hbridgeA, 2) ;
  ledcSetup(3, PWM_Freq, PWM_Res); ledcAttachPin(leftMotor.hbridgeB, 3) ;
  leftMotor.set_PWM_channels(2,3);

  leftMotorPid.begin();
  leftMotorPid.tune(10,0,0);
  leftMotorPid.setpoint(100);
  leftMotorPid.limit(-150,150);
  /*==========================================================================*/

  attachInterrupt(digitalPinToInterrupt(rightMotor.encoderPinA),read_rightEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(leftMotor.encoderPinA),read_leftEncoder,RISING);

  Serial.println("Setup Complete ...");
  vDelay(1000);
}


void loop(){
  
  long int leftMotor_pos= leftMotor.getPos();
  double leftMotor_rpm= leftMotor.getSpeed() *60.0/ENC_REV_COUNT ;
  
  long int rightMotor_pos= rightMotor.getPos();
  double rightMotor_rpm= rightMotor.getSpeed() *60.0/ENC_REV_COUNT;
  
  if (rightMotor_pos>50){
    leftMotor.setMotor(1,150);
  } 
  else if (rightMotor_pos<-50){
    leftMotor.setMotor(-1,150);
  }
  else {
    leftMotor.setMotor(0,0);
  }

 
  // Serial.print(String(leftMotor_pos) + " ");
  // Serial.print(String(rightMotor_pos) + "\n");

  Serial.print(String(leftMotor_rpm) + " ");
  Serial.print(String(rightMotor_rpm) + "\n");

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


