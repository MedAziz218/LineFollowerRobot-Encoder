#include "MotorController.h"
#include <Arduino.h>

// make sure to use set_PWM_channels function before this
void MotorController::setup(){
    pinMode(hbridgeA,OUTPUT); pinMode(hbridgeB,OUTPUT);
    pinMode(encoderPinA,INPUT);pinMode(encoderPinB,INPUT);
    ledcSetup(pwm_channel1, PWM_Freq, PWM_Res); ledcAttachPin(hbridgeA, pwm_channel1) ;
    ledcSetup(pwm_channel2, PWM_Freq, PWM_Res); ledcAttachPin(hbridgeB, pwm_channel2) ;
}
// returns the position in units of encoder counts 
long int MotorController::getPos(){
    long int pos_returned=0; noInterrupts(); pos_returned = pos; interrupts();
    return pos_returned;
}

// returns the speed in units of encoder counts per second
double MotorController::getSpeed(){ 
    double speed; unsigned long stop_indicator;
    noInterrupts(); 
    speed = 1/(last_elapsed_time/1.0e6) * movement_direction;
    stop_indicator = micros() - last_interrupt_time;
    interrupts();
    if (stop_indicator>ENC_STOP_TIME_THRESHOLD){ speed = 0; }
    return speed;
}
// set motor direction and voltage applied to the motor
void MotorController::setMotor(int pwm_val){
    int direction = (pwm_val >=0) ? 1:-1;
    pwm_val = abs(pwm_val);
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
void MotorController::set_PWM_channels(int channel1,int channel2){
    pwm_channel1 = channel1; pwm_channel2 = channel2;
}
void MotorController::increment_pos(){
    pos ++; movement_direction = 1;
}
void MotorController::decrement_pos(){
    pos --; movement_direction = -1;
}