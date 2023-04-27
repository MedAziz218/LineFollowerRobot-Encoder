#ifndef MotorControllerLib
#define MotorControllerLib

// pulse width modulation constants
#define PWM_Res      8
#define PWM_Freq  1200

// if this amount of time passes without any encoder pulses 
// then the motor has stopped
#define ENC_STOP_TIME_THRESHOLD 10000 // 10000 micro second

class MotorController{
  public:
    // control data
    int hbridgeA, hbridgeB, encoderPinA, encoderPinB;
    int pwm_channel1= -1 , pwm_channel2= -1;
    // sensor data
    volatile unsigned long last_elapsed_time = 0;
    volatile unsigned long last_interrupt_time = 0;
    volatile long int pos = 0;  
    volatile int  movement_direction = 0;

    MotorController(int hbridgeA,int hbridgeB,int encoderPinA,int encoderPinB,int pos=0):
        hbridgeA(hbridgeA),hbridgeB(hbridgeB),
        encoderPinA(encoderPinA),encoderPinB(encoderPinB),
        pos(pos) {};
    void setup();
    long int getPos();
    double getSpeed();
    void setMotor(int pwm_val);
    void set_PWM_channels(int channel1,int channel2);
    void increment_pos();
    void decrement_pos();
};
#endif
