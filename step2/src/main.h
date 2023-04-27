#pragma once
#include <Arduino.h>
template <int j>
void readEncoder();
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
void vDelay(int ms){ vTaskDelay(ms/portTICK_PERIOD_MS);}
void read_rightEncoder();
void read_leftEncoder();


// void sendToPC(int* data)
// {
//   byte* byteData = (byte*)(data);
//   Serial.write(byteData, 2);
// }

// void sendToPC(double* data)
// {
//   byte* byteData = (byte*)(data);
//   Serial.write(byteData, 4);
// }
