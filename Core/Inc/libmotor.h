#pragma once

#include "main.h"

void set_Pwm(int32_t motor1, int32_t motor2);
int pid_Balance_Velocity(float angle, float gyro, int encoder_left, int encoder_right);
