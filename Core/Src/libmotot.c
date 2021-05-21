#include "libmotor.h"

#define PWMA TIM1->CCR1    //PA8
#define PWMB TIM1->CCR4    //PA11
#define PWM_Amplitude 6900 //max 7200 limit 6900
#define Balance_Kp 450
#define Balance_Kd 1
#define Velocity_Kp 80
#define Velocity_Ki 1
#define ZHONGZHI 0
#define Gyro_Banlance 77
#define MAX_Encoder_Integral 8000 //10000

uint16_t Target_Velocity, Turn_Amplitude;

uint8_t Flag_Qian, Flag_Hou, Flag_Left, Flag_Right;

void set_Pwm(int32_t motor1, int32_t motor2)
{
    if (motor1 > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   //AIN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); //AIN2
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); //AIN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   //AIN2
    }
    motor1 = (motor1 < -PWM_Amplitude) ? -PWM_Amplitude : motor1;
    motor1 = (motor1 > PWM_Amplitude) ? PWM_Amplitude : motor1;
    PWMA = abs(motor1);
    if (motor2 > 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   //BIN2
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); //BIN1
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //BIN2
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   //BIN1
    }
    motor2 = (motor2 < -PWM_Amplitude) ? -PWM_Amplitude : motor2;
    motor2 = (motor2 > PWM_Amplitude) ? PWM_Amplitude : motor2;
    PWMB = abs(motor2);
}

// 平衡PID 速度PID
int pid_Balance_Velocity(float angle, float gyro, int encoder_left, int encoder_right)
{
    int balance;
    static float Velocity, Encoder, Movement;
    float Encoder_Integral;
    Target_Velocity = 110;

    // 平衡
    balance = Balance_Kp * (angle - ZHONGZHI) + Balance_Kd * (gyro + Gyro_Banlance); //===计算平衡控制的电机PWM  PD控制

    if (1 == Flag_Qian)
        Movement = -Target_Velocity; //===前进标志位置1
    else if (1 == Flag_Hou)
        Movement = Target_Velocity;
    else
        Movement = 0;

    Encoder = 0.8 * Encoder + 0.2 * (encoder_left + encoder_right);
    Encoder_Integral = Encoder_Integral + Encoder - Movement;

    if (Encoder_Integral > MAX_Encoder_Integral)
        Encoder_Integral = MAX_Encoder_Integral;
    if (Encoder_Integral < -MAX_Encoder_Integral)
        Encoder_Integral = -MAX_Encoder_Integral;

    Velocity = Encoder * Velocity_Kp + Encoder_Integral * Velocity_Ki; //===速度控制

    return balance + Velocity;
}
