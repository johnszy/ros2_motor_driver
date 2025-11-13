#include <stdio.h>
#include "motor_ctrl.h"
#include "mcc_generated_files/mcc.h"

// Example: global variable for current RPM target (optional)
static int16_t current_rpm = 0;
extern volatile bool MotorRunning;
#include "motor_ctrl.h"

pid_t speed_pid = {
    .kp = 5.5f,
    .ki = 1.2f,
    .kd = 0.05f,
    .prev_error = 0,
    .integral = 0
};

int16_t target_rpm = 0;
//int16_t measured_rpm = 0;

int16_t PID_Compute(pid_t *pid, int16_t target, int16_t measured)
{
    float error = target - measured;

    // Integral accumulation
    pid->integral += error;

    // Derivative
    float derivative = error - pid->prev_error;

    // PID output
    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    pid->prev_error = error;

    // Limit output to PWM range -1023 to +1023
    if (output > 1023) output = 1023;
    else if (output < -1023) output = -1023;

    return (int16_t)output;
}

void Motor_ApplyPWM(int16_t pwm)
{
    if (pwm >= 0)
    {
        PWM3_LoadDutyValue(pwm); // forward
        PWM4_LoadDutyValue(0);
    }
    else
    {
        PWM3_LoadDutyValue(0);
        PWM4_LoadDutyValue(-pwm); // reverse
    }
}
void Motor_Start(int16_t rpm)
{
    target_rpm = rpm;
    speed_pid.prev_error = 0;
    speed_pid.integral = 0;

    // Start with zero duty (PID will ramp up)
    MotorRunning = true;
    Motor_ApplyPWM(rpm);

    printf("Motor_Start(): Target RPM = %d\n", rpm);
    printf("\n/> ");
}

void Motor_Stop(void)
{
    target_rpm = 0;
    MotorRunning = false;
    Motor_ApplyPWM(0);

    speed_pid.prev_error = 0;
    speed_pid.integral = 0;

    printf("Motor_Stop\n");
    printf("/> ");
}

