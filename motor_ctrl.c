#include <stdio.h>
#include "motor_ctrl.h"
#include "mcc_generated_files/mcc.h"

// Example: global variable for current RPM target (optional)
static int16_t current_rpm = 0;

void Motor_Start(int16_t target_rpm)
{
    current_rpm = target_rpm;

    // TODO: add real motor control logic
    // Example placeholders:
    // PWM_DutyCycle(target_rpm_to_pwm(target_rpm));
    // Enable motor driver pins
    // Start closed-loop speed control

    if (current_rpm >0)
    {
        PWM3_LoadDutyValue(current_rpm);
        PWM4_LoadDutyValue(0);
    }
    else
    {
        PWM3_LoadDutyValue(0);
        PWM4_LoadDutyValue(-current_rpm);
    }
    printf("Motor_Start(): Target RPM = %d\n", current_rpm);
    printf("\n/> ");
}

void Motor_Stop(void)
{
    current_rpm = 0;

    // TODO: add real motor stop logic
    // Disable PWM / motor driver
    // Zero duty cycles
    // Brake or coast if needed
    PWM3_LoadDutyValue(0);
    PWM4_LoadDutyValue(0);
    printf("Motor_Stop(): Motor stopped\n");
    printf("\n/> ");
}

