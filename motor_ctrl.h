/* 
 * File:   motor_ctrl.h
 * Author: johns
 *
 * Created on November 5, 2025, 11:06 AM
 */



#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_error;
    float integral;
} pid_t;

// Extern lets other .c files access these variables
extern pid_t speed_pid;
extern int16_t target_rpm;
//extern int16_t measured_rpm;

typedef enum {
    MTR_CTRL_OPEN_LOOP = 0,
    MTR_CTRL_CLOSED_LOOP = 1
} mtr_ctrl_mode_t;

int16_t PID_Compute(pid_t *pid, int16_t target, int16_t measured);
void I2C_Set_PWM(void);
void init_PID(void);
void Motor_Init(void);
void Motor_Task(void);
void Motor_ApplyPWM(int16_t pwm);
void Motor_Start(int16_t rpm);
void Motor_Stop(void);
void Motor_ClosedLoopStep(void);
void config_mtr_params(void);
void update_globals_from_regs(void);

#ifdef	__cplusplus
extern "C" {
#endif



#endif	/* MOTOR_CTRL_H */

