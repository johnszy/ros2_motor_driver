/* 
 * File:   motor_ctrl.h
 * Author: johns
 *
 * Created on November 5, 2025, 11:06 AM
 */



#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <stdint.h>

void Motor_Start(int16_t target_rpm);
void Motor_Stop(void);


#ifdef	__cplusplus
extern "C" {
#endif



#endif	/* MOTOR_CTRL_H */

