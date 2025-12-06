/* 
 * File:   motor_regs.h
 * Author: johns
 *
 * Created on November 12, 2025, 6:24 PM
 */

#ifndef MOTOR_REGS_H
#define	MOTOR_REGS_H


#include <stdint.h>

#define REG_LEN 16
volatile int8_t motor_regs[REG_LEN];

int16_t reg_get_word(int8_t);
void reg_set_word(int8_t reg, int16_t);

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_REGS_H */
