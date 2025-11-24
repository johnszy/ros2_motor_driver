/* 
 * File:   motor_regs.h
 * Author: johns
 *
 * Created on November 12, 2025, 6:24 PM
 */

#ifndef MOTOR_REGS_H
#define	MOTOR_REGS_H


#include <stdint.h>
volatile uint8_t motor_regs[16];

uint16_t reg_get_word(uint8_t);
void reg_set_word(uint8_t reg, uint16_t);

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* MOTOR_REGS_H */

