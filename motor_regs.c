#include <stdio.h>
#include "motor_regs.h"


uint16_t reg_get_word(uint8_t reg)
{
    return ((uint16_t)motor_regs[reg] << 8) | motor_regs[reg+1];
}

void reg_set_word(uint8_t reg, uint16_t value)
{
    motor_regs[reg]   = value >> 8;
    motor_regs[reg+1] = value & 0xFF;
}
