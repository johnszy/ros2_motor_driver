#include <stdio.h>
#include "motor_regs.h"


int16_t reg_get_word(int8_t reg)
{
    return ((int16_t)motor_regs[reg] << 8) | motor_regs[reg+1];
}

void reg_set_word(int8_t reg, int16_t value)
{
    motor_regs[reg+1]   = (int8_t)(value >> 8); //msb
    motor_regs[reg] = (int8_t)(value & 0xFF);  //lsb
}
