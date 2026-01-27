#include "motor_regs.h"

volatile int8_t motor_regs[REG_LEN] = {0};

int16_t reg_get_word(motor_reg_index_t msb_index)
{
    uint8_t msb = (uint8_t)motor_regs[msb_index];
    uint8_t lsb = (uint8_t)motor_regs[msb_index + 1];

    uint16_t combined = ((uint16_t)msb << 8) | (uint16_t)lsb;
    return (int16_t)combined;  // signed two's complement preserved
}

void reg_set_word(motor_reg_index_t msb_index, int16_t value)
{

    uint16_t v = (uint16_t)value;

    motor_regs[msb_index]     = (int8_t)((v >> 8) & 0xFFu);
    motor_regs[msb_index + 1] = (int8_t)( v       & 0xFFu);

    /* Mark DIRTY only for PID + TPR writes */
    if (stat1_should_mark_dirty_on_word_write(msb_index)) {
        stat1_mark_dirty();
    }

}

void init_regs(void)
{
    reg_set_word(REG_MTR_STAT0,0);          // motor run, reverse, mode bits- flags (running bit)
    reg_set_word(REG_MSB_MEAS_RPM,0);       // measured rpm used with PID mode
    reg_set_word(REG_MSB_TARGET_RPM,0);     // target rpm used with PID mode
    reg_set_word(REG_MSB_PWM,0);            // PWM value - used in Open Loop mode 
    reg_set_word(REG_MSB_KP,5500);          // P gain /1000 = 5.5
    reg_set_word(REG_MSB_KI,1200);          // I gain /1000 = 1.2
    reg_set_word(REG_MSB_KD,50);            // D gain /1000 = 0.05
    reg_set_word(REG_MSB_TPR,205);          // default encoder ticks per single shaft rotation = 205
    publish_ticks_to_regs(0);               // init ticks0-ticks3 to 0
   
}
