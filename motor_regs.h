/* 
 * File:   motor_regs.h
 * Author: johns
 *
 * Created on November 12, 2025, 6:24 PM
 */

#ifndef MOTOR_REGS_H
#define MOTOR_REGS_H

#include <stdint.h>
#include <stdbool.h>

/* Total register length */
#define REG_LEN 20

/* ---- Register index enum: positions in motor_regs[] ---- */

typedef enum {
    REG_MTR_STAT0 = 0,   // byte 0
    REG_MTR_STAT1,       // byte 1

    REG_MSB_MEAS_RPM,    // byte 2
    REG_LSB_MEAS_RPM,    // byte 3

    REG_MSB_TARGET_RPM,  // byte 4
    REG_LSB_TARGET_RPM,  // byte 5

    REG_MSB_PWM,         // byte 6
    REG_LSB_PWM,         // byte 7

    REG_MSB_KP,          // byte 8
    REG_LSB_KP,          // byte 9

    REG_MSB_KI,          // byte 10
    REG_LSB_KI,          // byte 11

    REG_MSB_KD,          // byte 12
    REG_LSB_KD,          // byte 13

    // int32 encoder ticks (LSB..MSB)
    REG_POS_TICKS_B0,    // byte 14 (LSB)
    REG_POS_TICKS_B1,    // byte 15
    REG_POS_TICKS_B2,    // byte 16
    REG_POS_TICKS_B3,     // byte 17 (MSB)
            
    REG_MSB_TPR,          // byte 18  ticks per revolution
    REG_LSB_TPR,          // byte 19  ticks per revolution
} motor_reg_index_t;

/* ---- Global register array (define in motor_regs.c) ---- */
extern volatile int8_t motor_regs[REG_LEN];
extern volatile long en0;

/* ---- 16-bit word helpers (MSB at index, LSB at index+1) ---- */
int16_t reg_get_word(motor_reg_index_t msb_index);
void    reg_set_word(motor_reg_index_t msb_index, int16_t value);

/* Optionally: simple byte helpers */
static inline int8_t reg_get_byte(motor_reg_index_t idx)
{
    return motor_regs[idx];
}

static inline void reg_set_byte(motor_reg_index_t idx, int8_t val)
{
    motor_regs[idx] = val;
}

/* ============================================================
 *  MTR_STAT0 bit definitions & helpers
 *  bit0: RUN     (0 = stop, 1 = run)
 *  bit1: DIR     (0 = forward, 1 = reverse)
 *  bit2..3: MODE (0=open, 1=closed, 2=config, 3=reserved)
 * ============================================================ */

#define MTR_STAT0_RUN_MASK      (1u << 0)
#define MTR_STAT0_DIR_MASK      (1u << 1)

/* 2-bit MODE field in bits [3:2] */
#define MTR_STAT0_MODE_SHIFT    (2u)
#define MTR_STAT0_MODE_MASK     (3u << MTR_STAT0_MODE_SHIFT)   /* 0b1100 */


/* Direction enum */
typedef enum {
    MTR_DIR_FORWARD = 0,
    MTR_DIR_REVERSE = 1
} mtr_dir_t;

/* Mode enum */
typedef enum {
    MTR_MODE_OPEN_LOOP  = 0,
    MTR_MODE_CLOSED_LOOP = 1,
    MTR_MODE_CONFIG = 2
} mtr_mode_t;

/* Raw access for MTR_STAT0 */
static inline uint8_t mtr_stat0_get_raw(void)
{
    return (uint8_t)motor_regs[REG_MTR_STAT0];
}

static inline void mtr_stat0_set_raw(uint8_t value)
{
    motor_regs[REG_MTR_STAT0] = (int8_t)value;
}

/* ---- RUN helpers ---- */
static inline void mtr_stat0_set_run(bool run)
{
    uint8_t s = mtr_stat0_get_raw();
    if (run)   s |=  MTR_STAT0_RUN_MASK;
    else       s &= ~MTR_STAT0_RUN_MASK;
    mtr_stat0_set_raw(s);
}

static inline bool mtr_stat0_is_running(void)
{
    return (mtr_stat0_get_raw() & MTR_STAT0_RUN_MASK) != 0u;
}

/* ---- DIR helpers ---- */
static inline void mtr_stat0_set_dir(mtr_dir_t dir)
{
    uint8_t s = mtr_stat0_get_raw();
    if (dir == MTR_DIR_REVERSE) s |=  MTR_STAT0_DIR_MASK;
    else                        s &= ~MTR_STAT0_DIR_MASK;
    mtr_stat0_set_raw(s);
}

static inline mtr_dir_t mtr_stat0_get_dir(void)
{
    return (mtr_stat0_get_raw() & MTR_STAT0_DIR_MASK) ?
           MTR_DIR_REVERSE : MTR_DIR_FORWARD;
}

/* ---- MODE helpers (bits 2..3) ---- */
static inline void mtr_stat0_set_mode(mtr_mode_t mode)
{
    uint8_t s = mtr_stat0_get_raw();
    s &= (uint8_t)~MTR_STAT0_MODE_MASK; /* clear bits 2..3 */
    s |= (uint8_t)(((uint8_t)mode << MTR_STAT0_MODE_SHIFT) & MTR_STAT0_MODE_MASK);
    mtr_stat0_set_raw(s);
}

static inline mtr_mode_t mtr_stat0_get_mode(void)
{
    uint8_t raw = mtr_stat0_get_raw();
    uint8_t m = (uint8_t)((raw & MTR_STAT0_MODE_MASK) >> MTR_STAT0_MODE_SHIFT);
    return (mtr_mode_t)m; /* returns 0,1,2,3 (3 reserved) */
}

static inline bool mtr_mode_is_valid(mtr_mode_t mode)
{
    return (mode == MTR_MODE_OPEN_LOOP) ||
           (mode == MTR_MODE_CLOSED_LOOP) ||
           (mode == MTR_MODE_CONFIG);
}



/* ---- STAT1 bit definitions ----
 * bits1..0: SEQ (2-bit sequence counter)
 * bit2:     DIRTY_STATUS (0=clean, 1=dirty)
 */
#define STAT1_SEQ_MASK        (0x03u)      /* 0b00000011 */
#define STAT1_DIRTY_MASK      (1u << 2)    /* 0b00000100 */



static inline uint8_t stat1_get_seq(void)
{
    return (uint8_t)motor_regs[REG_MTR_STAT1] & STAT1_SEQ_MASK;
}

static inline void stat1_set_seq(uint8_t seq)
{
    uint8_t s = (uint8_t)motor_regs[REG_MTR_STAT1];
    s = (s & (uint8_t)~STAT1_SEQ_MASK) | (seq & STAT1_SEQ_MASK);
    motor_regs[REG_MTR_STAT1] = (int8_t)s;
}

static inline void stat1_bump_seq(void)
{
    uint8_t seq = (stat1_get_seq() + 1u) & 0x03u;
    stat1_set_seq(seq);
}

/* ---- DIRTY_STATUS helpers (STAT1 bit2) ---- */
static inline bool stat1_is_dirty(void)
{
    return (((uint8_t)motor_regs[REG_MTR_STAT1]) & STAT1_DIRTY_MASK) != 0u;
}

static inline void stat1_set_dirty(bool dirty)
{
    uint8_t s = (uint8_t)motor_regs[REG_MTR_STAT1];
    if (dirty) s |=  (uint8_t)STAT1_DIRTY_MASK;
    else       s &= (uint8_t)~STAT1_DIRTY_MASK;
    motor_regs[REG_MTR_STAT1] = (int8_t)s;
}

static inline void stat1_mark_dirty(void)
{
    stat1_set_dirty(true);
}

static inline void stat1_clear_dirty(void)
{
    stat1_set_dirty(false);
}

static inline bool stat1_should_mark_dirty_on_byte_write(uint8_t idx)
{
    return (idx == REG_MSB_KP) || (idx == REG_LSB_KP) ||
           (idx == REG_MSB_KI) || (idx == REG_LSB_KI) ||
           (idx == REG_MSB_KD) || (idx == REG_LSB_KD) ||
           (idx == REG_MSB_TPR) || (idx == REG_LSB_TPR);
}
static inline bool stat1_should_mark_dirty_on_word_write(motor_reg_index_t msb_index)
{
    return (msb_index == REG_MSB_KP) ||
           (msb_index == REG_MSB_KI) ||
           (msb_index == REG_MSB_KD) ||
           (msb_index == REG_MSB_TPR);
}

static inline void publish_ticks_to_regs(int32_t ticks)
{
    // Mark update start
    stat1_bump_seq();
    
    motor_regs[REG_POS_TICKS_B0] = (int8_t)((uint32_t)ticks >> 0);
    motor_regs[REG_POS_TICKS_B1] = (int8_t)((uint32_t)ticks >> 8);
    motor_regs[REG_POS_TICKS_B2] = (int8_t)((uint32_t)ticks >> 16);
    motor_regs[REG_POS_TICKS_B3] = (int8_t)((uint32_t)ticks >> 24);
    
    // Mark update complete
    stat1_bump_seq();
}

static inline int32_t snapshot_ticks_atomic(void)
{
    int32_t snap;
    di();              // disable interrupts (XC8)
    snap = en0;
    ei();              // enable interrupts
    return snap;
}



void init_regs(void);

void reg_set_word(motor_reg_index_t, int16_t );

int16_t reg_get_word(motor_reg_index_t );

#endif /* MOTOR_REGS_H */