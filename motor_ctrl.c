#include <stdio.h>
#include "motor_ctrl.h"
#include "mcc_generated_files/mcc.h"
#include "motor_ctrl.h"
#include "motor_regs.h"


// Example external from your RPM calc code (TMR3 ISR, etc.)
//extern volatile int16_t wheel_speed_rpm;   // measured RPM
// Example: global variable for current RPM target (optional)
static int16_t current_rpm = 0;
extern volatile bool MotorRunning;
extern volatile int16_t ticks_per_rot;
int16_t target_rpm = 0;
//int16_t measured_rpm = 0;

#if DEBUG
static mtr_ctrl_mode_t s_ctrl_mode = MTR_CTRL_OPEN_LOOP;

// For edge detection so we only act on changes
static bool       s_last_run   = false;
static mtr_dir_t  s_last_dir   = MTR_DIR_FORWARD;
static mtr_mode_t s_last_mode  = MTR_MODE_OPEN_LOOP;
#endif

pid_t speed_pid = {
    .kp = 5.5f,
    .ki = 1.2f,
    .kd = 0.05f,
    .prev_error = 0,
    .integral = 0
};

void Motor_Init(void)
{
    // TODO: set up PWM, direction GPIOs, disable outputs at startup, etc.
    
    Motor_Stop();
    
    //s_ctrl_mode = MTR_CTRL_OPEN_LOOP;
    //s_last_run  = false;
    //s_last_dir  = MTR_DIR_FORWARD;
    //s_last_mode = MTR_MODE_OPEN_LOOP;
}

//mtr_ctrl_mode_t Motor_GetControlMode(void)
//{
//    //return s_ctrl_mode;
//}
#if DEBUG
void Motor_Task(void)
{
    // 1. Read current status bits from MTR_STAT0
    bool       run  = mtr_stat0_is_running();  // bit0
    mtr_dir_t  dir  = mtr_stat0_get_dir();     // bit1
    mtr_mode_t mode = mtr_stat0_get_mode();    // bit2

    // 2. Handle RUN start/stop edges
    if (run && !s_last_run) {
        // Transition: STOP -> RUN
        Motor_EnableOutput();
    } 
    else if (!run && s_last_run) {
        // Transition: RUN -> STOP
        Motor_Stop();
        // Optionally set PWM to zero
        Motor_ApplyPWM(0);
    }

    // 3. Handle direction changes
    if (dir != s_last_dir) {
        Motor_SetDir(dir);
    }

    // 4. Handle mode changes (open <-> closed loop)
    if (mode != s_last_mode) {
        s_ctrl_mode = (mode == MTR_MODE_CLOSED_LOOP)
                        ? MTR_CTRL_CLOSED_LOOP
                        : MTR_CTRL_OPEN_LOOP;

        // Optional: re-init PID when entering closed loop
        // PID_ResetIntegrators();
    }

    // 5. If motor should be running, apply appropriate control
    if (run) {
        if (s_ctrl_mode == MTR_CTRL_OPEN_LOOP) {
            // ---- OPEN LOOP: motor_regs PWM drives the hardware ----
            int16_t pwm_word = reg_get_word(REG_MSB_PWM);
            if (pwm_word < 0) pwm_word = 0;        // clamp for safety
            if (pwm_word > 1023) pwm_word = 1023;  // adjust to your PWM resolution
            Motor_ApplyPwm((uint16_t)pwm_word);

        } else { 
            // ---- CLOSED LOOP: use PID with target RPM ----
            int16_t target_rpm = reg_get_word(REG_MSB_TARGET_RPM);

            // Option 1 (recommended): PID in a timer ISR uses target_rpm & wheel_speed_rpm
            //   - Here you just make sure target_rpm is visible to that ISR
            //   - For example, store it in a global variable:
            extern volatile int16_t g_target_rpm;
            g_target_rpm = target_rpm;

            // Option 2 (simple but less precise): do PID here (software loop)
            //   - Only if your main loop is reasonably periodic.
            //   - I?d recommend the timer interrupt approach instead.
        }
    }

    // 6. Optionally mirror measured RPM into the MEAS_RPM registers
    //    This lets the I2C master read back actual speed.
    reg_set_word(REG_MSB_MEAS_RPM, wheel_speed_rpm);

    // 7. Update last-known state for change detection
    s_last_run  = run;
    s_last_dir  = dir;
    s_last_mode = mode;
}
#endif
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

void init_PID(void)
{
    float temp;
    temp =  reg_get_word(REG_MSB_KP) /1000;
    speed_pid.kp = temp;
    temp = reg_get_word(REG_MSB_KI) /1000;
    speed_pid.ki = temp;
    temp = reg_get_word(REG_MSB_KD) /1000;
    speed_pid.kd = temp;
    
}

void I2C_Set_PWM(void)
{
    //get direction, get pwm
    mtr_dir_t cur_dir = mtr_stat0_get_dir();
    int16_t cur_pwm = reg_get_word( REG_MSB_PWM );
    //set PWMs based on direction and pwm value
    if (cur_dir == MTR_DIR_FORWARD)
    {
        PWM3_LoadDutyValue(cur_pwm); // forward
        PWM4_LoadDutyValue(0);
    }  
    else
    {
        PWM3_LoadDutyValue(0);
        PWM4_LoadDutyValue(cur_pwm); // reverse
    }
}

// used for UART commands
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

//used for UART commands
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

// used for UART commands
void Motor_Stop(void)
{
    target_rpm = 0;
    MotorRunning = false;
    Motor_ApplyPWM(0);

    speed_pid.prev_error = 0;
    speed_pid.integral = 0;

    //printf("Motor_Stop\n");
    //printf("/> ");
}

void Motor_ClosedLoopStep(void)
{
    
}

void config_mtr_params(void)
{
    if (stat1_is_dirty()){
        update_globals_from_regs();
        stat1_clear_dirty();   
        // blink LED
        LED_EN_SetLow();  // turn off LED
        __delay_ms(500);
        LED_EN_SetHigh();  // turn on LED
        __delay_ms(500); 
        LED_EN_SetLow();  // turn off LED
        __delay_ms(500);
        LED_EN_SetHigh();  // turn on LED
        __delay_ms(500);
    }
        
}

void update_globals_from_regs(void)
{
    init_PID();
    ticks_per_rot = reg_get_word(REG_MSB_TPR);
    // TODO allow I2C to update POS_TICKS right now the publish ticks in main continually prevents it 
}
