/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC16F1619
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
 * 
 * RX - PIN9 RC7
 * TX - PIN10 RB7
 *   
 * RC0PPS = 0x0F;   //RC0->PWM4:PWM4OUT;  
    RXPPS = 0x17;   //RC7->EUSART:RX;    
    RB7PPS = 0x12;   //RB7->EUSART:TX;  
    RA2PPS = 0x0E;   //RA2->PWM3:PWM3OUT;    
    
    SSPCLKPPS = 0x0E;     // RB4 DAT I2C  pin 11
    SSPDATPPS = 0x0C;     // RB6 CLK I2C  Pin 13
    RB4PPS = 0x11;
    RB6PPS = 0x10;  
*/

#include "mcc_generated_files/mcc.h"
#include "uart_cmd_proc.h"
#include "motor_regs.h"
//#include "motor_ctrl.h"

#define _XTAL_FREQ 16000000
#define DEBUG 0

extern volatile unsigned long milli_sec;
extern volatile long en0;
extern volatile uint16_t wheel_speed_rpm;
int16_t output_pwm = 0;
volatile bool MotorRunning = false;
extern volatile int8_t motor_regs[REG_LEN]; 



static mtr_ctrl_mode_t s_ctrl_mode = MTR_CTRL_OPEN_LOOP;

// For edge detection so we only act on changes
static bool       s_last_run   = false;
static mtr_dir_t  s_last_dir   = MTR_DIR_FORWARD;
static mtr_mode_t s_last_mode  = MTR_MODE_OPEN_LOOP;

volatile bool       run  = false;  // bit0
static mtr_dir_t  dir  = MTR_DIR_FORWARD;     // bit1
volatile mtr_mode_t op_mode = MTR_MODE_OPEN_LOOP;    // bit2
/*
                         Main application
 */
void main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    
    init_regs();
    
    Motor_Init();

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    __delay_ms(500);
    
    MTR_EN_SetHigh();   // enable motor driver
    LED_EN_SetHigh();  // turn on LED
    printf("\n/> ");

    while(1)
    {
        
        // 1. Read current status bits from MTR_STAT0
        run  = mtr_stat0_is_running();  // bit0
        dir  = mtr_stat0_get_dir();     // bit1
        op_mode = mtr_stat0_get_mode();    // bit2
        
        while( op_mode == MTR_MODE_OPEN_LOOP )
        {
            if(run)
            {
                I2C_Set_PWM();
            }
            else
            {
                Motor_Stop();
            }
            run  = mtr_stat0_is_running();  // bit0
            dir  = mtr_stat0_get_dir();     // bit1
            op_mode = mtr_stat0_get_mode();    // bit2
            
            int32_t snap = snapshot_ticks_atomic();
            publish_ticks_to_regs(snap);
  
        }
        while( op_mode == MTR_MODE_CLOSED_LOOP )
        {
            if(run)
            {
                //I2C_Set_PWM();
            }
            else
            {
                Motor_Stop();
            }
            run  = mtr_stat0_is_running();  // bit0
            dir  = mtr_stat0_get_dir();     // bit1
            op_mode = mtr_stat0_get_mode();    // bit2
            
            int32_t snap = snapshot_ticks_atomic();
            publish_ticks_to_regs(snap);
        }
        
        
        //UART_CommandProcess();
        //Motor_Task();
     /*   
        // Add your application code
        printf("elapsed time (ms): %lu %ld %d\r\n", milli_sec, en0, wheel_speed_rpm);
        __delay_ms(1000);
        
        PWM3_LoadDutyValue(300);
        PWM4_LoadDutyValue(0);
     
        __delay_ms(500);
        printf("elapsed time (ms): %lu %ld %d\r\n", milli_sec, en0, wheel_speed_rpm);
        __delay_ms(500);
        printf("elapsed time (ms): %lu %ld %d\r\n", milli_sec, en0, wheel_speed_rpm);
        __delay_ms(500);
        printf("elapsed time (ms): %lu %ld %d\r\n", milli_sec, en0, wheel_speed_rpm);
        __delay_ms(500);
        printf("elapsed time (ms): %lu %ld %d\r\n", milli_sec, en0, wheel_speed_rpm);
        
        
        PWM3_LoadDutyValue(0);
        PWM4_LoadDutyValue(0);
        __delay_ms(1000);
        printf("elapsed time (ms): %lu %ld %d\r\n", milli_sec, en0, wheel_speed_rpm);
        __delay_ms(500);
        printf("elapsed time (ms): %lu %ld %d\r\n", milli_sec, en0, wheel_speed_rpm);
        
        PWM3_LoadDutyValue(0);
        PWM4_LoadDutyValue(300);
        __delay_ms(500);
        printf("elapsed time (ms): %lu %ld %d\r\n", milli_sec, en0, wheel_speed_rpm);
        __delay_ms(500);
        printf("elapsed time (ms): %lu %ld %d\r\n", milli_sec, en0, wheel_speed_rpm);
        __delay_ms(500);
        printf("elapsed time (ms): %lu %ld %d\r\n", milli_sec, en0, wheel_speed_rpm);
        __delay_ms(500);
        printf("elapsed time (ms): %lu %ld %d\r\n", milli_sec, en0, wheel_speed_rpm);
        
        
        PWM3_LoadDutyValue(0);
        PWM4_LoadDutyValue(0);
        __delay_ms(1000);
        printf("elapsed time (ms): %lu %ld %d\r\n", milli_sec, en0, wheel_speed_rpm);
        __delay_ms(500);
        printf("elapsed time (ms): %lu %ld %d\r\n", milli_sec, en0, wheel_speed_rpm);
*/        
    }
}
/**
 End of File
*/