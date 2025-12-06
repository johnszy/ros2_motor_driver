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
*/

#include "mcc_generated_files/mcc.h"
#include "uart_cmd_proc.h"
#include "motor_regs.h"
//#include "motor_ctrl.h"

#define _XTAL_FREQ 16000000

extern volatile unsigned long milli_sec;
extern volatile long en0;
extern volatile uint16_t wheel_speed_rpm;
int16_t output_pwm = 0;
volatile bool MotorRunning = false;
extern volatile int8_t motor_regs[REG_LEN]; 
/*
                         Main application
 */
void main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    
    reg_set_word(0,1);          // velocity_target
    reg_set_word(2,1);          // velocity_measured
    reg_set_word(4,1);          // pwm_value
    reg_set_word(6,1);     // flags (running bit)
    reg_set_word(8,5500);       // P gain *1000 = 5.5
    reg_set_word(10,1200);       // I gain *1000 = 1.2
    reg_set_word(12,50);        // D gain *1000 = 0.05
    reg_set_word(14,0);         // reserved

   /*
    motor_regs[0] = 1;     // velocity_target
    motor_regs[1] = 1;     // velocity_measured
    motor_regs[2] = 1;     // pwm_value
    motor_regs[3] = 0x0001;// flags (running bit)
    motor_regs[4] = 5500;   // P gain *1000 = 5.5
    motor_regs[5] = 1200;    // I gain *1000 = 1.2
    motor_regs[6] = 50;    // D gain *1000 = 0.05
*/

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

    while (1)
    {
        
        UART_CommandProcess();

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