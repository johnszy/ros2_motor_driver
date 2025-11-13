#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "mcc_generated_files/mcc.h"
#include "uart_cmd_proc.h"
#include "motor_ctrl.h"

#define MAX_COMMAND_LEN 30
#define VERSION_DATE "v1.0.0 11-01-25"

// buffer for incoming characters
static char command[MAX_COMMAND_LEN];
static uint8_t cmd_index = 0;
extern volatile uint16_t wheel_speed_rpm;
extern volatile long int delta;
extern volatile long en0;
// You will implement these somewhere else:
//extern void Motor_Start(uint16_t target_rpm);
//extern void Motor_Stop(void);
//extern void System_Reset(void);
//extern void Exit_CommandMode(void);

/**
 * @brief Call repeatedly in main() loop.
 * Assembles characters, calls executeCommand when '$' received.
 */
void UART_CommandProcess(void)
{
    char c = getch();

    if (c != '\n' && c != '\r')
    {
        command[cmd_index++] = c;
        if (cmd_index >= MAX_COMMAND_LEN)
            cmd_index = 0;
    }

    if (c == '$')    // end of command symbol
    {
        command[cmd_index] = '\0';
        cmd_index = 0;

        printf("received: %s\n", command);
        executeCommand(command);
    }
}


/**
 * @brief Parses command strings
 *
 * Supported:
 *   start,NN   (NN = RPM)
 *   stop
 *   reset
 *   exit
 *   help
 *   aR,G,B,W   - Set all PWM channels
 *   rNNN / gNNN / bNNN / wNNN  - single PWM
 *   v          - version
 *   f          - flush notification
 */
void executeCommand(char cmd[])
{
    // remove trailing '$'
    size_t len = strlen(cmd);
    if (len > 0)
        cmd[len-1] = '\0';

    // ----- motor commands -----

    if (strncmp(cmd, "start,", 6) == 0)
    {
        // extract rpm value after ','
        char *p = cmd + 6;
        int rpm = atoi(p);
        //if (rpm < 0) rpm = 0;

        printf("Starting motor target %d RPM\n", rpm);
        
        Motor_Start((int16_t)rpm);
        return;
    }

    if (strcmp(cmd, "stop") == 0)
    {
        printf("Motor stop\n");
        
        Motor_Stop();
        return;
    }
    
    if (strcmp(cmd, "getrpm") == 0)
    {
        printf("Motor rpm: %d %d\n", wheel_speed_rpm, delta);
        printf("\n/> ");
        return;
    }
    if (strcmp(cmd, "getenc") == 0)
    {
        printf("en0 raw: %ld\n", en0);
        printf("\n/> ");
        return;
    }
/*
    if (strcmp(cmd, "reset") == 0)
    {
        printf("System reset\n");
        System_Reset();
        return;
    }

    if (strcmp(cmd, "exit") == 0)
    {
        printf("Exiting command mode\n");
        Exit_CommandMode();
        return;
    }
*/
    if (strcmp(cmd, "help") == 0)
    {
        printf("Commands:\n");
        printf(" start,NN   - start motor, NN = RPM\n");
        printf(" stop       - stop motor\n");
        printf(" getrpm     - returns rpm\n");
        printf(" exit       - exit command mode\n");
        printf(" help       - show commands\n");
        printf(" rNNN,gNNN,bNNN,wNNN - set speeds 0-1023\n");
        printf(" aR,G,B,W   - set all PWMs\n");
        printf(" v          - version\n");
        printf(" f          - flush\n");
        printf("\n/> ");
        return;
    }

    // ----- PWM multi-channel -----
    if (cmd[0] == 'a')
    {
        uint16_t pwm_nums[4] = {0};
        char *token = strtok(cmd+1, ",");
        int count = 0;

        while (token != NULL && count < 4)
        {
            int v = atoi(token);
            if (v < 0 || v > 1023) v = 0;
            pwm_nums[count++] = (uint16_t)v;
            token = strtok(NULL, ",");
        }
    }
/*
        PWM6_LoadDutyValue(pwm_nums[0]);
        PWM5_LoadDutyValue(pwm_nums[1]);
        PWM4_LoadDutyValue(pwm_nums[2]);
        PWM3_LoadDutyValue(pwm_nums[3]);
        return;
    }

    // ----- single PWM channels -----
    if (cmd[0] == 'r' || cmd[0] == 'g' || cmd[0] == 'b' || cmd[0] == 'w')
    {
        uint16_t pwm_val = (uint16_t)atoi(cmd+1);
        if (pwm_val > 1023)
            pwm_val = 0;

        switch (cmd[0])
        {
            case 'r': PWM6_LoadDutyValue(pwm_val); break;
            case 'g': PWM5_LoadDutyValue(pwm_val); break;
            case 'b': PWM4_LoadDutyValue(pwm_val); break;
            case 'w': PWM3_LoadDutyValue(pwm_val); break;
        }
        return;
    }
*/
    // ----- misc -----
    if (cmd[0] == 'v')
    {
        printf("version = %s\n", VERSION_DATE);
        printf("\n/> ");
        return;
    }

    if (cmd[0] == 'f')
    {
        printf("flush buffer received\n");
        printf("\n/> ");
        return;
    }

    printf("Unknown command: %s\n", cmd);
    printf("\n/> ");
}

