/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.8
        Device            :  PIC16F1619
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.36 and above
        MPLAB 	          :  MPLAB X 6.00	
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

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>



#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set ENC0 aliases
#define ENC0_TRIS                 TRISAbits.TRISA0
#define ENC0_LAT                  LATAbits.LATA0
#define ENC0_PORT                 PORTAbits.RA0
#define ENC0_WPU                  WPUAbits.WPUA0
#define ENC0_OD                   ODCONAbits.ODA0
#define ENC0_ANS                  ANSELAbits.ANSA0
#define ENC0_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define ENC0_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define ENC0_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define ENC0_GetValue()           PORTAbits.RA0
#define ENC0_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define ENC0_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define ENC0_SetPullup()          do { WPUAbits.WPUA0 = 1; } while(0)
#define ENC0_ResetPullup()        do { WPUAbits.WPUA0 = 0; } while(0)
#define ENC0_SetPushPull()        do { ODCONAbits.ODA0 = 0; } while(0)
#define ENC0_SetOpenDrain()       do { ODCONAbits.ODA0 = 1; } while(0)
#define ENC0_SetAnalogMode()      do { ANSELAbits.ANSA0 = 1; } while(0)
#define ENC0_SetDigitalMode()     do { ANSELAbits.ANSA0 = 0; } while(0)

// get/set ENC1 aliases
#define ENC1_TRIS                 TRISAbits.TRISA1
#define ENC1_LAT                  LATAbits.LATA1
#define ENC1_PORT                 PORTAbits.RA1
#define ENC1_WPU                  WPUAbits.WPUA1
#define ENC1_OD                   ODCONAbits.ODA1
#define ENC1_ANS                  ANSELAbits.ANSA1
#define ENC1_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define ENC1_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define ENC1_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define ENC1_GetValue()           PORTAbits.RA1
#define ENC1_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define ENC1_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define ENC1_SetPullup()          do { WPUAbits.WPUA1 = 1; } while(0)
#define ENC1_ResetPullup()        do { WPUAbits.WPUA1 = 0; } while(0)
#define ENC1_SetPushPull()        do { ODCONAbits.ODA1 = 0; } while(0)
#define ENC1_SetOpenDrain()       do { ODCONAbits.ODA1 = 1; } while(0)
#define ENC1_SetAnalogMode()      do { ANSELAbits.ANSA1 = 1; } while(0)
#define ENC1_SetDigitalMode()     do { ANSELAbits.ANSA1 = 0; } while(0)

// get/set RA2 procedures
#define RA2_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define RA2_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define RA2_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define RA2_GetValue()              PORTAbits.RA2
#define RA2_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define RA2_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define RA2_SetPullup()             do { WPUAbits.WPUA2 = 1; } while(0)
#define RA2_ResetPullup()           do { WPUAbits.WPUA2 = 0; } while(0)
#define RA2_SetAnalogMode()         do { ANSELAbits.ANSA2 = 1; } while(0)
#define RA2_SetDigitalMode()        do { ANSELAbits.ANSA2 = 0; } while(0)

// get/set RB7 procedures
#define RB7_SetHigh()            do { LATBbits.LATB7 = 1; } while(0)
#define RB7_SetLow()             do { LATBbits.LATB7 = 0; } while(0)
#define RB7_Toggle()             do { LATBbits.LATB7 = ~LATBbits.LATB7; } while(0)
#define RB7_GetValue()              PORTBbits.RB7
#define RB7_SetDigitalInput()    do { TRISBbits.TRISB7 = 1; } while(0)
#define RB7_SetDigitalOutput()   do { TRISBbits.TRISB7 = 0; } while(0)
#define RB7_SetPullup()             do { WPUBbits.WPUB7 = 1; } while(0)
#define RB7_ResetPullup()           do { WPUBbits.WPUB7 = 0; } while(0)
#define RB7_SetAnalogMode()         do { ANSELBbits.ANSB7 = 1; } while(0)
#define RB7_SetDigitalMode()        do { ANSELBbits.ANSB7 = 0; } while(0)

// get/set RC0 procedures
#define RC0_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define RC0_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define RC0_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define RC0_GetValue()              PORTCbits.RC0
#define RC0_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define RC0_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define RC0_SetPullup()             do { WPUCbits.WPUC0 = 1; } while(0)
#define RC0_ResetPullup()           do { WPUCbits.WPUC0 = 0; } while(0)
#define RC0_SetAnalogMode()         do { ANSELCbits.ANSC0 = 1; } while(0)
#define RC0_SetDigitalMode()        do { ANSELCbits.ANSC0 = 0; } while(0)

// get/set MTR_EN aliases
#define MTR_EN_TRIS                 TRISCbits.TRISC1
#define MTR_EN_LAT                  LATCbits.LATC1
#define MTR_EN_PORT                 PORTCbits.RC1
#define MTR_EN_WPU                  WPUCbits.WPUC1
#define MTR_EN_OD                   ODCONCbits.ODC1
#define MTR_EN_ANS                  ANSELCbits.ANSC1
#define MTR_EN_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define MTR_EN_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define MTR_EN_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define MTR_EN_GetValue()           PORTCbits.RC1
#define MTR_EN_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define MTR_EN_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define MTR_EN_SetPullup()          do { WPUCbits.WPUC1 = 1; } while(0)
#define MTR_EN_ResetPullup()        do { WPUCbits.WPUC1 = 0; } while(0)
#define MTR_EN_SetPushPull()        do { ODCONCbits.ODC1 = 0; } while(0)
#define MTR_EN_SetOpenDrain()       do { ODCONCbits.ODC1 = 1; } while(0)
#define MTR_EN_SetAnalogMode()      do { ANSELCbits.ANSC1 = 1; } while(0)
#define MTR_EN_SetDigitalMode()     do { ANSELCbits.ANSC1 = 0; } while(0)

// get/set LED_EN aliases
#define LED_EN_TRIS                 TRISCbits.TRISC2
#define LED_EN_LAT                  LATCbits.LATC2
#define LED_EN_PORT                 PORTCbits.RC2
#define LED_EN_WPU                  WPUCbits.WPUC2
#define LED_EN_OD                   ODCONCbits.ODC2
#define LED_EN_ANS                  ANSELCbits.ANSC2
#define LED_EN_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define LED_EN_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define LED_EN_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define LED_EN_GetValue()           PORTCbits.RC2
#define LED_EN_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define LED_EN_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define LED_EN_SetPullup()          do { WPUCbits.WPUC2 = 1; } while(0)
#define LED_EN_ResetPullup()        do { WPUCbits.WPUC2 = 0; } while(0)
#define LED_EN_SetPushPull()        do { ODCONCbits.ODC2 = 0; } while(0)
#define LED_EN_SetOpenDrain()       do { ODCONCbits.ODC2 = 1; } while(0)
#define LED_EN_SetAnalogMode()      do { ANSELCbits.ANSC2 = 1; } while(0)
#define LED_EN_SetDigitalMode()     do { ANSELCbits.ANSC2 = 0; } while(0)

// get/set RC3 procedures
#define RC3_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define RC3_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define RC3_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define RC3_GetValue()              PORTCbits.RC3
#define RC3_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define RC3_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)
#define RC3_SetPullup()             do { WPUCbits.WPUC3 = 1; } while(0)
#define RC3_ResetPullup()           do { WPUCbits.WPUC3 = 0; } while(0)
#define RC3_SetAnalogMode()         do { ANSELCbits.ANSC3 = 1; } while(0)
#define RC3_SetDigitalMode()        do { ANSELCbits.ANSC3 = 0; } while(0)

// get/set RC4 procedures
#define RC4_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define RC4_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define RC4_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define RC4_GetValue()              PORTCbits.RC4
#define RC4_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define RC4_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define RC4_SetPullup()             do { WPUCbits.WPUC4 = 1; } while(0)
#define RC4_ResetPullup()           do { WPUCbits.WPUC4 = 0; } while(0)

// get/set RC7 procedures
#define RC7_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define RC7_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define RC7_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define RC7_GetValue()              PORTCbits.RC7
#define RC7_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define RC7_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define RC7_SetPullup()             do { WPUCbits.WPUC7 = 1; } while(0)
#define RC7_ResetPullup()           do { WPUCbits.WPUC7 = 0; } while(0)
#define RC7_SetAnalogMode()         do { ANSELCbits.ANSC7 = 1; } while(0)
#define RC7_SetDigitalMode()        do { ANSELCbits.ANSC7 = 0; } while(0)


volatile long en0 = 0;


/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);


/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handler for the IOCAF0 pin functionality
 * @Example
    IOCAF0_ISR();
 */
void IOCAF0_ISR(void);

/**
  @Summary
    Interrupt Handler Setter for IOCAF0 pin interrupt-on-change functionality

  @Description
    Allows selecting an interrupt handler for IOCAF0 at application runtime
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    InterruptHandler function pointer.

  @Example
    PIN_MANAGER_Initialize();
    IOCAF0_SetInterruptHandler(MyInterruptHandler);

*/
void IOCAF0_SetInterruptHandler(void (* InterruptHandler)(void));

/**
  @Summary
    Dynamic Interrupt Handler for IOCAF0 pin

  @Description
    This is a dynamic interrupt handler to be used together with the IOCAF0_SetInterruptHandler() method.
    This handler is called every time the IOCAF0 ISR is executed and allows any function to be registered at runtime.
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCAF0_SetInterruptHandler(IOCAF0_InterruptHandler);

*/
extern void (*IOCAF0_InterruptHandler)(void);

/**
  @Summary
    Default Interrupt Handler for IOCAF0 pin

  @Description
    This is a predefined interrupt handler to be used together with the IOCAF0_SetInterruptHandler() method.
    This handler is called every time the IOCAF0 ISR is executed. 
    
  @Preconditions
    Pin Manager intializer called

  @Returns
    None.

  @Param
    None.

  @Example
    PIN_MANAGER_Initialize();
    IOCAF0_SetInterruptHandler(IOCAF0_DefaultInterruptHandler);

*/
void IOCAF0_DefaultInterruptHandler(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/