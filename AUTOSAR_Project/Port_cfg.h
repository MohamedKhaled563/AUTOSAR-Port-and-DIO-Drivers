 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Khaled Mohamed Radi
 *
 ******************************************************************************/
#ifndef PORT_CFG_H
#define PORT_CFG_H

/******************************************************************************
 *                             AR and SW Versions                             *
 ******************************************************************************/

/* Module Version 1.0.0 */
#define PORT_CFG_SW_MAJOR_VERSION           (1U)
#define PORT_CFG_SW_MINOR_VERSION           (0U)
#define PORT_CFG_SW_PATCH_VERSION           (0U)

/* Autosar Version 4.0.3 */
#define PORT_CFG_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_CFG_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_CFG_AR_RELEASE_PATCH_VERSION   (3U)

/******************************************************************************
 *                         Pre-Compile Configurations                         *
 ******************************************************************************/

/* The number of specified PortPins  */
#define PORT_PINS_CONFIGURED_NUMBER          (39U)

/* Pre-compile option for Development Error Detect */
#define PORT_DEV_ERROR_DETECT                (STD_ON)

/* Pre-compile option for presence of Port_SetPinDirection Function */
#define PORT_SET_PIN_DIRECTION_API           (STD_ON)

/* Pre-compile option for presence of Port_SetPinMode Function */
#define PORT_SET_PIN_MODE_API                (STD_ON)

/* Pre-compile option for presence of Port_VersionInfo Function */
#define PORT_VERSION_INFO_API                (STD_ON)

/******************************************************************************
 *             Array indexes and Values for Post Build Structure              *
 ******************************************************************************/
/* Configuration Struction Indexes */
/* Port A */
#define PORTCONFIG_A_PIN_0_ID_INDEX         (uint8)0x00
#define PORTCONFIG_A_PIN_1_ID_INDEX         (uint8)0x01
#define PORTCONFIG_A_PIN_2_ID_INDEX         (uint8)0x02
#define PORTCONFIG_A_PIN_3_ID_INDEX         (uint8)0x03 
#define PORTCONFIG_A_PIN_4_ID_INDEX         (uint8)0x04
#define PORTCONFIG_A_PIN_5_ID_INDEX         (uint8)0x05 
#define PORTCONFIG_A_PIN_6_ID_INDEX         (uint8)0x06
#define PORTCONFIG_A_PIN_7_ID_INDEX         (uint8)0x07
/* Port B */
#define PORTCONFIG_B_PIN_0_ID_INDEX         (uint8)0x08
#define PORTCONFIG_B_PIN_1_ID_INDEX         (uint8)0x09 
#define PORTCONFIG_B_PIN_2_ID_INDEX         (uint8)0x0A   
#define PORTCONFIG_B_PIN_3_ID_INDEX         (uint8)0x0B
#define PORTCONFIG_B_PIN_4_ID_INDEX         (uint8)0x0C
#define PORTCONFIG_B_PIN_5_ID_INDEX         (uint8)0x0D
#define PORTCONFIG_B_PIN_6_ID_INDEX         (uint8)0x0E 
#define PORTCONFIG_B_PIN_7_ID_INDEX         (uint8)0x0F
/* Port C */   
/*PC0:PC3 reserved for JTAG*/
#define PORTCONFIG_C_PIN_4_ID_INDEX         (uint8)0x10 
#define PORTCONFIG_C_PIN_5_ID_INDEX         (uint8)0x11
#define PORTCONFIG_C_PIN_6_ID_INDEX         (uint8)0x12 
#define PORTCONFIG_C_PIN_7_ID_INDEX         (uint8)0x13
/* Port D */   
#define PORTCONFIG_D_PIN_0_ID_INDEX         (uint8)0x14 
#define PORTCONFIG_D_PIN_1_ID_INDEX         (uint8)0x15
#define PORTCONFIG_D_PIN_2_ID_INDEX         (uint8)0x16 
#define PORTCONFIG_D_PIN_3_ID_INDEX         (uint8)0x17
#define PORTCONFIG_D_PIN_4_ID_INDEX         (uint8)0x18
#define PORTCONFIG_D_PIN_5_ID_INDEX         (uint8)0x19
#define PORTCONFIG_D_PIN_6_ID_INDEX         (uint8)0x1A 
#define PORTCONFIG_D_PIN_7_ID_INDEX         (uint8)0x1B
/* Port E */   
#define PORTCONFIG_E_PIN_0_ID_INDEX         (uint8)0x1C 
#define PORTCONFIG_E_PIN_1_ID_INDEX         (uint8)0x1D
#define PORTCONFIG_E_PIN_2_ID_INDEX         (uint8)0x1E 
#define PORTCONFIG_E_PIN_3_ID_INDEX         (uint8)0x1F
#define PORTCONFIG_E_PIN_4_ID_INDEX         (uint8)0x20 
#define PORTCONFIG_E_PIN_5_ID_INDEX         (uint8)0x21
/* Port F */   
#define PORTCONFIG_F_PIN_0_ID_INDEX         (uint8)0x22 
#define PORTCONFIG_F_PIN_1_ID_INDEX         (uint8)0x23
#define PORTCONFIG_F_PIN_2_ID_INDEX         (uint8)0x24 
#define PORTCONFIG_F_PIN_3_ID_INDEX         (uint8)0x25
#define PORTCONFIG_F_PIN_4_ID_INDEX         (uint8)0x26

/* Port Numbers */
#define PORTCONFIG_PORT_A_NUM               (0x00)
#define PORTCONFIG_PORT_B_NUM               (0x01)  
#define PORTCONFIG_PORT_C_NUM               (0x02)  
#define PORTCONFIG_PORT_D_NUM               (0x03)  
#define PORTCONFIG_PORT_E_NUM               (0x04)  
#define PORTCONFIG_PORT_F_NUM               (0x05) 

/* Pin Numbers */
#define PORTCONFIG_PIN_0_NUM                      (0x00)
#define PORTCONFIG_PIN_1_NUM                      (0x01)  
#define PORTCONFIG_PIN_2_NUM                      (0x02)  
#define PORTCONFIG_PIN_3_NUM                      (0x03)  
#define PORTCONFIG_PIN_4_NUM                      (0x04)  
#define PORTCONFIG_PIN_5_NUM                      (0x05) 
#define PORTCONFIG_PIN_6_NUM                      (0x06)
#define PORTCONFIG_PIN_7_NUM                      (0x07)

/* Modes */
/* DIO */
#define PORTCONFIG_PIN_MODE_DIO                   (0x00)

/* Analog */
#define PORTCONFIG_PIN_MODE_ANALOG                (0x0A)
   
/* 1 */
#define PORTCONFIG_PIN_MODE_UART0_RX              (0x01)
#define PORTCONFIG_PIN_MODE_UART0_TX              (0x01)
#define PORTCONFIG_PIN_MODE_UART1_RX              (0x01)
#define PORTCONFIG_PIN_MODE_UART1_TX              (0x01)
#define PORTCONFIG_PIN_MODE_UART1_RTS             (0x01)
#define PORTCONFIG_PIN_MODE_UART1_CTS             (0x01)
#define PORTCONFIG_PIN_MODE_UART2_RX              (0x01)
#define PORTCONFIG_PIN_MODE_UART2_TX              (0x01)
#define PORTCONFIG_PIN_MODE_UART3_RX              (0x01)
#define PORTCONFIG_PIN_MODE_UART3_TX              (0x01)
#define PORTCONFIG_PIN_MODE_UART4_RX              (0x01)
#define PORTCONFIG_PIN_MODE_UART4_TX              (0x01)
#define PORTCONFIG_PIN_MODE_UART5_RX              (0x01)
#define PORTCONFIG_PIN_MODE_UART5_TX              (0x01)
#define PORTCONFIG_PIN_MODE_UART6_RX              (0x01)
#define PORTCONFIG_PIN_MODE_UART6_TX              (0x01)
#define PORTCONFIG_PIN_MODE_SSI3_CLK              (0x01)
#define PORTCONFIG_PIN_MODE_SSI3_FSS              (0x01)
#define PORTCONFIG_PIN_MODE_SSI3_RX               (0x01)
#define PORTCONFIG_PIN_MODE_SSI3_TX               (0x01)

/* 2 */
#define PORTCONFIG_PIN_MODE_SSI0_CLK              (0x02)
#define PORTCONFIG_PIN_MODE_SSI0_FSS              (0x02)
#define PORTCONFIG_PIN_MODE_SSI0_RX               (0x02)
#define PORTCONFIG_PIN_MODE_SSI0_TX               (0x02)
#define PORTCONFIG_PIN_MODE_SSI2_CLK              (0x02)
#define PORTCONFIG_PIN_MODE_SSI2_FSS              (0x02)
#define PORTCONFIG_PIN_MODE_SSI2_RX               (0x02)
#define PORTCONFIG_PIN_MODE_SSI2_TX               (0x02)
#define PORTCONFIG_PIN_MODE_SSI1_CLK              (0x02)
#define PORTCONFIG_PIN_MODE_SSI1_FSS              (0x02)
#define PORTCONFIG_PIN_MODE_SSI1_RX               (0x02)
#define PORTCONFIG_PIN_MODE_SSI1_TX               (0x02)

/* 3 */
#define PORTCONFIG_PIN_MODE_I2C0_SCL              (0x03)
#define PORTCONFIG_PIN_MODE_I2C0_SDA              (0x03)
#define PORTCONFIG_PIN_MODE_I2C1_SCL              (0x03)
#define PORTCONFIG_PIN_MODE_I2C1_SDA              (0x03)
#define PORTCONFIG_PIN_MODE_I2C2_SCL              (0x03)
#define PORTCONFIG_PIN_MODE_I2C2_SDA              (0x03)
#define PORTCONFIG_PIN_MODE_I2C3_SCL              (0x03)
#define PORTCONFIG_PIN_MODE_I2C3_SDA              (0x03) 

/* 4 */
#define PORTCONFIG_PIN_MODE_M0PWM2                (0x04)
#define PORTCONFIG_PIN_MODE_M0PWM3                (0x04)
#define PORTCONFIG_PIN_MODE_M0PWM0                (0x04)
#define PORTCONFIG_PIN_MODE_M0PWM1                (0x04)
#define PORTCONFIG_PIN_MODE_M0PWM4                (0x04)
#define PORTCONFIG_PIN_MODE_M0PWM5                (0x04)
#define PORTCONFIG_PIN_MODE_M0PWM6                (0x04)
#define PORTCONFIG_PIN_MODE_M0PWM7                (0x04)
#define PORTCONFIG_PIN_MODE_M0FAULT0              (0x04)

/* 5 */
#define PORTCONFIG_PIN_MODE_M1PWM2                (0x05)
#define PORTCONFIG_PIN_MODE_M1PWM3                (0x05)
#define PORTCONFIG_PIN_MODE_M1PWM0                (0x05)
#define PORTCONFIG_PIN_MODE_M1PWM1                (0x05)
#define PORTCONFIG_PIN_MODE_M1PWM3                (0x05)
#define PORTCONFIG_PIN_MODE_M1PWM4                (0x05)
#define PORTCONFIG_PIN_MODE_M1PWM5                (0x05)
#define PORTCONFIG_PIN_MODE_M1PWM6                (0x05)
#define PORTCONFIG_PIN_MODE_M1PWM7                (0x05)
#define PORTCONFIG_PIN_MODE_M1FAULT0              (0x05)

/* 6 */
#define PORTCONFIG_PIN_MODE_IDX1                  (0x06)
#define PORTCONFIG_PIN_MODE_IDX0                  (0x06)
#define PORTCONFIG_PIN_MODE_PHA1                  (0x06)
#define PORTCONFIG_PIN_MODE_PHB1                  (0x06)
#define PORTCONFIG_PIN_MODE_PHA0                  (0x06)
#define PORTCONFIG_PIN_MODE_PHB0                  (0x06)

/* 7 */
#define PORTCONFIG_PIN_MODE_T0CCP0                (0x07)
#define PORTCONFIG_PIN_MODE_T0CCP1                (0x07)
#define PORTCONFIG_PIN_MODE_T1CCP0                (0x07)
#define PORTCONFIG_PIN_MODE_T1CCP1                (0x07)
#define PORTCONFIG_PIN_MODE_T2CCP0                (0x07)
#define PORTCONFIG_PIN_MODE_T2CCP1                (0x07)
#define PORTCONFIG_PIN_MODE_T3CCP0                (0x07)
#define PORTCONFIG_PIN_MODE_T3CCP1                (0x07)
#define PORTCONFIG_PIN_MODE_WT0CCP0               (0x07)
#define PORTCONFIG_PIN_MODE_WT0CCP1               (0x07)
#define PORTCONFIG_PIN_MODE_WT1CCP0               (0x07)
#define PORTCONFIG_PIN_MODE_WT1CCP1               (0x07)
#define PORTCONFIG_PIN_MODE_WT2CCP0               (0x07)
#define PORTCONFIG_PIN_MODE_WT2CCP1               (0x07)
#define PORTCONFIG_PIN_MODE_WT3CCP0               (0x07)
#define PORTCONFIG_PIN_MODE_WT3CCP1               (0x07)
#define PORTCONFIG_PIN_MODE_WT4CCP0               (0x07)
#define PORTCONFIG_PIN_MODE_WT4CCP1               (0x07)
#define PORTCONFIG_PIN_MODE_WT5CCP0               (0x07)
#define PORTCONFIG_PIN_MODE_WT5CCP1               (0x07)

/* 8 */
#define PORTCONFIG_PIN_MODE_CAN1_RX               (0x08)
#define PORTCONFIG_PIN_MODE_CAN1_TX               (0x08)
#define PORTCONFIG_PIN_MODE_CAN0_RX               (0x08)
#define PORTCONFIG_PIN_MODE_CAN0_TX               (0x08)
#define PORTCONFIG_PIN_MODE_USB0_EPEN             (0x08)
#define PORTCONFIG_PIN_MODE_USB0_PFLT             (0x08)
#define PORTCONFIG_PIN_MODE_NMI                   (0x08)

/* 9 */
#define PORTCONFIG_PIN_MODE_C0                    (0x09)
#define PORTCONFIG_PIN_MODE_C1                    (0x09)

/* 14 */
#define PORTCONFIG_PIN_MODE_TRD1                  (0x0E)
#define PORTCONFIG_PIN_MODE_TRD0                  (0x0E)
#define PORTCONFIG_PIN_MODE_TRCLK                 (0x0E)

/* Special Values */
#define PORTCONFIG_SPECIAL_C4_PIN_MODE_UART1_RX   (0x02) /* PORTC PIN4 */
#define PORTCONFIG_SPECIAL_C5_PIN_MODE_UART1_TX   (0x02) /* PORTC PIN5 */
#define PORTCONFIG_SPECIAL_F0_PIN_MODE_CAN0_RX    (0x03) /* PORTF PIN0 */
#define PORTCONFIG_SPECIAL_F3_PIN_MODE_CAN0_TX    (0x03) /* PORTF PIN3 */
#define PORTCONFIG_SPECIAL_C4_PIN_MODE_U1RTS      (0x08) /* PORTC PIN4 */
#define PORTCONFIG_SPECIAL_C5_PIN_MODE_U1CTS      (0x08) /* PORTC PIN5 */

#endif /* PORT_CFG_H */
