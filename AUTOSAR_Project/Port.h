 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Khaled Mohamed Radi
 *
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/******************************************************************************
 *                  Define IDs, AR and SW Versions                            *
 ******************************************************************************/

/* Id for the company in the AUTOSAR
 * for example Mohamed Khaled's ID = 1000 */
#define PORT_VENDOR_ID                  (1000U)

/* Port Module Id */
#define PORT_MODULE_ID                  (124U)

/* Port Instance Id */
#define PORT_INSTANCE_ID                (0U)

/* Module Version 1.0.0 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/* Autosar Version 4.0.3 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/******************************************************************************
 *      Include AUTOSAR and Non-AUTOSAR files and apply Version Checking      *
 ******************************************************************************/

/* Standards AUTOSAR Types */
#include "Std_Types.h"

/* AUTOSAR Checking Between Std_Types and PORT Modules */
#if ( (STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION) \
  ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION) \
  ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION)) 
  #error "The AR version of Std_Types.h does not match the expected version"
#endif

/* Port Pre-Compile configurations Header File*/
#include "Port_cfg.h"
    
/* AUTOSAR Version Checking Between Port_Cfg.h and Port.h Files */
#if ( (PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION) \
  ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION) \
  ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION)) 
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif
    
/* Software Version Checking Between Port_Cfg.h and Port.h Files */
#if ( (PORT_CFG_SW_RELEASE_MAJOR_VERSION != PORT_SW_RELEASE_MAJOR_VERSION) \
  ||  (PORT_CFG_SW_RELEASE_MINOR_VERSION != PORT_SW_RELEASE_MINOR_VERSION) \
  ||  (PORT_CFG_SW_RELEASE_PATCH_VERSION != PORT_SW_RELEASE_PATCH_VERSION)) 
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif    

/* Non AUTOSAR Files */
#include "Common_Macros.h"
    
 /*******************************************************************************
 *                              Port Status                                 *
 *******************************************************************************/
#define PORT_INITIALIZED                     (1U)
#define PORT_NOT_INITIALIZED                 (0U)
    
/*******************************************************************************
 *                          Define DET Error Codes                             *
 *******************************************************************************/
/* Invalid Port Pin ID requested */   
#define PORT_E_PARAM_PIN                     (uint8)0x0A
/* Port Pin not configured as changeable */    
#define PORT_E_DIRECTION_UNCHANGEABLE        (uint8)0x0B
/* API Port_Init service called with wrong parameter */
#define PORT_E_PARAM_CONFIG                  (uint8)0x0C 
/* API Port_SetPinMode service called with invalid mode */
#define PORT_E_PARAM_INVALID_MODE            (uint8)0x0D   
/* API Port_SetPinMode service called when mode is unchangeable. */
#define PORT_E_MODE_UNCHANGEABLE             (uint8)0x0E
/*API service called without module initialization */
#define PORT_E_UNINIT                        (uint8)0x0F  
/* APIs called with a Null Pointer */
#define PORT_E_PARAM_POINTER                 (uint8)0x10   
    
/******************************************************************************
 *                          Define Service IDs                                *
 ******************************************************************************/
/* Service ID for port init */
#define PORT_PORT_INIT_SID                 (uint8)0x00
/* Service ID for Port SetPinDirection */
#define PORT_SET_PIN_DIRECTION_SID         (uint8)0x01
/* Service ID for Port RefreshPortDirection */
#define PORT_REFRESH_PORT_DIRECTION_SID    (uint8)0x02
/* Service ID for Port GetVersionInfo*/
#define PORT_GET_VERSION_INFO_SID          (uint8)0x03
/* Service ID for Port SetPinMode */
#define PORT_SET_PIN_MODE_SID              (uint8)0x04   
    
/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/
/* PORT046 */ /* PORT220 */
/* Possible directions of a port pin */
typedef enum  
{
  PORT_PIN_IN, PORT_PIN_OUT 
} Port_PinDirectionType;

/* PORT013 */  /* PORT219 */
 /* Data type for the symbolic name of a port pin. */ 
typedef uint8 Port_PinType;

 /* Data type for the symbolic name of a port. */ 
typedef uint8 Port_Type;

/* Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
} Port_InternalResistorType;

/* PORT221 */
/* Different port pin modes */
typedef uint8 Port_PinModeType;

/* PORT129_Conf*/
typedef enum 
{
  PORT_PIN_LEVEL_HIGH, PORT_PIN_LEVEL_LOW
} Port_PinLevelValueType;

 /*PORT072*/
/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *      3. the direction of pin --> INPUT or OUTPUT
 *      4. the direction is changable or not
 *      5. Mode
 *      6. Level
 *      7. the mode is changable or not
 *      8. the internal resistor --> Disable, Pull up or Pull down
 */
 typedef struct 
{
    Port_Type port_num; 
    Port_PinType pin_num; 
    Port_PinDirectionType Port_Pin_Direction;
    boolean Port_Pin_Direction_Changeable;
    Port_PinModeType mode;
    Port_PinLevelValueType Port_PinLevelValue;
    boolean Port_Pin_Mode_Changeable;
    Port_InternalResistorType resistor;
}Port_ConfigPortPinType;

/* PORT073 */ 
/* Type of the external data structure containing the initialization data for this module.*/
typedef struct 
{
  Port_ConfigPortPinType portPins[PORT_PINS_CONFIGURED_NUMBER];
} Port_ConfigType;

/*******************************************************************************
 *                      Function Prototypes                                    *
 *******************************************************************************/

/* Function for Port Init API */
void Port_Init( const Port_ConfigType* ConfigPtr );

/* Function for Poer Set Pin Direction API */
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction );

/* Function for Port Refresh Port Direction API */
void Port_RefreshPortDirection( void );

/* Function for Std Version Info Type API */
void Port_GetVersionInfo( Std_VersionInfoType* versioninfo );

/* Function for Port Set Pin Mode API */
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode );    
    
/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/
extern const Port_ConfigType Port_Configuration;

#endif /* PORT_H */
