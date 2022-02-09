 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Mohamed Khaled Mohamed Radi
 *
 ******************************************************************************/

/******************************************************************************
 *                  Include ModName.h and non AUTOSAR files                   *
 ******************************************************************************/
#include "Port.h"
#include "Port_Regs.h"

/******************************************************************************
 *               Include AUTOSAR files and apply Version Checking             *
 ******************************************************************************/
/* AUTOSAR checking between Det and Port Modules */
#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
#if ( (PORT_AR_RELEASE_MAJOR_VERSION != DET_AR_MAJOR_VERSION)\
   || (PORT_AR_RELEASE_MINOR_VERSION != DET_AR_MINOR_VERSION)\ 
   || (PORT_AR_RELEASE_PATCH_VERSION != DET_AR_PATCH_VERSION) )
#error "The AR version of Det.h does not match the expected version"
#endif
   
#endif
   
/******************************************************************************
 *       Pointer to the configuration array and Module status variable        *
 ******************************************************************************/
STATIC const Port_ConfigPortPinType* Port_pins = NULL_PTR;
STATIC uint8 Port_status = PORT_NOT_INITIALIZED;

/******************************************************************************
 *                             Functions Definations                          *
 ******************************************************************************/
/************************************************************************************
* Service Name: Port_Init
* Service ID[hex]: 0x00
* Sync/Async: Synchronous
* Reentrancy: Non reentrant
* Parameters (in): ConfigPtr - Pointer to configuration set.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Initializes the Port Driver module.
************************************************************************************/
/* PORT140 */ 
/* PORT121 */ /* The function Port_Init shall always have a pointer as a parameter. */
void Port_Init( const Port_ConfigType* ConfigPtr )
{
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    volatile uint32 delay = 0;
    
  /*PORT105*/ 
  /* If development error detection for the Port Driver module is enabled and function
   * Port_Init is called with a NULL ConfigPtr, the function Port_Init shall raise the 
   * development error PORT_E_PARAM_CONFIG and return without any action.*/
#if ( PORT_DEV_ERROR_DETECT == STD_ON )
    if ( NULL_PTR == ConfigPtr )
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_PORT_INIT_SID, 
                        PORT_E_PARAM_CONFIG);
    }
    else
#endif 
    {
        /* PORT002 */
        /* The function Port_Init shall initialize all variables used by the PORT
         * driver module to an initial state. */
        Port_pins = ConfigPtr->portPins;
        /* PORT213 */
        /* If Port_Init function is not called first, then no operation can occur on
         * the MCU ports and port pins. */
        Port_status = PORT_INITIALIZED;
        
        /* Enable clock for all pins and allow time for clock to start*/
        SYSCTL_REGCGC2_REG |= 0x3F;
        delay = SYSCTL_REGCGC2_REG;
        
        /* PORT041 */
        /* The function Port_Init shall initialize ALL ports and port pins with the
         * configuration set pointed to by the parameter ConfigPtr. */
        /* PORT042 */
        /* The function Port_Init shall initialize all configured resources */ 
        for (uint8 portPinIndex = PORTCONFIG_A_PIN_0_ID_INDEX; portPinIndex < PORT_CONFIGURED_PORT_PINS; portPinIndex++)
        {
            /* Get Port Base Register */
            switch(Port_pins[portPinIndex].port_num)
            {
            case PORTCONFIG_PORT_A_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                                         break;
            case PORTCONFIG_PORT_B_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                                         break;
            case PORTCONFIG_PORT_C_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                                         break;                             
            case PORTCONFIG_PORT_D_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                                         break;                             
            case PORTCONFIG_PORT_E_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                                         break;                             
            case PORTCONFIG_PORT_F_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                                         break;                                                       
            }
            
            /* Special Cases*/
            /* Pins Need Unlocking */
            if( ((Port_pins[portPinIndex].port_num == PORTCONFIG_PORT_D_NUM) && (Port_pins[portPinIndex].pin_num == PORTCONFIG_PIN_7_NUM))
             || ((Port_pins[portPinIndex].port_num == PORTCONFIG_PORT_F_NUM) && (Port_pins[portPinIndex].pin_num == PORTCONFIG_PIN_0_NUM)) ) /* PD7 or PF0 */
            {
                *(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;                     /* Unlock the GPIOCR register */   
                SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_COMMIT_REG_OFFSET) , ConfigPtr.pin_num);  /* Set the corresponding bit in GPIOCR register to allow changes on this pin */
            }
            /* JTAG Pins */
            /* These cases will not happen because any time so it can be removed */
            else if( ( PORTCONFIG_PORT_C_NUM == Port_pins[portPinIndex].port_num ) && ( PORTCONFIG_PIN_3_NUM >= Port_pins[portPinIndex].pin_num ) ) /* PC0 to PC3 */
            {
                /* Do Nothing ...  this is the JTAG pins */
                continue;
            }
            else
            {
                /* Do Nothing ... No need to unlock the commit register for this pin */
            }
            
            /* Set Analog mode and Digital Mode registers */
            if( PORTCONFIG_PIN_MODE_ANALOG == Port_pins[portPinIndex].mode )
            {
                /* Set Analog Mode only for pins with analog mode */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_pins[portPinIndex].pin_num);
                /* Clear Digital enable for pins with analog mode */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_pins[portPinIndex].pin_num);
            }
            /* All other modes are digital */
            else 
            {
                /* Clear Analog Mode only for pins with analog mode */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_pins[portPinIndex].pin_num);
                /* Set Digital enable for pins with analog mode */
                SET_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_pins[portPinIndex].pin_num);
            }
   
            /* Clear the PMCx bits in CTL Register */
            *(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << Port_pins[portPinIndex].pin_num * 4);

            /* Clear Alternative Function if GPIO */
            if( (Port_pins[portPinIndex].mode == PORTCONFIG_PIN_MODE_DIO) || (Port_pins[portPinIndex].mode == PORTCONFIG_PIN_MODE_ANALOG) )
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_pins[portPinIndex].pin_num);
            }
            /* For al other modes, Set Alternative Function bit and set the mode in the CTL register */
            else 
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_pins[portPinIndex].pin_num);
                *(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x0000000F << Port_pins[portPinIndex].pin_num * 4);
            }
            
            /* Set the Direction of pin */
            /* 1 if Output and set initial value */
            if ( PORT_PIN_OUT == Port_pins[portPinIndex].Port_Pin_Direction )
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_pins[portPinIndex].pin_num);
                /* Set Initial value in data register */
                if (Port_pins[portPinIndex].Port_PinLevelValue == PORT_PIN_LEVEL_HIGH)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_pins[portPinIndex].pin_num);
                }
                else (Port_pins[portPinIndex].Port_PinLevelValue == PORT_PIN_LEVEL_LOW)
                {
                    CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_DATA_REG_OFFSET), Port_pins[portPinIndex].pin_num);
                }
            }
            /* 0 if input */
            else if ( PORT_PIN_IN == Port_pins[portPinIndex].Port_Pin_Direction )
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_pins[portPinIndex].pin_num);
                /* Set the internal resistor value */
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Port_pins[portPinIndex].pin_num);
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_pins[portPinIndex].pin_num);
                if(Port_pins[portPinIndex].resistor == PULL_UP)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_PULL_UP_REG_OFFSET), Port_pins[portPinIndex].pin_num);
                }
                else if(Port_pins[portPinIndex].resistor == PULL_UP)
                {
                    SET_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_PULL_DOWN_REG_OFFSET), Port_pins[portPinIndex].pin_num);           
                }
                else 
                {
                  /* Do Nothing */
                } 
            }
            else 
            {
                /* Do Nothing */
            }
        }
    }
}

/************************************************************************************
* Service Name: Port_SetPinDirection
* Service ID[hex]: 0x01
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin - Port Pin ID number (INDEX).
                   Direction - Port Pin Direction. 
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin direction.
************************************************************************************/
/* PORT086 */
/* The function Port_SetPinDirection shall only be available to the user if the 
 * pre-compile parameter PortSetPinDirectionApi is set to TRUE.  */
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
/* PORT141 */
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction )
{
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    boolean error = FALSE;
    
    /* Error Checking */
#if ( PORT_DEV_ERROR_DETECT == STD_ON )
    /* API service called without module initialization */
    if( PORT_NOT_INITIALIZED == Port_status )
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, 
                        PORT_E_UNINIT);
        error = TRUE;
    }
    else 
    {
        /* Do Nothing */
    }
    /* Invalid Port Pin ID requested */   
    if( Pin >= PORT_CONFIGURED_PORT_PINS )
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, 
                        PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else 
    {
        /* Do Nothing */
    }
    /* Port Pin not configured as changeable */    
    if( STD_OFF == Port_pins[Pin].Port_Pin_Direction_Changeable )
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, 
                        PORT_E_DIRECTION_UNCHANGEABLE);
        error = TRUE;
    }
    else 
    {
        /* Do Nothing */
    }
#endif
    /* In-case there are no errors */
    /* PORT063 */
    /* The function Port_SetPinDirection shall set the port pin direction during runtime */
    if( FALSE == error )
    {
        /* Get Port Base Register */
        switch(Port_pins[portPinIndex].port_num)
        {
        case PORTCONFIG_PORT_A_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                                     break;
        case PORTCONFIG_PORT_B_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                                     break;
        case PORTCONFIG_PORT_C_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                                     break;                             
        case PORTCONFIG_PORT_D_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                                     break;                             
        case PORTCONFIG_PORT_E_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                                     break;                             
        case PORTCONFIG_PORT_F_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                                     break;                                                       
        }
        if ( PORT_PIN_OUT == Direction  )
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_pins[Pin].pin_num);
        }
        /* 0 if input */
        else if ( PORT_PIN_IN == Direction )
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_pins[portPinIndex].pin_num);
        }
        else 
        {
            /* Do Nothing */
        }
    }
}
#endif

/************************************************************************************
* Service Name: Port_RefreshPortDirection
* Service ID[hex]: 0x02
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None 
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Refreshes port direction.
************************************************************************************/
/* PORT142 */
/* PORT060 */
/* The function Port_RefreshPortDirection shall refresh the direction of all configured 
 * ports to the configured direction */
void Port_RefreshPortDirection( void )
{
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    boolean error = FALSE;
    
    /* Error Checking */
#if ( PORT_DEV_ERROR_DETECT == STD_ON )
    /* API service called without module initialization */
    if( PORT_NOT_INITIALIZED == Port_status )
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_PORT_DIRECTION_SID, 
                        PORT_E_UNINIT);
        error = TRUE;
    }
    else 
    {
        /* Do Nothing */
    }
#endif
    /* In-case there are no errors */
    if( FALSE == error )
    {
        for (uint8 portPinIndex = PORTCONFIG_A_PIN_0_ID_INDEX; portPinIndex < PORT_CONFIGURED_PORT_PINS; portPinIndex++)
        {
            switch(Port_pins[portPinIndex].port_num)
            {
            case PORTCONFIG_PORT_A_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                                         break;
            case PORTCONFIG_PORT_B_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                                         break;
            case PORTCONFIG_PORT_C_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                                         break;                             
            case PORTCONFIG_PORT_D_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                                         break;                             
            case PORTCONFIG_PORT_E_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                                         break;                             
            case PORTCONFIG_PORT_F_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                                         break;                                                       
            }
            /* 1 if Output and set initial value */
            if (Port_pins[portPinIndex].Port_Pin_Direction == PORT_PIN_OUT)
            {
                SET_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_pins[portPinIndex].pin_num);
            }
            /* 0 if input */
            else if (Port_pins[portPinIndex].Port_Pin_Direction == PORT_PIN_IN)
            {
                CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_DIR_REG_OFFSET), Port_pins[portPinIndex].pin_num);
            }
            else 
            {
                /* Do Nothing */
            }
        }
    }
}

/************************************************************************************
* Service Name: Port_GetVersionInfo
* Service ID[hex]: 0x03
* Sync/Async: Synchronous
* Reentrancy: Non Reentrant
* Parameters (in): None
* Parameters (inout): None
* Parameters (out): versioninfo - Pointer to where to store the version information of this module
* Return value: None
* Description: Returns the version information of this module.
************************************************************************************/
/* PORT103 */
/* The function Port_GetVersionInfo shall be pre compile time configurable On/Off by 
 * the configuration parameter PortVersionInfoApi. */
#if ( PORT_VERSION_INFO_API == STD_ON )
/* PORT145 */
void Port_GetVersionInfo( Std_VersionInfoType* versioninfo )
{
    boolean error = FALSE;
    
    /* Error Checking */
#if ( PORT_DEV_ERROR_DETECT == STD_ON )
    /* API service called without module initialization */
    if( PORT_NOT_INITIALIZED == Port_status )
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, 
                        PORT_E_UNINIT);
        error = TRUE;
    }
    else 
    {
        /* Do Nothing */
    }
    /* PORT225 */
    /* if Det is enabled, the parameter versioninfo shall be checked for being NULL.
     *  The error PORT_E_PARAM_POINTER shall be reported in case the value is a NULL pointer.*/
    if( NULL_PTR == versioninfo )
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, 
                        PORT_E_PARAM_POINTER);
        error = TRUE;
    }
    else 
    {
        /* Do Nothing */
    }    
#endif
    /* In-case there are no errors */
    if( FALSE == error )
    {
        versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
        versioninfo->moduleID = (uint16)PORT_MODULE_ID;
        versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
        versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
        versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }   
}
#endif

/************************************************************************************
* Service Name: Port_SetPinMode
* Service ID[hex]: 0x04
* Sync/Async: Synchronous
* Reentrancy: Reentrant
* Parameters (in): Pin - Port Pin ID number
                   Mode - New Port Pin mode to be set on port pin.
* Parameters (inout): None
* Parameters (out): None
* Return value: None
* Description: Sets the port pin mode.
************************************************************************************/
#if( PORT_SET_PIN_MODE_API == STD_ON )

/* PORT145 */
/* PORT125 */
/* The function Port_SetPinMode shall set the port pin mode of the referenced pin during runtime. */
void Port_SetPinMode( Port_PinType Pin, Port_PinModeType Mode )
{
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    boolean error = FALSE;
    
    /* Error Checking */
#if ( PORT_DEV_ERROR_DETECT == STD_ON )
    /* API service called without module initialization */
    if( PORT_NOT_INITIALIZED == Port_status )
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, 
                        PORT_E_UNINIT);
        error = TRUE;
    }
    else 
    {
        /* Do Nothing */
    }
    /* Invalid Port Pin ID requested */   
    if( Pin >= PORT_CONFIGURED_PORT_PINS )
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, 
                        PORT_E_PARAM_PIN);
        error = TRUE;
    }
    else 
    {
        /* Do Nothing */
    }
    /* API Port_SetPinMode service called when mode is unchangeable. */
    if( FALSE == Port_pins[Pin].Port_Pin_Mode_Changeable )
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, 
                        PORT_E_MODE_UNCHANGEABLE);
        error = TRUE;
    }
    else 
    {
        /* Do Nothing */
    }
    /* Check if mode is invalid */
    if( (Mode > 0x09) && (Mode != 0x0E) &&(Mode != (0x0A)) )
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, 
                        PORT_E_PARAM_INVALID_MODE);
        error = TRUE;
    }
#endif
      /* In-case there are no errors */
    if( FALSE == error )
    {
        switch(Port_pins[Pin].port_num)
        {
        case PORTCONFIG_PORT_A_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
                                     break;
        case PORTCONFIG_PORT_B_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
                                     break;
        case PORTCONFIG_PORT_C_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
                                     break;                             
        case PORTCONFIG_PORT_D_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
                                     break;                             
        case PORTCONFIG_PORT_E_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
                                     break;                             
        case PORTCONFIG_PORT_F_NUM : PortGpio_Ptr = (volatile uint32 *) GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
                                     break;                                                       
        }
        /* Set Analog mode and Digital Mode registers */
        if( PORTCONFIG_PIN_MODE_ANALOG == Port_pins[portPinIndex].mode )
        {
            /* Set Analog Mode only for pins with analog mode */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_pins[portPinIndex].pin_num);
            /* Clear Digital enable for pins with analog mode */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_pins[portPinIndex].pin_num);
        }
        /* All other modes are digital */
        else 
        {
            /* Clear Analog Mode only for pins with analog mode */
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_pins[portPinIndex].pin_num);
            /* Set Digital enable for pins with analog mode */
            SET_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_pins[portPinIndex].pin_num);
        }

        /* Clear the PMCx bits in CTL Register */
        *(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_CTL_REG_OFFSET) &= ~(0x0000000F << Port_pins[portPinIndex].pin_num * 4);

        /* Clear Alternative Function if GPIO */
        if( (Port_pins[portPinIndex].mode == PORTCONFIG_PIN_MODE_DIO) || (Port_pins[portPinIndex].mode == PORTCONFIG_PIN_MODE_ANALOG) )
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_pins[portPinIndex].pin_num);
        }
        /* For al other modes, Set Alternative Function bit and set the mode in the CTL register */
        else 
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_pins[portPinIndex].pin_num);
            *(volatile uint32 *)((volatile uint8 *) PortGpio_Ptr + PORT_CTL_REG_OFFSET) |= (0x0000000F << Port_pins[portPinIndex].pin_num * 4);
        }
    }
}

#endif 
