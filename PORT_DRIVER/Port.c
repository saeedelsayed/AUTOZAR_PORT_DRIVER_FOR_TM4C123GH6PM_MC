 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.c
 *
 * Description: Source file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Saeed Elsayed
 ******************************************************************************/

#include "Port.h"
#include "Port_Regs.h"

#if (PORT_DEV_ERROR_DETECT == STD_ON)

#include "Det.h"
/* AUTOSAR Version checking between Det and Dio Modules */
#if ((DET_AR_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 || (DET_AR_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 || (DET_AR_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Det.h does not match the expected version"
#endif

#endif

STATIC const Port_ConfigType * Port_ConfigPtr = NULL_PTR;
STATIC uint8 Port_Status = PORT_NOT_INITIALIZED;   
   
/***********************************************************************************
*                                functions                                         *
***********************************************************************************/

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
void Port_Init(const Port_ConfigType* ConfigPtr)
{
  boolean error = FALSE;
      
#if( PORT_DEV_ERROR_DETECT == STD_ON )
    /* check if the input configuration pointer is not a NULL_PTR */
    if( ConfigPtr == NULL_PTR )
    {
      Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_INIT_SID,
		     PORT_E_PARAM_CONFIG);
      error = TRUE;
    }
  else
#endif    
  {
       /*
       * Set the module state to initialized and point to the PB configuration structure using a global pointer.
       * This global pointer is global to be used by other functions to read the PB configuration structures
       */
      Port_Status    = PORT_INITIALIZED;
      Port_ConfigPtr = ConfigPtr; /* address of the first configuration of the pins --> pinsConfgrations[0] */
  }
  
  if(error == FALSE)
  {
      for(uint8 i =0; i<PORT_PIN_NUMBERS; i++)
      {
        volatile uint32 *Port_Ptr = NULL_PTR;
        switch (ConfigPtr->pinsConfgrations[i].port_num)
        {
        case PORTA_ID:
            Port_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;
            break;
        case PORTB_ID:
            Port_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;
            break;
        case PORTC_ID:
            Port_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;
            break;
        case PORTD_ID:
            Port_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;
            break;
        case PORTE_ID:
            Port_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;
            break;
        case PORTF_ID:
            Port_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;
            break;
        }
        uint8 delay =0;
        /* Enable clock of the chosen port */
        SYSCTL_REGCGC2_REG |= ConfigPtr->pinsConfgrations[i].port_num;
        /* delay to allow the clock to initialize */
        delay = SYSCTL_REGCGC2_REG;
        /* do not change anything in JTAG pins */
        if( ( ConfigPtr->pinsConfgrations[i].port_num == PORTC_ID ) && ( ConfigPtr->pinsConfgrations[i].pin_num <= PIN3_ID ) )
        {
          continue;  /* do not complete the code */
        }
        /* unlock and commit for PD7 and PF0*/
        else if( ( ( ConfigPtr->pinsConfgrations[i].port_num == PORTD_ID ) && ( ConfigPtr->pinsConfgrations[i].pin_num <= PIN7_ID ) )  \
                  || ( ( ConfigPtr->pinsConfgrations[i].port_num == PORTF_ID ) && ( ConfigPtr->pinsConfgrations[i].pin_num <= PIN0_ID ) ) )
        {
          /* Unlock the GPIO_PORTX_CR_REG */
          *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_LOCK_REG_OFFSET) = 0x4C4F434B;
          /* Enable changes on the required pin */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_COMMIT_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num); 
          
        }
        
/*-----------------------------------------------------------------------------------------------------------------------*/
                                       /* Configure direction properities */
        
        /* set configurations for pin if it is output pin */
        if( ConfigPtr->pinsConfgrations[i].direction == PORT_PIN_OUTPUT )
        {
              /* Configure the pin as output pin */
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIR_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num);
            
            
            /* check the initial value if it is high */
            if( ConfigPtr->pinsConfgrations[i].initial_value == Port_Pin_STD_HIGH )
            {
              /* Set the corresponding pin in the data register */
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DATA_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num);            }
            else
            {
              /* clear the corresponding pin in the data register */
              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DATA_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num);
            }
        }
        
        /* set configurations for pin if it is input pin */
       else if( ConfigPtr->pinsConfgrations[i].direction == PORT_PIN_INPUT )
        {
              /* Configure the pin as output pin */
              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIR_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num); 
            
            
            /* check the initial value if it is high */
            if( ConfigPtr->pinsConfgrations[i].resistor == PULL_UP )
            {
              /* Set the corresponding pin in the data register */
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_UP_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num);
            }
            else if( ConfigPtr->pinsConfgrations[i].resistor == PULL_DOWN )
            {
              /* clear the corresponding pin in the data register */
              SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_DOWN_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num);
            }
        
            /* the clear the pull up and pull down modes */
            else
            {
              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_UP_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num);
              CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_PULL_DOWN_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num);
            }
        
        }
        
/*-----------------------------------------------------------------------------------------------------------------------*/
                               /* Configure mode properities */
        
        if(ConfigPtr->pinsConfgrations[i].pin_mode == Analog_mode)
        {
          /* clear the corresponding bit in the digital enable register */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num);
          
          /* Set the corresponding bit in the analog mode select register */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num);
        }
        else
        {
          /* Set the corresponding bit in the digital enable register */
          SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num);
          
          /* CLEAR_BIT the corresponding bit in the analog mode select register */
          CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num);
        }
        
        
       if(ConfigPtr->pinsConfgrations[i].pin_mode == GPIO_mode)
       {
         /* disable the alternative function mode */
         CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num);
         /* clear the control register mode of this pin */
         *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) &= ~(PMCx_MASK << (ConfigPtr->pinsConfgrations[i].pin_num * 4));
       }
       else
       {
        /* enable the alternative function mode */
         SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ALT_FUNC_REG_OFFSET), ConfigPtr->pinsConfgrations[i].pin_num);
         /* Clear all PMCx bits then set them in the control register according to the required mode */
         *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) &= ~(PMCx_MASK << (ConfigPtr->pinsConfgrations[i].pin_num * 4));
         *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= ((ConfigPtr->pinsConfgrations[i].pin_mode & PMCx_MASK) << (ConfigPtr->pinsConfgrations[i].pin_num * 4));
        }       }
      }
      
  }
   
/************************************************************************************
 * Service Name: Port_SetPinDirection
 * Service ID[hex]: 0x01
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): Pin - Port Pin ID number, Direction - Port Pin Direction
 * Parameters (inout): None
 * Parameters (out): None
 * Return value: None
 * Description: Sets the port pin direction.
 ************************************************************************************/
#if (PORT_SET_PIN_DIRECTION_API == STD_ON )  
void Port_SetPinDirection( Port_PinType Pin, Port_PinDirectionType Direction )
{
  
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    
    /* check for the initialization of the port */
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_UNINIT);
    }
    else
    {
        /* No Action Required */
    }
    /* Check if the used pin is within the valid range */
    if (Port_ConfigPtr->pinsConfgrations[Pin].pin_num < 0 || Port_ConfigPtr->pinsConfgrations[Pin].pin_num >= PORT_PIN_NUMBERS)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_PARAM_PIN);
    }
    else
    {
        /* No Action Required */
    }
    /* check if the pin is configured unchangeable */
    if (Port_ConfigPtr->pinsConfgrations[Pin].direction_changeability == DIRECTION_NOT_CHANGABLE)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_DIRECTION_SID, PORT_E_DIRECTION_UNCHANGEABLE);
    }
    else
    {
        /* No Action Required */
    }
#endif
    
    if( ( Port_ConfigPtr->pinsConfgrations[Pin].port_num == PORTC_ID ) && ( Port_ConfigPtr->pinsConfgrations[Pin].pin_num <= PIN3_ID ) )
        {
          return;  /* if it is JTAG pins, end the function */
        }
    else
    {
      /* No Action Required */
    }
    
    volatile uint32 * PortGpio_Ptr = NULL_PTR; /* point to the required Port Registers base address */
    
    switch(Port_ConfigPtr->pinsConfgrations[Pin].port_num)
    {
        case PORTA_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS; /* PORTA Base Address */
		 break;
	case PORTB_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS; /* PORTB Base Address */
		 break;
	case PORTC_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS; /* PORTC Base Address */
		 break;
	case PORTD_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS; /* PORTD Base Address */
		 break;
        case PORTE_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS; /* PORTE Base Address */
		 break;
        case PORTF_ID: PortGpio_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS; /* PORTF Base Address */
		 break;
    }
    
    
    if(Direction == PORT_PIN_OUTPUT)
      
    {   /* Set the corresponding bit in the GPIODIR register to configure it as output pin */ 
	SET_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_ConfigPtr->pinsConfgrations[Pin].port_num);  
    }
    else if(Direction == PORT_PIN_INPUT)
    {
        /* Clear the corresponding bit in the GPIODIR register to configure it as input pin */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)PortGpio_Ptr + PORT_DIR_REG_OFFSET) , Port_ConfigPtr->pinsConfgrations[Pin].port_num);
    }
    else
    {
        /* Do Nothing */
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
void Port_RefreshPortDirection(void)
{
            /* check det errors */
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check it the port is not initialized */
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_REFRESH_PORT_DIRECTION_SID, PORT_E_UNINIT);
    }
    else
    {
        /* Do Nothing */
    }
#endif
    for (uint8 i = PA0; i < PORT_PIN_NUMBERS; i++)
    {
        /* point to the required Port Registers base address */
        volatile uint32 *Port_Ptr = NULL_PTR;
        switch (Port_ConfigPtr->pinsConfgrations[i].pin_num)
        {
        case PORTA_ID:
            Port_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;
            break;
        case PORTB_ID:
            Port_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;
            break;
        case PORTC_ID:
            Port_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;
            break;
        case PORTD_ID:
            Port_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;
            break;
        case PORTE_ID:
            Port_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;
            break;
        case PORTF_ID:
            Port_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;
            break;
        }

        /* If PC0-PC3, then do nothing (JTAG Pins) */
        if ((Port_ConfigPtr->pinsConfgrations[i].pin_num == PORTC_ID) && (Port_ConfigPtr->pinsConfgrations[i].pin_num <= PIN3_ID))
        {
            continue;
        }
        /* exclude pins that are configured as pin direction changeable during runtime */
        if(Port_ConfigPtr->pinsConfgrations[i].direction_changeability == DIRECTION_CHANGABLE)
        {
            continue; 
        }

        /* check if the pin is input */
        if (Port_ConfigPtr->pinsConfgrations[i].pin_num == PORT_PIN_INPUT)
        {
            CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIR_REG_OFFSET), Port_ConfigPtr->pinsConfgrations[i].pin_num);
        }
        /* check if the pin is output */
        else if (Port_ConfigPtr->pinsConfgrations[i].direction == PORT_PIN_OUTPUT)
        {
            SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIR_REG_OFFSET), Port_ConfigPtr->pinsConfgrations[i].pin_num);
        }
        else
        {
            /* Do Nothing */
        }
    }
}


/************************************************************************************
 * Service Name: Port_GetVersionInfo
 * Service ID[hex]: 0x03
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): None
 * Parameters (inout): None
 * Parameters (out): versioninfo - Pointer to where to store the version information of this module.
 * Return value: None
 * Description: Returns the version information of this module.
 ************************************************************************************/
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo)
{
          /* check det errors */
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* Check if pointer is Null pointer */
    if (NULL_PTR == versioninfo)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_GET_VERSION_INFO_SID, PORT_E_PARAM_POINTER);
    }
    else
#endif
    {
        /* get vendor Id */
        versioninfo->vendorID = (uint16)PORT_VENDOR_ID;
        /* get module Id */
        versioninfo->moduleID = (uint16)PORT_MODULE_ID;
        /* get Software Major Version */
        versioninfo->sw_major_version = (uint8)PORT_SW_MAJOR_VERSION;
        /* get Software Minor Version */
        versioninfo->sw_minor_version = (uint8)PORT_SW_MINOR_VERSION;
        /* get Software Patch Version */
        versioninfo->sw_patch_version = (uint8)PORT_SW_PATCH_VERSION;
    }
}
#endif

/************************************************************************************
 * Service Name: Port_SetPinMode
 * Service ID[hex]: 0x04
 * Sync/Async: Synchronous
 * Reentrancy: Reentrant
 * Parameters (in): Pin - Port Pin ID number, Mode - New Port Pin mode to be set on port pin
 * Parameters (inout): None
 * Parameters (out): versioninfo - Pointer to where to store the version information of this module.
 * Return value: None
 * Description: Sets the port pin mode.
 ************************************************************************************/
#if (PORT_SET_PIN_MODE_API == STD_ON)
void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode)
{
         /* check det errors */
#if (PORT_DEV_ERROR_DETECT == STD_ON)
    /* check it the port is not initialized */
    if (Port_Status == PORT_NOT_INITIALIZED)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_UNINIT);
    }
    else
    {
        /* Do nothing */
    }
    /* check if the pin in the valid range */
    if (Port_ConfigPtr->pinsConfgrations[Pin].pin_num < 0 || Port_ConfigPtr->pinsConfgrations[Pin].pin_num >= PORT_PIN_NUMBERS)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_PARAM_PIN);
    }
    else
    {
        /* Do nothing */
    }
    /* check if the pin mode is unchangable */
    if (Port_ConfigPtr->pinsConfgrations[Pin].mode_changeability == MODE_NOT_CHANGABLE)
    {
        Det_ReportError(PORT_MODULE_ID, PORT_INSTANCE_ID, PORT_SET_PIN_MODE_SID, PORT_E_MODE_UNCHANGEABLE);
    }
    else
    {
        /* Do nothing */
    }
#endif

    /* if the pin is JTAG pin, then end the function */
    if ((Port_ConfigPtr->pinsConfgrations[Pin].port_num == PORTC_ID) && (Port_ConfigPtr->pinsConfgrations[Pin].pin_num <= PIN3_ID))
    {
        return;
    }
    else
    {
        /* Do nothing */
    }

    volatile uint32 *Port_Ptr = NULL_PTR;
    switch (Port_ConfigPtr->pinsConfgrations[Pin].pin_num)
    {
    case PORTA_ID:
        Port_Ptr = (volatile uint32 *)GPIO_PORTA_BASE_ADDRESS;
        break;
    case PORTB_ID:
        Port_Ptr = (volatile uint32 *)GPIO_PORTB_BASE_ADDRESS;
        break;
    case PORTC_ID:
        Port_Ptr = (volatile uint32 *)GPIO_PORTC_BASE_ADDRESS;
        break;
    case PORTD_ID:
        Port_Ptr = (volatile uint32 *)GPIO_PORTD_BASE_ADDRESS;
        break;
    case PORTE_ID:
        Port_Ptr = (volatile uint32 *)GPIO_PORTE_BASE_ADDRESS;
        break;
    case PORTF_ID:
        Port_Ptr = (volatile uint32 *)GPIO_PORTF_BASE_ADDRESS;
        break;
    }

    /* Analog Mode */
    if (Mode == Analog_mode)
    {
        /* Clear the corresponding bit in the digital enable register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_ConfigPtr->pinsConfgrations[Pin].pin_num);

        /* Set the corresponding bit in the analog mode select register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_ConfigPtr->pinsConfgrations[Pin].pin_num);
    }
    /* Digital Mode */
    else
    {
        /* Set the corresponding bit in the digital enable register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_DIGITAL_ENABLE_REG_OFFSET), Port_ConfigPtr->pinsConfgrations[Pin].pin_num);

        /* Clear the corresponding bit in the analog mode select register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ANALOG_MODE_SEL_REG_OFFSET), Port_ConfigPtr->pinsConfgrations[Pin].pin_num);
    }

    /* DIO mode */
    if (Mode == GPIO_mode)
    {
        /* Clear the corresponding bit in the alternate function register */
        CLEAR_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_ConfigPtr->pinsConfgrations[Pin].pin_num);

        /* Clear PMCx bits in port control register */
        *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) &= ~(PMCx_MASK << (Port_ConfigPtr->pinsConfgrations[Pin].pin_num * 4));
    }
    /* alternate mode */
    else
    {
        /* Set the corresponding bit in the alternate function register */
        SET_BIT(*(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_ALT_FUNC_REG_OFFSET), Port_ConfigPtr->pinsConfgrations[Pin].pin_num);

        /* Clear all PMCx bits then set them in the control register according to the required mode */
        *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) &= ~(PMCx_MASK << (Port_ConfigPtr->pinsConfgrations[Pin].pin_num * 4));
        *(volatile uint32 *)((volatile uint8 *)Port_Ptr + PORT_CTL_REG_OFFSET) |= ((Mode & PMCx_MASK) << (Port_ConfigPtr->pinsConfgrations[Pin].pin_num * 4));
    }
}
#endif
