 /******************************************************************************
 *
 * Module: Port
 *
 * File Name: Port.h
 *
 * Description: Header file for TM4C123GH6PM Microcontroller - Port Driver.
 *
 * Author: Saeed elsayed
 ******************************************************************************/

#ifndef PORT_H
#define PORT_H

/* Id for the company in the AUTOSAR */
#define PORT_VENDOR_ID    (1000U)

/* Dio Module Id */
#define PORT_MODULE_ID    (120U)

/* Dio Instance Id */
#define PORT_INSTANCE_ID  (0U)

/*
 * Module Version 1.0.0
 */
#define PORT_SW_MAJOR_VERSION           (1U)
#define PORT_SW_MINOR_VERSION           (0U)
#define PORT_SW_PATCH_VERSION           (0U)

/*
 * AUTOSAR Version 4.0.3
 */
#define PORT_AR_RELEASE_MAJOR_VERSION   (4U)
#define PORT_AR_RELEASE_MINOR_VERSION   (0U)
#define PORT_AR_RELEASE_PATCH_VERSION   (3U)

/*
 * Macros for PORT Status
 */
#define PORT_INITIALIZED                (1U)
#define PORT_NOT_INITIALIZED            (0U)

#include "Common_Macros.h"
#include "Std_Types.h"

/* AUTOSAR checking between Std Types and Port Modules */
#if ((STD_TYPES_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (STD_TYPES_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Std_Types.h does not match the expected version"
#endif
   
/* Port Pre-Compile Configuration Header file */
#include "Port_Cfg.h"

/* AUTOSAR Version checking between Port_Cfg.h and Port.h files */
#if ((PORT_CFG_AR_RELEASE_MAJOR_VERSION != PORT_AR_RELEASE_MAJOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_MINOR_VERSION != PORT_AR_RELEASE_MINOR_VERSION)\
 ||  (PORT_CFG_AR_RELEASE_PATCH_VERSION != PORT_AR_RELEASE_PATCH_VERSION))
  #error "The AR version of Port_Cfg.h does not match the expected version"
#endif   
   
/* Software Version checking between Port_Cfg.h and Port.h files */
#if ((Port_CFG_SW_MAJOR_VERSION != Port_SW_MAJOR_VERSION)\
 ||  (Port_CFG_SW_MINOR_VERSION != Port_SW_MINOR_VERSION)\
 ||  (Port_CFG_SW_PATCH_VERSION != Port_SW_PATCH_VERSION))
  #error "The SW version of Port_Cfg.h does not match the expected version"
#endif   
   
/******************************************************************************
 *                      API Service Id Macros                                 *
 ******************************************************************************/

/* Service ID for Port Init */   
#define PORT_INIT_SID                       (uint8)0x00   

/* Service ID for Port Set Pin Direction */
#define PORT_SET_PIN_DIRECTION_SID          (uint8)0x01   
   
/* Service ID for Port Refresh Port Direction */
#define PORT_REFRESH_PORT_DIRECTION_SID     (uint8)0x02

/* Service ID for Port Get Version Info */
#define PORT_GET_VERSION_INFO_SID           (uint8)0x03

/* Service ID for Port Set Pin Mode */
#define PORT_SET_PIN_MODE_SID               (uint8)0x04
   
/*******************************************************************************
 *                      DET Error Codes                                        *
 *******************************************************************************/

/* DET code to report Invalid Port Pin */   
#define PORT_E_PARAM_PIN                  (uint8)0x0A   

/* DET code to report Port Pin not configured as changeable  */   
#define PORT_E_DIRECTION_UNCHANGEABLE     (uint8)0x0B
  
/* DET code to report Port API init service is called with wrong parameter */      
#define PORT_E_PARAM_CONFIG               (uint8)0x0C   
   
/* DET code to report Port API Port_SetPinMode service called with invalid mode */   
#define PORT_E_PARAM_INVALID_MODE         (uint8)0x0D   

/* DET code to report Port API Port_SetPinMode service called when mode is unchangable */   
#define PORT_E_MODE_UNCHANGEABLE          (uint8)0x0E   

/* DET code to report Port API service called without module initialization  */     
#define PORT_E_UNINIT                     (uint8)0x0F    

/* DET code to report Port APIs called with null pointers */    
#define PORT_E_PARAM_POINTER              (uint8)0x10    

/*----------------------------------------------------------------------------------*/
                               /* Port Pins ID numbers*/   
   
#define PA0 (Port_PinType)0
#define PA1 (Port_PinType)1
#define PA2 (Port_PinType)2
#define PA3 (Port_PinType)3
#define PA4 (Port_PinType)4
#define PA5 (Port_PinType)5
#define PA6 (Port_PinType)6
#define PA7 (Port_PinType)7

#define PB0 (Port_PinType)8
#define PB1 (Port_PinType)9
#define PB2 (Port_PinType)10
#define PB3 (Port_PinType)11
#define PB4 (Port_PinType)12
#define PB5 (Port_PinType)13
#define PB6 (Port_PinType)14
#define PB7 (Port_PinType)15

#define PC0 (Port_PinType)16
#define PC1 (Port_PinType)17
#define PC2 (Port_PinType)18
#define PC3 (Port_PinType)19
#define PC4 (Port_PinType)20
#define PC5 (Port_PinType)21
#define PC6 (Port_PinType)22
#define PC7 (Port_PinType)23

#define PD0 (Port_PinType)24
#define PD1 (Port_PinType)25
#define PD2 (Port_PinType)26
#define PD3 (Port_PinType)27
#define PD4 (Port_PinType)28
#define PD5 (Port_PinType)29
#define PD6 (Port_PinType)30
#define PD7 (Port_PinType)31

#define PE0 (Port_PinType)32
#define PE1 (Port_PinType)33
#define PE2 (Port_PinType)34
#define PE3 (Port_PinType)35
#define PE4 (Port_PinType)36
#define PE5 (Port_PinType)37

#define PF0 (Port_PinType)38
#define PF1 (Port_PinType)39
#define PF2 (Port_PinType)40
#define PF3 (Port_PinType)41
#define PF4 (Port_PinType)42   
  
/*----------------------------------------------------------------------------------*/   
   

/*******************************************************************************
 *                              Module Data Types                              *
 *******************************************************************************/

/* Description: define the pin used */    
typedef uint8 Port_PinType;   
   
/* Description: Enum to hold PIN direction */
typedef enum
{
    PORT_PIN_INPUT,PORT_PIN_OUTPUT
}Port_PinDirectionType;

/* Description: Enum to hold internal resistor type for PIN */
typedef enum
{
    OFF,PULL_UP,PULL_DOWN
}Port_InternalResistor;

/* choose the level of the port pin */
typedef enum
{
  Port_Pin_STD_LOW, Port_Pin_STD_HIGH
}Port_PinInitialLevel;

/* determine if the pin direction is changable or not */
typedef enum
{
  DIRECTION_NOT_CHANGABLE, DIRECTION_CHANGABLE
}Port_Pin_Direction_Changeability;

/* determine if the pin mode is changable or not */
typedef enum
{
  MODE_NOT_CHANGABLE, MODE_CHANGABLE
}Port_Pin_MODE_Changeability;

/* to choose the pin mode needed */
typedef uint8 Port_PinModeType;

/* Description: Structure to configure each individual PIN:
 *	1. the PORT Which the pin belongs to. 0, 1, 2, 3, 4 or 5
 *	2. the number of the pin in the PORT.
 *      3. the direction of pin --> INPUT or OUTPUT
 *      4. the internal resistor --> Disable, Pull up or Pull down
 */
typedef struct 
{
    uint8 port_num; 
    uint8 pin_num; 
    Port_PinDirectionType direction;
    Port_InternalResistor resistor;
    Port_PinInitialLevel initial_value;
    Port_PinModeType pin_mode;
    Port_Pin_Direction_Changeability direction_changeability;
    Port_Pin_MODE_Changeability mode_changeability;
    
}Port_PinConfigType;

/* array of structures to configure all the pins */
typedef struct
{
  Port_PinConfigType pinsConfgrations[PORT_PIN_NUMBERS];
}Port_ConfigType;

/*******************************************************************************
 *                           Function Prototypes                               *
 *******************************************************************************/
/* function to Initialize the Port Driver module */
void Port_Init(const Port_ConfigType *ConfigPtr);

/* function to set port pin direction */
#if (PORT_SET_PIN_DIRECTION_API == STD_ON)
void Port_SetPinDirection(Port_PinType Pin, Port_PinDirectionType Direction);
#endif

/* function to Refreshe the port direction */
void Port_RefreshPortDirection(void);

/* function to Return version of this module */
#if (PORT_VERSION_INFO_API == STD_ON)
void Port_GetVersionInfo(Std_VersionInfoType *versioninfo);
#endif

/* Sets the port pin mode */
#if (PORT_SET_PIN_MODE == STD_ON)
void Port_SetPinMode(Port_PinType Pin, Port_PinModeType Mode);
#endif

/*******************************************************************************
 *                       External Variables                                    *
 *******************************************************************************/

/* Extern PB structures to be used by Port and other modules */
extern const Port_ConfigType Pins_InitialConfigurations;

#endif /* PORT.H */