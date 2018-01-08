;/*******************************************************************************
;*                                                                              *
;*   Abstract    : This file contains interrupt vector and startup code.        *
;*                                                                              *
;*   Functions   : Reset_Handler                                                *
;*                                                                              *
;*   Target      : Cypress FM microcontrollers                                  *
;*                                                                              *
;*   Environment : IAR Embedded Workbench                                       *
;*                                                                              *
;*   Distribution: The file is distributed "as is," without any warranty        *
;*                 of any kind.                                                 *
;*                                                                              *
;*******************************************************************************/

;
;Copyright (C) 2013-2017, Fujitsu Electronics Europe GmbH or a               
;subsidiary of Fujitsu Electronics Europe GmbH.  All rights reserved.        
;                                                                            
;This software, including source code, documentation and related             
;materials ("Software"), is owned by Fujitsu Electronics Europe GmbH or      
;one of its subsidiaries ("Fujitsu").
;                                                                            
;If no EULA applies, Fujitsu hereby grants you a personal, non-exclusive,    
;non-transferable license to copy, modify, and compile the                   
;Software source code solely for use in connection with Fujitsu's            
;integrated circuit products.  Any reproduction, modification, translation,  
;compilation, or representation of this Software except as specified         
;above is prohibited without the express written permission of Fujitsu.      
;                                                                            
;Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO                        
;WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,                        
;BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED                                
;WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A                             
;PARTICULAR PURPOSE. Fujitsu reserves the right to make                      
;changes to the Software without notice. Fujitsu does not assume any         
;liability arising out of the application or use of the Software or any      
;product or circuit described in the Software. Fujitsu does not              
;authorize its products for use in any products where a malfunction or       
;failure of the Fujitsu product may reasonably be expected to result in      
;significant property damage, injury or death ("High Risk Product"). By      
;including Fujitsu's product in a High Risk Product, the manufacturer        
;of such system or application assumes all risk of such use and in doing     
;so agrees to indemnify Fujitsu against all liability.                       
;

;/*****************************************************************************/
;/*  Startup for IAR                                                          */
;/*  Version     V1.0                                                         */
;/*  Date        2013-03-22                                                   */
;/*****************************************************************************/


                MODULE  ?cstartup

                ;; Forward declaration of sections.
                SECTION CSTACK:DATA:NOROOT(3)

                SECTION .intvec:CODE:NOROOT(2)

                EXTERN  __iar_program_start
                EXTERN  SystemInit
                PUBLIC  __vector_table

                DATA
__vector_table  DCD     sfe(CSTACK)               ; Top of Stack
		        DCD     Reset_Handler             ; Reset
                DCD     NMI_Handler               ; NMI
                DCD     HardFault_Handler         ; Hard Fault
                DCD     MemManage_Handler         ; MPU Fault
                DCD     BusFault_Handler          ; Bus Fault
                DCD     UsageFault_Handler        ; Usage Fault
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall
                DCD     DebugMon_Handler          ; Debug Monitor
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV
                DCD     SysTick_Handler           ; SysTick

; Numbered IRQ handler vectors

; Note: renaming to device dependent ISR function names are done in
;       pdl.h (section "IRQ name definition for all type MCUs"

                DCD     BLE_WAKEUP_LP_Handler            				
                DCD     BLE_FINETGTIM_Handler     
                DCD     BLE_GROSSTGTIM_Handler    
                DCD     BLE_CSCNT_Handler          
                DCD     BLE_SLP_Handler 
                DCD     BLE_ERROR_Handler 
                DCD     BLE_RX_Handler
                DCD     BLE_EVENT_Handler	                
                DCD     SWTIM_Handler
                DCD     WKUP_QUADEC_Handler	
                DCD     BLE_RF_DIAG_Handler	
                DCD     BLE_CRYPT_Handler	
                DCD     UART_Handler		
                DCD     UART2_Handler    
                DCD     I2C_Handler    
                DCD     SPI_Handler    
                DCD     ADC_Handler    
                DCD     KEYBRD_Handler    
                DCD     RFCAL_Handler    
                DCD     GPIO0_Handler
                DCD     GPIO1_Handler
                DCD     GPIO2_Handler
                DCD     GPIO3_Handler
                DCD     GPIO4_Handler


                THUMB
; Dummy Exception Handlers (infinite loops which can be modified)

                PUBWEAK Reset_Handler
                SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__iar_program_start
                BX      R0
                
                PUBWEAK NMI_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
                B       NMI_Handler

                PUBWEAK HardFault_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
                B       HardFault_Handler

                PUBWEAK MemManage_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
MemManage_Handler
                B       MemManage_Handler

                PUBWEAK BusFault_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
BusFault_Handler
                B       BusFault_Handler

                PUBWEAK UsageFault_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
UsageFault_Handler
                B       UsageFault_Handler

                PUBWEAK SVC_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
                B       SVC_Handler

                PUBWEAK DebugMon_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
DebugMon_Handler
                B       DebugMon_Handler

                PUBWEAK PendSV_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
                B       PendSV_Handler

                PUBWEAK SysTick_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
                B       SysTick_Handler

                
                PUBWEAK BLE_WAKEUP_LP_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
BLE_WAKEUP_LP_Handler
                B       BLE_WAKEUP_LP_Handler

                PUBWEAK BLE_FINETGTIM_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
BLE_FINETGTIM_Handler
                B       BLE_FINETGTIM_Handler                

                PUBWEAK BLE_GROSSTGTIM_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
BLE_GROSSTGTIM_Handler
                B       BLE_GROSSTGTIM_Handler      

                PUBWEAK BLE_CSCNT_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
BLE_CSCNT_Handler
                B       BLE_CSCNT_Handler
                
                PUBWEAK BLE_SLP_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
BLE_SLP_Handler
                B       BLE_SLP_Handler
                
                PUBWEAK BLE_ERROR_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
BLE_ERROR_Handler
                B       BLE_ERROR_Handler
                
                PUBWEAK BLE_RX_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
BLE_RX_Handler
                B       BLE_RX_Handler
                
                PUBWEAK BLE_EVENT_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
BLE_EVENT_Handler
                B       BLE_EVENT_Handler
                
                PUBWEAK SWTIM_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
SWTIM_Handler
                B       SWTIM_Handler
                
                PUBWEAK WKUP_QUADEC_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
WKUP_QUADEC_Handler
                B       WKUP_QUADEC_Handler

                PUBWEAK BLE_RF_DIAG_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
BLE_RF_DIAG_Handler
                B       BLE_RF_DIAG_Handler

                PUBWEAK BLE_CRYPT_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
BLE_CRYPT_Handler
                B       BLE_CRYPT_Handler

                PUBWEAK UART_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
UART_Handler
                B       UART_Handler

                PUBWEAK UART2_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
UART2_Handler
                B       UART2_Handler

                PUBWEAK I2C_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
I2C_Handler
                B       I2C_Handler

                PUBWEAK SPI_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
SPI_Handler
                B       SPI_Handler

                PUBWEAK ADC_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
ADC_Handler
                B       ADC_Handler

                PUBWEAK KEYBRD_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
KEYBRD_Handler
                B       KEYBRD_Handler

                PUBWEAK RFCAL_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
RFCAL_Handler
                B       RFCAL_Handler

                PUBWEAK GPIO0_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
GPIO0_Handler
                B       GPIO0_Handler

                PUBWEAK GPIO1_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
GPIO1_Handler
                B       GPIO1_Handler

                PUBWEAK GPIO2_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
GPIO2_Handler
                B       GPIO2_Handler

                PUBWEAK GPIO3_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
GPIO3_Handler
                B       GPIO3_Handler

                PUBWEAK GPIO4_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
GPIO4_Handler
                B       GPIO4_Handler
                
                PUBWEAK Dummy
                SECTION .text:CODE:REORDER:NOROOT(1)
Dummy
                B       Dummy

                END
