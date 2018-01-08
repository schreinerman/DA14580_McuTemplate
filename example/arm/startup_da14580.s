;/*******************************************************************************
;*                                                                              *
;*   Abstract    : This file contains interrupt vector and startup code.        *
;*                                                                              *
;*   Functions   : Reset_Handler                                                *
;*                                                                              *
;*   Target      : microcontrollers                                             *
;*                                                                              *
;*   Environment : KEIL µVISION	                                                *
;*                                                                              *
;*   Distribution: The file is distributed "as is," without any warranty        *
;*                 of any kind.                                                 *
;*                                                                              *
;********************************************************************************
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

; Stack Configuration
;  Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; Heap Configuration
;  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>

Heap_Size       EQU     0x00000800

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

; Numbered IRQ handler vectors				
				
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


__Vectors_End

__Vectors_Size 	EQU 	__Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

;remap 
                IF      :DEF:__REMAP_SYSRAM
                LDR     R0, =0x0
                LDR     R1, [R0]
                LDR     R0, =0x20000000
                LDR     R2, [R0]
                CMP     R2, R1
                BEQ     remap_done
                LDR     R0, =0x50000012
                LDRH    R1, [R0]
                LSRS    R2, R1, #2
                LSLS    R1, R2, #2
                MOVS    R2, #0x2
                ADDS    R1, R1, R2            ;remap SYSRAM to 0
                LSLS    R2, R2, #14
                ADDS    R1, R1, R2         ;SW RESET
                STRH    R1, [R0]
remap_done                
                ENDIF
                    
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC
                EXPORT BLE_WAKEUP_LP_Handler   [WEAK]
                EXPORT BLE_FINETGTIM_Handler   [WEAK]
                EXPORT BLE_GROSSTGTIM_Handler  [WEAK]
                EXPORT BLE_CSCNT_Handler       [WEAK]
                EXPORT BLE_SLP_Handler         [WEAK]
                EXPORT BLE_ERROR_Handler       [WEAK]
                EXPORT BLE_RX_Handler          [WEAK]
                EXPORT BLE_EVENT_Handler	   [WEAK]
                EXPORT SWTIM_Handler           [WEAK]
                EXPORT WKUP_QUADEC_Handler     [WEAK]
                EXPORT BLE_RF_DIAG_Handler     [WEAK]
		EXPORT BLE_CRYPT_Handler	   [WEAK]
                EXPORT UART_Handler		       [WEAK]
                EXPORT UART2_Handler           [WEAK]
                EXPORT I2C_Handler             [WEAK]
                EXPORT SPI_Handler             [WEAK]
                EXPORT ADC_Handler             [WEAK]
                EXPORT KEYBRD_Handler          [WEAK]
                EXPORT RFCAL_Handler           [WEAK]
                EXPORT GPIO0_Handler           [WEAK]
                EXPORT GPIO1_Handler           [WEAK]
                EXPORT GPIO2_Handler           [WEAK]
                EXPORT GPIO3_Handler           [WEAK]
                EXPORT GPIO4_Handler           [WEAK]

                
                EXPORT  Dummy	          [WEAK]

BLE_WAKEUP_LP_Handler
BLE_FINETGTIM_Handler     
BLE_GROSSTGTIM_Handler    
BLE_CSCNT_Handler          
BLE_SLP_Handler 
BLE_ERROR_Handler 
BLE_RX_Handler
BLE_EVENT_Handler	
SWTIM_Handler
WKUP_QUADEC_Handler 	
BLE_RF_DIAG_Handler 
BLE_CRYPT_Handler	
UART_Handler		
UART2_Handler    
I2C_Handler    
SPI_Handler    
ADC_Handler    
KEYBRD_Handler    
RFCAL_Handler    
GPIO0_Handler
GPIO1_Handler
GPIO2_Handler
GPIO3_Handler
GPIO4_Handler

Dummy


                B       .

                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB
                
                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit
                
                ELSE
                
                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, = Heap_Mem
                LDR     R1, = (Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem + Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF


                END
