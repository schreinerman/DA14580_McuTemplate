/*******************************************************************************
*                                                                              * 
*   Abstract    : This file contains interrupt vector and startup code.        *
*                                                                              *
*   Functions   : Reset_Handler                                                *
*                                                                              *
*   Target      : microcontrollers                                                   *
*                                                                              * 
*   Environment : GNU Toolchain(Make)                                          *
*                                                                              *
*   Distribution: The file is distributed "as is," without any warranty        *
*                 of any kind.                                                 *
*                                                                              *
*******************************************************************************/

/*
Copyright (C) 2013-2017, Fujitsu Electronics Europe GmbH or a               
subsidiary of Fujitsu Electronics Europe GmbH.  All rights reserved.        
                                                                            
This software, including source code, documentation and related             
materials ("Software"), is owned by Fujitsu Electronics Europe GmbH or      
one of its subsidiaries ("Fujitsu").
                                                                            
If no EULA applies, Fujitsu hereby grants you a personal, non-exclusive,    
non-transferable license to copy, modify, and compile the                   
Software source code solely for use in connection with Fujitsu's            
integrated circuit products.  Any reproduction, modification, translation,  
compilation, or representation of this Software except as specified         
above is prohibited without the express written permission of Fujitsu.      
                                                                            
Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO                        
WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,                        
BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED                                
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A                             
PARTICULAR PURPOSE. Fujitsu reserves the right to make                      
changes to the Software without notice. Fujitsu does not assume any         
liability arising out of the application or use of the Software or any      
product or circuit described in the Software. Fujitsu does not              
authorize its products for use in any products where a malfunction or       
failure of the Fujitsu product may reasonably be expected to result in      
significant property damage, injury or death ("High Risk Product"). By      
including Fujitsu's product in a High Risk Product, the manufacturer        
of such system or application assumes all risk of such use and in doing     
so agrees to indemnify Fujitsu against all liability.                       

*/


/*
// <h> Stack Configuration
//   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
// </h>
*/

 .equ    Stack_Size, 0x00000400
    .section ".stack", "w"
    .align  3
    .globl  __FM_stack_mem
    .globl  __FM_stack_size
__MCU_stack_mem:
    .if     Stack_Size
    .space  Stack_Size
    .endif
    .size   __MCU_stack_mem,  . - __MCU_stack_mem
    .set    __MCU_stack_size, . - __MCU_stack_mem


/*
// <h> Heap Configuration
//   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
// </h>
*/

    .equ    Heap_Size,  0x00000800
    
    .section ".heap", "w"
    .align  3
    .globl  __MCU_heap_start
    .globl  __MCU_heap_end
__MCU_heap_start:
    .if     Heap_Size
    .space  Heap_Size
    .endif
__MCU_heap_end:


/* Vector Table */

    .section ".MCU.interrupt_vector"
    .globl  __MCU_interrupt_vector_cortex_m
    .type   __MCU_interrupt_vector_cortex_m, %object

__MCU_interrupt_vector_cortex_m:
    .long   __MCU_stack                 /* Top of Stack                 */
    .long   __MCU_reset                 /* Reset Handler                */
    .long   NMI_Handler                 /* NMI Handler                  */
    .long   HardFault_Handler           /* Hard Fault Handler           */
    .long   MemManage_Handler           /* MPU Fault Handler            */
    .long   BusFault_Handler            /* Bus Fault Handler            */
    .long   UsageFault_Handler          /* Usage Fault Handler          */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   SVC_Handler                 /* SVCall Handler               */
    .long   DebugMon_Handler            /* Debug Monitor Handler        */
    .long   0                           /* Reserved                     */
    .long   PendSV_Handler              /* PendSV Handler               */
    .long   SysTick_Handler             /* SysTick Handler              */

    /* External Interrupts */


	.long   BLE_WAKEUP_LP_Handler            				
	.long   BLE_FINETGTIM_Handler     
	.long   BLE_GROSSTGTIM_Handler    
	.long   BLE_CSCNT_Handler          
	.long   BLE_SLP_Handler 
	.long   BLE_ERROR_Handler 
	.long   BLE_RX_Handler
	.long   BLE_EVENT_Handler	                
	.long   SWTIM_Handler
	.long   WKUP_QUADEC_Handler	
	.long   BLE_RF_DIAG_Handler	
	.long   BLE_CRYPT_Handler	
	.long   UART_Handler		
	.long   UART2_Handler    
	.long   I2C_Handler    
	.long   SPI_Handler    
	.long   ADC_Handler    
	.long   KEYBRD_Handler    
	.long   RFCAL_Handler    
	.long   GPIO0_Handler
	.long   GPIO1_Handler
	.long   GPIO2_Handler
	.long   GPIO3_Handler
	.long   GPIO4_Handler
    .size   __MCU_interrupt_vector_cortex_m, . - __MCU_interrupt_vector_cortex_m


    .thumb


/* Reset Handler */

    .section .MCU.reset,"x",%progbits
    .thumb_func
    .globl  __MCU_reset_cortex_m
    .type   __MCU_reset_cortex_m, %function
__MCU_reset_cortex_m:
    .fnstart
.ifdef Debug_RAM
/* this routine is used for .bss sections initialization by RAM debugging "Debug_RAM = 1" */
/* Clear .bss section (Zero init) */
 MOV     R0, #0
 LDR     R1, =_sbss
 LDR     R2, =_ebss
 CMP     R1,R2
 BEQ     start_main
Loop_bss:
 CMP     R1, R2
 BHS  start_main
 STR    R0, [R1]
 ADD  R1, #4
 BLO     Loop_bss

.else
/* this routine is used for .data and .bss sections initialization by ROM debugging "Debug_RAM = 0" */
init_data:
 LDR  R0, =_sdata
 LDR  R1, =_etext
 LDR  R2, =data_size
 MOV  R3, #0
loop_init_data:
 CMP  R3, R2
 BCS  zero_bss
    LDR  R4,[R1,#0]
 STR  R4,[R0,#0]
 ADD  R3, #4
 ADD  R1, #4
 ADD  R0, #4
 B  loop_init_data

/* Clear .bss section (Zero init) */

zero_bss:
 LDR  R0, =_sbss
 LDR  R2, =bss_size
 MOV  R3, #0
 MOV  R4, #0
loop_zero_bss:
 CMP  R3, R2
 BCS  start_main
 STR  R4,[R0,#0]
 ADD  R3, #4
 ADD  R0, #4
 B  loop_zero_bss
.endif
start_main:

  /*enable fpu begin*/
  /*; enable cp10,cp11 */
  
  
  
  /*enable fpu end*/ 
  
    LDR     R0, =SystemInit
    BLX     R0
    LDR     R0,=main
    BX      R0

    .pool
    .cantunwind
    .fnend
    .size   __MCU_reset_cortex_m,.-__MCU_reset_cortex_m

    .section ".text"

/* Exception Handlers */

    .weak   NMI_Handler
    .type   NMI_Handler, %function
NMI_Handler:
    B       .
    .size   NMI_Handler, . - NMI_Handler

    .weak   HardFault_Handler
    .type   HardFault_Handler, %function
HardFault_Handler:
    B       .
    .size   HardFault_Handler, . - HardFault_Handler

    .weak   MemManage_Handler
    .type   MemManage_Handler, %function
MemManage_Handler:
    B       .
    .size   MemManage_Handler, . - MemManage_Handler

    .weak   BusFault_Handler
    .type   BusFault_Handler, %function
BusFault_Handler:
    B       .
    .size   BusFault_Handler, . - BusFault_Handler

    .weak   UsageFault_Handler
    .type   UsageFault_Handler, %function
UsageFault_Handler:
    B       .
    .size   UsageFault_Handler, . - UsageFault_Handler

    .weak   SVC_Handler
    .type   SVC_Handler, %function
SVC_Handler:
    B       .
    .size   SVC_Handler, . - SVC_Handler

    .weak   DebugMon_Handler
    .type   DebugMon_Handler, %function
DebugMon_Handler:
    B       .
    .size   DebugMon_Handler, . - DebugMon_Handler

    .weak   PendSV_Handler
    .type   PendSV_Handler, %function
PendSV_Handler:
    B       .
    .size   PendSV_Handler, . - PendSV_Handler

    .weak   SysTick_Handler
    .type   SysTick_Handler, %function
SysTick_Handler:
    B       .
    .size   SysTick_Handler, . - SysTick_Handler


.weak BLE_WAKEUP_LP_Handler
  .thumb_set BLE_WAKEUP_LP_Handler, Default_Handler

  .weak BLE_FINETGTIM_Handler
  .thumb_set BLE_FINETGTIM_Handler, Default_Handler

  .weak BLE_GROSSTGTIM_Handler
  .thumb_set BLE_GROSSTGTIM_Handler, Default_Handler

  .weak BLE_CSCNT_Handler
  .thumb_set BLE_CSCNT_Handler, Default_Handler

  .weak BLE_SLP_Handler
  .thumb_set BLE_SLP_Handler, Default_Handler

  .weak BLE_ERROR_Handler
  .thumb_set BLE_ERROR_Handler, Default_Handler

  .weak BLE_RX_Handler
  .thumb_set BLE_RX_Handler, Default_Handler

  .weak BLE_EVENT_Handler
  .thumb_set BLE_EVENT_Handler, Default_Handler

  .weak SWTIM_Handler
  .thumb_set SWTIM_Handler, Default_Handler

  .weak WKUP_QUADEC_Handler
  .thumb_set WKUP_QUADEC_Handler, Default_Handler

  .weak BLE_RF_DIAG_Handler
  .thumb_set BLE_RF_DIAG_Handler, Default_Handler

  .weak BLE_CRYPT_Handler
  .thumb_set BLE_CRYPT_Handler, Default_Handler

  .weak UART_Handler
  .thumb_set UART_Handler, Default_Handler

  .weak UART2_Handler
  .thumb_set UART2_Handler, Default_Handler

  .weak I2C_Handler
  .thumb_set I2C_Handler, Default_Handler

  .weak SPI_Handler
  .thumb_set SPI_Handler, Default_Handler

  .weak ADC_Handler
  .thumb_set ADC_Handler, Default_Handler

  .weak KEYBRD_Handler
  .thumb_set KEYBRD_Handler, Default_Handler

  .weak RFCAL_Handler
  .thumb_set RFCAL_Handler, Default_Handler

  .weak GPIO0_Handler
  .thumb_set GPIO0_Handler, Default_Handler

  .weak GPIO1_Handler
  .thumb_set GPIO1_Handler, Default_Handler

  .weak GPIO2_Handler
  .thumb_set GPIO2_Handler, Default_Handler

  .weak GPIO3_Handler
  .thumb_set GPIO3_Handler, Default_Handler

  .weak GPIO4_Handler
  .thumb_set GPIO4_Handler, Default_Handler

/* IRQ Handlers */

    .globl  Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    B       .
    .size   Default_Handler, . - Default_Handler

    .macro  IRQ handler
    .weak   \handler
    .set    \handler, Default_Handler
    .endm



    .end
