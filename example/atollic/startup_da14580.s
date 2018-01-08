/*******************************************************************************
*                                                                              *
*   Abstract    : This file contains interrupt vector and startup code.        *
*                                                                              *
*   Functions   : Reset_Handler                                                *
*                                                                              *
*   Target      : microcontrollers                                  *
*                                                                              *
*   Environment : Atollic TrueSTUDIO(R)                                        *
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

  .syntax unified
  .thumb

  .global Reset_Handler
  .global InterruptVector
  .global Default_Handler

  /* Linker script definitions */
  /* start address for the initialization values of the .data section */
  .word _sidata
  /* start address for the .data section */
  .word _sdata
  /* end address for the .data section */
  .word _edata
  /* start address for the .bss section */
  .word _sbss
  /* end address for the .bss section */
  .word _ebss

/**
**===========================================================================
**  Program - Reset_Handler
**  Abstract: This code gets called after reset.
**===========================================================================
*/
  .section  .text.Reset_Handler,"ax", %progbits
  .type Reset_Handler, %function
Reset_Handler:
  /* Set stack pointer */
  ldr r0,=_estack
  mov sp, r0
  
  
  /* Branch to SystemInit function */
  bl SystemInit

  /* Copy data initialization values */
  ldr r1,=_sidata
  ldr r2,=_sdata
  ldr r3,=_edata
  b cmpdata
CopyLoop:
  ldr r0, [r1]
  adds r1, r1, #4
  str r0, [r2]
  adds r2, r2, #4
cmpdata:
  cmp r2, r3
  blt CopyLoop

  /* Clear BSS section */
  movs r0, #0
  ldr r2,=_sbss
  ldr r3,=_ebss
  b cmpbss
ClearLoop:
  str r0, [r2]
  adds r2, r2, #4
cmpbss:
  cmp r2, r3
  blt ClearLoop

  /* Call static constructors */
  bl __libc_init_array

  /* Branch to main */
  bl main

  /* If main returns, branch to Default_Handler. */
  b Default_Handler

  .size  Reset_Handler, .-Reset_Handler

/**
**===========================================================================
**  Program - Default_Handler
**  Abstract: This code gets called when the processor receives an
**    unexpected interrupt.
**===========================================================================
*/
  .section  .text.Default_Handler,"ax", %progbits
Default_Handler:
  b  Default_Handler

  .size  Default_Handler, .-Default_Handler

/**
**===========================================================================
**  Interrupt vector table
**===========================================================================
*/
  .section .isr_vector,"a", %progbits
InterruptVector:
  .word _estack                   /* 0 - Stack pointer */
  .word Reset_Handler             /* 1 - Reset */
  .word NMI_Handler               /* 2 - NMI  */
  .word HardFault_Handler         /* 3 - Hard fault */
  .word MemManage_Handler         /* 4 - Memory management fault */
  .word BusFault_Handler          /* 5 - Bus fault */
  .word UsageFault_Handler        /* 6 - Usage fault */
  .word 0                         /* 7 - Reserved */
  .word 0                         /* 8 - Reserved */
  .word 0                         /* 9 - Reserved */
  .word 0                         /* 10 - Reserved */
  .word SVC_Handler               /* 11 - SVCall */
  .word DebugMonitor_Handler      /* 12 - Reserved for Debug */
  .word 0                         /* 13 - Reserved */
  .word PendSV_Handler            /* 14 - PendSV */
  .word SysTick_Handler           /* 15 - Systick */

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


/**
**===========================================================================
**  Weak interrupt handlers redirected to Default_Handler. These can be
**  overridden in user code.
**===========================================================================
*/
  .weak NMI_Handler
  .thumb_set NMI_Handler, Default_Handler

  .weak HardFault_Handler
  .thumb_set HardFault_Handler, Default_Handler

  .weak MemManage_Handler
  .thumb_set MemManage_Handler, Default_Handler

  .weak BusFault_Handler
  .thumb_set BusFault_Handler, Default_Handler

  .weak UsageFault_Handler
  .thumb_set UsageFault_Handler, Default_Handler

  .weak SVC_Handler
  .thumb_set SVC_Handler, Default_Handler

  .weak DebugMonitor_Handler
  .thumb_set DebugMonitor_Handler, Default_Handler

  .weak PendSV_Handler
  .thumb_set PendSV_Handler, Default_Handler

  .weak SysTick_Handler
  .thumb_set SysTick_Handler, Default_Handler
  
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

  .end
