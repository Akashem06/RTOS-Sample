/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "scheduler.h"
#include "task.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARM 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void edf_schedule(void) {
	uint32_t min_deadline = UINT32_MAX;
	uint8_t next_task_index = current_task_index;

	for (uint8_t i = 0; i < active_tasks; i++) {
		if (task_list[i].state == TASK_READY && task_list[i].deadline < min_deadline) {
			min_deadline = task_list[i].deadline;
			next_task_index = i;
		}

		// Deadline updating
		if (task_list[i].deadline == system_time) {
			// Prevent dividing by 0
			task_list[i].deadline = (task_list[i].deadline + task_list[i].period) % UINT32_MAX;
		}
	}

	// Update the task state and switch context
	if (next_task_index != current_task_index) {
		task_list[next_task_index].state = TASK_RUNNING;
		current_task_index = next_task_index;
	} else {
		task_list[0].state = TASK_RUNNING; // Idle state
		current_task_index = 0;
	}

#ifdef ARM
	// Set the Process Stack Pointer to the stack pointer of the next
#elif x86
	task_list[current_task_index].taskFunction(NULL);
#endif
}

void SVC_Handler(void) {
	__asm volatile (
		"TST lr, #4 		\n" // Test bit 2 of LR to determine which stack pointer was used
		"ITE EQ 			\n"	// If-Then-Else
		"MRSEQ r0, MSP 		\n"	// If MSP was used, move MSP to R0
		"MRSNE r0, PSP 		\n" // If PSP was used, move PSP to R0
		"B SVC_Handler_Main \n"	// Branch to Handler function with R0 containing stack pointer
	);
}

void SVC_Handler_Main(unsigned int *svc_args) {
	unsigned int svc_number = -1;
	uint32_t min_deadline = UINT32_MAX;

#ifdef ARM
	/*
	 * Stack contains:
	 * r0, r1, r2, r3, r12, r14, LR and xPSR
	 * First argument (r0) is svc_args[0]
	 */

	// Cast the return address to a char pointer and access the byte before
	// cppcheck-suppress ctuArrayIndex
	uintptr_t return_address = (uintptr_t)svc_args[6];
	svc_number = ((char *)return_address)[-2];
#elif x86
	svc_number = *svc_args;
#endif

	switch (svc_number) {
		case RUN_FIRST_TASK_SVC:
			for (uint8_t i = 0; i < active_tasks; i++) {
				if (task_list[i].deadline < min_deadline) {
					min_deadline = task_list[i].deadline;
					current_task_index = i;
				}
			}
			task_list[current_task_index].state = TASK_RUNNING;
			task_list[current_task_index].deadline = (task_list[current_task_index].deadline + task_list[current_task_index].period) % UINT32_MAX;
#ifdef ARM
			__asm(
				"POP {R7}               		 \n"
				"POP {R7}               		 \n"
				"LDR r1, =task_list     		 \n"  // Load address of task_list

				"LDR r2, =current_task_index     \n"  // Load current_task_index address
				"LDR r2, [r2]              		 \n"  // Get the value at current_task_index address
				"LSL r3, r2, #5          		 \n"  // Calculate address of TaskControlBlock: r2 = current_task_index * 2^5 (32 bytes)
				"LSL r12, r2, #3          		 \n"  // Calculate address of TaskControlBlock: r2 = current_task_index * 2^3 (8 bytes)
				"ADD r12, r12, r3                \n"  // current_task_index * (2^5 + 2^3) = current_task_index * 40 [Size of TCB]
				"ADD r12, r1, r12                \n"
				"LDR r0, [r12, #4]        		 \n"  // Load the stackPointer field (offset 4 bytes) from the current TaskControlBlock into r0

				"MOV LR, #0xFFFFFFFD    		 \n"
				"LDMIA R0!,{R4-R11}     		 \n"
				"MSR PSP, R0            		 \n"
				"BX LR                 			 \n"
			);
#elif x86
			task_list[current_task_index].taskFunction(NULL);
#endif

			break;

		case STOP_TASK_SVC:
			// Pend an interrupt to do the context switch
			task_list[current_task_index].state = TASK_BLOCKED;
#ifdef ARM
			_ICSR |= 1 << 28;
			__asm("isb");
#elif x86
// Simulate context switch
#endif

			break;

		default: /* unknown SVC */
			break;
	}
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
//void SVC_Handler(void)
//{
//  /* USER CODE BEGIN SVCall_IRQn 0 */
//
//  /* USER CODE END SVCall_IRQn 0 */
//  /* USER CODE BEGIN SVCall_IRQn 1 */
//
//  /* USER CODE END SVCall_IRQn 1 */
//}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  #ifdef ARM

	// Store the values of r0 and r3 into C variables
	__asm volatile(
	    "MRS r0, PSP                     \n"  // Get current process stack pointer
	    "STMDB r0!, {r4-r11}             \n"  // Save r4-r11 on the process stack
		"STR r0, [r12, #4]               \n"  // Store the new stack pointer in the task's TCB
		"ISB 							 \n"

		// Save PSP into current task's TCB
		"LDR r1, =current_task_index      \n"
		"LDR r1, [r1]                     \n"
		"LDR r2, =task_list               \n"
		"LSL r3, r1, #5                   \n"  // Calculate address of TaskControlBlock: r1 * 32 bytes
		"LSL r12, r1, #3                  \n"  // Calculate address of TaskControlBlock: r1 * 8 bytes
		"ADD r12, r12, r3                 \n"  // r1 * (32 + 8) = r1 * 40 [Size of TCB]
		"ADD r12, r2, r12                 \n"  // task_list + calculated_address
		"STR r0, [r12, #4]                \n"  // Store the new stack pointer in the task's TCB (stackPointer field)

	    "BL edf_schedule                 \n"

		"LDR r1, =task_list     		 \n"  // Load address of task_list

		"LDR r2, =current_task_index     \n"  // Load current_task_index address
		"LDR r2, [r2]              		 \n"  // Get the value at current_task_index address
		"LSL r3, r2, #5          		 \n"  // Calculate address of TaskControlBlock: r2 = current_task_index * 2^5 (32 bytes)
		"LSL r12, r2, #3          		 \n"  // Calculate address of TaskControlBlock: r2 = current_task_index * 2^3 (8 bytes)
		"ADD r12, r12, r3                \n"  // current_task_index * (2^5 + 2^3) = current_task_index * 40 [Size of TCB]
		"ADD r12, r1, r12                 \n"
		"LDR r0, [r12, #4]        		 \n"  // Load the stackPointer field (offset 4 bytes) from the current TaskControlBlock into r0

		// Restore context of the next task
	    "MOV LR, #0xFFFFFFFD             \n"  // Load return address value
	    "LDMIA r0!, {r4-r11}             \n"  // Restore r4-r11 from the new task's stack
	    "MSR PSP, r0                     \n"  // Set PSP to the new task's stack pointer
		"ISB							 \n"
	    "BX LR                           \n"  // Return from exception
	);
	#endif
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  /* USER CODE BEGIN SysTick_IRQn 1 */
  HAL_IncTick();
  system_time = (system_time + 1) % UINT32_MAX;
  bool context_switch_flag = false;
  // Iterate through tasks to find the earliest deadline task
  for (int i = 0; i < active_tasks; i++) {
    if (task_list[i].deadline <= task_list[current_task_index].deadline && task_list[i].state == TASK_READY) {
        context_switch_flag = true;
    }
    if (task_list[i].deadline == system_time) {
	  task_list[i].deadline = (task_list[i].deadline + task_list[i].period) % UINT32_MAX;
	  task_list[i].state = TASK_READY;
    }
  }

  // Check and update the currently running task's deadline if necessary
  // If the next task is different from the current task, switch the task context
  if (context_switch_flag) {
	task_list[current_task_index].state = TASK_READY;
    #ifdef ARM
          _ICSR |= 1 << 28;
          __asm("isb");
    #elif x86
          edf_schedule();
    #endif
      }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
