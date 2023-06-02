/*
 * USART_Test.c
 *
 *  Created on: 14-Jul-2021
 *      Author: vermas
 */
/* Description : File used to Initialized the FPU(Floating point unit)
 */

//enable the FPU
/* *********************************Documentation Section **********************************************
 * @func                         : Enable_FPU
 * @brief                        : Function that enable the Floating point unit.
 * @return                       : None
 * Special Note                  : This is pure assembly function with no epilogue and prologue (indicated by __attribute__((naked)) )
 *                                 When function gets called then setting up of stack frame is done by epilogue.
 *                                 Wnen function returs then its restoring its stack frame of calling (Parent) function.
 *                                 Here programmer is responsible for whatever happens and that's why it need to return to valid
 *                                 address.
 *                                 volatile keyword is used to fetch the data from memory instead of accessing from temporary register.
 *                                 Refer to page 4-52 in Cortex M4 generic user guide.
 *
 */

__attribute__((naked)) volatile void Enable_FPU(void)
{
         //Giving the access privileges for coprocessor
	//Step1 : load the address of CPACR (Coprocessor Access Control Register) into R0
	__asm volatile ("LDR.W R0, =0xE000ED88");
        //Step2 : Write the value at CPACR address to R1 Register
	__asm volatile ("LDR R1, [R0]");
        //Step3 : Set the bit from 20 to 23 to enable the Full access
	__asm volatile ("ORR R1, R1, #(0xF << 20)");
        //Step4 : Writing the value back to value at CPACR register address
	__asm volatile ("STR R1, [R0]");

	//Step5: Enable the memory access order
	__asm volatile ("DSB");
	__asm volatile ("ISB");
        //Step6 : Jump to calling function
	__asm volatile ("BLX LR");
}
