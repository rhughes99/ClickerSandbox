// Use shared memory to communicate with host
#include <stdint.h>
#include <pru_cfg.h>
#include "resource_table_empty.h"

#define PRU0_DRAM		0x00000			// Offset to DRAM
// Skip first 0x200 byte of DRAM since Makefile allocates
// 0x100 for STACK and 0x100 for HEAP
volatile unsigned int *pru0_dram = (unsigned int *) (PRU0_DRAM + 0x200);

volatile register uint32_t __R30;
volatile register uint32_t __R31;

void main(void)
{
	uint32_t i, code, sample;
	uint32_t LED, IR_IN, TEST;

	// Set I/O constants
	LED   = 0x1<<1;		// P9_29, output
	IR_IN = 0x1<<0;		// P9_31, input
	TEST  = 0x1<<3;		// P9_28, output

	// Turn LED off & TEST=0
	__R30 &= ~(LED | TEST);

	// Clear SYSCFG[STANDBY_INIT] to enable OCP master port
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	// Put something in shared memory just for fun
	for(i=0; i<128; i++)
		pru0_dram[i] = i;

	while (1)
	{
		__delay_cycles(90000);	// n us

		// Wait for 0 input from clicker; quiescent output of sensor = 1
		while ((__R31 & IR_IN) == IR_IN);

		__R30 |= LED;		// LED on, start sampling

		// Start sampling, delay 450 usec to sample pulse centers
		__delay_cycles(90000);	// 450 us

		// Looking for 0 1 0 preamble
//		__R30 |= TEST;		// TEST=1
		if ((__R31 & IR_IN) == 0)
		{
			__delay_cycles(180000);		// 900 us
			__R30 &= ~TEST;		// TEST=0
			if ((__R31 & IR_IN) == 1)
			{
				__delay_cycles(180000);
//				__R30 |= TEST;		// TEST=1
				if ((__R31 & IR_IN) == 0)
				{
					// Now ready for 25 bits of clicker code
					code = 0;
					for (i=0; i<25; i++)
					{
						__delay_cycles(90000);
						__R30 &= ~TEST;		// TEST=0
						__delay_cycles(90000);
						__R30 |= TEST;		// TEST=1, so rising edge of TEST is sample time

						sample = __R31 & IR_IN;
						code = (code<<1) | sample;		// shift code left & insert sample as lsb
					}

					// Complete code received, put in shared memory
					// Not thrilled with order, but it matches original verison of ClickerDecoder
					// for reference
//					payload[1] =  code & 0x000000FF;
//					payload[2] = (code & 0x0000FF00) >> 8;
//					payload[3] = (code & 0x00FF0000) >> 16;
//					payload[4] = (code & 0xFF000000) >> 24;
					pru0_dram[0] = code;

					__R30 &= ~LED;		// LED off
				}
			}
		}
	}
}
