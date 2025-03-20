/*	Clicker Decoder Controller Sandbox - to try out new stuff
	Tuned for Philips / Magnavox TV remote
	(Smaller controller only for TV)

	Expects two I2C devices:
		8x8 matrix display at 0x71
		MCP23017 I/O expander at 0x20

	The relay board connected to the MCP23017 uses negative logic.
	That is, 0 = relay energized
*/
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <sys/mman.h>

#include <inttypes.h>
#include <math.h>
#include <time.h>

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define PRU_ADDR		0x4A300000		// Start of PRU memory Page 184 am335x TRM
#define PRU_LEN			0x80000			// Length of PRU memory
#define PRU0_DRAM		0x00000			// Offset to DRAM
#define PRU1_DRAM		0x02000
#define PRU_SHAREDMEM	0x10000			// Offset to shared memory

unsigned int *pru0DRAM_32int_ptr;		// Points to the start of PRU 0 usable RAM
unsigned int *pru1DRAM_32int_ptr;		// Points to the start of PRU 1 usable RAM
unsigned int *prusharedMem_32int_ptr;	// Points to the start of shared memory

// Display globals
#define MATRIX_ADDR 0x71

unsigned char dispBuffer[16];

// MCP globals
#define MCP_ADDR0 0x20

// MCP23017 register numbers
#define IODIRA 0x00
#define IODIRB 0x01
#define GPIOA  0x12
#define GPIOB  0x13

// MCP output and "low" status
#define OUTPUT	0x00
#define ALL_OFF 0x00
#define ALL_ON	0xFF

unsigned char mcpBuffer[2];			// Buffer for writes to MCP
int mcp0;

void myShutdown(int sig);
void ClearDisplay(void);
void SetUpX(void);
void SetUp8Square(void);
void SetUp6Square(void);
void SetUp4Square(void);
void SetUp2Square(void);

void McpAllOff(void);
void McpAllOn(void);
int	 McpInit(void);
void McpSendBytes(unsigned char byteA, unsigned char byteB);

void DisplayNumber(int file, int guess);
void SetUp0(int file);
void SetUp1(int file);
void SetUp2(int file);
void SetUp3(int file);
void SetUp4(int file);
void SetUp5(int file);
void SetUp6(int file);
void SetUp7(int file);
void SetUp8(int file);
void SetUp9(int file);

void DoTimeTag(int n);
int  CheckForEntryFinish(void);

// Times in seconds
#define TIME_BETWEEN_NUMBERS	1.5

float lastTimeTag;
int workingEntry, finishedEntry;

unsigned char running;

//____________________
int main(int argc, char *argv[])
{
	unsigned int *pru;		// Points to start of PRU memory
	unsigned int theCmd, lastCmd;
	unsigned int *pruDRAM_32int_ptr;
	int i, fd, file;
	char cmdBuffer[1];

	fd = open ("/dev/mem", O_RDWR | O_SYNC);
	if (fd == -1)
	{
		printf ("*** ERROR: could not open /dev/mem.\n");
		return EXIT_FAILURE;
	}
	pru = mmap (0, PRU_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, PRU_ADDR);
	if (pru == MAP_FAILED)
	{
		printf ("*** ERROR: could not map memory.\n");
		return EXIT_FAILURE;
	}
	close(fd);

	// Set memory pointers
	pru0DRAM_32int_ptr =	 pru + PRU0_DRAM/4 + 0x200/4;	// Points to 0x200 of PRU0 memory
	pru1DRAM_32int_ptr =	 pru + PRU1_DRAM/4 + 0x200/4;	// Points to 0x200 of PRU1 memory
	prusharedMem_32int_ptr = pru + PRU_SHAREDMEM/4;			// Points to start of shared memory

	pruDRAM_32int_ptr = pru0DRAM_32int_ptr;

	for (i=0; i<8; i++)
	{
		if (pruDRAM_32int_ptr[i] != i)
			printf("*** Unexpected value read from PRU: %d %d\n", i, pruDRAM_32int_ptr[i]);
	}

	if (McpInit() != 0)
		printf("\n*** Problem in McpInit()\n");;

	// Display setup
	if ((file = open("/dev/i2c-2", O_RDWR)) < 0)
	{
		perror("*** Failed to open I2C bus dev file\n");
		return EXIT_FAILURE;
	}

	if (ioctl(file, I2C_SLAVE, MATRIX_ADDR) < 0)
	{
		perror("*** Failed to connect to I2C device\n");
		return EXIT_FAILURE;
	}

	// Internal system clock enable
	cmdBuffer[0] = 0x21;
	if (write(file, cmdBuffer, 1) != 1)
		perror("*** Failed to send System Setup cmd: Device not present\n");

	// Display on
	cmdBuffer[0] = 0x81;		// no blink
	write(file, cmdBuffer, 1);

	// ROW/INT output pin set
	cmdBuffer[0] = 0xA0;
	write(file, cmdBuffer, 1);

	// Dimming = default

	// Blinking = default

	// Clear display
	ClearDisplay();
	write(file, dispBuffer, 16);

	(void) signal(SIGINT, myShutdown);

	lastTimeTag = 0.0;
	workingEntry = 0;
	finishedEntry = -1;

	running = 1;
	lastCmd = 0;
	printf("\nClickerDecoder Sandbox running...\n");
	do
	{
		sleep(0.1);			// 1 command takes ~24 ms
		CheckForEntryFinish();

		theCmd = pruDRAM_32int_ptr[0];
		if (theCmd != lastCmd)
		{
			switch (theCmd)
			{
				case 11184981:						// 0xAAAB55
				case 19573589:						// 0x12AAB55
					printf("STATUS/EXIT\n");
					ClearDisplay();
					write(file, dispBuffer, 16);
					McpAllOff();					// deactivate all relays
					break;

				case 11186859:						// 0xAAB2AB
				case 19575467:						// 0x12AB2AB
					printf("CH +\n");
					break;

				case 11184971:
				case 19573579:
					printf("--- POWER ---\n");
					break;

				case 11185325:						// 0xAAACAD
				case 19573933:						// 0x12AACAD
					printf("VOL\n");
					break;

				case 11187027:						// 0xAAB353
				case 19575635:						// 0x12AB353
					printf("MENU\n");
					break;

				case 11185323:						// 0xAAACAB
				case 19573931:						// 0x12AACAB
					printf("VOL +\n");
					break;

				case 11184973:						// 0xAAAB4D
				case 19573581:						// 0x12AAB4D
					printf("MUTE\n");
					break;

				case 11186861:						// 0xAAB2AD
				case 19575469:						// 0x12AB2AD
					printf("CH -\n");
					break;

				case 11186867:						// 0xAAB2B3
				case 19575475:						// 0x12AB2B3
					printf("A/CH\n");
					break;

				case 11184813:						// 0xAAAAAD
				case 19573421:						// 0x12AAAAD
					DoTimeTag(1);
					printf("1\n");
					DisplayNumber(file, 1);
					break;

				case 11184819:						// 0xAAAAB3
				case 19573427:						// 0x12AAAB3
					DoTimeTag(2);
					printf("2\n");
					DisplayNumber(file, 2);
					break;

				case 11184821:						// 0xAAAAB5
				case 19573429:						// 0x12AAAB5
					DoTimeTag(3);
					printf("3\n");
					DisplayNumber(file, 3);
					break;

				case 11184843:						// 0xAAAACB
				case 19573451:						// 0x12AAACB
					DoTimeTag(4);
					printf("4\n");
					DisplayNumber(file, 4);
					break;

				case 11184845:						// 0xAAAACD
				case 19573453:						// 0x12AAACD
					DoTimeTag(5);
					printf("5\n");
					DisplayNumber(file, 5);
					break;

				case 11184851:						// 0xAAAAD3
				case 19573459:						// 0x12AAAD3
					DoTimeTag(6);
					printf("6\n");
					DisplayNumber(file, 6);
					break;

				case 11184853:						// 0xAAAAD5
				case 19573461:						// 0x12AAAD5
					DoTimeTag(7);
					printf("7\n");
					DisplayNumber(file, 7);
					break;

				case 11184939:						// 0xAAAB2B
				case 19573547:						// 0x12AAB2B
					DoTimeTag(8);
					printf("8\n");
					DisplayNumber(file, 8);
					break;

				case 11184941:						// 0xAAAB2D
				case 19573549:						// 0x12AAB2D
					DoTimeTag(9);
					printf("9\n");
					DisplayNumber(file, 9);
					break;

				case 11184811:						// 0xAAAAAB
				case 19573419:						// 0x12AAAAB
					DoTimeTag(0);
					printf("0\n");
					DisplayNumber(file, 0);
					break;

				case 11185491:						// 0xAAAD53
				case 19574099:						// 0x12AAD53
					printf("CC\n");
					break;

				case 11186899:						// 0xAAB2D3
				case 19575507:						// 0x12AB2D3
					printf("SLEEP\n");
					break;

				default:
					printf("Unknown cmd: %d (0x%X)\n", theCmd, theCmd);
			}
			lastCmd = theCmd;
		}
	} while (running);

	printf ("---Shutting down...\n");

	if(munmap(pru, PRU_LEN))
		printf("*** ERROR: munmap failed at Shutdown\n");

	return EXIT_SUCCESS;
}

//____________________
void myShutdown(int sig)
{
	// ctrl-c
	running = 0;
	(void) signal(SIGINT, SIG_DFL);		// reset signal handling of SIGINT
}

//____________________
void ClearDisplay(void)
{
	// Clear dispBuffer; note that every other byte is used
	int i;
	for (i=0; i<16; i++)
		dispBuffer[i] = 0;
}

//____________________
void SetUpX(void)
{
	dispBuffer[0x1] = 0xC0;
	dispBuffer[0x3] = 0x21;
	dispBuffer[0x5] = 0x12;
	dispBuffer[0x7] = 0x0C;
	dispBuffer[0x9] = dispBuffer[0x7];
	dispBuffer[0xB] = dispBuffer[0x5];
	dispBuffer[0xD] = dispBuffer[0x3];
	dispBuffer[0xF] = dispBuffer[0x1];
}

//____________________
void SetUp8Square(void)
{
	dispBuffer[0x1] = 0xFF;
	dispBuffer[0x3] = 0xC0;
	dispBuffer[0x5] = 0xC0;
	dispBuffer[0x7] = 0xC0;
	dispBuffer[0x9] = 0xC0;
	dispBuffer[0xB] = 0xC0;
	dispBuffer[0xD] = 0xC0;
	dispBuffer[0xF] = 0xFF;
}

//____________________
void SetUp6Square(void)
{
	dispBuffer[0x1] = 0x00;
	dispBuffer[0x3] = 0x3F;
	dispBuffer[0x5] = 0x21;
	dispBuffer[0x7] = 0x21;
	dispBuffer[0x9] = 0x21;
	dispBuffer[0xB] = 0x21;
	dispBuffer[0xD] = 0x3F;
	dispBuffer[0xF] = 0x00;
}

//____________________
void SetUp4Square(void)
{
	dispBuffer[0x1] = 0x00;
	dispBuffer[0x3] = 0x00;
	dispBuffer[0x5] = 0x1E;
	dispBuffer[0x7] = 0x12;
	dispBuffer[0x9] = 0x12;
	dispBuffer[0xB] = 0x1E;
	dispBuffer[0xD] = 0x00;
	dispBuffer[0xF] = 0x00;
}

//____________________
void SetUp2Square(void)
{
	dispBuffer[0x1] = 0x00;
	dispBuffer[0x3] = 0x00;
	dispBuffer[0x5] = 0x00;
	dispBuffer[0x7] = 0x0C;
	dispBuffer[0x9] = 0x0C;
	dispBuffer[0xB] = 0x00;
	dispBuffer[0xD] = 0x00;
	dispBuffer[0xF] = 0x00;
}

//____________________
void McpAllOff(void)
{
	// Sets all MCP outputs to 0
	// This activates all relays

	mcpBuffer[0] = GPIOA;
	mcpBuffer[1] = ALL_OFF;
	write(mcp0, mcpBuffer, 2);

	mcpBuffer[0] = GPIOB;
	mcpBuffer[1] = ALL_OFF;
	write(mcp0, mcpBuffer, 2);
}

//____________________
void McpAllOn(void)
{
	// Sets all MCP outputs to 1
	// This deactivates all relays

	mcpBuffer[0] = GPIOA;
	mcpBuffer[1] = ALL_ON;
	write(mcp0, mcpBuffer, 2);

	mcpBuffer[0] = GPIOB;
	mcpBuffer[1] = ALL_ON;
	write(mcp0, mcpBuffer, 2);
}

//____________________
int McpInit(void)
{
	// Chip initialization:
	// The system (and kernel in particular) must set up communication
	// with the MCP23017 chips over the I2C bus. It's done by opening
	// the device file (/dev/i2c-1) and calling the "ioctl" function.

	// The chip must also "know" that it's used for providing outputs.
	// All registers used are specified in chip's datasheet:
	// http://ww1.microchip.com/downloads/en/DeviceDoc/21952b.pdf

	// Initialize 1st MCP23017 with 0x20 address:
	mcp0 = open("/dev/i2c-2", O_RDWR);
	if (mcp0 < 0)
	{
		perror("*** Failed to open I2C bus file for MCP device\n");
		return -1;
	}
	if (ioctl(mcp0, I2C_SLAVE, MCP_ADDR0) < 0)
	{
		perror("*** Failed to connect to MCP\n");
		return -1;
	}

	/*
		Writing bytes to registers:
			Each MCP23017 has two input/output banks, and we can read or write
			bytes to/from them at once.
			Bank A (pins 21...28, named GPA0...GPA7)
			Bank B (pins 1...8, named GPB0...GPB7)
		The MCP23017 has many registers we can write bytes from, but the ones
		that interest us are (with their hex values):
			IODIRA - 0x00  - for setting bank direction: 0 - output, 1 - input
			IODIRB - 0x01  - as above for bank B
			GPIOA  - 0x12  - for writing data to outputs or reading inputs of bank A
			GPIOB  - 0x13  - as above for bank B
		First, we need to set IODIRA and IODIRB registers which control
		input or output direction for all GPIO pins of bank A or B.
		Write 0x00 for all outputs, 0xFF for all inputs,
		or specify pins to be outputs and inputs.
	*/

	// First, set I/O direction to output
	mcpBuffer[0] = IODIRA;
	mcpBuffer[1] = OUTPUT;
	write(mcp0, mcpBuffer, 2);	// set IODIRA to all outputs

	mcpBuffer[0] = IODIRB;
	mcpBuffer[1] = OUTPUT;
	write(mcp0, mcpBuffer, 2);	// set IODIRB to all outputs

	// Initialize all outputs to 0 to deactivate relays
	McpAllOff();
	return 0;
}

//____________________
void McpSendBytes(unsigned char byteA, unsigned char byteB)
{
  // Sends two bytes to GPIOA & GPIOB registers

  mcpBuffer[0] = GPIOA;
  mcpBuffer[1] = byteA;
  write(mcp0, mcpBuffer, 2);	// GPIOA

  mcpBuffer[0] = GPIOB;
  mcpBuffer[1] = byteB;
  write(mcp0, mcpBuffer, 2);	// GPIOB
}

//____________________
void DisplayNumber(int file, int guess)
{
	if (guess >= 0 && guess <= 9)
	switch (guess)
	{
		case 0:
			SetUp0(file);	break;
		case 1:
			SetUp1(file);	break;
		case 2:
			SetUp2(file);	break;
		case 3:
			SetUp3(file);	break;
		case 4:
			SetUp4(file);	break;
		case 5:
			SetUp5(file);	break;
		case 6:
			SetUp6(file);	break;
		case 7:
			SetUp7(file);	break;
		case 8:
			SetUp8(file);	break;
		case 9:
			SetUp9(file);	break;
	}
}

//____________________
void SetUp0(int file)
{
	unsigned char i, dispBuffer[16];
	for (i=0; i<16; i++)
		dispBuffer[i] = 0;

	dispBuffer[0x1] = 0x00;
	dispBuffer[0x3] = 0x00;
	dispBuffer[0x5] = 0x7F;
	dispBuffer[0x7] = 0x41;
	dispBuffer[0x9] = 0x41;
	dispBuffer[0xB] = 0x41;
	dispBuffer[0xD] = 0x7F;
	dispBuffer[0xF] = 0x00;
	write(file, dispBuffer, 16);
}

//____________________
void SetUp1(int file)
{
	unsigned char i, dispBuffer[16];
	for (i=0; i<16; i++)
		dispBuffer[i] = 0;

	dispBuffer[0x1] = 0x00;
	dispBuffer[0x3] = 0x00;
	dispBuffer[0x5] = 0x00;
	dispBuffer[0x7] = 0x00;
	dispBuffer[0x9] = 0x7F;
	dispBuffer[0xB] = 0x02;
	dispBuffer[0xD] = 0x00;
	dispBuffer[0xF] = 0x00;
	write(file, dispBuffer, 16);
}

//____________________
void SetUp2(int file)
{
	unsigned char i, dispBuffer[16];
	for (i=0; i<16; i++)
		dispBuffer[i] = 0;

	dispBuffer[0x1] = 0x00;
	dispBuffer[0x3] = 0x00;
	dispBuffer[0x5] = 0x4F;
	dispBuffer[0x7] = 0x49;
	dispBuffer[0x9] = 0x49;
	dispBuffer[0xB] = 0x49;
	dispBuffer[0xD] = 0x79;
	dispBuffer[0xF] = 0x00;
	write(file, dispBuffer, 16);
}

//____________________
void SetUp3(int file)
{
	unsigned char i, dispBuffer[16];
	for (i=0; i<16; i++)
		dispBuffer[i] = 0;

	dispBuffer[0x1] = 0x00;
	dispBuffer[0x3] = 0x00;
	dispBuffer[0x5] = 0x7F;
	dispBuffer[0x7] = 0x49;
	dispBuffer[0x9] = 0x49;
	dispBuffer[0xB] = 0x49;
	dispBuffer[0xD] = 0x41;
	dispBuffer[0xF] = 0x00;
	write(file, dispBuffer, 16);
}

//____________________
void SetUp4(int file)
{
	unsigned char i, dispBuffer[16];
	for (i=0; i<16; i++)
		dispBuffer[i] = 0;

	dispBuffer[0x1] = 0x00;
	dispBuffer[0x3] = 0x00;
	dispBuffer[0x5] = 0x7F;
	dispBuffer[0x7] = 0x08;
	dispBuffer[0x9] = 0x08;
	dispBuffer[0xB] = 0x08;
	dispBuffer[0xD] = 0x0F;
	dispBuffer[0xF] = 0x00;
	write(file, dispBuffer, 16);
}

//____________________
void SetUp5(int file)
{
	unsigned char i, dispBuffer[16];
	for (i=0; i<16; i++)
		dispBuffer[i] = 0;

	dispBuffer[0x1] = 0x00;
	dispBuffer[0x3] = 0x00;
	dispBuffer[0x5] = 0x79;
	dispBuffer[0x7] = 0x49;
	dispBuffer[0x9] = 0x49;
	dispBuffer[0xB] = 0x49;
	dispBuffer[0xD] = 0x4F;
	dispBuffer[0xF] = 0x00;
	write(file, dispBuffer, 16);
}

//____________________
void SetUp6(int file)
{
	unsigned char i, dispBuffer[16];
	for (i=0; i<16; i++)
		dispBuffer[i] = 0;

	dispBuffer[0x1] = 0x00;
	dispBuffer[0x3] = 0x00;
	dispBuffer[0x5] = 0x79;
	dispBuffer[0x7] = 0x49;
	dispBuffer[0x9] = 0x49;
	dispBuffer[0xB] = 0x49;
	dispBuffer[0xD] = 0x7F;
	dispBuffer[0xF] = 0x00;
	write(file, dispBuffer, 16);
}

//____________________
void SetUp7(int file)
{
	unsigned char i, dispBuffer[16];
	for (i=0; i<16; i++)
		dispBuffer[i] = 0;

	dispBuffer[0x1] = 0x00;
	dispBuffer[0x3] = 0x00;
	dispBuffer[0x5] = 0x07;
	dispBuffer[0x7] = 0x09;
	dispBuffer[0x9] = 0x71;
	dispBuffer[0xB] = 0x01;
	dispBuffer[0xD] = 0x01;
	dispBuffer[0xF] = 0x00;
	write(file, dispBuffer, 16);
}

//____________________
void SetUp8(int file)
{
	unsigned char i, dispBuffer[16];
	for (i=0; i<16; i++)
		dispBuffer[i] = 0;

	dispBuffer[0x1] = 0x00;
	dispBuffer[0x3] = 0x00;
	dispBuffer[0x5] = 0x7F;
	dispBuffer[0x7] = 0x49;
	dispBuffer[0x9] = 0x49;
	dispBuffer[0xB] = 0x49;
	dispBuffer[0xD] = 0x7F;
	dispBuffer[0xF] = 0x00;
	write(file, dispBuffer, 16);
}

//____________________
void SetUp9(int file)
{
	unsigned char i, dispBuffer[16];
	for (i=0; i<16; i++)
		dispBuffer[i] = 0;

	dispBuffer[0x1] = 0x00;
	dispBuffer[0x3] = 0x00;
	dispBuffer[0x5] = 0x7F;
	dispBuffer[0x7] = 0x49;
	dispBuffer[0x9] = 0x49;
	dispBuffer[0xB] = 0x49;
	dispBuffer[0xD] = 0x4F;
	dispBuffer[0xF] = 0x00;
	write(file, dispBuffer, 16);
}

//____________________
void DoTimeTag(int n)
{
	// Updates workingEntry and lastTimeTag
	// Called when numeral entered
	
	long ms;
	time_t s;
	struct timespec spec;

	clock_gettime(CLOCK_REALTIME, &spec);
	s = spec.tv_sec;
	ms = (spec.tv_nsec / 1.0e6);		// convert nanoseconds to milliseconds
	if (ms >999)
	{
		s++;
		ms = 0;
	}
//	printf("Current time: %"PRIdMAX".%03ld seconds since the Epoch\n", (intmax_t)s, ms);
	lastTimeTag = s + ms/1000.0;
	workingEntry = 10*workingEntry + n;
	printf("workingEntry= %d\n", workingEntry);
}

//____________________
int CheckForEntryFinish(void)
{
	// Determines if enough time has elapsed to declare entry finished
	// Called very often
	
	long ms;
	time_t s;
	struct timespec spec;
	float currentTime;

	clock_gettime(CLOCK_REALTIME, &spec);
	s = spec.tv_sec;
	ms = (spec.tv_nsec / 1.0e6);		// convert nanoseconds to milliseconds
	if (ms >999)
	{
		s++;
		ms = 0;
	}
	currentTime = s + ms/1000.0;
	if ((currentTime - lastTimeTag) > TIME_BETWEEN_NUMBERS)
	{
		finishedEntry = workingEntry;
		printf("Finished number: %d\n", finishedEntry);

		workingEntry = 0;
		return finishedEntry;
	}
	else
		return -1;
}
