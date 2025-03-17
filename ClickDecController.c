/*	Clicker Decoder Controller
	Tuned for Philips / Magnavox TV remote
    (Smaller controller only for TV)
	Shared memory example

	Checks for two I2C devices:
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

unsigned char displayPresent = 1;    // assume display on I2C bus
unsigned char dispBuffer[16];

// MCP globals
#define MCP_ADDR0 0x20

// MCP23017 register numbers
#define IODIRA 0x00
#define IODIRB 0x01
#define GPIOA  0x12
#define GPIOB  0x13

// MCP output and "low" status
#define OUTPUT  0x00
#define ALL_OFF 0x00
#define ALL_ON  0xFF

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
int  McpInit(void);
void McpSendBytes(unsigned char byteA, unsigned char byteB);

unsigned char running;

//____________________
int main(int argc, char *argv[])
{
	unsigned int *pru;		// Points to start of PRU memory
	unsigned int theCmd, lastCmd;
	unsigned int *pruDRAM_32int_ptr;
	int	i, fd, file;
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
//	printf ("Using /dev/mem.\n");

	// Set memory pointers
	pru0DRAM_32int_ptr =     pru + PRU0_DRAM/4 + 0x200/4;	// Points to 0x200 of PRU0 memory
	pru1DRAM_32int_ptr =     pru + PRU1_DRAM/4 + 0x200/4;	// Points to 0x200 of PRU1 memory
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
        return 1;
    }

    if (ioctl(file, I2C_SLAVE, MATRIX_ADDR) < 0)
    {
        perror("*** Failed to connect to I2C device\n");
        return 1;
    }

    // Internal system clock enable
    cmdBuffer[0] = 0x21;
    if (write(file, cmdBuffer, 1) != 1)
    {
        perror("*** Failed to send System Setup cmd: Device not present\n");
        displayPresent = 0;
    }

    if (displayPresent)
    {
        // Display on
        cmdBuffer[0] = 0x81;        // no blink
        write(file, cmdBuffer, 1);

        // ROW/INT output pin set
        cmdBuffer[0] = 0xA0;
        write(file, cmdBuffer, 1);

        // Dimming = default

        // Blinking = default

        // Clear display
        ClearDisplay();
        write(file, dispBuffer, 16);
     }

	(void) signal(SIGINT, myShutdown);

	running = 1;
	lastCmd = 0;
	printf("\nClickerDecoder running...\n");
	if (displayPresent)
		printf("   (Display present)\n");
	else
		printf("   (Display not present)\n");

	do
	{
		sleep(0.1);			// 1 command takes ~24 ms

		theCmd = pruDRAM_32int_ptr[0];
		if (theCmd != lastCmd)
		{
			switch (theCmd)
			{
                case 11184981:                          // 0xAAAB55
                case 19573589:                          // 0x12AAB55
                    printf("STATUS/EXIT\n");
                    if (displayPresent)
                    {
                        ClearDisplay();
                        write(file, dispBuffer, 16);

                        // Deactivate relays
						McpAllOff();

                    }
                    break;

				case 11186859:                          // 0xAAB2AB
                case 19575467:                          // 0x12AB2AB
                    printf("CH +\n");
                    break;

				case 11184971:
				case 19573579:
					printf("--- POWER ---\n");
					if (displayPresent)
					{
						SetUpX();
				        write(file, dispBuffer, 16);
					}
					break;

				case 11185325:                          // 0xAAACAD
                case 19573933:                          // 0x12AACAD
                    printf("VOL - (Relay 13)\n");
                    McpSendBytes(0x00, 0x10);
                    break;

				case 11187027:                          // 0xAAB353
                case 19575635:                          // 0x12AB353
                    printf("MENU (Relay 14)\n");
                    McpSendBytes(0x00, 0x20);
                    break;

				case 11185323:                          // 0xAAACAB
                case 19573931:                          // 0x12AACAB
                    printf("VOL + (Relay 15)\n");
                    McpSendBytes(0x00, 0x40);
                    break;

                case 11184973:                          // 0xAAAB4D
				case 19573581:                          // 0x12AAB4D
                    printf("MUTE (Relay 16)\n");
                    McpSendBytes(0x00, 0x80);
                    break;

                case 11186861:                          // 0xAAB2AD
				case 19575469:                          // 0x12AB2AD
                    printf("CH -\n");
                    break;

                case 11186867:                          // 0xAAB2B3
				case 19575475:                          // 0x12AB2B3
                    printf("A/CH\n");
                    break;

                case 11184813:                          // 0xAAAAAD
				case 19573421:                          // 0x12AAAAD
                    printf("1 (Relay 1)\n");
                    McpSendBytes(0x01, 0x00);
                    break;

                case 11184819:                          // 0xAAAAB3
				case 19573427:                          // 0x12AAAB3
                    printf("2 (Relay 2)\n");
					if (displayPresent)
					{
						SetUp2Square();
				        write(file, dispBuffer, 16);
					}
                   McpSendBytes(0x02, 0x00);
                    break;

                case 11184821:                          // 0xAAAAB5
				case 19573429:                          // 0x12AAAB5
                    printf("3 (Relay 3)\n");
                    McpSendBytes(0x04, 0x00);
                    break;

                case 11184843:                          // 0xAAAACB
				case 19573451:                          // 0x12AAACB
                    printf("4 (Relay 4)\n");
					if (displayPresent)
					{
						SetUp4Square();
				        write(file, dispBuffer, 16);
					}
					McpSendBytes(0x08, 0x00);
                    break;

                case 11184845:                          // 0xAAAACD
				case 19573453:                          // 0x12AAACD
                    printf("5 (Relay 5)\n");
                    McpSendBytes(0x10, 0x00);
                    break;

                case 11184851:                          // 0xAAAAD3
				case 19573459:                          // 0x12AAAD3
                    printf("6 (Relay 6)\n");
					if (displayPresent)
					{
						SetUp6Square();
				        write(file, dispBuffer, 16);
					}
					McpSendBytes(0x20, 0x00);
                    break;

                case 11184853:                          // 0xAAAAD5
				case 19573461:                          // 0x12AAAD5
                    printf("7 (Relay 7)\n");
                    McpSendBytes(0x40, 0x00);
                    break;

                case 11184939:                          // 0xAAAB2B
				case 19573547:                          // 0x12AAB2B
                    printf("8 (Relay 8)\n");
					if (displayPresent)
					{
						SetUp8Square();
				        write(file, dispBuffer, 16);
					}
					McpSendBytes(0x80, 0x00);
                    break;

                case 11184941:                          // 0xAAAB2D
				case 19573549:                          // 0x12AAB2D
                    printf("9 (Relay 9)\n");
                    McpSendBytes(0x00, 0x01);
                    break;

                case 11184811:                          // 0xAAAAAB
				case 19573419:                          // 0x12AAAAB
                    printf("0 (Relay 10)\n");
                    if (displayPresent)
                    {
                        ClearDisplay();
                        write(file, dispBuffer, 16);
                    }
                    McpSendBytes(0x00, 0x02);
                    break;

                case 11185491:                          // 0xAAAD53
				case 19574099:                          // 0x12AAD53
                    printf("CC (Relay 11)\n");
                    McpSendBytes(0x00, 0x04);
                    break;

                case 11186899:                          // 0xAAB2D3
				case 19575507:                          // 0x12AB2D3
                    printf("SLEEP (Relay 12)\n");
                    McpSendBytes(0x00, 0x08);
                    break;

/* These are VCR commands used by TV/VCR remote
				case 11324235:							// 0x0ACCB4B
				case 19712843:							// 0x12CCB4B
					printf("--- VCR POWER ---\n");
					break;

				case 11326803:							// 0x0ACD553
				case 19715411:							// 0x12CD553
					printf("VCR/TV\n");
					break;

				case 11326285:							// 0x0ACD34D
				case 19714893:							// 0x12CD34D
					printf("VCR: EJECT\n");
					break;

				case 11326669:							// 0x0ACD4CD
				case 19715277:							// 0x12CD4CD
					printf("VCR: PLAY\n");
					break;

				case 11326643:
				case 19715251:
					printf("VCR: REW\n");
					break;
*/

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

//    printf("McpAllOn\n");
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

