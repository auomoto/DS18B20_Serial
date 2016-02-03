#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#define DEFAULTSERIAL	"/dev/ttyS2"	// For Cygwin, Raspberry PI
#define	msleep(t)	usleep(1000*t)		// Milliseconds sleep

// DS2480B
#define COMMANDMODE	0xE3
#define DATAMODE	0xE1
#define RESETCOMMAND	0xC5			// DS2480B flex speed reset
#define SEARCHACCOFF	0xA1			// Search accelerator off
#define	SEARCHACCON	0xB1			// Search accelerator on
#define TIMINGBYTE	0xC1

// 1-Wire
#define MATCHROM	0x55
#define READROM		0x33
#define	SEARCHROM	0xF0
#define SKIPROM		0xCC

// DS18B20
#define CONVERT		0x44
#define READSCRATCHPAD	0xBE

// MISC
#define TRUE		1
#define FALSE		0

// Globals
uint8_t currentMode;
uint8_t nDevices;				// Number of devices found in a SearchROM
int lastDeviceFlag;				// Flag indicating search found the last device

// Function Prototypes
int ds18b20_convert(int);
float ds18b20_readTemperature(int);
int ds2480b_detect(int);
int ds2480b_matchROM(int, uint8_t*);
int ds2480b_mode(int, uint8_t);
void printBits(uint8_t*);
int ds2480b_readROM(int, uint8_t*);
int ds2480b_recv(int, uint8_t*, int);
int ds2480b_reset(int);
int ds2480b_send(int, uint8_t*, int);
int ds2480b_skipROM(int);
void getROM(uint8_t*, uint8_t*);
int initialize(int, char**);
int main(int, char**);
int OWSearch(int, int, uint8_t*);
void OWSearchX(int, uint8_t*);
int lastDiscrep(uint8_t*);
void serialBreak(int);
int serialInit(char*);
void setDirectionBit(int, uint8_t*);
uint8_t twoIntoOne(uint8_t*);

int main(int argc, char *argv[])
{

	uint8_t addr[8];				// 1-Wire ROM address
	int i, fd, temp, currentTime, lastDiscrepancy;
	float Celsius, Fahrenheit;

	fd = initialize(argc, argv);	// Open the DS2480B serial port
/////////

	lastDiscrepancy = -1;

	lastDiscrepancy = OWSearch(fd, lastDiscrepancy, addr);
/*
if (ds2480b_detect(fd) == -1) {
	fprintf(stderr, "initialize: first ds2480b_detect failed\n");
	msleep(100);
	ds2480b_detect(fd);
}
*/
	ds2480b_reset(fd);
	lastDiscrepancy = OWSearch(fd, lastDiscrepancy, addr);
exit(0);
/////////
//	ds2480b_readROM(fd, addr);
	ds2480b_skipROM(fd);
	currentTime = (int) time(NULL);	// POSIX time (no leap seconds)
	ds18b20_convert(fd);			// Get temperature reading
	for (;;) {
		/* Read 1-Wire ROM addresses from stdin */
		for (i = 0; i < 8; i++) {
			if (scanf("%x", &temp) == EOF) {
				return(1);		// unexpected EOF
			}
			addr[i] = (uint8_t) temp;
		}
		ds2480b_matchROM(fd, addr);
		Celsius = ds18b20_readTemperature(fd);
		Fahrenheit = 32.0 + 9.0 * Celsius / 5.0;
		printf("%d %6.2f %5.1f [ ", currentTime, Celsius, Fahrenheit);
		for (i = 0; i < 8; i++) {
			printf("%02X ", addr[i]);
		}
		printf("]\n");
	}
	exit(0);

}

/*------------------------------------------------------------------------------

	int initialize(argc, argv)

	Open the serial port connected to the DS2480B. DEFAULTSERIAL port is used
	unless a port is named on the command line (anything with a leading '/' will
	be interpreted as a serial port).

------------------------------------------------------------------------------*/
int initialize(int argc, char *argv[])
{

	char portName[25];
	int i, fd;

	strcpy(portName, DEFAULTSERIAL);
	for (i = 1; i < argc; i++) {
		if (argv[i][0] == '/') {
			strcpy(portName, argv[i]);
		}	
	}
	fd = serialInit(portName);	// Open the serial port
	if (ds2480b_detect(fd) == -1) {
		fprintf(stderr, "initialize: first ds2480b_detect failed\n");
		msleep(100);
		ds2480b_detect(fd);
	}
	return(fd);

}

/*------------------------------------------------------------------------------

	int serialInit("/dev/ttyS2")

	Set the serial port to 9600 baud, non-blocking, and return the file
	descriptor.

------------------------------------------------------------------------------*/
int serialInit(char *portName)
{

	int fd, flags;
	struct termios tty_attrib;

	fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd < 0) {
		fprintf(stderr, "initialize: can't open serial port %s\n", portName);	
		exit(1);
	}
	flags = fcntl(fd, F_GETFL);			// retrieve access mode and status flags
	flags |= O_NONBLOCK;				// set non-blocking
	fcntl(fd, F_SETFL, flags);			// save new flags
	tcgetattr(fd, &tty_attrib);			// retrieve current attributes
	cfsetspeed(&tty_attrib, B9600);		// set baud rate
	cfmakeraw(&tty_attrib);				// set raw mode
	tcflush(fd, TCIOFLUSH);				// flush read and write data
	tcsetattr(fd, TCSANOW, &tty_attrib);	// save new attributes
	return(fd);

}

/*------------------------------------------------------------------------------

	void serialBreak(fd)

	Send a break to the serial port connected to fd.

------------------------------------------------------------------------------*/
void serialBreak(int fd)
{

	if (tcsendbreak(fd, 0) != 0) {
		fprintf(stderr, "serialBreak: tcsendbreak failed\n");
		exit(1);
	}

}

/*------------------------------------------------------------------------------

	ds2480b_detect(fd)

	Sets up the DS2480B timing and pulses. Leaves it in Command mode.

	The last byte in the response is supposed to be 93 but it sometimes
	comes back as 90. The explanation in AN192 suggests that the discrepancy
	is due to unsolicited presence pulses although that doesn't jibe with
	Maxim demonstration code found on the web.

------------------------------------------------------------------------------*/
int ds2480b_detect(fd)
{

	uint8_t i, n, tbuf[6], response[] = {0x16, 0x44, 0x5A, 0x00, 0x93};

	serialBreak(fd);			// Break resets the DS2480B
	tbuf[0] = TIMINGBYTE;
	ds2480b_send(fd, tbuf, 1);	// Send the timing byte

							// Pulse shaping & detect: See p4 & p5 of AN192
	tbuf[0] = 0x17;			// Pull-down slew rate (PDSRC) 1.37V/us
	tbuf[1] = 0x45;			// Write-low time 10 us (W1LT or W1LD)
	tbuf[2] = 0x5B;			// DSO/WORT 8 us
	tbuf[3] = 0x0F;			// Read baud rate register command request
	tbuf[4] = 0x91;			// 1-Wire bit

	// Send the commands
	n = ds2480b_send(fd, tbuf, 5);
	if (n == -1) {
		fprintf(stderr, "ds2480b_detect: error sending pulse shape parameters\n");
		exit(1);
	}

	// Get the response
	n = ds2480b_recv(fd, tbuf, 5);
	if (n == -1) {
		fprintf(stderr, "ds2480b_detect: error receiving response packet\n");
		exit(1);
	}

	// Check the response
	for (i = 0; i < 5; i++) {
		if (tbuf[i] != response[i]) {
			if (i < 4) {
				fprintf(stderr, "ds2480b_detect: bad response packet\n");
				fprintf(stderr, "tbuf[%d] = %02X should be %02X\n", i, tbuf[i], response[i]);
				return(-1);
			} else {
				if (tbuf[i] != 0x90) {		// 0x90 is OK (AN192 p4)
					fprintf(stderr, "ds2480b_detect: bad response packet\n");
					fprintf(stderr, "tbuf[%d] = %02X should be 0x90 or 0x93\n", i, tbuf[i]);
					return(-1);
				}
			}
		}
	}
	currentMode = COMMANDMODE;
	return(0);
}

/*------------------------------------------------------------------------------

	int ds2480b_matchROM(fd, addr)

	Send the match ROM command

------------------------------------------------------------------------------*/
int ds2480b_matchROM(int fd, uint8_t *addr)
{

	uint8_t tbuf[9];
	int i, n;

	ds2480b_reset(fd);
	ds2480b_mode(fd, DATAMODE);
	tbuf[0] = MATCHROM;
	for (i = 1; i < 9; i++) {
		tbuf[i] = addr[i-1];
	}

	// Send the matchROM command and the ROM address
	n = ds2480b_send(fd, tbuf, 9);
	if (n == -1) {
		fprintf(stderr, "ds2480b_matchROM: error sending MATCHROM command\n");
		return(-1);
	}

	// The DS2480B echos back the sent command string
	n = ds2480b_recv(fd, tbuf, 9);
	if (n == -1) {
		fprintf(stderr, "ds2480b_matchROM: error reading reply from DS2480B\n");
		return(-1);
	}
	return(0);

}

/*------------------------------------------------------------------------------

	int ds2480b_mode(fd, mode)

	Set the DS2480B mode and the global currentMode flag.

------------------------------------------------------------------------------*/
int ds2480b_mode(int fd, uint8_t mode)
{

	uint8_t tbuf[1];
	int n;

	if (currentMode == mode) {
		return(0);
	}
	tbuf[0] = mode;
	n = ds2480b_send(fd, tbuf, 1);
	if (n == -1) {
		fprintf(stderr, "ds2480b_mode: error sending mode change command\n");
		return(-1);
	}
	currentMode = mode;
	return(0);

}

/*------------------------------------------------------------------------------

	int ds2480b_reset(fd)

	Reset the 1-Wire bus and report wire irregularities.
	The DS2480B returns a 1 byte response:

	11xvvvrr where:
		x	- undefined bit
		vvv	- DS2480 version
			- 010 means DS2480
			- 011 means DS2480B
		rr	- 00 shorted
			- 01 presence detected (OK)
			- 10 alarm detected
			- 11 no presence detected

	A good response is 0xCD

------------------------------------------------------------------------------*/
int ds2480b_reset(int fd)
{

	uint8_t tbuf[2];
	int n;

	if (currentMode == DATAMODE) {
		ds2480b_mode(fd, COMMANDMODE);
	}
	tbuf[0] = RESETCOMMAND;
	n = ds2480b_send(fd, tbuf, 1);
	if (n == -1) {
		fprintf(stderr, "ds2480b_reset: error sending 1-Wire RESET to DS2480B\n");
		return(-1);
	}
sleep(1);
printf("before:\n");
	n = ds2480b_recv(fd, tbuf, 1);
printf("after\n");
	if (tbuf[0] != 0xCD) {
		fprintf(stderr, "ds2480b_reset: 1-Wire ");
		n = (tbuf[0] & 0x03);
		switch (n) {
		case 0:
			fprintf(stderr, "shorted\n");
			return(-1);
		case 2:
			fprintf(stderr, "alarm\n");
			return(-1);
		case 3:
			fprintf(stderr, "devices not present\n");
			return(-1);
		default:
			fprintf(stderr, "unknown response [%0x]; should be 0xCD\n", tbuf[0]);
			return(-1);
		}
	}
	return(0);

}

/*------------------------------------------------------------------------------

	int ds2480b_readROM(fd, addr)

	Read the ROM address of a single 1-Wire device on the bus and print it.

------------------------------------------------------------------------------*/
int ds2480b_readROM(int fd, uint8_t *addr)
{

	int i, n;
	uint8_t tbuf[9];

	ds2480b_reset(fd);
	ds2480b_mode(fd, DATAMODE);
	tbuf[0] = READROM;
	for (i = 1; i < 9; i++) {
		tbuf[i] = 0xFF;
	}
	n = ds2480b_send(fd, tbuf, 9);
	if (n == -1) {
		fprintf(stderr, "ds2480b_readROM: error sending readROM command\n");
		return(-1);
	}
	n = ds2480b_recv(fd, tbuf, 9);
	if (n == -1) {
		fprintf(stderr, "ds2480b_readROM: error receiving ROM address\n");
		return(-1);
	}
	for (i = 0; i < 8; i++) {
		addr[i] = tbuf[i+1];
	}
	for (i = 0; i < 8; i++) {
		printf("%02X ", addr[i]);
	}
	printf("\n");
	return(0);

}

/*------------------------------------------------------------------------------

	int ds2480b_recv(fd, tbuf, nbytes)

	Receive a stream of bytes from the serial port. Returns the number of bytes
	or -1 if there's an error.

------------------------------------------------------------------------------*/
int ds2480b_recv(int fd, uint8_t *tbuf, int nbytes)
{

	int n;

	msleep(50);
	n = read(fd, tbuf, nbytes);
	if (n != nbytes) {
		fprintf(stderr, "ds2480b_recv: read from serial port failed\n");
		tcflush(fd, TCIOFLUSH);		// flush read and write data
		return(-1);
	}
	tcflush(fd, TCIOFLUSH);			// flush read and write data
	return(n);

}

/*------------------------------------------------------------------------------

	int ds2480b_send(fd, data, nbytes)

	Send a stream of bytes out the serial port. Returns -1 on error.

------------------------------------------------------------------------------*/
int ds2480b_send(int fd, uint8_t *data, int nbytes)
{

	uint8_t tbuf[32];
	int i, n, len;

	tcflush(fd, TCIOFLUSH);			// flush read and write data
	len = 0;
	for (i = 0; i < nbytes; i++) {
		tbuf[len] = data[i];
		if (tbuf[len] == COMMANDMODE) {	// Send twice if byte is COMMANDMODE
			len++;
			tbuf[len] = COMMANDMODE;
		}
		len++;
	}
	nbytes = len;
	n = write(fd, data, nbytes);
	if (n != nbytes) {
		fprintf(stderr, "ds2480b_send: write to serial port failed\n");
		return(-1);
	}
	msleep(50);
	return(0);

}

/*------------------------------------------------------------------------------

	int ds2480b_skipROM(fd)

	Sends the skip ROM command. Returns -1 on error.

------------------------------------------------------------------------------*/
int ds2480b_skipROM(int fd)
{

	uint8_t tbuf[1];
	int n;

	ds2480b_reset(fd);
	n = ds2480b_mode(fd, DATAMODE);
	if (n != 0) {
		fprintf(stderr, "ds2480b_skipROM: error reported from ds2480b_mode\n");
		return(-1);
	}
	tbuf[0] = SKIPROM;
	n = ds2480b_send(fd, tbuf, 1);
	if (n == -1) {
		fprintf(stderr, "ds2480b_skipROM: error reported from ds2480b_send\n");
		return(-1);
	}
	n = ds2480b_recv(fd, tbuf, 1);
	if (n != 1) {
		fprintf(stderr, "ds2480b_skipROM: error reported from ds2480b_recv\n");
		return(-1);
	}
	if (tbuf[0] != SKIPROM) {
		fprintf(stderr, "ds2480b_skipROM: reply error from DS2480B\n");
		return(-1);
	}
	return(0);

}

/*------------------------------------------------------------------------------

	int ds18b20_convert(fd)

	Send the convert command (0x44) to the DS18B20 sensor and read
	back the reply (command echo) from the DS2480B bridge. Returns -1
	on error. This includes the 750ms conversion time wait.

------------------------------------------------------------------------------*/
int ds18b20_convert(int fd)
{

	uint8_t tbuf[3];
	int n;
	
	ds2480b_mode(fd, DATAMODE);
	tbuf[0] = CONVERT;
	n = ds2480b_send(fd, tbuf, 1);
	if (n == -1) {
		fprintf(stderr, "ds18b20_convert: error sending convert command\n");
		return(-1);
	}
	msleep(750);		// 12-bit conversion time
	n = ds2480b_recv(fd, tbuf, 1);
	if (n != 1) {
		fprintf(stderr, "ds18b20_convert: error receiving convert command confirmation\n");
		return(-1);
	}
	if (tbuf[0] != CONVERT) {
		fprintf(stderr, "ds18b20_convert: read-back error from DS2480B\n");
		return(-1);
	}
	return(0);

}

/*------------------------------------------------------------------------------

	float ds18b20_readTemperature(fd)

	Returns the floating point value. Do a CONVERT first.

------------------------------------------------------------------------------*/
float ds18b20_readTemperature(int fd)
{

	uint8_t i, n, tbuf[9];
	int C;
	float sign;

	ds2480b_mode(fd, DATAMODE);
	tbuf[0] = READSCRATCHPAD;
	for (i = 1; i < 9; i++) {
		tbuf[i] = 0xFF;
	}
	n = ds2480b_send(fd, tbuf, 9);
	if (n == -1) {
		fprintf(stderr, "ds2480b_readTemperature: error sending READSCRATCHPAD command\n");
		return(-1);
	}

	n = ds2480b_recv(fd, tbuf, 9);
	if (n == -1) {
		fprintf(stderr, "ds2480b_readTemperature: error receiving scratchpad data\n");
		return(-1);
	}
	C = tbuf[2];
	C = C << 8;
	C += tbuf[1];
	// C = 0xFE6F;		// Test value -25.00
	if (C & 0x8000) {
		sign = -1.0;
		C ^= 0xFFFF;
	} else {
		sign = 1.0;
	}
	return(sign * 0.0625 * (float) C);

}

void OWSearchX(int fd, uint8_t *addr)
{

	uint8_t i, id_bit_number, cmd[1], tbuf[16], response[16], romAddr[8];
	int lastDiscrepancy;

	if (ds2480b_reset(fd) != 0) {
		fprintf(stderr, "OWSearch: ds2480b_reset error\n");
		exit(1);
	}

	ds2480b_mode(fd, DATAMODE);
	cmd[0] = SEARCHROM;			// Send the searchROM command
	ds2480b_send(fd, cmd, 1);
	ds2480b_recv(fd, cmd, 1);		// Receive command echo

	ds2480b_mode(fd, COMMANDMODE);
	cmd[0] = SEARCHACCON;			// Turn on the search accelerator
	ds2480b_send(fd, cmd, 1);		// No reply from a Search acclerator command

	for (i = 0; i < 16; i++) {
		tbuf[i] = 0;
	}
lastDiscrepancy = 64;
	do {
		ds2480b_mode(fd, DATAMODE);
		ds2480b_send(fd, tbuf, 16);		// Send 16 bytes
		ds2480b_recv(fd, response, 16);		// Receive 16 bytes
		printBits(response);
		printf("Last discrepancy = %d\n", lastDiscrepancy = lastDiscrep(response));
		getROM(romAddr, response);		// Unpack ROM address
	} while (lastDiscrepancy < 63);

}

int OWSearch(int fd, int lastDiscrepancy, uint8_t *addr)
{

	uint8_t i, cmd[1], tbuf[16], response[16], romAddr[8];

	if (ds2480b_reset(fd) != 0) {
		fprintf(stderr, "OWSearch: ds2480b_reset error\n");
		exit(1);
	}
printf("OWSearch: Reset ok\n");

	ds2480b_mode(fd, DATAMODE);
	cmd[0] = SEARCHROM;			// Send the searchROM command
	ds2480b_send(fd, cmd, 1);
	ds2480b_recv(fd, cmd, 1);		// Receive command echo
printf("sent searchrom command\n");
	ds2480b_mode(fd, COMMANDMODE);
	cmd[0] = SEARCHACCON;			// Turn on the search accelerator
	ds2480b_send(fd, cmd, 1);		// No reply from a Search acclerator command
printf("sent search accelerator command\n");
	setDirectionBit(lastDiscrepancy, tbuf);
printf("here's tbuf\n");
printBits(tbuf);
	ds2480b_mode(fd, DATAMODE);
printf("set datamode\n");
	ds2480b_send(fd, tbuf, 16);		// Send 16 bytes
	ds2480b_recv(fd, response, 16);		// Receive 16 bytes
	ds2480b_mode(fd, COMMANDMODE);
	cmd[0] = SEARCHACCOFF;
	ds2480b_send(fd, cmd, 1);		// No reply from a Search accelerator command
	printBits(response);
	printf("Last discrepancy = %d\n", lastDiscrepancy = lastDiscrep(response));
	getROM(romAddr, response);		// Unpack ROM address
printf("printed ROM address\n");
	ds2480b_reset(fd);
	return(lastDiscrepancy);

}

void setDirectionBit(int lastDiscrepancy, uint8_t *tbuf)
{

	int i, byte, shiftAmt;

	for (i = 0; i < 16; i++) {
		tbuf[i] = 0;
	}
	if (lastDiscrepancy < 0) {
		return;
	}
	byte = lastDiscrepancy / 4;
	shiftAmt = lastDiscrepancy % 4;
	tbuf[byte] += (0x03 << shiftAmt);

}

void printBits(uint8_t *tbuf)
{

	int i, j, k;
	uint8_t tmp;

	printf("MSB to LSB order\n");
	for (i = 0; i < 16; i++) {
		tmp = tbuf[15-i];
		for (j = 0; j < 8; j++) {
			k = (tmp & 0x80) ? 1 : 0;
			printf("%d", k);
			tmp <<= 1;
		}
		printf(" ");
	}
	printf("\n");

}

int lastDiscrep(uint8_t *tbuf)
{

	uint8_t tmp;
	int i, j, discrepBit, returnbit, abit, last;

	printf("\nDiscrepancies");

	for (i = 0; i < 16; i++) {
		tmp = tbuf[i];
		printf("\nByte %02d ", i);
		for (j = 0; j < 8; j+=2) {
			discrepBit = ((tmp >> j) & 0x01) ? 1 : 0;
			returnbit = ((tmp >> (j+1)) & 0x01) ? 1 : 0;
			abit = (i*8+j)/2;
			if (discrepBit) last = abit;
			printf("Bit%d(%03d):%d,%d  ", j, abit, returnbit, discrepBit);
		}
	}
	printf("\n");
	return(last);

}

void getROM(uint8_t *romAddr, uint8_t *tbuf)
{

	int i;

	for (i = 0; i < 16; i += 2) {
		romAddr[i/2] = twoIntoOne(&tbuf[i]);
	}

	printf("romAddr (LSB to MSB):\n");
	for (i = 0; i < 8; i++) {
		printf("%02X ", romAddr[i]);
	}
	printf("\n");

}

uint8_t twoIntoOne(uint8_t *pair)
{

	uint8_t i, j, temp;

	temp = 0;
	for (i = 0; i < 2; i++) {
		for (j = 0; j < 4; j++) {
			temp <<= 1;
			temp += (pair[1-i] & 0x80) ? 1 : 0;
			pair[1-i] <<= 2;
		}
	}
	return(temp);
}
