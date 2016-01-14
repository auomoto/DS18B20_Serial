/*-----------------------------------------------------------------------------

	Read data from a network of 1-Wire DS18B20 temperature
	sensors through a DS2480B serial to 1-Wire line driver.

	Compile on POSIX systems:

		% cc -Wall -O3 ds18b20_with_ds2480b.c


	Usage:

		./a.out [/dev/ttyS2] [x]


	Optional command line parameters:

		/dev/ttyS2 is the serial port. The default /dev/ttyS2 will
		be used if it is not on the command line. It is recognized
		by a leading '/' character. 

		x is any character that is not a '/'. If present, the program
		will print the ROM address of a single 1-Wire device on the bus.
		This only works if there is only one device on the network.


	Output:

		The unix time() followed by the temperature in C to stdout.
		This repeats forever with the interval controlled by the
		sleep() in main().


	Program #defines:

		o The default serial port
		o The number of sensors on the network
		o The ROM address of each sensor
		
	2015-10-08 au

------------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>

#define msleep(t)	usleep(1000*t)	// millseconds

// DS2480B and DS18B20 commands
#define COMMANDMODE	0xE3
#define CONVERT		0x44
#define DATAMODE	0xE1
#define MATCHROM	0x55
#define READROM		0x33
#define READSCRATCHPAD	0xBE
#define RESETCOMMAND	0xC5		// DS2480B flex speed reset
#define SKIPROM		0xCC

// Function prototypes
int ds18b20_convert(int);
float ds18b20_readTemperature(int);
void ds2480b_dataMode(int);
int ds2480b_detect(int);
int ds2480b_matchROM(int, uint8_t *);
int ds2480b_readROM(int, uint8_t *);
int ds2480b_reset(int);
int ds2480b_skipROM(int);
int serialBreak(int);
int serialSetup(int, char**);


/*-------------- Change these to match your situation ------------------------*/

#define DEFAULTSERIAL	"/dev/ttyS2"
#define NSENSORS	1
uint8_t sensor[NSENSORS][8] = {
	{ 0x28, 0x1A, 0x82, 0xC3, 0x06, 0x00, 0x00, 0xA2 },
//	{ 0x28, 0x38, 0xCA, 0x95, 0x06, 0x00, 0x00, 0x7C },
//	{ 0x28, 0x42, 0xDE, 0x94, 0x06, 0x00, 0x00, 0xC7 },
//	{ 0x28, 0xE3, 0xD1, 0x95, 0x06, 0x00, 0x00, 0x4C },
//	{ 0x28, 0x46, 0xC9, 0x94, 0x06, 0x00, 0x00, 0x36 },
//	{ 0x28, 0x89, 0x80, 0x95, 0x06, 0x00, 0x00, 0x18 },
};

/*----------------------------------------------------------------------------*/

int main(int argc, char *argv[])
{

	uint8_t addr[8];
	int i, get_addr, fd;
	float C, F;

	get_addr = 0;
	fd = serialSetup(argc, argv);
	for (i = 1; i < argc; i++) {
		if (argv[i][0] != '/') {
			get_addr = 1;
		}
	}
	
	ds2480b_detect(fd);

	if (get_addr) {
		ds2480b_readROM(fd, addr);
		exit(0);
	}

	for (;;) {
		ds2480b_skipROM(fd);
		ds18b20_convert(fd);
		printf("%d ", (int) time(NULL));
		for (i = 0; i < NSENSORS; i++) {
			ds2480b_matchROM(fd, sensor[i]);
			C = ds18b20_readTemperature(fd);
			F = 32.0 + (9.0 * C/5.0);
			printf("%6.2f %6.2f", C, F);
		}
		printf("\n");
		fflush(stdout);
		sleep(58);
	}

	close(fd);
	return(0);

}

/*------------------------------------------------------------------------------

	int ds18b20_convert(fd) - send convert command to DS18B20
	The convert command tells the DS18B20 temperature sensor to
	measure the temperature and convert the value to a digital
	12-bit number. fd is the serial port file descriptor.

	DS2480B serial interface must be in data mode upon entry.
	Exits with DS2480B in data mode.

------------------------------------------------------------------------------*/

int ds18b20_convert(int fd)
{

	uint8_t tbuf[3];
	int n;

	tbuf[0] = CONVERT;

	n = write(fd, tbuf, 1);
	msleep(50);

	if (n != 1) {
		fprintf(stderr, "ds18b20_convert: error writing convert command\n");
		fflush(stderr);
		exit(0);
	}

	msleep(750);	// wait for 12-bit conversion
	tbuf[0] = 0xFF;
	n = read(fd, tbuf, 1);
	if ((n != 1) || (tbuf[0] != CONVERT)) {
		fprintf(stderr, "ds18b20_convert: read-back error\n");
		fflush(stderr);
		exit(0);
	}

	tcflush(fd, TCIOFLUSH);
	msleep(50);

	return(0);

}

/*------------------------------------------------------------------------------

	float ds18b20_readTemperature(fd) - returns temperature
	Reads the DS18B20 scratchpad for a 12-bit number, then converts to C.

	Do a ds2480b_matchROM or ds2480b_skipROM first.

------------------------------------------------------------------------------*/

float ds18b20_readTemperature(int fd)
{

	uint8_t tbuf[10], scratch[10];
	int i, n, len, Celsius;

	len = 0;
	tbuf[len++] = READSCRATCHPAD;
	for (i = 0; i < 9; i++) {
		tbuf[len++] = 0xFF;
	}
	n = write(fd, tbuf, len);
	msleep(50);

	if (n != len) {
		fprintf(stderr, "ds18b20_readTemperature: error writing command to DS2480B\n");
		exit(0);
	}

	n = read(fd, scratch, len);
	if (n != len) {
		fprintf(stderr, "ds18b20_readTemperature: error writing command to DS2480B\n");
		exit(0);
	}

	Celsius = scratch[2];
	Celsius <<= 8;
	Celsius |= scratch[1];

	tcflush(fd, TCIOFLUSH);
	msleep(50);

	return(Celsius * 0.0625);

}

/*------------------------------------------------------------------------------

	ds2480b_dataMode(fd) - puts the DS2480B line driver into data mode

------------------------------------------------------------------------------*/

void ds2480b_dataMode(int fd)
{

	uint8_t tbuf;
	int n;

	tbuf = DATAMODE;

	n = write(fd, &tbuf, 1);
	if (n != 1) {
		fprintf(stderr, "ds2480b_dataMode: error writing DATAMODE request\n");
		fflush(stdout);
		exit(0);
	}

	return;

}

/*------------------------------------------------------------------------------

	int ds2480b_detect(fd) - initialize the DS2480B interface

	Resets the DS2480B and loads configuration parameters. We're
	using mostly defaults for mid-sized networks here. See p4 of
	AN192.

------------------------------------------------------------------------------*/

int ds2480b_detect(fd)
{

	uint8_t tbuf[6], response[] = {0x16, 0x44, 0x5A, 0x00, 0x93};
	int i, n;

	tbuf[0] = 0x17;		// Pull-down slew rate control (PDSRC) 1.35V/us
	tbuf[1] = 0x45;		// Write-1 low time 10 us (W1LT aka WILD)
	tbuf[2] = 0x5B;		// DSO/WORT 8 us
	tbuf[3] = 0x0F;		// Read baud rate request
	tbuf[4] = 0x91;		// 1-Wire bit result

	if (serialBreak(fd) != 0) {
		exit(0);
	}

	n = ds2480b_reset(fd);
	if ((n != 0xCD) & (n != 0xE3)) {
		fprintf(stderr, "ds2480b_detect: reset failed\n");
		exit(0);
	}

	n = write(fd, tbuf, 5);
	msleep(50);		// minimum by experiment is 10 ms

	if (n != 5) {
		fprintf(stderr, "ds2480b_detect: error writing configuration parameters\n");
		exit(0);
	}

	n = read(fd, tbuf, 5);
	if (n != 5) {
		fprintf(stderr, "ds2480b_detect: error reading back configuration parameters\n");
		exit(0);
	}
	for (i = 0; i < 5; i++) {
		if (tbuf[i] != response[i]) {
			fprintf(stderr, "ds2480b_detect: config parameter read-back error\n");
			exit(0);
		}
	}
	tcflush(fd, TCIOFLUSH);
	msleep(50);
	return(0);

}

/*------------------------------------------------------------------------------

	ds2480b_matchROM(int fd, uint8_t *addr) - select a 1-Wire device

	Select the 1-Wire slave with the matching ROM address.

------------------------------------------------------------------------------*/

int ds2480b_matchROM(int fd, uint8_t *addr)
{

	uint8_t tbuf[18];
	int i, n, len;

	ds2480b_reset(fd);
	ds2480b_dataMode(fd);

	len = 0;
	tbuf[len++] = MATCHROM;
	for (i = 0; i < 8; i++) {
		tbuf[len++] = addr[i];
		if (tbuf[len-1] == COMMANDMODE) {
			tbuf[len++] = COMMANDMODE;
		}
	}

	n = write(fd, tbuf, len);
	msleep(50);
	if (n != len) {
		fprintf(stderr, "ds2480b_matchROM: error writing MATCHROM command\n");
		fflush(stderr);
		exit(0);
	}

	n = read(fd, tbuf, 9);
	if (n != 9) {
		fprintf(stderr, "ds2480b_matchROM: error reading reply from DS2480B\n");
		fflush(stderr);
		exit(0);
	}

	tcflush(fd, TCIFLUSH);
	msleep(50);

	return(0);

}

/*------------------------------------------------------------------------------

	ds2480b_readROM(fd, *addr) - reads the ROM address of a 1-Wire device

	Only one device on the network.

------------------------------------------------------------------------------*/

int ds2480b_readROM(int fd, uint8_t *addr)
{

	uint8_t tbuf[10];
	int i, n, len;

	ds2480b_reset(fd);
	ds2480b_dataMode(fd);

	len = 0;
	tbuf[len++] = READROM;
	for (i = 1; i < 10; i++) {
		tbuf[len++] = 0xFF;
	}

	n = write(fd, tbuf, len);
	msleep(50);
	if (n != len) {
		fprintf(stderr, "ds2480b_readROM: error writing READROM request\n");
		fflush(stderr);
		return(-1);
	}

	n = read(fd, addr, 10);
	if (n != 10) {
		fprintf(stderr, "ds2480b_readROM: error reading ROM address\n");
		fflush(stdout);
		tcflush(fd, TCIOFLUSH);
		msleep(50);
		return(-1);
	}

	for (i = 0; i < 9; i++) {
		addr[i] = addr[i+1];
	}
	printf("ROM address is [ ");
	for (i = 0; i < 8; i++) {
		printf("%02X ", addr[i]);
	}
	printf("]\n");
	fflush(stdout);

	tcflush(fd, TCIOFLUSH);
	msleep(50);

	return(0);

}

/*------------------------------------------------------------------------------

	int ds2480b_reset(fd) - resets the 1-Wire bus and leaves DS2480B in command mode

	returns:
		0xE3 - no reply from DS2480B (this happens after sending a BREAK)
		0xCC - 1-Wire bus shorted
		0xCD - Device presence detected
		0xCE - Alarm received
		0xCF - No 1-Wire device presence

------------------------------------------------------------------------------*/

int ds2480b_reset(int fd)
{

	uint8_t tbuf[2];
	int n;

	tbuf[0] = COMMANDMODE;
	tbuf[1] = RESETCOMMAND;		// timing byte

	n = write(fd, tbuf, 2);
	msleep(50);			// see page 16 of the DS2480B data sheet

	if (n != 2) {
		fprintf(stderr, "ds2480b_reset: failed 1-Wire reset command\n");
		return(-1);
	}
	n = read(fd, tbuf, 2);
	if ((tbuf[0] != 0xCD) && (tbuf[0] != 0xE3)) {
		fprintf(stderr, "ds2480b_reset: abnormal 1-Wire reply is %02X\n", tbuf[0]);
		fflush(stderr);
	}
	tcflush(fd, TCIOFLUSH);
	msleep(50);
	return(tbuf[0]);

}

/*------------------------------------------------------------------------------

	ds2480b_skipROM(int fd) - use to send a command to all 1-Wire devices

------------------------------------------------------------------------------*/

int ds2480b_skipROM(int fd)
{

	uint8_t tbuf[1];
	int n;

	ds2480b_reset(fd);
	ds2480b_dataMode(fd);

	tbuf[0] = SKIPROM;
	n = write(fd, tbuf, 1);
	msleep(50);
	if (n != 1) {
		fprintf(stderr, "ds2480b_skipROM: error writing SKIPROM request\n");
		fflush(stderr);
		exit(0);
	}

	n = read(fd, tbuf, 1);
	if (n != 1) {
		fprintf(stderr, "ds2480b_skipROM: error in read-back\n");
		fflush(stderr);
		exit(0);
	}
	tcflush(fd, TCIOFLUSH);
	msleep(50);
	return(0);

}

/*------------------------------------------------------------------------------

	ds2480b_serialBreak(int fd) - Send a BREAK to the DS2480B line driver

	This resets the DS2480B to its power-up state.

------------------------------------------------------------------------------*/

int serialBreak(int fd)
{

	if (tcsendbreak(fd, 0) != 0) {
		fprintf(stderr, "serialBreak: tcsendbreak(fd, 0) failed\n");
		fflush(stderr);
		return(-1);
	} else {
		return(0);
	}
}

/*------------------------------------------------------------------------------

	serialSetup(int argc, char* argv[]) - Get serial port file descriptor

	Assumes a command line argument starting with '/' is a port name.
	Returns the file descriptor or exits.

------------------------------------------------------------------------------*/

int serialSetup(int argc, char* argv[])
{

	char portName[25];
	int i, fd, flags;
	struct termios tty_attrib;

	strcpy(portName, DEFAULTSERIAL);

	for (i = 1; i < argc; i++) {
		if (argv[i][0] == '/') {
			strcpy(portName, argv[1]);
		}
	}

	fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);

	if (fd < 0) {
		fprintf(stderr, "serialSetup: can't open serial port %s\n", portName);
		exit(0);
	}

	flags = fcntl(fd, F_GETFL);		// retrieve access mode and status flags
	flags |= O_NONBLOCK;			// set non-blocking
	fcntl(fd, F_SETFL, flags);		// save new flags
	tcgetattr(fd, &tty_attrib);		// retrieve current attributes
	cfsetspeed(&tty_attrib, B9600);		// set baud rate
	cfmakeraw(&tty_attrib);			// set raw mode
	tcflush(fd, TCIOFLUSH);			// flush read and write data
	tcsetattr(fd, TCSANOW, &tty_attrib);	// save new attributes

	return(fd);

}
