/*
	This program reads temperatures from a string of DS18B20 digital sensors
	through a DS2480B 1-Wire to serial interface chip. The serial interface
	chip handles the 1-Wire network timing so non-real-time computers can
	use the temperature sensors. The DS2480B also allows pulse-shape control
	that can improve long-distance performance on a 1-Wire network.

	The DS2480B data sheet and the Maxim Application Note 192 have information
	on how to use this device.

	This program is known to compile with gcc but should work with any POSIX.
	The development environment is Cygwin on Windows 10 or Debian on a Raspberry PI.
	
	$ cc ds18b20_with_ds2480b.c
	$ ./a.exe /dev/ttyS2

*/
	
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

// Change these if you'd like
#define DEFAULTSERIAL	"/dev/ttyS2"
#define MAXDEVICES		25				// Array size, # of 1-Wire devices

// DS2480B serial to 1-Wire driver
#define COMMANDMODE		0xE3
#define DATAMODE		0xE1
#define RESETCOMMAND	0xC5			// DS2480B flex speed reset
#define SEARCHACCOFF	0xA1			// Search accelerator off command
#define	SEARCHACCON		0xB1			// Search accelerator on command
#define TIMINGBYTE		0xC1

// 1-Wire
#define MATCHROM		0x55
#define READROM			0x33
#define	SEARCHROM		0xF0
#define SKIPROM			0xCC

// DS18B20 temperature sensor
#define CONVERT			0x44
#define READSCRATCHPAD	0xBE

// Helpers
#define	msleep(t)	usleep(1000*t)		// Milliseconds sleep

// Globals
uint8_t currentMode;					// Command or Data mode flag for the DS2480B

// Function Prototypes
int ds18b20_convert(int);
float ds18b20_readTemperature(int);
int ds2480b_detect(int);
int ds2480b_matchROM(int, uint8_t*);
int ds2480b_mode(int, uint8_t);
int ds2480b_readROM(int, uint8_t*);
int ds2480b_recv(int, uint8_t*, int);
int ds2480b_reset(int);
void ds2480b_searchROM(int, uint8_t*, uint8_t*);
int ds2480b_send(int, uint8_t*, int);
int ds2480b_skipROM(int);
void getROM(uint8_t*, uint8_t*);
int initialize(int, char**);
int lastDiscrep(uint8_t*);
void loadBuf(uint8_t*, uint8_t*);
int main(int, char**);
void parse(uint8_t*, uint8_t*);
void prArray(uint8_t*);
void prParsed(uint8_t*);
void prROM(uint8_t*);
int scanBus(int, uint8_t[][8]);
void serialBreak(int);
int serialInit(char*);
void setDirectionBit(int, uint8_t*);
uint8_t twoIntoOne(uint8_t*);

/*------------------------------------------------------------------------------
	This program starts by scanning the 1-Wire bus for all devices and saving
	their ROM addresses. Then for every DS18B20 device (those with a first
	address byte of 0x28) the temperature is read out and printed on stdout
	along with the POSIX time and 1-Wire ROM address.

	For G-CLEF, we probably want to hold the ROM addresses in a database along
	with the location of each sensor.
------------------------------------------------------------------------------*/
int main(int argc, char *argv[])
{

	uint8_t romAddresses[MAXDEVICES][8];
	int i, fd, nDevices, currentTime;
	float Celsius, Fahrenheit;

	fd = initialize(argc, argv);				// Open the DS2480B serial port
	if (fd < 0) {
		fprintf(stderr, "DS2480B detect failed.\n");
		return(0);
	}

	nDevices = scanBus(fd, romAddresses);		// Get all 1-Wire addresses

	for (;;) {									// Loop forever, reading and printing temperatures
		ds2480b_skipROM(fd);					// Address all sensors
		currentTime = (int) time(NULL);			// POSIX time (no leap seconds)
		ds18b20_convert(fd);					// Ask all sensors to save temperature
		for (i = 0; i < nDevices; i++) {
			if (romAddresses[i][0] == 0x28) {	// 0x28 is the DS18B20 device family
				ds2480b_matchROM(fd, romAddresses[i]);
				Celsius = ds18b20_readTemperature(fd);
				Fahrenheit = 32.0 + 9.0 * Celsius / 5.0;
				printf("%d %9.5f %5.1f [ ", currentTime, Celsius, Fahrenheit);
				prROM(romAddresses[i]);
				printf("]\n");
				fflush(stdout);
			}
		}
	}
	return(0);

}

/*------------------------------------------------------------------------------
	int initialize(argc, argv)

	Open the serial port connected to the DS2480B. DEFAULTSERIAL port is used
	unless a port is named on the command line. Anything with a leading '/' will
	be interpreted as a serial port.
	
	After the serial port is opened, do a DS2480B DETECT command. This sets up
	the DS2480B by setting the pulse shaping parameters and baud rate.
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
		return(-1);
	}
	return(fd);

}

/*------------------------------------------------------------------------------
	int scanBus(fd, romAddresses)

	Scans the 1-Wire bus putting the addresses into romAddress and returning the
	number of devices found.

	We use the 64-byte directions[] array to store the idBit path to take and to
	receive discrepancy flags for the searchROM function. The actual 16-byte data
	arrays sent to and received from the DS2480B are packed and unpacked from the
	directions array at time of use.
------------------------------------------------------------------------------*/
int scanBus(int fd, uint8_t romAddresses[][8])
{

	uint8_t directions[64], addr[8];
	int i, lastDiscrepancy, nDevices;
	
	for (i = 0; i < 64; i++) {
		directions[i] = 0x00;
	}
	lastDiscrepancy = -1;
	nDevices = 0;
	while (lastDiscrepancy) {
		ds2480b_searchROM(fd, directions, addr);
		for (i = 0; i < 8; i++) {
			romAddresses[nDevices][i] = addr[i];
		}
		lastDiscrepancy = lastDiscrep(directions);
		setDirectionBit(lastDiscrepancy, directions);
		nDevices++;
		if (nDevices == MAXDEVICES) {
			break;
		}
	}
	return(nDevices);

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
		fprintf(stderr, "serialInit: can't open serial port %s\n", portName);	
		exit(1);
	}
	flags = fcntl(fd, F_GETFL);				// retrieve access mode and status flags
	flags |= O_NONBLOCK;					// set non-blocking
	fcntl(fd, F_SETFL, flags);				// save new flags
	tcgetattr(fd, &tty_attrib);				// retrieve current attributes
	cfsetspeed(&tty_attrib, B9600);			// set baud rate
	cfmakeraw(&tty_attrib);					// set raw mode
	tcflush(fd, TCIOFLUSH);					// flush read and write data
	tcsetattr(fd, TCSANOW, &tty_attrib);	// save new attributes
	return(fd);

}

/*------------------------------------------------------------------------------
	void serialBreak(fd)

	Send a break to the serial port on fd.
------------------------------------------------------------------------------*/
void serialBreak(int fd)
{

	if (tcsendbreak(fd, 0) != 0) {
		fprintf(stderr, "serialBreak: tcsendbreak failed\n");
		exit(1);
	}

}

/* BEGIN DS18B20 =============================================================*/
/*------------------------------------------------------------------------------
	int ds18b20_convert(fd)

	Send the convert command (0x44) to the DS18B20 sensor and read back the
	reply (a command echo) from the DS2480B bridge. Returns -1 on error. This
	includes the 750ms conversion time wait.
------------------------------------------------------------------------------*/
int ds18b20_convert(int fd)
{

	uint8_t tbuf[1];
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

	Returns the floating point temperature value in C. Do a CONVERT first.
------------------------------------------------------------------------------*/
float ds18b20_readTemperature(int fd)
{

	uint8_t i, n, tbuf[9];
	int16_t C;

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
	C = (uint16_t) tbuf[2];
	C = C << 8;
	C |= (uint16_t) tbuf[1];
//C = 0xFE6F;		// Test value for -25.0625
//C = 0x0191;		// Test value for +25.0625
//C = 0xFFF8;		// Test value for -0.5
	return(0.0625 * (float) C);

}
/* END DS18B20 ===============================================================*/

/* BEGIN DS2480B =============================================================*/
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

	uint8_t i, n, tbuf[5], response[] = {0x16, 0x44, 0x5A, 0x00, 0x93};

	serialBreak(fd);			// Break resets the DS2480B
	tbuf[0] = TIMINGBYTE;
	ds2480b_send(fd, tbuf, 1);	// Send the timing byte

								// Pulse shaping & detect: See p4 & p5 of AN192
	tbuf[0] = 0x17;				// Pull-down slew rate (PDSRC) 1.37V/us
	tbuf[1] = 0x45;				// Write-low time 10 us (W1LT or W1LD)
	tbuf[2] = 0x5B;				// DSO/WORT 8 us
	tbuf[3] = 0x0F;				// Read baud rate register command request
	tbuf[4] = 0x91;				// 1-Wire bit

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

	Send the matchROM command. Communicate with the selected device only.
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
		rr	- 00 shorted 1-Wire bus
			- 01 presence detected (OK)
			- 10 alarm detected on a device
			- 11 no presence pulses detected

	A good response is 0xCD
------------------------------------------------------------------------------*/
int ds2480b_reset(int fd)
{

	uint8_t tbuf[1];
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
	n = ds2480b_recv(fd, tbuf, 1);
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

	If there is only one 1-Wire device on the bus, read its address and
	return it.
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
	return(0);

}

/*------------------------------------------------------------------------------
	int ds2480b_recv(fd, tbuf, nbytes)

	Receive a stream of bytes from the serial port. Returns the number of bytes
	or -1 if there's an error. There's an msleep() at the top of this routine
	to give commands sent immediately before time to finish. Not all commands
	take the same amount of time but 50ms seems to be long enough to cover the
	DS18B20 situations.
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

	Send a stream of bytes out the serial port. Returns -1 on error. The longest
	byte stream we've needed for the DS18B20 is 16 bytes, needed to send and
	receive the searchROM data. Because sending a 0xE3 byte (COMMANDMODE) as
	data requires sending it twice in succession tbuf is sized 32 bytes for
	the worst-case (likely impossible) situation.
	
	There's a 50ms sleep after sending to give devices a chance to react. This
	is much longer than commonly needed but pretty much guarantees we won't ever
	try to read back data too soon. Combined with the 50ms wait at the top of
	a read data sequence, we have a minimum 0.1 seconds between the start of a
	send data and the end of a receive data sequence.
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

	Sends the skip ROM command, telling every device on the bus to listen.
	Returns -1 on error.
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
	ds2480b_searchROM(fd, directions, addr)

	Retrieve the next 1-Wire device address during a searchROM loop. The
	directions array controls the path taken when a discrepancy is noted.
	The directions array is modified within this routine to reflect the
	response from the searchROM, that is, it's equivalent to the response
	from the searchROM command but packed in an easier-to-use format.
------------------------------------------------------------------------------*/
void ds2480b_searchROM(int fd, uint8_t *directions, uint8_t *addr)
{

	uint8_t cmd[1], tbuf[16];

	if (ds2480b_reset(fd) != 0) {
		fprintf(stderr, "OWSearch: ds2480b_reset error\n");
		exit(1);
	}

	ds2480b_mode(fd, DATAMODE);
	cmd[0] = SEARCHROM;				// Send the searchROM command
	ds2480b_send(fd, cmd, 1);
	ds2480b_recv(fd, cmd, 1);		// Receive searchROM command echo
	ds2480b_mode(fd, COMMANDMODE);
	cmd[0] = SEARCHACCON;			// Turn on the search accelerator
	ds2480b_send(fd, cmd, 1);		// No reply expected
	ds2480b_mode(fd, DATAMODE);
	loadBuf(directions, tbuf);		// Create an output data buffer
	ds2480b_send(fd, tbuf, 16);		// Send 16 byte output data array
	ds2480b_recv(fd, tbuf, 16);		// Receive 16 byte reply data array
	parse(directions, tbuf);		// Unpack the reply into the directions array
	getROM(addr, tbuf);				// Unpack the ROM address
	ds2480b_mode(fd, COMMANDMODE);
	cmd[0] = SEARCHACCOFF;			// Turn off the search accelerator
	ds2480b_send(fd, cmd, 1);		// No reply expected
	ds2480b_reset(fd);				// Reset the 1-Wire bus

}

/*------------------------------------------------------------------------------
	getROM(romAddr, tbuf)

	Extracts the ROM address from a searchROM response array.
------------------------------------------------------------------------------*/
void getROM(uint8_t *romAddr, uint8_t *tbuf)
{

	int i;

	for (i = 0; i < 16; i += 2) {
		romAddr[i/2] = twoIntoOne(&tbuf[i]);
	}

}

/*------------------------------------------------------------------------------
	lastDiscrep(directions)

	Scans the directions array from high to low to find the last discrepancy
	idBit position.
------------------------------------------------------------------------------*/
int lastDiscrep(uint8_t *directions)
{

	int i;

	for (i = 63; i; i--) {
		if (directions[i] == 0x01) {
			break;
		}
	}
	return(i);

}

/*------------------------------------------------------------------------------
	loadBuf(directions, tbuf)

	Creates a searchROM output data array from the directions array.
------------------------------------------------------------------------------*/
void loadBuf(uint8_t *directions, uint8_t *tbuf)
{

	int i, j, idBit;

	idBit = 0;
	for (i = 0; i < 16; i++) {
		for (j = 0; j < 4; j++) {
			tbuf[i] >>= 2;
			tbuf[i] |= (directions[idBit] << 6);
			idBit++;
		}
	}

}

/*------------------------------------------------------------------------------
	parse(directions, response)

	Takes the response from a searchROM command and creates the directions
	array. The each directions array byte then contains the 2-bits of
	information for each idBit.
------------------------------------------------------------------------------*/
void parse(uint8_t *directions, uint8_t *response)
{

	uint8_t tmp;
	int i, j, idBit;

	idBit = 0;
	for (i = 0; i < 16; i++) {
		tmp = response[i];
		for (j = 0; j < 8; j+=2) {
			directions[idBit] = ((tmp >> j) & 0x03);
			idBit++;
		}
	}

}

/*------------------------------------------------------------------------------
	prArray(array)

	Prints the 16-byte output data and response array.
------------------------------------------------------------------------------*/
void prArray(uint8_t *array)
{

	uint8_t tmp;
	int i, j, r, d, abit;

	for (i = 0; i < 16; i++) {
		printf("\n");
		tmp = array[i];
		for (j = 0; j < 8; j+=2) {
			d = ((tmp >> j) & 0x01) ? 1 : 0;
			r = ((tmp >> (j+1)) & 0x01) ? 1: 0;
			abit = (i*8+j)/2;
			printf("(%03d):%d,%d ", abit, r, d);
		}
	}

}

/*------------------------------------------------------------------------------
		prParsed(array)

		Prints the directions array.
------------------------------------------------------------------------------*/
void prParsed(uint8_t *array)
{

	int i, r, d;

	for (i = 0; i < 64; i++) {
		if (!(i%8)) {
			printf("\n");
		}
		r = (array[i] & 0x02) ? 1 : 0;
		d = (array[i] & 0x01) ? 1 : 0;
//		printf("B%02d:(%02X) ", i, array[i]);
		printf("B%02d:(%01X%01X) ", i, r, d);
	}
	printf("\n");

}

/*------------------------------------------------------------------------------
	prROM(addr)

	Prints a ROM address.
------------------------------------------------------------------------------*/
void prROM(uint8_t *addr)
{

	int i;

	for (i = 0; i < 8; i++) {
		printf("%02X ", addr[i]);
	}

}

/*------------------------------------------------------------------------------
	setDirectionBit(lastDiscrepancy, directions)

	Modifies the directions array to put a (1,1) into the last
	discrepancy position and zeroes out higher idBit positions.
------------------------------------------------------------------------------*/
void setDirectionBit(int lastDiscrepancy, uint8_t *directions)
{

	int i;

	directions[lastDiscrepancy] = 0x03;
	for (i = lastDiscrepancy + 1; i < 64; i++) {
		directions[i] = 0x00;
	}

}
/*------------------------------------------------------------------------------
	twoIntoOne(pair)

	Helper routine for getROM
------------------------------------------------------------------------------*/
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
/* END DS2480B ===============================================================*/

/*------------------------------------------------------------------------------
------------------------------------------------------------------------------*/
