/**
 * @file cflash.c
 * @brief COVEA MCS flashing utility
 * @author Telecom Design S.A.
 * @version 1.0.0
 */

#include <features.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <string.h>

// ****************************************************************************
// TYPES:
// ****************************************************************************

typedef struct _HDR
{
	unsigned int  address:20;
	unsigned int  command:4;
	unsigned int  product:8;
} HDR;

// ****************************************************************************
// DEFINES:
// ****************************************************************************

#define FRM_SZ                  60
#define HDR_SZ                  sizeof(HDR)
#define DAT_SZ                  (FRM_SZ - HDR_SZ)

#define WRITE_RAM 		        0x40
#define WRITE_FLASH		        0x50
#define ERASE_ANDWRITEFLASH	    0x60
#define REBOOT			        0x70
#define JUMP			        0x80

#define ACK                     0x30

#define ESC                     0x1b

#define VERSION                 "1.1"

// ****************************************************************************
// STATICS:
// ****************************************************************************

/* The following arrays must have equal length and the values must
 * correspond.
 */
static int baudrates[] = {
  0, 9600, 19200, 38400, 57600, 115200, 230400, 460800
};

static speed_t baud_bits[] = {
  0, B9600, B19200, B38400, B57600, B115200, B230400, B460800
};

static struct {
    char *  name;
    int     code;
} products[] = {
    { "gtw",    0x01    },
    { "smk",    0x02    },
    { "kbd",    0x03    },
    { "mot",    0x81    },
    { NULL,     0x00    }
};

static unsigned long    totalBytes;
static int              serial_fd;

static unsigned char    product     = 1;
static unsigned int     baudrate    = 115200;
static bool             trace       = false;

static struct termios   orig_termios;

// ****************************************************************************
// CODE:
// ****************************************************************************
/**
 * keyboard core
 */
static void reset_terminal_mode(void)
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

static void set_conio_terminal_mode(void)
{
    struct termios new_termios;

    /* take two copies - one for now, one for later */
    tcgetattr(0, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    /* register cleanup handler, and set the new terminal mode */
    atexit(reset_terminal_mode);
    cfmakeraw(&new_termios);
    tcsetattr(0, TCSANOW, &new_termios);
}

static int kbhit(void)
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

static int getch(void)
{
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}

/**
 * Determine baud rate index
 */
static int indexOfBaud(int baudrate)
{
  int i;

  for (i = 0; i < sizeof(baudrates) / sizeof(baudrates[0]); ++i) {
    if (baudrates[i] == baudrate)
      return i;
  }

  return 0;
}

/* Opens serial port, set's it to 57600bps 8N1 RTS/CTS mode.
 *
 * PARAMS:
 * dev - device name
 * RETURNS :
 * file descriptor or -1 on error
 */
static int open_serialport(char *dev)
{
    int fd;

    if ((fd = open(dev, O_RDWR | O_NOCTTY)) != -1) {

        int index = indexOfBaud(baudrate);
        struct termios options_cpy;
        struct termios options;

        printf("Serial port %s opened at speed %d.\r\n", dev, baudrate);

        fcntl(fd, F_SETFL, 0);

        tcgetattr(fd, &options);                                                // Get the parameters

        if (index > 0) {
            cfsetispeed(&options, 0);                                           // Do like minicom: set 0 in speed options
            cfsetospeed(&options, 0);
        }

        options.c_cflag = (CLOCAL | CREAD | CS8 | HUPCL);                       // Enable the receiver and set local mode and 8N1

        if (index > 0) {
            options.c_cflag |= baud_bits[index];                                // Set speed
        } else {
            printf("Invalid serial speed %d.\r\n", baudrate);
        }

        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);                     // Set raw input
        options.c_iflag  = IGNBRK;

        options.c_oflag &= ~(OPOST | OLCUC | ONLRET | ONOCR | OCRNL);           // Set raw output

        options.c_cc[VTIME] = 0;                                                // Non blocking read
        options.c_cc[VMIN]  = 0;

        /**
         * Set serial port options. Then switch baudrate to zero for a while
         * and then back up. This is needed to get some modems
         * (such as Siemens MC35i) to wake up.
         */

        tcflush(fd, TCIFLUSH);

        options_cpy = options;                                                  // Set the new options for the port...
        tcsetattr(fd, TCSANOW, &options);
        options = options_cpy;

        options.c_cflag &= ~baud_bits[index];                                   // Do like minicom: set speed to 0 and back
        tcsetattr(fd, TCSANOW, &options);
        options = options_cpy;

        sleep(1);

        options.c_cflag |= baud_bits[index];
        tcsetattr(fd, TCSANOW, &options);
    }

    return fd;
}

void dump(char *text, unsigned char *s, unsigned char sz)
{
    static unsigned char dump[3 * 16];
    unsigned char i, j, k;
    static char hex[10];

    printf("%s", text);

    for (i = 0, k = 0; i < sz; i++) {
        sprintf(hex, "%02x ", s[i]);
        for (j = 0; hex[j]; j++) {
            dump[k++] = hex[j];
        }
        if (k == 16*3) {
            dump[k] = '\0';
            printf("%s\r\n", dump);
            k = 0;
        }
    }
    if (k > 0) {
        dump[k] = '\0';
        printf("%s\r\n", dump);
    }
}

static unsigned char waitack(unsigned short delay)
{
    struct timeval timeout;
    unsigned char tmp[80];
    fd_set rfds;
    int ret;

    FD_ZERO(&rfds);
    FD_SET(serial_fd, &rfds);

    timeout.tv_sec  = delay / 1000;                                             // Set the requested select timeout
    timeout.tv_usec = 0;

    if (select(serial_fd + 1, &rfds, NULL, NULL, &timeout) > 0) {               // Wait for characters
        if (FD_ISSET(serial_fd, &rfds)) {
            if ((ret = read(serial_fd, tmp, sizeof(tmp))) > 0) {
                if (trace) {
                    dump("ACK: ", tmp, ret);
                }
                return tmp[0];
            }
        }
    }

    return 0;                                                                   // Nothing received
}

static bool upgrade(char *filename) 
{ 
    unsigned int i, ret, sz, percent, old = -1;
    unsigned char binline[FRM_SZ];
    HDR *hdr = (HDR *)binline;
	unsigned char c;
	char prompt[20];
	int count = 0;
	FILE *fp;

	if ((fp = fopen(filename, "rb")) == NULL) {
		printf("Failure : couldn't open file %s\r\n", filename);
		return false;
	}

    fseek(fp, 0L, SEEK_END);
	totalBytes = ftell(fp);                                                     // Get File size (progress bar)
    fseek(fp, 0L, SEEK_SET);
	
    printf("Flashing %ld bytes on products id %#-2.2x\r\n", totalBytes, product);
	printf("Acquiring, 'U' to upgrade, any other key to abort...\r\n");

	for (i = 0; i < FRM_SZ; i++) {
        write(serial_fd, "X", 1);                                               // padding...
    }

	sleep(1);

	hdr->address = 0;
	hdr->command = 0;
	hdr->product = product;

	while (!kbhit()) {                                                          // loop sending upgrade pattern

		ret = write(serial_fd, binline, FRM_SZ);

		if ((c = waitack(1000)) != ACK) {
			printf("Did not get local ACK (%02x)\r\n", c);
			fclose(fp);
			return false;
		}

		usleep(20000);
	}

    if (getch() != 'U') {
        printf("Interrupted...\r\n");
        return false;
    }

    printf("Upgrading, hit ESC to abort...\r\n");

    hdr->command = WRITE_FLASH;

    while (true) {	

        if (kbhit() && (getch() == ESC)) {
            printf("\r\nInterrupted\r\n");                                      // keyboard interrupt
            return false;
        }

        if ((sz = fread(&binline[HDR_SZ], 1, DAT_SZ, fp)) < 0) {
            printf("\r\nRead error (%s)\r\n", strerror(errno));                 // file read error
            return false;
        }

        if (sz == 0) {
            break;                                                              // end of file
        }

        if (trace) {
            sprintf(prompt, "FRM %d\r\n", ++count);
            dump(prompt, binline, FRM_SZ);
        }

	    if ((ret = write(serial_fd, binline, FRM_SZ)) != FRM_SZ) {              // send data packet
            printf("\r\nWrite error (%d/%d)\r\n", ret, FRM_SZ);
            return false;
	    }

	    if (waitack(1000) != ACK) {                                             // wait for local acknowledge
			printf("\r\nWrite ack error\r\n");
			return false;
		}

	    hdr->address += DAT_SZ;

	    if ((percent = (hdr->address * 100) / totalBytes) != old) {             // update completion indicator
	        printf("\r%d%%", percent);
	        old = percent;
	        fflush(stdout);
	    }

	    usleep(50000);                                                          // important: wait 50ms
    }	

    printf("\r\nUpgrade OK\r\n");
		
    hdr->command = REBOOT;                                                      // reboot product
    memset(&binline[HDR_SZ], 'X', DAT_SZ);

    if (trace) {
        dump("", binline, FRM_SZ);
    }

    write(serial_fd, binline, FRM_SZ);
	
    if (waitack(3000) != ACK) {
		printf("Did not get local ACK for reboot\r\n");
		return false;
	}

    return true;
}

// shows how to use this program
static void usage(void)
{
  printf("cflash version %s\n", VERSION);
  printf("Usage: [options] <file>\n");
  printf("options:\n");
  printf("  -d <device>         : Serial port device to connect to [/dev/ttyUSB0]\n");
  printf("  -b <baudrate>       : Serial port speed (0,9600,19200, ...)\n");
  printf("  -p <product>        : Product type (gtw, smk, kbd, mot, or value 1..255)\n");
  printf("  -t                  : Activate trace mode\n");
  printf("  -h                  : Show this help message\n");
}

int main(int argc, char *argv[])
{
    char *dev = "/dev/ttyUSB0";
    char *filename;
    int opt, i;
    bool ok;

    printf("cflash version #%s\n", VERSION);

    while ((opt = getopt(argc, argv, "d:p:b:th?")) > 0) {

        switch (opt) {
            case 'p':
                if (optarg[0] > '9') {
                    for (i = 0; products[i].name; i++) {
                        if (strcasecmp(optarg, products[i].name) == 0) {
                            product = products[i].code;
                            break;
                        }
                    }
                } else {
                    product = strtoul(optarg, 0, 0);
                }
                break;
            case 'd':
                dev = optarg;
                break;
            case 'b':
                baudrate = atoi(optarg);
                break;
            case 't':
                trace = true;
                break;
            case 'h':
            case '?':
                usage();
                exit(1);
                break;
        }
    }

    if (((filename = argv[optind]) == NULL) || (access(filename, 0) != 0)) {
        printf("Can't find file %s.\r\n", filename);
        exit(1);
    }

    if ((serial_fd = open_serialport(dev)) < 0) {                               // Open the serial port
        printf("Can't open %s. %s (%d).\r\n", dev, strerror(errno), errno);
        exit(1);
    }

    set_conio_terminal_mode();
    ok = upgrade(filename);
    reset_terminal_mode();

    close(serial_fd);
    exit(ok? 0: 1);
}

// ****************************************************************************
