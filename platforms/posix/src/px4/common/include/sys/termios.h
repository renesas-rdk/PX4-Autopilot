#pragma once

#if defined(__PX4_FREERTOS)


/* Minimal stub to satisfy termios dependencies on bare-metal builds. */
struct termios {
	unsigned int c_iflag;
	unsigned int c_oflag;
	unsigned int c_cflag;
	unsigned int c_lflag;
	unsigned char c_cc[32];
};

#ifndef IGNBRK
#define IGNBRK    (1u << 0)  /* Ignore break condition */
#endif
#ifndef BRKINT
#define BRKINT    (1u << 1)  /* Signal interrupt on break */
#endif
#ifndef IGNPAR
#define IGNPAR    (1u << 2)  /* Ignore characters with parity errors */
#endif
#ifndef PARMRK
#define PARMRK    (1u << 3)  /* Mark parity errors */
#endif
#ifndef INPCK
#define INPCK     (1u << 4)  /* Enable input parity check */
#endif
#ifndef ISTRIP
#define ISTRIP    (1u << 5)  /* Strip character */
#endif
#ifndef INLCR
#define INLCR     (1u << 6)  /* Map NL to CR on input */
#endif
#ifndef IGNCR
#define IGNCR     (1u << 7)  /* Ignore CR */
#endif
#ifndef ICRNL
#define ICRNL     (1u << 8)  /* Map CR to NL on input */
#endif
#ifndef IXON
#define IXON      (1u << 10) /* Enable start/stop output control */
#endif
#ifndef IXANY
#define IXANY     (1u << 11) /* Any character will restart */
#endif
#ifndef IXOFF
#define IXOFF     (1u << 12) /* Enable start/stop input control */
#endif

#ifndef OPOST
#define OPOST    (1u << 0)  /* Post-process output */
#endif
#ifndef ONLCR
#define ONLCR    (1u << 2)  /* Map NL to CR-NL on output */
#endif
#ifndef OCRNL
#define OCRNL    (1u << 3)  /* Map CR to NL on output */
#endif
#ifndef ONOCR
#define ONOCR    (1u << 4)  /* No CR output at column 0 */
#endif
#ifndef ONLRET
#define ONLRET   (1u << 5)  /* NL performs CR function */
#endif
#ifndef OFILL
#define OFILL    (1u << 6)  /* Use fill characters for delay */
#endif

#ifdef __cplusplus
using tcflag_t = unsigned int;
using cc_t = unsigned char;
using speed_t = unsigned int;
#else
typedef unsigned int tcflag_t;
typedef unsigned char cc_t;
typedef unsigned int speed_t;
#endif

/* c_cflag bits */
#define CSIZE    0000060  /* Character size mask */
#define   CS5    0000000  /* 5 bits */
#define   CS6    0000020  /* 6 bits */
#define   CS7    0000040  /* 7 bits */
#define   CS8    0000060  /* 8 bits */
#define CSTOPB   0000100  /* Set two stop bits */
#define CREAD    0000200  /* Enable receiver */
#define PARENB   0000400  /* Parity enable */
#define PARODD   0001000  /* Odd parity */
#define HUPCL    0002000  /* Hang up on last close */
#define CLOCAL   0004000  /* Ignore modem status lines */
#ifndef CRTSCTS
#define CCTS_OFLOW (1u << 29)
#define CRTS_IFLOW (1u << 31)
#define CRTSCTS   (CCTS_OFLOW | CRTS_IFLOW)
#endif

#ifndef ISIG
#define ISIG      (1u << 0)  /* Enable signals */
#endif
#ifndef ICANON
#define ICANON    (1u << 1)  /* Canonical input */
#endif
#ifndef ECHO
#define ECHO      (1u << 3)  /* Enable echo */
#endif
#ifndef ECHOE
#define ECHOE     (1u << 4)  /* Visual erase */
#endif
#ifndef ECHOK
#define ECHOK     (1u << 5)  /* Echo KILL */
#endif
#ifndef ECHONL
#define ECHONL    (1u << 6)  /* Echo NL */
#endif
#ifndef IEXTEN
#define IEXTEN    (1u << 15) /* Enable extended input processing */
#endif

/* c_cc indices */
#define VMIN     16  /* Minimum number of characters */
#define VTIME    17  /* Time out value */

/* Baud rate values */
#define B0       0
#define B50      50
#define B75      75
#define B110     110
#define B134     134
#define B150     150
#define B200     200
#define B300     300
#define B600     600
#define B1200    1200
#define B1800    1800
#define B2400    2400
#define B4800    4800
#define B9600    9600
#define B19200   19200
#define B38400   38400
#define B57600   57600
#define B115200  115200
#define B230400  230400
#define B460800  460800
#define B500000  500000
#define B576000  576000
#define B921600  921600
#define B1000000 1000000
#define B1152000 1152000
#define B1500000 1500000
#define B2000000 2000000

/* tcsetattr optional actions */
#define TCSANOW   0  /* Change immediately */
#define TCSADRAIN 1  /* Change when output drained */
#define TCSAFLUSH 2  /* Flush and change */

/* tcflush queue selector */
#define TCIFLUSH  0  /* Flush input queue */
#define TCOFLUSH  1  /* Flush output queue */
#define TCIOFLUSH 2  /* Flush both queues */

// Implement proper termios functions (overrides inline stubs)
#ifdef __cplusplus
extern "C" {
#endif

int tcgetattr(int fd, struct termios *termios_p);
int tcsetattr(int fd, int optional_actions, const struct termios *termios_p);
int cfsetispeed(struct termios *termios_p, speed_t speed);
int cfsetospeed(struct termios *termios_p, speed_t speed);
int tcflush(int fd, int queue_selector);
int isatty(int fd);

#ifdef __cplusplus
}
#endif
#else
#include_next <sys/termios.h>
#endif /* __PX4_FREERTOS */
