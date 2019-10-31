/***************************************************************************
 *   Copyright (C) 2010 by Dave Huseby <dave at linuxprogrammer dot org>   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termio.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>

#include <jtag/interface.h>
#include "bitbang.h"

#define _POSIX_SOURCE 1

#define KB(x) (x * 1024)

/* goodfet apps */
#define MONITOR				0x00
#define OPENOCD				0x18

/* goodfet verbs */
#define PEEK				0x02
#define SETUP				0x10
#define START				0x20
#define STOP				0x21
#define NOK					0x7E
#define OK					0x7F
#define OPENOCD_RESET		0x80
#define OPENOCD_READ		0x81
#define OPENOCD_WRITE		0x82
#define OPENOCD_LED			0x83
#define DEBUGHEX			0xFE
#define DEBUGSTRING			0xFF

/* error codes */
#define FAILED_NO_DEVICE		1
#define FAILED_OPEN			2
#define FAILED_COMMAND_SEND		3
#define FAILED_DATA_READ		4
#define FAILED_INVALID_PARAMS		5
#define FAILED_DATA_WRITE		6

/* GoodFET lowlevel data */
typedef struct goodfet_s {
	int data;
	int status;
	int err;
	int rbyte;
	int rcount;
	fd_set readfds;
	fd_set exceptfds;
	struct termios old_tios_device;
} goodfet_t;

/* GoodFET packet */
typedef struct goodfet_packet_s {
	uint8_t app;			/* app number */
	uint8_t verb;			/* verb number */
	uint16_t count;			/* data size */
	uint8_t *data;			/* the data */
} goodfet_packet_t;

/* global goodfet handle */
static int gf_speed;
static uint32_t gf_toggling_time_ns = 10000;
static char *gf_device;
static goodfet_t *g_gf;

/* allocate a goodfet struct */
static goodfet_t *goodfet_new(void)
{
	return (goodfet_t *)calloc(1, sizeof(goodfet_t));
}

/* free a goodfet struct */
static void goodfet_free(void *gf)
{
	if (gf == NULL)
		return;

	free(gf);
}

/* read data from the serial port */
ssize_t goodfet_lowlevel_read
(
	goodfet_t *gf,
	void *buf,
	size_t count,
	struct timeval *timeout
)
{
	ssize_t ret = 0;
	unsigned int i = 0;
	int bytes = 0;

	if ((count <= 0) || (buf == 0)) {
		LOG_ERROR("goodfet_lowlevel_read invalid params");
		gf->err = FAILED_INVALID_PARAMS;
		return -gf->err;
	}

	/* try to read as much data as was requested */
	while (i < count) {
		/* check to see if we've run out of time */
		if ((timeout != 0) && (timeout->tv_sec <= 0) && (timeout->tv_usec <= 0)) {
			LOG_ERROR("timeout break");
			break;
		}

		/* clear the fd sets */
		FD_ZERO(&gf->readfds);
		FD_ZERO(&gf->exceptfds);

		/* add the file descriptors to the test sets */
		FD_SET(gf->data, &gf->readfds);
		FD_SET(gf->data, &gf->exceptfds);

		/* wait for input */
		ret = select(gf->data + 1, &gf->readfds, 0, &gf->exceptfds, timeout);
		if (ret < 0) {
			LOG_ERROR("select error");
			gf->err = FAILED_DATA_READ;
			return -gf->err;
		}

		/* we've got data ready to read */
		if ((ret > 0) && FD_ISSET(gf->data, &gf->readfds)) {
			bytes = read(gf->data, (buf + i), (count - i));
			if (bytes < 0) {
				LOG_ERROR("read error");
				gf->err = FAILED_DATA_READ;
				return -gf->err;
			}
			i += bytes;
		}
	}

	return (ssize_t)i;
}

/* write data to the goodfet */
ssize_t goodfet_lowlevel_write(goodfet_t *gf, void *buf, size_t size)
{
	ssize_t ret = 0;

	if ((size <= 0) || (buf == 0)) {
		gf->err = FAILED_INVALID_PARAMS;
		return -gf->err;
	}

	ret = write(gf->data, buf, size);
	if (ret < 0) {
		gf->err = FAILED_DATA_WRITE;
		return -gf->err;
	}

	return ret;
}


void goodfet_packet_free(goodfet_packet_t *p)
{
	if (p == NULL)
		return;

	if (p->data != NULL)
		free(p->data);

	free(p);
}

/* packet reading states */
#define PACKET_START		0
#define PACKET_READ_APP		1
#define PACKET_READ_VERB	2
#define PACKET_READ_COUNT	3
#define PACKET_READ_DATA	4
#define PACKET_END		5

ssize_t goodfet_read_packet(goodfet_t *gf, goodfet_packet_t **p, struct timeval timeout)
{
	int bytes = 0;
	int i = 0;
	int state = PACKET_START;
	uint8_t tmpbyte = 0;
	uint16_t tmpshort = 0;
	goodfet_packet_t *pkt = NULL;
	struct timeval t;

	t.tv_sec = timeout.tv_sec;
	t.tv_usec = timeout.tv_usec;

	while (state != PACKET_END) {
		switch (state) {
			case PACKET_START:
			{
				/* allocate the packet struct */
				*p = calloc(1, sizeof(goodfet_packet_t));
				pkt = *p;

				/* move to the next state */
				state = PACKET_READ_APP;

				break;
			}

			case PACKET_READ_APP:
			{
				/* reset timeval struct */
				t.tv_sec = timeout.tv_sec;
				t.tv_usec = timeout.tv_usec;

				/* read in the app byte */
				if (goodfet_lowlevel_read(gf, &tmpbyte, sizeof(uint8_t), &t) != sizeof(uint8_t)) {
					gf->err = FAILED_DATA_READ;
					return -gf->err;
				}

				/* store the app */
				pkt->app = tmpbyte;

				/* move to the next state */
				state = PACKET_READ_VERB;

				break;
			}

			case PACKET_READ_VERB:
			{
				/* reset timeval struct */
				t.tv_sec = timeout.tv_sec;
				t.tv_usec = timeout.tv_usec;

				/* read in the app byte */
				if (goodfet_lowlevel_read(gf, &tmpbyte, sizeof(uint8_t), &t) != sizeof(uint8_t)) {
					gf->err = FAILED_DATA_READ;
					return -gf->err;
				}

				/* store the verb */
				pkt->verb = tmpbyte;

				/* move to the next state */
				state = PACKET_READ_COUNT;
				break;
			}

			case PACKET_READ_COUNT:
			{
				/* reset timeval struct */
				t.tv_sec = timeout.tv_sec;
				t.tv_usec = timeout.tv_usec;

				/* read in the app byte */
				if (goodfet_lowlevel_read(gf, &tmpshort, sizeof(uint16_t), &t) != sizeof(uint16_t)) {
					gf->err = FAILED_DATA_READ;
					return -gf->err;
				}

				pkt->count = tmpshort;

				/* if the packet has payload, allocate a block of memory for it */
				if (pkt->count > 0) {
					/* allocate a buffer for the data */
					pkt->data = calloc(pkt->count, sizeof(uint8_t));

					/* move to the next state */
					state = PACKET_READ_DATA;

					/* set up the indexes */
					i = 0;
				} else {
					/* move to the end state */
					state = PACKET_END;
				}
				break;
			}

			case PACKET_READ_DATA:
			{
				/* reset timeval struct */
				t.tv_sec = timeout.tv_sec;
				t.tv_usec = timeout.tv_usec;

				/* read the bytes */
				bytes = 0;
				if (i < pkt->count) {
					/* read the data */
					bytes = goodfet_lowlevel_read(gf, &pkt->data[i], (pkt->count - i), &t);

					/* track how many bytes we've read */
					i += bytes;
				} else {
					/* move to the next state */
					state = PACKET_END;
				}
				break;
			}
		}
	}

	return (ssize_t)i;
}

ssize_t goodfet_get_next_packet(goodfet_t *gf, goodfet_packet_t **pkt)
{
	char *str = NULL;
	ssize_t size = 0;
	struct timeval t = { 1L, 0L };
	goodfet_packet_t *p;

	while ((size = goodfet_read_packet(gf, &p, t)) >= 0) {
		switch (p->verb) {
			case DEBUGSTRING:
				/* copy the string so we can print it out */
				str = calloc(p->count + 1, sizeof(char));
				memcpy((void *)str, (void *)p->data, p->count);
				LOG_ERROR("DEBUG: %s", str);
				free(str);
				str = NULL;
				goodfet_packet_free(p);
				p = NULL;
				break;
			case DEBUGHEX:
				if (p->count == 2)
					LOG_ERROR("DEBUG: %02X %02X", p->data[0], p->data[1]);
				else if (p->count == 4)
					LOG_ERROR("DEBUG: %02X %02X %02X %02X", p->data[0], p->data[1],
					    p->data[2], p->data[3]);
				goodfet_packet_free(p);
				p = NULL;
				break;
			default:
				(*pkt) = p;
				return size;
		}
	}

	gf->err = FAILED_DATA_READ;
	return -gf->err;
}

ssize_t goodfet_write_packet(goodfet_t *gf, goodfet_packet_t *pkt)
{
	uint8_t *data;
	uint16_t count;
	ssize_t size;

	/* convert the packet to a buffer */
	size = 4 + pkt->count;
	data = calloc(size, sizeof(uint8_t));
	data[0] = pkt->app;
	data[1] = pkt->verb;
	count = htons(pkt->count);
	data[2] = count >> 8;
	data[3] = count & 0x0ff;
	memcpy((void *)(&data[4]), (void *)pkt->data, (size_t)pkt->count);

	/* send the packet */
	if (goodfet_lowlevel_write(gf, (void *)data, size) != size) {
		gf->err = FAILED_DATA_WRITE;
		return -gf->err;
	}

	/* free the memory */
	free(data);

	return size;
}

static int goodfet_monitor_peek_8(uint16_t addr, uint8_t *val)
{
	uint8_t data[2];
	uint16_t tmp;
	goodfet_packet_t pout;
	goodfet_packet_t *pin;

	if (val == NULL) {
		g_gf->err = FAILED_INVALID_PARAMS;
		return -g_gf->err;
	}

	pout.app = MONITOR;
	pout.verb = PEEK;
	pout.count = 2;
	tmp = htons(addr);
	data[0] = (tmp >> 8);
	data[1] = tmp & 0x0ff;
	pout.data = data;

	/* send the monitor peek command */
	goodfet_write_packet(g_gf, &pout);

	/* read the response */
	if (goodfet_get_next_packet(g_gf, &pin) > 0) {
		(*val) = pin->data[0];
		goodfet_packet_free(pin);
		return ERROR_OK;
	}

	LOG_ERROR("failed to read response to monitor peek command");
	g_gf->err = FAILED_DATA_READ;
	return -g_gf->err;
}

static void goodfet_monitor_info(void)
{
	uint8_t info[2];

	/* get the MCU number */
	goodfet_monitor_peek_8(0x0ff0, &info[0]);
	goodfet_monitor_peek_8(0x0ff1, &info[1]);

	LOG_INFO("GoodFET with %02x%02x MCU", info[0], info[1]);

	/* get the monitor clocking */
	goodfet_monitor_peek_8(0x0056, &info[0]);
	goodfet_monitor_peek_8(0x0057, &info[1]);

	LOG_INFO("Clocked at 0x%04x", (uint16_t)(info[0] + (info[1] << 8)));
}

static int goodfet_read(void)
{
	int val = 0;
	goodfet_packet_t pout;
	goodfet_packet_t *pin;

	pout.app = OPENOCD;
	pout.verb = OPENOCD_READ;
	pout.count = 0;
	pout.data = NULL;

	/* send the monitor peek command */
	goodfet_write_packet(g_gf, &pout);

	/* read the response */
	if (goodfet_get_next_packet(g_gf, &pin) > 0) {
		val = pin->data[0];
		goodfet_packet_free(pin);
		return val;
	}

	LOG_ERROR("failed to read response to goodfet_read command");
	g_gf->err = FAILED_DATA_READ;
	return -g_gf->err;
}

static void goodfet_write(int tck, int tms, int tdi)
{
	uint8_t data[3];
	goodfet_packet_t pout;
	goodfet_packet_t *pin;

	pout.app = OPENOCD;
	pout.verb = OPENOCD_WRITE;
	pout.count = 3;
	data[0] = (uint8_t)tck;
	data[1] = (uint8_t)tms;
	data[2] = (uint8_t)tdi;
	pout.data = data;

	/* send the monitor peek command */
	goodfet_write_packet(g_gf, &pout);

	/* read the response */
	if (goodfet_get_next_packet(g_gf, &pin) == 0) {
		goodfet_packet_free(pin);
		return;
	}

	LOG_ERROR("failed to read response to goodfet_write command");
	g_gf->err = FAILED_DATA_READ;
}

static void goodfet_reset(int trst, int srst)
{
	uint8_t data[2];
	goodfet_packet_t pout;
	goodfet_packet_t *pin;

	pout.app = OPENOCD;
	pout.verb = OPENOCD_RESET;
	pout.count = 2;
	data[0] = (uint8_t)trst;
	data[1] = (uint8_t)srst;
	pout.data = data;

	/* send the monitor peek command */
	goodfet_write_packet(g_gf, &pout);

	/* read the response */
	if (goodfet_get_next_packet(g_gf, &pin) == 0) {
		goodfet_packet_free(pin);
		return;
	}

	LOG_ERROR("failed to read response to goodfet_reset command");
	g_gf->err = FAILED_DATA_READ;
}

static void goodfet_led(int on)
{
	uint8_t data;
	goodfet_packet_t pout;
	goodfet_packet_t *pin;

	pout.app = OPENOCD;
	pout.verb = OPENOCD_LED;
	pout.count = 1;
	data = (uint8_t)on;
	pout.data = &data;

	/* send the monitor peek command */
	goodfet_write_packet(g_gf, &pout);

	/* read the response */
	if (goodfet_get_next_packet(g_gf, &pin) == 0) {
		goodfet_packet_free(pin);
		return;
	}

	LOG_ERROR("failed to read response to goodfet_led command");
	g_gf->err = FAILED_DATA_READ;
}

static void goodfet_setup(void)
{
	goodfet_packet_t pout;
	goodfet_packet_t *pin;

	pout.app = OPENOCD;
	pout.verb = SETUP;
	pout.count = 0;
	pout.data = NULL;

	/* send the monitor peek command */
	goodfet_write_packet(g_gf, &pout);

	/* read the response */
	if (goodfet_get_next_packet(g_gf, &pin) == 0) {
		goodfet_packet_free(pin);
		return;
	}

	LOG_ERROR("failed to read response to goodfet_setup command");
	g_gf->err = FAILED_DATA_READ;
}

static void goodfet_start(void)
{
	goodfet_packet_t pout;
	goodfet_packet_t *pin;

	pout.app = OPENOCD;
	pout.verb = START;
	pout.count = 0;
	pout.data = NULL;

	/* send the monitor peek command */
	goodfet_write_packet(g_gf, &pout);

	/* read the response */
	if (goodfet_get_next_packet(g_gf, &pin) == 0) {
		goodfet_packet_free(pin);
		return;
	}

	LOG_ERROR("failed to read response to goodfet_start command");
	g_gf->err = FAILED_DATA_READ;
}

static void goodfet_stop(void)
{
	goodfet_packet_t pout;
	goodfet_packet_t *pin;

	LOG_ERROR("goodfet_stop()");

	pout.app = OPENOCD;
	pout.verb = STOP;
	pout.count = 0;
	pout.data = NULL;

	/* send the monitor peek command */
	goodfet_write_packet(g_gf, &pout);

	/* read the response */
	if (goodfet_get_next_packet(g_gf, &pin) == 0) {
		goodfet_packet_free(pin);
		return;
	}

	LOG_ERROR("failed to read response to goodfet_stop command");
	g_gf->err = FAILED_DATA_READ;
}

static int goodfet_speed(int speed)
{
	gf_speed = speed;
	return ERROR_OK;
}

static int goodfet_khz(int khz, int *jtag_speed)
{
	if (khz == 0) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}

	*jtag_speed = 499999 / (khz * gf_toggling_time_ns);
	return ERROR_OK;
}

static int goodfet_speed_div(int speed, int *khz)
{
	uint32_t denominator = (speed + 1) * gf_toggling_time_ns;

	*khz = (499999 + denominator) / denominator;
	return ERROR_OK;
}

static struct bitbang_interface goodfet_bitbang = {
	.read = &goodfet_read,
	.write = &goodfet_write,
	.reset = &goodfet_reset,
	.blink = &goodfet_led
};

/*
 * NOTE: getting the serial driver configured correctly was a little tricky to
 * figure out but thanks to the awesome Serial Programming Guide for POSIX
 * Operating Systems <http://www.easysw.com/~mike/serial/serial.html> by
 * Michael R. Sweet, I was able to figure it out.
 */
static void goodfet_configure_tios(struct termios *tios)
{
	/* set up 115.2kB baud rate */
	cfsetispeed(tios, B115200);
	cfsetospeed(tios, B115200);

	/* enable receiver, make local */
	tios->c_cflag |= (CLOCAL | CREAD);

	/* set 8N1: 8-bit width, no parity, and one stop bit */
	tios->c_cflag &= ~PARENB;
	tios->c_cflag &= ~CSTOPB;
	tios->c_cflag &= ~CSIZE;
	tios->c_cflag |= CS8;

	/* enable hardware flow control */
	/*tios->c_cflag |= CRTSCTS;*/

	/* set up for raw input */
	tios->c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG);

	/*
	 * turn off all parity checking, marking, and stripping...also turn off
	 * all software flow control and all of the bullshit mapping of CR's to LF's
	 * and LF's to CR's...blah..who thought it would be this hard to receive
	 * 8-bit binary data over a serial port?!?  I beat my head against this one
	 * for about a week before figuring out the correct settings for this flag!
	 */
	tios->c_iflag &= ~(INPCK | IGNPAR | PARMRK | ISTRIP | IXON | IXOFF | IXANY | ICRNL | INLCR | IUCLC | BRKINT);

	/* set up raw output */
	tios->c_oflag &= ~OPOST;
}

static void goodfet_configure_device(goodfet_t *gf)
{
	struct termios dataios;

	/* save the current termios settings for the two devices */
	tcgetattr(gf->data, &gf->old_tios_device);

	/* get the current data channel settings */
	bzero(&dataios, sizeof(dataios));
	tcgetattr(gf->data, &dataios);

	/* configure the data channel tios */
	goodfet_configure_tios(&dataios);

	/* set the new settings for the data device */
	tcsetattr(gf->data, TCSAFLUSH, &dataios);

	/* set the descriptor to non-blocking */
	fcntl(gf->data, F_SETFL, FNDELAY);
}

/* flush the I/O buffers in the CopyNES */
void goodfet_flush(goodfet_t *gf)
{
	/* flush I/O buffers on the serial device */
	tcflush(gf->data, TCIOFLUSH);
}

/* init the GoodFET device */
static int goodfet_init(void)
{
	char *str = NULL;
	int flags = 0;
	int attempts = 0;
	struct timeval t = { 10L, 0L }; /* 10 second timeout */
	goodfet_packet_t *pkt = NULL;

	/*if (g_gf != NULL) {
		LOG_ERROR("goodfet already initialized");
		return ERROR_FAIL;
	}*/

	/* allocate the goodfet struct */
	g_gf = goodfet_new();

	/* make sure we have a goodfet device name */
	if (gf_device == NULL) {
		LOG_ERROR("no GoodFET device specified");
		g_gf->err = FAILED_NO_DEVICE;
		return ERROR_FAIL;
	}

	/* open the serial device */
	g_gf->data = open(gf_device, O_RDWR  | O_NOCTTY | O_NDELAY);

	if (g_gf->data == -1) {
		LOG_ERROR("failed to open GoodFET device: %s", gf_device);
		g_gf->err = FAILED_OPEN;
		return ERROR_FAIL;
	}
	LOG_INFO("opened serial port");

	/* configure the serial device */
	goodfet_configure_device(g_gf);
	LOG_INFO("device configured");

	for (;;) {
		/* flush the buffers */
		goodfet_flush(g_gf);

		LOG_INFO("Attempting to reset the board: %d", attempts);

		/* get the current control signals state */
		ioctl(g_gf->data, TIOCMGET, &flags);

		/* explicitly set RST and DTR to halt the board */
		flags |= TIOCM_RTS;		/* RTS = 1 */
		flags |= TIOCM_DTR;		/* DTR = 1 */
		ioctl(g_gf->data, TIOCMSET, &flags);

		/* drop DTR, which is !RST, low to begin the app. */
		flags &= ~TIOCM_DTR;	/* DTR = 0 */
		ioctl(g_gf->data, TIOCMSET, &flags);

		/* flush the buffers */
		goodfet_flush(g_gf);

		sleep(1); /* sleep for 1 second */

		/* update number of attmepts */
		attempts++;

		/* read a packet */
		LOG_INFO("looking for initial signature packet");
		if (goodfet_read_packet(g_gf, &pkt, t) > 0) {
			if ((pkt->app == MONITOR) && (pkt->verb == OK) &&
			    (memcmp(pkt->data, "http://goodfet.sf.net/", pkt->count) == 0)) {
				LOG_INFO("got OK packet");
				break;
			}
		}

		/* free the packet */
		goodfet_packet_free(pkt);
		pkt = NULL;

		if (attempts > 100) {
			LOG_ERROR("tried over 100 times to reset the GoodFET device");
			g_gf->err = FAILED_OPEN;
			return ERROR_FAIL;
		}
	}

	/* copy the string so we can print it out */
	str = calloc(pkt->count + 1, sizeof(char));
	memcpy((void *)str, (void *)pkt->data, pkt->count);
	LOG_INFO("%s", str);
	free(str);

	/* free the packet */
	goodfet_packet_free(pkt);

	/* output monitor info */
	goodfet_monitor_info();

	/* do any GoodFET initialization here */
	goodfet_setup();
	goodfet_start();
	goodfet_reset(0, 0);
	goodfet_write(0, 0, 0);
	goodfet_led(1);

	/* hook up the function pointers */
	bitbang_interface = &goodfet_bitbang;

	return ERROR_OK;
}

static int goodfet_quit(void)
{
	goodfet_stop();

	/* reset the termios settings */
	tcsetattr(g_gf->data, TCSAFLUSH, &g_gf->old_tios_device);

	/* close the device */
	close(g_gf->data);

	/* free the device string */
	if (gf_device != NULL)
		free(gf_device);

	/* free the goodfet struct */
	if (g_gf != NULL)
		goodfet_free(g_gf);

	return ERROR_OK;
}

COMMAND_HANDLER(goodfet_handle_device_command)
{
	if (CMD_ARGC == 0)
		return ERROR_OK;

	/* only if the cable name wasn't overwritten by cmdline */
	gf_device = calloc(strlen(CMD_ARGV[0]) + sizeof(char), sizeof(char));
	if (gf_device == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	strcpy(gf_device, CMD_ARGV[0]);
	LOG_INFO("Setting GoodFET device to: %s", gf_device);

	return ERROR_OK;
}

static const struct command_registration goodfet_command_handlers[] = {
	{
		.name = "goodfet_device",
		.handler = goodfet_handle_device_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the name of the serial device for the GoodFET",
		.usage = "[serial device]",
	},
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface goodfet_interface = {
	.name = "goodfet",
	.commands = goodfet_command_handlers,

	.init = goodfet_init,
	.quit = goodfet_quit,

	.khz = goodfet_khz,
	.speed = goodfet_speed,
	.speed_div = goodfet_speed_div,
	.execute_queue = bitbang_execute_queue,
};

