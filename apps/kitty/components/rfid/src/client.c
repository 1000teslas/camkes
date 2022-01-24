/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

/**
 * RFID card reader, serial port version.
 *
 * Consult saison:/data/hg_root/kitty/python_client
 * Device is located at NRL level 4 fun room.
 */

#include <stdio.h>
#include <stdlib.h>
#include <util.h>
#include <rfid_string.h>
#include <camkes.h>

#include <string.h>

#ifdef DEBUGLEVEL
    #undef DEBUGLEVEL
#endif
#define DEBUGLEVEL DBG_ERR

/* RFID protocol strings */
#define RFID_INIT    "\x08\x01\x41"   // RFID reader initialization
#define RFID_CONN    "\x01\x01\x03"   // Connect to the RFID reader
#define RFID_RESET   "\x01\x02\x52"   // Reset reader
#define RFID_BEEP    "\x06\x01"       // Beep
#define RFID_HALT    "\x04\x02"       // Halt device
#define RFID_ANT     "\x0c\x01"       // Antenna control
#define RFID_LED     "\x07\x01"       // LED control

/* RFID data exchange */
#define RFID_MAGIC   "\xaa\xbb"       // Magic number for sending data/command
#define RFID_SEND    "\x00\x00\x00"   // Send data
#define RFID_RECV    "\x01\x02\x26"   // Receive data
#define RFID_TYPE    "\x02\x02"       // Card type
#define RFID_SEL     "\x03\x02"       // Card selection

/*
 * The RFID reader protocol always stays the same,
 *
 *     MAGIC + LENGTH + SEND_PADDING + COMMAND + CHECKSUM
 *
 * There are only a few bytes difference each time.
 * Moreover, we run this component in a single thread, so it is safe to
 * use a global packet to improve efficiency.
 */
#define RFID_MSG_LEN 20
static char rfid_packet[RFID_MSG_LEN] = {RFID_MAGIC};

/** RFID control commands */
enum rfid_cmd {
	CMD_INIT = 0,
	CMD_CONN,
	CMD_RESET,
	CMD_BEEP,
	CMD_HALT,
	CMD_ANT,
	CMD_LED,
	CMD_RECV,
	CMD_TYPE,
	CMD_SEL
};

/** Antenna control */
enum rfid_antenna {
	ANT_DISABLE = 0,
	ANT_ENABLE  = 1,
};

/** LED control */
enum rfid_led {
	LED_OFF    = 0,
	LED_RED    = 1,
	LED_GREEN  = 2,
	LED_YELLOW = 3,
};

/**
 * Write to serial port
 *
 * It is a wrapper which simulates passing pointers.
 */
static size_t rfid_write(const void *buf_arg, size_t count)
{
	D(DBG_LOG, "buf(%p), size(%d)", buf, count);

	int ret;
	int sent;

	if (count <= 0) {
		return 0;
	}

	sent = 0;
	do {
		memcpy((void*)buf, (char*)buf_arg + sent, count - sent);
		ret = serial_write(0, count);
		if (ret < 0) {
			D(DBG_ERR, "error: %d\n", ret);
			break;
		}

		sent += ret;
	} while (sent != count);

	return sent;
}

/**
 * Read from serial port
 */
static size_t rfid_read(void *buf_arg, size_t count)
{
	D(DBG_LOG, "buf(%p), size(%d)", buf, count);

	int ret;
	int read;

	if (count <= 0) {
		return 0;
	}
	read = 0;
	do {

		ret = serial_read(0, count - read);
		if (ret < 0) {
			D(DBG_ERR, "Serial read error: %d\n", ret);
			break;
		}
		if (ret == 0) {
		    has_serial_wait();
		}
		memcpy((char*)buf_arg + read, (void*)buf, ret);
		read += ret;
	} while (read != count);

	return read;
}

/**
 * Calculate checksum for the card reader.
 *
 * @param payload Data which to be calculated.
 * @param count   Size of the payload in bytes.
 */
static char checksum(const void *payload, size_t count)
{
	char ret = 0;
	char *data = (char*)payload;

	for (int i = 0; i < count; i++) {
		ret ^= data[i];
	}

	D(DBG_LOG, "%d", ret);
	return ret;
}

/**
 * Send data/command to the RFID card reader.
 *
 * @param cmd     Command to be sent.
 * @param buf     Extra parameters.
 * @param size    Number of extra parameters.
 *
 * @note The first extra parameter must be the size and followed by
 *       a character buffer.
 */
static void rfid_send(const int cmd, const void *buf, int size)
{
	D(DBG_LOG, "cmd(%d), extra(%d)\n", cmd, size);

	int len;    // Length of payload.
	int offset; // Payload offset also the length of the header.

	/*
	 * We do NOT care about MAGIC.
	 * It skips the length byte which will be filled in later.
	 */
	offset = sizeof(RFID_MAGIC RFID_SEND);

	char *payload = &rfid_packet[offset];

	switch (cmd) {
		case CMD_INIT:
			len = sizeof(RFID_INIT) - 1;
			memcpy(payload, RFID_INIT, len);
			break;
		case CMD_CONN:
			len = sizeof(RFID_CONN) - 1;
			memcpy(payload, RFID_CONN, len);
			break;
		case CMD_RESET:
			len = sizeof(RFID_RESET) - 1;
			memcpy(payload, RFID_RESET, len);
			break;
		case CMD_BEEP:
			len = sizeof(RFID_BEEP) - 1;
			memcpy(payload, RFID_BEEP, len);
			memcpy(payload + len, buf, size);
			len += size;
			break;
		case CMD_HALT:
			len = sizeof(RFID_HALT) - 1;
			memcpy(payload, RFID_HALT, len);
			break;
		case CMD_ANT:
			len = sizeof(RFID_ANT) - 1;
			memcpy(payload, RFID_ANT, len);
			memcpy(payload + len, buf, size);
			len += size;
			break;
		case CMD_LED:
			len = sizeof(RFID_LED) - 1;
			memcpy(payload, RFID_LED, len);
			memcpy(payload + len, buf, size);
			len += size;
			break;
		case CMD_RECV:
			len = sizeof(RFID_RECV) - 1;
			memcpy(payload, RFID_RECV, len);
			break;
		case CMD_TYPE:
			len = sizeof(RFID_TYPE) - 1;
			memcpy(payload, RFID_TYPE, len);
			memcpy(payload + len, buf, size);
			len += size;
			break;
		case CMD_SEL:
			len = sizeof(RFID_SEL) - 1;
			memcpy(payload, RFID_SEL, len);
			memcpy(payload + len, buf, size);
			len += size;
			break;
		default:
			D(DBG_ERR, "Unknown command!\n");
			return;
	}

	/* Fill in checksum */
	*(payload + len) = checksum(payload, len);

	/*
	 * Fill in the length of the payload, plus send padding.
	 */
	rfid_packet[sizeof(RFID_MAGIC) - 1] = len + sizeof(RFID_SEND) - 1;
	memcpy(&rfid_packet[sizeof(RFID_MAGIC)], RFID_SEND, sizeof(RFID_SEND) - 1);

	/* Write to the card reader(header + payload + checksum). */
	rfid_write(rfid_packet, offset + len + 1);

	print_buffer(DBG_INFO, rfid_packet, RFID_MSG_LEN);

	return;
}

/**
 * Receive rfid response.
 *
 * @param payload Extra data pointer.
 * @param size    Number of extra bytes received.
 *
 * @return Card reader's return value.
 */
static int rfid_recv(unsigned int *payload, int *size)
{
	char len;  // Character received.
	char chk;
	int offset;

	/* Receive rfid magic */
	rfid_read(rfid_packet, sizeof(RFID_MAGIC) - 1);

	if (memcmp(rfid_packet, RFID_MAGIC, sizeof(RFID_MAGIC) - 1)) {
		D(DBG_ERR,"Invalid magic: %02X %02X\n",
			rfid_packet[0], rfid_packet[1]);
		return -1;
	}
	offset = sizeof(RFID_MAGIC) - 1;

	/* Length of response */
	rfid_read(&len, 1);
	rfid_packet[offset] = len;
	offset++;

	/* Receive payload */
	rfid_read(&rfid_packet[offset], len);

	rfid_read(&chk, 1);
	if (chk != checksum(&rfid_packet[offset], len)) {
		D(DBG_ERR, "Checksum failed.\n");
	}

	print_buffer(DBG_INFO, rfid_packet, RFID_MSG_LEN);

	if (len <= 0x6) {
		payload = NULL;
		size = NULL;
		return rfid_packet[offset + len - 1];
	} else {
		*payload = (unsigned int)&rfid_packet[offset + 0x6];
		*size = len - 0x6;
		return rfid_packet[offset + 0x6 - 1];
	}
}

static void rfid_beep(int ms)
{
	int ret;
	char time = (char)(ms / 10);

	rfid_send(CMD_BEEP, &time, 1);

	ret = rfid_recv(NULL, NULL);
	if (ret != 0) {
		D(DBG_ERR, "Beep error: 0x%x\n", ret);
	}
}

static void rfid_connect(void)
{
    int ret;
    rfid_send(CMD_CONN, NULL, 0);

    ret = rfid_recv(NULL, NULL);

    if (ret != 0) {
        D(DBG_ERR, "Connect error: 0x%x\n", ret);
    }
}

static void rfid_antenna(int enable)
{
	int ret;

	if (enable < ANT_DISABLE || enable > ANT_ENABLE) {
		D(DBG_ERR, "Invalid antenna cmd");
		return;
	}

	rfid_send(CMD_ANT, &enable, 1);

	ret = rfid_recv(NULL, NULL);
	if (ret != 0) {
		D(DBG_ERR, "Antenna error: 0x%x\n", ret);
	}
}

static void rfid_init(void)
{
	int ret;
	rfid_send(CMD_INIT, NULL, 0);

	ret = rfid_recv(NULL, NULL);
	if (ret != 0) {
		D(DBG_ERR, "Init error: 0x%x\n", ret);
	}
}

static void rfid_led(int color)
{
	int ret;

	if (color < LED_OFF || color > LED_YELLOW) {
		D(DBG_ERR, "Invalid color");
		return;
	}

	rfid_send(CMD_LED, &color, 1);

	ret = rfid_recv(NULL, NULL);
	if (ret != 0) {
		D(DBG_ERR, "LED error: 0x%x\n", ret);
	}
}

#if 0
static void prep_card_event(const char *buf, int size)
{
	int *lock;
	int *count;
	char *data;

	if (!buf) {
		return;
	}

	lock = (int*)rfid_card;
	count = (int*)((int*)rfid_card + 1);
	data = (char*)((int*)rfid_card + 2);

	spinlock_lock(lock);
	*count = size;
	memcpy(data, buf, size);
	spinlock_unlock(lock);
}
#endif

static int rfid_card_internal(rfid_string_t *client_var)
{
	int ret;
	unsigned int addr = 0;
	int size;

	rfid_send(CMD_RECV, NULL, 0);

	ret = rfid_recv(&addr, &size);
	if (ret == 0x14) {
		D(DBG_INFO, "No card!\n");
	} else if (ret == 0) {
		rfid_send(CMD_TYPE, (char*)addr, 1);

		ret = rfid_recv(&addr, &size);
		if (ret != 0) {
			D(DBG_ERR, "Type error: 0x%x\n", ret);
			return -1;
		}
		client_var->size = size;
		memcpy(client_var->buf, (void*)addr, size);

		print_buffer(DBG_INFO, (char*)addr, size);
		/* Copy card id into client buffer */
		//prep_card_event((char*)addr, size);

		rfid_send(CMD_SEL, (char*)addr, size);
		ret = rfid_recv(&addr, &size);
		if (ret != 0 && (*(char*)addr != 0x8 || *(char*)addr != 0x18)) {
			D(DBG_ERR, "Select error: 0x%x\n", ret);
			return -1;
		}


		return 1;
	} else {
		D(DBG_ERR, "Card error: 0x%x\n", ret);
	}

	return 0;
}

static void rfid_halt(void)
{
	int ret;

	rfid_send(CMD_HALT, NULL, 0);

	ret = rfid_recv(NULL, NULL);
	if (ret != 0) {
		D(DBG_ERR, "Halt error: 0x%x\n", ret);
	}
}
int beep;

void beeps_beep(void) {
	beep = 1;
}

int run()
{
    int ret;
    memset((char*)rfid_card, 0, 10);
    serial_ready();

    serial_set_speed(19200);
    rfid_connect();
    printf("%s: %d\n", __func__,__LINE__);
    rfid_antenna(ANT_DISABLE);
    rfid_led(LED_RED);
    rfid_init();
    rfid_beep(200);
    rfid_antenna(ANT_ENABLE);
    rfid_led(LED_GREEN);
    rfid_string_t client_var;
    //printf("%s: %d\n", __func__,__LINE__);
    while (1) {
        ret = rfid_card_internal(&client_var);
        if (beep) {
            rfid_beep(100);
            beep = 0;
        }
        if (ret == 0) {
            continue;
        } else if (ret < 0) {
            rfid_led(LED_YELLOW);
            // rfid_beep(10);
            // rfid_beep(10);
            // rfid_beep(10);
            rfid_led(LED_GREEN);
        } else {
            //rfid_beep(100);
            rfid_evt_emit();
            rfid_led(LED_RED);
            rfid_halt();
            process_card_process(client_var);
            rfid_led(LED_GREEN);

        }
    }

    return 0;
}
