/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <util.h>

#include <camkes.h>
#include <camkes/dma.h>

/* remove the camkes ERR_IF definition to not overlap with lwip */
#undef ERR_IF

#include <ethdrivers/lwip.h>
#include <ethdrivers/imx6.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <rfid_string.h>
#include <lwip/init.h>
#include <lwip/tcp.h>
#include <lwip/dhcp.h>
#include <lwip/timeouts.h>
#include <lwip/ip_addr.h>
#include <netif/etharp.h>
#include <sel4utils/sel4_zf_logif.h>

#define TXID_MAX             1000000

#define CMD_VALIDATE_CARDID  100
#define CMD_HEARTBEAT        200

#define RESP_DISPLAY_MSG     100
#define RESP_SEED_TXID       101
#define RESP_ALL_GOOD        200
#define RESP_NO_ACCOUNT      400
#define RESP_UNKNOWN_ERR     410
#define RESP_ACC_DISABLED    420

static int txid = 0;

/* Whether the TCP socket is connected */
static int isConnected = 0;
/* Whether the process card thread is blocked waiting for a response */
static int notifyOnCard = 0;
static uint64_t notifyOnCardStarted;

/* Whether the network link is up or not */
static int network_connected = 0;
/* Count the connection so if we reconnect we will know if we
 * get data from an old connection */
static uintptr_t connectionCounter = 0;
/* pcb for the tcb connection. only valid if isConnected is true */
static struct tcp_pcb *tcp_pcb = NULL;

static void display_printf(unsigned int clear_columns, unsigned int xpos, unsigned int ypos,
        uint8_t red, uint8_t green, uint8_t blue, const char *fmt, ...) {
    if (clear_columns > 0) {
        display_clearText(xpos, ypos, clear_columns);
    }
    va_list ap, ap2;
    va_start(ap, fmt);
    va_copy(ap2, ap);
    int len = vsnprintf(NULL, 0, fmt, ap2);
    va_end(ap2);
    char buf[len + 1];
    vsprintf(buf, fmt, ap);
    va_end(ap);
    display_text(buf, xpos, ypos, red, green, blue);
}

#define DPRINTF(...) display_printf(1, 40, 700, 255, 0, 0, __VA_ARGS__)

#define DIE(...) do { \
    DPRINTF(__VA_ARGS__); \
    ZF_LOGF(__VA_ARGS__); \
    } while (0)

static int sendDataViaSocket(int cmdid, char *buffer) {
    err_t err;
    if (isConnected == 0) {
        return -1;
    }

    char data[30];
    sprintf(data, "%d %d %s\n", cmdid, txid, buffer);
    display_printf(1, 512, 450, 40, 120, 220, "%s", data);

    err = tcp_write(tcp_pcb, data, strlen(data), TCP_WRITE_FLAG_COPY);
    if (err != ERR_OK) {
        return -1;
    }
    tcp_output(tcp_pcb);

    if (cmdid == CMD_VALIDATE_CARDID) {
        txid = (txid + 1) % TXID_MAX;
    }
    return 1;
}

static int sendToNetwork (char * cid) {
    char chk = cid[0];
    int i;
    for (i = 1; i < 4; i++) {
        chk = chk ^ cid[i];
    }

    char data[15];
    sprintf(data, "%02x%02x%02x%02x%02x 1.0", cid[0], cid[1], cid[2], cid[3], chk);

    return sendDataViaSocket(CMD_VALIDATE_CARDID,data);
}

void process_card_process(rfid_string_t client_var)
{
    int result;
    lock_lock();
    if (!isConnected) {
        lock_unlock();
        return;
    }

    result = sendToNetwork(client_var.buf);
    if (result > 0) {
        notifyOnCard = 1;
        notifyOnCardStarted = timer_get_time();
    }
    lock_unlock();
    if (result > 0) {
        card_received_wait();
    }
}

void readDataFromNetwork(char *bufferData) {
    static int text_remain = 0;
    static int text_remain_count = 0;
    static int kitty_repeats = 0;
    static int imageCounter = 0;
    int cmd;
    int result;
    char payload[1024];
    result = sscanf(bufferData, "%d %s", &cmd, payload);
    if (result != 2) {
        return;
    }

    // Do processing of data
    if (cmd == RESP_SEED_TXID) {
        int generation;
        int ignored;
        result = sscanf(bufferData,"%d %d %d",&ignored, &txid, &generation);
        if (result != 3) {
            return;
        }
        display_printf(0, 512, 170, 0, 0, 0, "Number of times restarted: %d", generation);
        display_text("Goal uptime (num 9s): 5", 530, 184, 0,0,0);
        display_text("Actual uptime (num 9s): 0", 512, 198, 0,0,0);
    } else if (cmd == RESP_DISPLAY_MSG) {
        if (text_remain) {
            text_remain_count++;
        }
        if (text_remain_count == 5) {
            display_clearText(512,350,1);
            display_clearText(512,364,1);
            text_remain_count = 0;
            text_remain = 0;
        }
    } else {
        /* some kind of card response */
        switch (cmd) {
        case RESP_ALL_GOOD:
            beeps_beep();
        case RESP_NO_ACCOUNT:
            text_remain = 1;
            text_remain_count = 0;
            /* all other responses are equivalently ignored */
        }
        /* Make the text fit on two lines if needed */
        int display = 1;
        if (strlen(bufferData) > 40) {
            /* split on a comma */
            int len = strlen(bufferData);
            char temp[len + 1];
            strcpy(temp, bufferData);
            char *first = strtok(temp, ",");
            if (first) {
                char *second = strtok(NULL, ",");
                if (second) {
                    char *last = strtok(NULL, ",");
                    if (!last) {
                        display = 0;
                        display_printf(1, 512, 350, 0, 0, 0, "%s,", first);
                        display_printf(1, 521, 364, 0, 0, 0, "%s", second);
                    }
                }
            }
        }
        if (display) {
            display_printf(1, 512, 350, 0, 0, 0, "%s", bufferData);
        }
        if (notifyOnCard) {
            notifyOnCard = 0;
            card_received_post();
        }
    }

    display_msg(1, 506, 220);
    display_text("Sending to Server:", 512, 420, 40, 120, 220);
    display_text("From Server:", 512, 320, 0,0,0);
    if (kitty_repeats == 0) {
        display_kitty(imageCounter);
        imageCounter = (imageCounter + 1) % 3;
    }
    kitty_repeats = (kitty_repeats + 1) % 5;
    sendDataViaSocket(CMD_HEARTBEAT,"1234");

}

static err_t connection_callback(void *arg, struct tcp_pcb *pcb, err_t err) {
    if (err != ERR_OK) {
        tcp_abort(pcb);
        tcp_pcb = NULL;
        return ERR_ABRT;
    }
    isConnected = 1;
    /* Clear the connection... message by printing an empty line */
    DPRINTF("");
    return ERR_OK;
}

static void error_callback(void *arg, err_t err) {
    isConnected = 0;
    if (notifyOnCard) {
        notifyOnCard = 0;
        card_received_post();
    }
    tcp_pcb = NULL;
}

static err_t recv_callback(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err) {
    uintptr_t id = (uintptr_t)arg;
    if (err != ERR_OK || p == NULL || id != connectionCounter) {
        tcp_abort(pcb);
        isConnected = 0;
        if (notifyOnCard) {
            notifyOnCard = 0;
            card_received_post();
        }
        tcp_pcb = NULL;
        return ERR_ABRT;
    }
    /* process the input. we assume that all data for a request comes at once */
    char buf[1024];
    int len = MIN(p->len, sizeof(buf) - 1);
    memcpy(buf, p->payload, len);
    buf[len] = NULL;
    readDataFromNetwork(buf);
    pbuf_free(p);
    return ERR_OK;
}

static int attempt_to_connect() {
    err_t err;
    if (!network_connected) {
        return -1;
    }
    if (isConnected) {
        return 0;
    }
    /* connection attempt already in progress? */
    if (tcp_pcb) {
        return -1;
    }
    tcp_pcb = tcp_new();
    if (!tcp_pcb) {
        DIE("Failed to allocate tcp pcb");
    }
    tcp_pcb->so_options |= SOF_KEEPALIVE;
    connectionCounter++;
    tcp_arg(tcp_pcb, (void*)connectionCounter);
    ip_addr_t addr;
    addr.addr = inet_addr("10.13.0.32");
    err = tcp_connect(tcp_pcb, &addr, 3737, connection_callback);
    if (err != ERR_OK) {
        DIE("Failed to enqueue TCP connection request");
    }
    DPRINTF("Connecting to server...");
    tcp_err(tcp_pcb, error_callback);
    tcp_recv(tcp_pcb, recv_callback);
    return -1;
}

int run() {
    while (1) {
        /* wait for timer interrupt */
        timer_update_wait();
        lock_lock();
        sys_check_timeouts();
        if (notifyOnCard && (timer_get_time() - notifyOnCardStarted) > 1000) {
            notifyOnCard = 0;
            card_received_post();
            tcp_abort(tcp_pcb);
            tcp_pcb = NULL;
            isConnected = 0;
        }
        attempt_to_connect();
        lock_unlock();
    }
    return 0;
}

static lwip_iface_t *fec_iface = NULL;

void irq_handle(void)
{
	lock_lock();
    assert(fec_iface->netif);
	ethif_lwip_handle_irq(fec_iface, 150);

    irq_acknowledge();
    lock_unlock();
}

static void *ioremap(void *cookie, uintptr_t paddr, size_t size, int cached, ps_mem_flags_t flags)
{
        switch (paddr) {
		case 0x2188000:
			return (void*)mmio_base;
		case 0x21BC000:
			return (void*)ocotp_base;
		case 0x20E0000:
			return (void*)iomux_base;
		case 0x20C4000:
			return (void*)ccm_base;
		case 0x20C8000:
			return (void*)analog_base;
		case 0x20A4000:
			return (void*)gpio3_base;
		case 0x20B0000:
			return (void*)gpio6_base;
		default:
			return NULL;
	}
}

static void iounmap(void *cookie UNUSED, void *vaddr UNUSED, size_t size UNUSED)
{
}

uint32_t sys_now(void)
{
    return timer_get_time();
}

void netif_link_callback(struct netif *netif) {
    if (netif_is_link_up(netif)) {
        network_connected = 1;
        attempt_to_connect();
    } else {
        network_connected = 0;
        if (notifyOnCard) {
            notifyOnCard = 0;
            card_received_post();
        }
    }
}

void pre_init(void)
{
	ps_io_ops_t io_ops;

	/* Use CAmkES DMA allocator. */
	if (camkes_dma_manager(&io_ops.dma_manager) != 0) {
        ZF_LOGF("Failed to initialise DMA manager");
    }

	io_ops.io_mapper.io_map_fn = ioremap;
	io_ops.io_mapper.io_unmap_fn = iounmap;

    lwip_init();

	/* Initialize network interface. */
	fec_iface = ethif_new_lwip_driver(io_ops, NULL, ethif_imx6_init, NULL);
    if (!fec_iface) {
        ZF_LOGF("Failed to initialize driver");
    }

    struct netif *netif = malloc(sizeof(struct netif));
    if (!netif) {
        ZF_LOGF("Failed to allocate netif");
    }

	netif_add(netif, NULL, NULL, NULL, fec_iface,
		  ethif_get_ethif_init(fec_iface), ethernet_input);
    netif_set_default(fec_iface->netif);
    netif_set_status_callback(fec_iface->netif, netif_link_callback);
    int err = dhcp_start(fec_iface->netif);
    DPRINTF("Waiting for DHCP...");
    ZF_LOGF_IF(err != ERR_OK, "Failed to start dhcp");
}
