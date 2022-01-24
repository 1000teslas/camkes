/*
 * Copyright 2014, NICTA
 *
 * This software may be distributed and modified according to the terms of
 * the BSD 2-Clause license. Note that NO WARRANTY is provided.
 * See "LICENSE_BSD2.txt" for details.
 *
 * @TAG(NICTA_BSD)
 */

#include <autoconf.h>
#include <stdio.h>
#include <string.h>
#include <utils/util.h>

/* Common headers */
#include <util.h>

/* CAmKEs headers */
#include <camkes.h>

/* Register definitions */
#define URXD  0x00 /* Receiver Register */
#define UTXD  0x40 /* Transmitter Register */
#define UCR1  0x80 /* Control Register 1 */
#define UCR2  0x84 /* Control Register 2 */
#define UCR3  0x88 /* Control Register 3 */
#define UCR4  0x8c /* Control Register 4 */
#define UFCR  0x90 /* FIFO Control Register */
#define USR1  0x94 /* Status Register 1 */
#define USR2  0x98 /* Status Register 2 */
#define UESC  0x9c /* Escape Character Register */
#define UTIM  0xa0 /* Escape Timer Register */
#define UBIR  0xa4 /* BRM Incremental Register */
#define UBMR  0xa8 /* BRM Modulator Register */
#define UBRC  0xac /* Baud Rate Count Register */
#define ONEMS 0xb0 /* One Millisecond register */
#define UTS   0xb4 /* UART Test Register */
#define UMCR  0xb8 /* RS-485 Mode Control Register */

/* UART Control Register Bit Fields.*/
#define  URXD_CHARRDY    (1<<15)
#define  URXD_ERR        (1<<14)
#define  URXD_OVRRUN     (1<<13)
#define  URXD_FRMERR     (1<<12)
#define  URXD_BRK        (1<<11)
#define  URXD_PRERR      (1<<10)
#define  UCR1_ADEN       (1<<15) /* Auto detect interrupt */
#define  UCR1_ADBR       (1<<14) /* Auto detect baud rate */
#define  UCR1_TRDYEN     (1<<13) /* Transmitter ready interrupt enable */
#define  UCR1_IDEN       (1<<12) /* Idle condition interrupt */
#define  UCR1_RRDYEN     (1<<9)	 /* Recv ready interrupt enable */
#define  UCR1_RDMAEN     (1<<8)	 /* Recv ready DMA enable */
#define  UCR1_IREN       (1<<7)	 /* Infrared interface enable */
#define  UCR1_TXMPTYEN   (1<<6)	 /* Transimitter empty interrupt enable */
#define  UCR1_RTSDEN     (1<<5)	 /* RTS delta interrupt enable */
#define  UCR1_SNDBRK     (1<<4)	 /* Send break */
#define  UCR1_TDMAEN     (1<<3)	 /* Transmitter ready DMA enable */
#define  UCR1_DOZE       (1<<1)	 /* Doze */
#define  UCR1_UARTEN     (1<<0)	 /* UART enabled */
#define  UCR2_ESCI     	 (1<<15) /* Escape seq interrupt enable */
#define  UCR2_IRTS  	 (1<<14) /* Ignore RTS pin */
#define  UCR2_CTSC  	 (1<<13) /* CTS pin control */
#define  UCR2_CTS        (1<<12) /* Clear to send */
#define  UCR2_ESCEN      (1<<11) /* Escape enable */
#define  UCR2_PREN       (1<<8)  /* Parity enable */
#define  UCR2_PROE       (1<<7)  /* Parity odd/even */
#define  UCR2_STPB       (1<<6)	 /* Stop */
#define  UCR2_WS         (1<<5)	 /* Word size */
#define  UCR2_RTSEN      (1<<4)	 /* Request to send interrupt enable */
#define  UCR2_ATEN       (1<<3)  /* Aging Timer Enable */
#define  UCR2_TXEN       (1<<2)	 /* Transmitter enabled */
#define  UCR2_RXEN       (1<<1)	 /* Receiver enabled */
#define  UCR2_SRST 	 (1<<0)	 /* SW reset */
#define  UCR3_DTREN 	 (1<<13) /* DTR interrupt enable */
#define  UCR3_PARERREN   (1<<12) /* Parity enable */
#define  UCR3_FRAERREN   (1<<11) /* Frame error interrupt enable */
#define  UCR3_DSR        (1<<10) /* Data set ready */
#define  UCR3_DCD        (1<<9)  /* Data carrier detect */
#define  UCR3_RI         (1<<8)  /* Ring indicator */
#define  UCR3_TIMEOUTEN  (1<<7)  /* Timeout interrupt enable */
#define  UCR3_RXDSEN	 (1<<6)  /* Receive status interrupt enable */
#define  UCR3_AIRINTEN   (1<<5)  /* Async IR wake interrupt enable */
#define  UCR3_AWAKEN	 (1<<4)  /* Async wake interrupt enable */
#define  UCR3_RXDMUXSEL	 (1<<2)  /* RXD Muxed Input Select */
#define  UCR3_INVT  	 (1<<1)  /* Inverted Infrared transmission */
#define  UCR3_BPEN  	 (1<<0)  /* Preset registers enable */
#define  UCR4_CTSTL_SHF  10      /* CTS trigger level shift */
#define  UCR4_CTSTL_MASK 0x3F    /* CTS trigger is 6 bits wide */
#define  UCR4_INVR  	 (1<<9)  /* Inverted infrared reception */
#define  UCR4_ENIRI 	 (1<<8)  /* Serial infrared interrupt enable */
#define  UCR4_WKEN  	 (1<<7)  /* Wake interrupt enable */
#define  UCR4_REF16 	 (1<<6)  /* Ref freq 16 MHz */
#define  UCR4_IRSC  	 (1<<5)  /* IR special case */
#define  UCR4_TCEN  	 (1<<3)  /* Transmit complete interrupt enable */
#define  UCR4_BKEN  	 (1<<2)  /* Break condition interrupt enable */
#define  UCR4_OREN  	 (1<<1)  /* Receiver overrun interrupt enable */
#define  UCR4_DREN  	 (1<<0)  /* Recv data ready interrupt enable */
#define  UFCR_RXTL_SHF   0       /* Receiver trigger level shift */
#define  UFCR_DCEDTE	 (1<<6)  /* DCE/DTE mode select */
#define  UFCR_RFDIV      (1<<7)  /* Reference freq divider mask(fixed divisor 5)*/
#define  UFCR_TXTL_SHF   10      /* Transmitter trigger level shift */
#define  USR1_PARITYERR  (1<<15) /* Parity error interrupt flag */
#define  USR1_RTSS  	 (1<<14) /* RTS pin status */
#define  USR1_TRDY  	 (1<<13) /* Transmitter ready interrupt/dma flag */
#define  USR1_RTSD  	 (1<<12) /* RTS delta */
#define  USR1_ESCF  	 (1<<11) /* Escape seq interrupt flag */
#define  USR1_FRAMERR    (1<<10) /* Frame error interrupt flag */
#define  USR1_RRDY       (1<<9)	 /* Receiver ready interrupt/dma flag */
#define  USR1_TIMEOUT    (1<<7)	 /* Receive timeout interrupt status */
#define  USR1_RXDS  	 (1<<6)	 /* Receiver idle interrupt flag */
#define  USR1_AIRINT	 (1<<5)	 /* Async IR wake interrupt flag */
#define  USR1_AWAKE 	 (1<<4)	 /* Aysnc wake interrupt flag */
#define  USR2_ADET  	 (1<<15) /* Auto baud rate detect complete */
#define  USR2_TXFE  	 (1<<14) /* Transmit buffer FIFO empty */
#define  USR2_DTRF  	 (1<<13) /* DTR edge interrupt flag */
#define  USR2_IDLE  	 (1<<12) /* Idle condition */
#define  USR2_IRINT 	 (1<<8)	 /* Serial infrared interrupt flag */
#define  USR2_WAKE  	 (1<<7)	 /* Wake */
#define  USR2_RTSF  	 (1<<4)	 /* RTS edge interrupt flag */
#define  USR2_TXDC  	 (1<<3)	 /* Transmitter complete */
#define  USR2_BRCD  	 (1<<2)	 /* Break condition */
#define  USR2_ORE        (1<<1)	 /* Overrun error */
#define  USR2_RDR        (1<<0)	 /* Recv data ready */
#define  UTS_FRCPERR	 (1<<13) /* Force parity error */
#define  UTS_LOOP        (1<<12) /* Loop tx and rx */
#define  UTS_TXEMPTY	 (1<<6)	 /* TxFIFO empty */
#define  UTS_RXEMPTY	 (1<<5)	 /* RxFIFO empty */
#define  UTS_TXFULL 	 (1<<4)	 /* TxFIFO full */
#define  UTS_RXFULL 	 (1<<3)	 /* RxFIFO full */
#define  UTS_SOFTRST	 (1<<0)	 /* Software reset */

/* Clock definitions(configurable via CMM) */
#define UART_PERCLK      80      /* UART peripheral clock in MHz(default) */
#define UART_MODCLK      66      /* UART module clock in MHz(default) */

/** UART driver */
#define TXBUF_SIZE       128     /* Transmit buffer size */
#define RXBUF_SIZE       128     /* Receiving buffer size */
struct ring_buf {
	unsigned char *buf;
	int           head;
	int           tail;
};

struct uart_port {
	unsigned char   *base;   /** UART base memory address */
	struct ring_buf tx;      /** Transmit buffer */
	struct ring_buf rx;      /** Receiving buffer */
	uint32_t        timeout; /** Unused */
	int             type;    /* RS-232/RS485/IrDA */
};
static struct uart_port *imx_port;

/* Default serial configuration */
#define DEF_BAUD         115200
#define DEF_PARITY       0
#define DEF_STOP         1
#define DEF_WORDSIZE     8

/* Message level */
#ifdef DEBUGLEVEL
    #undef DEBUGLEVEL
#endif
#define DEBUGLEVEL DBG_WARN

#if 0
/**
 * Print UART registers.
 */
static void print_reg(void)
{
	D(DBG_INFO, "UCR1: 0x%x\n", readl(imx_port->base + UCR1));
	D(DBG_INFO, "UCR2: 0x%x\n", readl(imx_port->base + UCR2));
	D(DBG_INFO, "UCR3: 0x%x\n", readl(imx_port->base + UCR3));
	D(DBG_INFO, "UCR4: 0x%x\n", readl(imx_port->base + UCR4));

	D(DBG_INFO, "UFCR: 0x%x\n", readl(imx_port->base + UFCR));

	D(DBG_INFO, "USR1: 0x%x\n", readl(imx_port->base + USR1));
	D(DBG_INFO, "USR2: 0x%x\n", readl(imx_port->base + USR2));

	D(DBG_INFO, "UESC: 0x%x\n", readl(imx_port->base + UESC));
	D(DBG_INFO, "UTIM: 0x%x\n", readl(imx_port->base + UTIM));
	D(DBG_INFO, "UBIR: 0x%x\n", readl(imx_port->base + UBIR));
	D(DBG_INFO, "UBMR: 0x%x\n", readl(imx_port->base + UBMR));
	D(DBG_INFO, "UBRC: 0x%x\n", readl(imx_port->base + UBRC));
	D(DBG_INFO, "ONEMS:0x%x\n", readl(imx_port->base + ONEMS));
	D(DBG_INFO, "UTS : 0x%x\n", readl(imx_port->base + UTS));
	D(DBG_INFO, "UMCR: 0x%x\n", readl(imx_port->base + UMCR));
}
#endif

int serial_set_speed(unsigned int baud);
void serial_set_parity(int enable, int check);
void serial_set_stop(int bits);
void serial_set_wordsize(int size);
static void serial_reset(struct uart_port *port);
void serial_set_wordsize_locked(int size);
void serial_set_stop_locked(int bits);
void serial_set_parity_locked(int enable, int check);
int serial_set_speed_locked(unsigned int baud);

/**
 * Fix a bug in u-boot.
 *
 * U-Boot(with Android fastboot support) which we are using currently,
 * enables UART1, but it doesn't set the input multiplexer correct.
 */
static void fix_iomux(void)
{
#define IOMUXC_UART1_UART_RX_DATA_SELECT_INPUT 0x0920
#define IOMUXC_SW_MUX_CTL_PAD_SD3_DATA7 0x02a8
#define IOMUXC_SW_MUX_CTL_PAD_SD3_DATA6 0x02ac

	unsigned char *base = (unsigned char*)iomux;
	writel(0x03, base + IOMUXC_UART1_UART_RX_DATA_SELECT_INPUT); /*
	writel(0x11, base + IOMUXC_SW_MUX_CTL_PAD_SD3_DATA7);
	writel(0x11, base + IOMUXC_SW_MUX_CTL_PAD_SD3_DATA6);
	int datMux7 = readl(base + IOMUXC_SW_MUX_CTL_PAD_SD3_DATA7);
	int datMux6 = readl(base + IOMUXC_SW_MUX_CTL_PAD_SD3_DATA6);
	printf("datMux6 = %d, datMux7 = %d\n", datMux6, datMux7); */
}

static bool serial_initialised = false;

void serial_ready(void) {
    serial_mutex_lock();
    bool init = serial_initialised;
    serial_mutex_unlock();
    if (!init) {
        serial_ready_wait();
        /* For reasons unknown we need to wait a bit longer here before returning */
        for (int i = 0; i < 100; i++) {
            seL4_Yield();
        }
    }
}

void lock_start(void) {
    serial_mutex_lock();
	fix_iomux();

	D(DBG_INFO, "uart port init\n");

	uint32_t val;

	/* Initialize driver structures */
	imx_port = (struct uart_port*)malloc(sizeof(struct uart_port));
	imx_port->base = (unsigned char*)mem;
	imx_port->tx.buf = (unsigned char*)malloc(TXBUF_SIZE);
	imx_port->tx.head = 0;
	imx_port->tx.tail = 0;
	imx_port->rx.buf = (unsigned char*)malloc(RXBUF_SIZE);
	imx_port->rx.head = 0;
	imx_port->rx.tail = 0;
	serial_reset(imx_port);

	/*
	 * FIFO control: reference clock divider is always 5,
	 * stay in DCE mode.
	 */
	val = ((2 << UFCR_TXTL_SHF) | UFCR_RFDIV | (1 << UFCR_RXTL_SHF));
	writel(val, imx_port->base + UFCR);

	/* Clear status register */
	writel(USR1_RTSD, imx_port->base + USR1);

	/* Enable interrupts and the UART */
	val = (UCR1_RRDYEN | UCR1_UARTEN);
	writel(val, imx_port->base + UCR1);

	/* Enable transmitter and receiver */
	val = readl(imx_port->base + UCR2);
	val |= (UCR2_IRTS | UCR2_RXEN | UCR2_TXEN | UCR2_SRST);
	writel(val, imx_port->base + UCR2);

	/* According to the hardware spec, this bit must always be set. */
	val = readl(imx_port->base + UCR3);
	val |= UCR3_RXDMUXSEL;
	writel(val & ~UCR3_AWAKEN, imx_port->base + UCR3);

	/* Set default baud speed, parity, stop bits and word size. */
	serial_set_speed_locked(DEF_BAUD);
	serial_set_parity_locked(DEF_PARITY, 0);
	serial_set_stop_locked(DEF_STOP);
	serial_set_wordsize_locked(DEF_WORDSIZE);

    serial_initialised = true;
    serial_ready_post();
    serial_mutex_unlock();
}

/**
 * Data received.
 */
static void serial_rx(struct uart_port *port)
{
	D(DBG_INFO, "\n");

	uint32_t ch, val;

	while (readl(port->base + USR2) & USR2_RDR) {
		ch = readl(port->base + URXD);

		/* Detect BREAK condition */
		val = readl(port->base + USR2);
		if (val & USR2_BRCD) {
			writel(USR2_BRCD, port->base + USR2);
		}

		/* Drop the character when overflow */
		if ((port->rx.tail + 1) % RXBUF_SIZE == port->rx.head) {
			D(DBG_WARN, "Rx buffer overflow\n");
			continue;
		}
		port->rx.buf[port->rx.tail] = ch;
		if (port->rx.head == port->rx.tail) {
		    /* went from no data to some data */
		    has_data_emit();
		}
		port->rx.tail = (port->rx.tail + 1) % RXBUF_SIZE;
	}
}

/**
 * Do real transmit.
 */
static void serial_transmit(struct uart_port *port)
{
	uint32_t val;
	struct ring_buf *xmit = &port->tx;
	val = readl(port->base + UTS);
	while ((xmit->head != xmit->tail) && !(val & UTS_TXFULL)) {
		writel(xmit->buf[xmit->head], port->base + UTXD);
		xmit->head = (xmit->head + 1) % TXBUF_SIZE;
		val = readl(port->base + UTS);
	}
	if (xmit->head == xmit->tail) {
		val = readl(port->base + UCR1);
		writel(val & ~UCR1_TXMPTYEN, port->base + UCR1);
	}
}

/**
 * Drain TXFIFO
 */
static void serial_drain(struct uart_port *port)
{
	while (!(readl(port->base + USR2) & USR2_TXDC));
}

/**
 * Data transmitted.
 */
static void serial_tx(struct uart_port *port)
{
	D(DBG_INFO, "\n");

	serial_transmit(port);
}

/**
 * Interrupt dispatcher
 */
void intr_handle(void)
{

	uint32_t status;
	serial_mutex_lock();

	status = readl(imx_port->base + USR1);

	D(DBG_INFO, "Intr fired: %u\n", status);

	if (status & USR1_RRDY) {
		serial_rx(imx_port);
	}

	if (status & USR1_TRDY &&
		    readl(imx_port->base + UCR1) & UCR1_TXMPTYEN) {
		serial_tx(imx_port);
	}

	if (status & USR1_AWAKE) {
		writel(USR1_AWAKE, imx_port->base + USR1);

	}

    intr_acknowledge();
	serial_mutex_unlock();
}

/**
 * Serial port software reset.
 */
static void serial_reset(struct uart_port *port)
{
	uint32_t val = 0;
    while (val == 0) {
        val = readl(port->base + UCR2);
    }
	writel(val & ~UCR2_SRST, port->base + UCR2);

	while (readl(port->base + UTS) & UTS_SOFTRST);
}

static void serial_irq_enable(struct uart_port *port)
{
	uint32_t val;

	val = readl(port->base + UCR1);
	val |= (UCR1_RRDYEN | UCR1_RTSDEN);
	writel(val, imx_port->base + UCR1);
}

static void serial_irq_disable(struct uart_port *port)
{
	uint32_t val;

	val = readl(port->base + UCR1);
	val &= ~(UCR1_RRDYEN | UCR1_RTSDEN);
	writel(val, imx_port->base + UCR1);
}

/**
 * Enable transmitter and receiver.
 */
static void serial_enable(struct uart_port *port)
{
	uint32_t val;

	val = readl(port->base + UCR2);
	val |= (UCR2_TXEN | UCR2_RXEN);
	writel(val, port->base + UCR2);
}

/**
 * Disable transmitter and receiver.
 */
static void serial_disable(struct uart_port *port)
{

	uint32_t val;

	serial_drain(port);

	val = readl(port->base + UCR2);
	val &= ~(UCR2_TXEN | UCR2_RXEN);
	writel(val, port->base + UCR2);
}

/**
 * Write to serial port
 */
int serial_write(unsigned int dummy, int count)
{

	D(DBG_LOG, "buf(%p), size(%d)", buf, count);
	int space;         // Space left in the tx buffer.
	int space_to_end;  // Space left to the end of the buffer.
	uint32_t val;
	struct ring_buf *xmit = &imx_port->tx;

	if (count <= 0) {
		return 0;
	}

	serial_mutex_lock();
	/* See if we have enough space */
	space = xmit->head - xmit->tail;
	if (space <= 0) {
		space += TXBUF_SIZE;
	}
	D(DBG_LOG, "TX buf space: %d\n", space);

	if (count > space) {
		count = space;
	}
	/*
	 * If tail is smaller than head,
	 * count will always smaller than space_to_end.
	 * So there is no need to worry about overlap.
	 */
	space_to_end = TXBUF_SIZE - xmit->tail;
	if (count <= space_to_end) {
		memcpy(&xmit->buf[xmit->tail], (void*)buf, count);
	} else {
		memcpy(&xmit->buf[xmit->tail], (void*)buf, space_to_end);
		memcpy(&xmit->buf[0],
			(char*)buf + space_to_end,
			count - space_to_end);
	}
	xmit->tail = (xmit->tail + count) % TXBUF_SIZE;
	/* Start tx */
	val = readl(imx_port->base + UCR1);
	writel(val | UCR1_TXMPTYEN, imx_port->base + UCR1);

    serial_transmit(imx_port);
	serial_mutex_unlock();
	return count;
}

/**
 * Read from serial port
 */
int serial_read(unsigned int dummy, int count)
{

	D(DBG_LOG, "buf(%p), size(%d)", buf, count);
	int num;
	int num_to_end;  // Data left to the end of the buffer.
	struct ring_buf *rx = &imx_port->rx;
	if (count <= 0) {
		return 0;
	}
	serial_mutex_lock();
	/* Empty buffer */
	if (rx->head == rx->tail) {
	    serial_mutex_unlock();
		return 0;
	}
	num = rx->tail - rx->head;
	if (num <= 0) {
		num += TXBUF_SIZE;
	}
	if (num < count) {
		count = num;
	}
	/*
	 * If head is smaller than tail,
	 * count will always smaller than num_to_end.
	 * So there is no need to worry about overlap.
	 */
	num_to_end = RXBUF_SIZE - rx->head;
	if (count <= num_to_end) {
		memcpy((void*)buf, &rx->buf[rx->head], count);
	} else {
		memcpy((void*)buf, &rx->buf[rx->head], num_to_end);
		memcpy((char*)buf + num_to_end,
			&rx->buf[0],
			count - num_to_end);
	}
	rx->head = (rx->head + count) % RXBUF_SIZE;
	serial_mutex_unlock();
	return count;
}

int serial_set_speed(unsigned int baud) {
    serial_mutex_lock();
    int ret = serial_set_speed_locked(baud);
    serial_mutex_unlock();
    return ret;
}

/**
 * Set Baud speed.
 *
 * Ugly hard-coded, but on Sabra, the UART reference clock is NOT adjustable,
 * we use a fixed divisor, and there is no demand for random baud rate currently.
 * So, I go for the easiest way.
 *
 * @param baud Baud speed per second(bps).
 */
int serial_set_speed_locked(unsigned int baud)
{

	int val = 0;
	switch (baud) {
		case 115200:
			val = 0x47;
			break;
		case 57600:
			val = 0x23;
			break;
		case 38400:
			val = 0x17;
			break;
		case 19200:
			val = 0x0B;
			break;
		case 9600:
			val = 0x05;
			break;
		default:
			D(DBG_WARN, "Baud rate unsupported!\n");
			return -1;
	}

	serial_irq_disable(imx_port);
	serial_drain(imx_port);
	serial_disable(imx_port);

	writel(val, imx_port->base + UBIR);
	writel(0x270, imx_port->base + UBMR);
	writel(UART_PERCLK * 1000 / 5 , imx_port->base + ONEMS);

	serial_enable(imx_port);
	serial_irq_enable(imx_port);

	return 0;
}

void serial_set_parity(int enable, int check)
{
    serial_mutex_lock();
    serial_set_parity_locked(enable, check);
    serial_mutex_unlock();
}

/**
 * Parity control.
 *
 * @param enable Enable/Disable parity generator(0 -- Disable, 1 -- Enable).
 * @param check  Parity checker(0 -- Even, 1 -- Odd).
 *
 * @attention: Any unexpected value will be ignored.
 */
void serial_set_parity_locked(int enable, int check)
{

	uint32_t val;

	serial_irq_disable(imx_port);
	serial_drain(imx_port);
	serial_disable(imx_port);

	val = readl(imx_port->base + UCR2);

	if (enable == 0) {
		val &= ~UCR2_PREN;
	} else if (enable == 1) {
		val |= UCR2_PREN;
		if (check == 0) {
			val &= ~UCR2_PROE;
		} else if (check == 1) {
			val |= UCR2_PROE;
		}
	}

	writel(val, imx_port->base + UCR2);

	serial_enable(imx_port);
	serial_irq_enable(imx_port);
}

void serial_set_stop(int bits)
{
    serial_mutex_lock();
    serial_set_stop_locked(bits);
    serial_mutex_unlock();
}

/**
 * Set stop bits.
 *
 * @param bits Number of stop bits after a character(1 -- 1 bit, 2 -- 2 bits).
 */
void serial_set_stop_locked(int bits)
{
	uint32_t val;

	serial_irq_disable(imx_port);
	serial_drain(imx_port);
	serial_disable(imx_port);

	val = readl(imx_port->base + UCR2);
	if (bits == 1) {
		val &= ~UCR2_STPB;
	} else if (bits == 2) {
		val |= UCR2_STPB;
	} else {
		return;
	}
	writel(val, imx_port->base + UCR2);

	serial_enable(imx_port);
	serial_irq_enable(imx_port);
}

void serial_set_wordsize(int size)
{
    serial_mutex_lock();
    serial_set_wordsize_locked(size);
    serial_mutex_unlock();
}

/**
 * Set serial word size.
 *
 * @param size Word size(Any size other than 7 and 8 will be ignored.
 */
void serial_set_wordsize_locked(int size)
{

	uint32_t val;

	serial_irq_disable(imx_port);
	serial_drain(imx_port);
	serial_disable(imx_port);

	val = readl(imx_port->base + UCR2);
	if (size == 7) {
		val &= ~UCR2_WS;
	} else if (size == 8) {
		val |= UCR2_WS;
	} else {
		return;
	}
	writel(val, imx_port->base + UCR2);

	serial_enable(imx_port);
	serial_irq_enable(imx_port);
}
