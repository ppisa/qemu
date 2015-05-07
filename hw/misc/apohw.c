/*
 *  Virtual I/O PCI card + simple terminal connected
 *  to this card via 8bit parallel bus.
 *  Hardware used for teaching A0B36APO:
 *  https://edux.feld.cvut.cz/courses/A0B36APO/tutorials/10/start
 *
 *  Authors: Rostislav Lisovy (lisovy@gmail.com)
 *           Pavel Pisa (pisa@cmp.felk.cvut.cz)
 *
 *  Licensed under GPLv3 license
 *
 *  Nomenclature:
 *   * APOHW -- Complete virtual HW to be used in A0B36APO class
 *   * APOIO -- PCI(e) I/O card
 *   * APOTERM -- Virtual "terminal" (Text LCD + matrix keyboard)
 *                connected to APOIO
*/

#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "qemu/event_notifier.h"
#include "qemu/osdep.h"
#include "qemu/thread.h"
#include "qemu/sockets.h"

#include <unistd.h>
#include <string.h>
#include <sys/types.h>

#define TYPE_APOHW_DEV "apohw"

#define APOHW_DEV(obj) \
    OBJECT_CHECK(apohw_state_t, (obj), TYPE_APOHW_DEV)

#define PCI_VENDOR_ID_ALTERA				0x1172
#define PCI_DEVICE_ID_ALTERA_CORPORATION_DEVICE		0x1f32

/* Memory locations of particular registers located in PCI(e) IO card */
#define APOIO_RAM_START					0x0
#define APOIO_RAM_SIZE					0x8000
#define APOIO_EMUL_BUS_CTRL				0x8080
#define APOIO_EMUL_BUS_ADDR				0x8060
#define APOIO_EMUL_BUS_DATA_OUT				0x8020
#define APOIO_EMUL_BUS_DATA_IN				0x8040
#define APOIO_LED_PIO					0x80A0

/* Masks for individual bits in emul_bus_addr */
#define APOTERM_ADDR_m					(1 | 1 << 1)

/* Decoding of 'fields' in emul_bus_ctrl */
#define APOTERM_RD_m					(1 << 0)
#define APOTERM_WR_m					(1 << 1)
#define APOTERM_CS0_m					(1 << 6)
#define APOTERM_PWR_m					(1 << 7)

/* Decoding of ADDR0 ADDR1 of emul_bus_ctrl */
#define APOTERM_WR_LCD_INST				0x0
#define APOTERM_WR_LED_WR				0x1
#define APOTERM_WR_LCD_WDATA				0x2
#define APOTERM_WR_KBD_WR				0x3
#define APOTERM_RD_KBD_RD				0x0
#define APOTERM_RD_LCD_STAT				0x1
#define APOTERM_RD_LCD_RDATA				0x3

/**** HD44780 ****/
#define APOTERM_HD44780_DISP_COLS			16
#define APOTERM_HD44780_DISP_LINES			2

/* RW bit */
#define APOTERM_HD44780_RD				0x1
#define APOTERM_HD44780_WR				0x0

/* RS bit */
#define APOTERM_HD44780_DATREG				0x1
#define APOTERM_HD44780_INSTREG				0x0

/* Data bits */
#define APOTERM_HD44780_CLEAR_DISP_m			(1 << 0)
#define APOTERM_HD44780_RETURN_HOME_m			(1 << 1)
#define APOTERM_HD44780_ENTRY_MODE_SET_m		(1 << 2)
#define APOTERM_HD44780_DISPLAY_ON_OFF_m		(1 << 3)
#define APOTERM_HD44780_CURSOR_DISPLAY_SHIFT_m		(1 << 4)
#define APOTERM_HD44780_FUNCTION_SET_m			(1 << 5)
#define APOTERM_HD44780_SET_CGRAM_ADDR_m		(1 << 6)
#define APOTERM_HD44780_SET_DDRAM_ADDR_m		(1 << 7)

/**** Keyboard parameters ****/

#define APOTERM_KBD_SCAN_LINES				3

typedef struct apoio_state_t {
	MemoryRegion bar0;
	uint8_t ram[APOIO_RAM_SIZE];
	uint8_t led_pio;
	//uint8_t timer_0;
	uint8_t emul_bus_ctrl_old;
	uint8_t emul_bus_ctrl_new;
	uint8_t emul_bus_addr;
	uint8_t emul_bus_data_out;
	uint8_t emul_bus_data_in;
} apoio_state_t;

typedef struct {
	int poweron;
	struct hd44780 {
		/* Shifting not supported
		   CGRAM not supported
		   4-bit interface not supported */

		/* TO BE IMPLEMENTED IN FUTURE?
		int CGROM[208+32];
		int CGRAM[16];
		int CGRAM_addr;
		*/
		int busyflag;
		int cursor_incr; /* Sets cursor move direction. This operation
				    is performed during data write and read. */

		int display_lines; /* Real count of display lines -- not the "N" bit*/
		int display_on;

#define APOTERM_HD44780_DDRAM_LINE1_START			0x0
#define APOTERM_HD44780_DDRAM_LINE2_START			0x40
#define APOTERM_HD44780_DDRAM_LINE_SIZE				0x27
#define APOTERM_HD44780_DDRAM_SIZE				0x80
		uint8_t DDRAM[APOTERM_HD44780_DDRAM_SIZE]; /* Always this size
					-- not corresponding to real display size */
		unsigned int DDRAM_addr;
	} lcd;
	uint8_t led;		/* LED reg */
	uint8_t kbd_scan_select;
	uint8_t kbd_scan_response[APOTERM_KBD_SCAN_LINES];
} apoterm_state_t;

typedef struct {
	/*< private >*/
	PCIDevice dev;
	/*< public >*/
	apoio_state_t apoio;
	apoterm_state_t apoterm;

	int socket_srv;
	int socket_tmp;
	uint32_t port;
	int telnet_iac_rx_state;
	int addr;
} apohw_state_t;

#ifndef TELNET_WILL
#define TELNET_WILL              251
#define TELNET_WONT              252
#define TELNET_DO                253
#define TELNET_DONT              254
#endif
#ifndef TELNET_IAC
#define TELNET_IAC               255
#endif
#ifndef TELNET_ECHO
#define TELNET_ECHO              1
#endif
#ifndef TELNET_SUPPRESS_GO_AHEAD
#define TELNET_SUPPRESS_GO_AHEAD 3
#endif
#ifndef TELNET_LINEMODE
#define TELNET_LINEMODE          34
#endif
#ifndef TELNET_SB
#define TELNET_SB                250
#endif
#ifndef TELNET_SE
#define TELNET_SE                240
#endif

/* IAC WILL ECHO IAC WILL SUPPRESS_GO_AHEAD IAC WONT LINEMODE */
/* 255  251    1 255  251                 3 255  252       34 */

char apohw_telnet_raw_mode[]={
  TELNET_IAC, TELNET_WILL, TELNET_ECHO,
  TELNET_IAC, TELNET_WILL, TELNET_SUPPRESS_GO_AHEAD,
  TELNET_IAC, TELNET_WILL, TELNET_LINEMODE,
};


int instance = 0; /* Shared among multiple APOHW devices */
#define DEFAULT_PORT			55555
#define STR_BUFF			256 // FIXME?
#define DEBUG_APOHW 			1
#undef DEBUG_APOHW

#define DEBUG_PREFIX			"APOHW: "
#ifdef DEBUG_APOHW
	#define DEBUG_PRINT(fmt, args...) \
		fprintf(stderr, DEBUG_PREFIX fmt, ## args)
#else
	#define DEBUG_PRINT(fmt, args...)
#endif

/* ------------------------------------------------------------------------- */

static void apoterm_kbd_process_char(apoterm_state_t *apoterm, int ch);
static void apoterm_display_update(apohw_state_t *d);

static int socket_write(apohw_state_t *d, const char* str, int len)
{
	int ret;
	int written = 0;

	if(d->socket_tmp == -1)
		return 0;

	do {
		ret = send(d->socket_tmp, str, len, 0);
		if (ret < 0) {
			if (socket_error() == -EINTR)
				continue;

			DEBUG_PRINT("Error writing into socket. "
				"Is there any client connected?\n");
			if (!ret)
				return -1;
		}
		if (ret <= 0)
			break;

		len -= ret;
		str += ret;
		written += ret;
	} while (len > 0);

	return written;
}

static void socket_read(apohw_state_t* d)
{
	char read_buffer[STR_BUFF];
	//char reg[STR_BUFF+1];
	//unsigned int val;
	int len = 0;
	//int ret;
	int i;

	while(1) {
		if (d->socket_tmp == -1)
			return;

		len = recv(d->socket_tmp, read_buffer, STR_BUFF-1, 0);
		if (len < 0) {
			perror("read()");
			return;
		}

		if (len == 0) {
			DEBUG_PRINT("Error while reading from socket. "
				 "Client disconnected?\n");
			return;
		}
		read_buffer[len] = '\0';


	//	ret = sscanf(read_buffer, "%[A-Z0-9]=%u", reg, &val);
	//	if (ret == 2) {
	//		if(!strcmp(reg, "CTRL")) {
	//			d->apoio.emul_bus_ctrl = val;
	//		} else if(!strcmp(reg, "DATAIN")) {
	//			d->apoio.emul_bus_data_in = val;
	//		} else {
	//			DEBUG_PRINT("reg = %s; val = %u\n", reg, val);
	//		}
	//	}


		for (i = 0; i < len; i++) {
			int ch = read_buffer[i];
			if (ch == TELNET_IAC) {
				d->telnet_iac_rx_state = TELNET_IAC;
			} else if (d->telnet_iac_rx_state) {
				if (d->telnet_iac_rx_state == TELNET_IAC) {
					d->telnet_iac_rx_state = ch;
					if ((ch == TELNET_WILL) || (ch == TELNET_WONT) ||
					    (ch == TELNET_DO) || (ch == TELNET_DONT))
						continue;
				}
				if (d->telnet_iac_rx_state == TELNET_SB)
					continue;

				printf("TELNET IAC 0x%02x 0x%02x\n", d->telnet_iac_rx_state, ch);

				d->telnet_iac_rx_state = 0;
				continue;
			}

			apoterm_kbd_process_char(&d->apoterm, ch);
		}
	}
}

static void* init_socket(void* ptr)
{
	struct sockaddr_in addr_client;
	struct sockaddr_in addr_srv;
	int port;
	int yes = 1;
	int ret;

	apohw_state_t* d = (apohw_state_t*)ptr;

	d->socket_tmp = -1;
	port = d->port;

	d->socket_srv = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (d->socket_srv == -1) {
		perror("socket()");
		return NULL;
	}

	ret = setsockopt(d->socket_srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
	if (ret == -1) {
		perror("setsockopt()");
		return NULL;
	}


	socklen_t len = sizeof(addr_srv);
	memset(&addr_srv, 0, len);
	addr_srv.sin_family = AF_INET;
	addr_srv.sin_addr.s_addr = htonl(INADDR_ANY);
	addr_srv.sin_port = htons(port);
	ret = bind(d->socket_srv, (struct sockaddr*)&addr_srv, len);
	if (ret == -1) {
		perror("bind()");
		return NULL;
	}

	ret = listen(d->socket_srv, 5);
	if (ret == -1) {
		perror("listen()");
		return NULL;
	}


	while(1) {
		socklen_t len_client = sizeof(addr_client);
		DEBUG_PRINT("Waiting on port %d for APOHW client to connect...\n", d->port);

		d->socket_tmp = accept(d->socket_srv, (struct sockaddr*)&addr_client, &len_client);
		if (d->socket_tmp == -1) {
			perror("accept()");

			continue;
		}
		d->telnet_iac_rx_state = 0;

		DEBUG_PRINT("Client connected.\n");

		socket_write(d, apohw_telnet_raw_mode, sizeof(apohw_telnet_raw_mode));

		apoterm_display_update(d);

		socket_read(d); /* Should run forever if everything is OK;
				   If error occurs (client disconnected), returns here */

		closesocket(d->socket_tmp);

		d->socket_tmp = -1;
	}

	return NULL;
}
/* ------------------------------------------------------------------------- */
static void apoterm_init(apohw_state_t *d)
{
	int i;

	apoterm_state_t *apoterm = &d->apoterm;

	apoterm->lcd.busyflag = 0;
	apoterm->lcd.DDRAM_addr = 0;
	apoterm->lcd.cursor_incr = 1;
	apoterm->lcd.display_lines = 1;

	apoterm->kbd_scan_select = ~0;
	for(i=0; i < APOTERM_KBD_SCAN_LINES; i++)
		apoterm->kbd_scan_response[i] = ~0;
}

static void apoterm_hd44780_increment_DDRAM_addr(struct hd44780 *lcd)
{
	//printf("apoterm_hd44780_increment_DDRAM_addr invoked lcd->DDRAM_addr = 0x%x "
	//	"lcd->cursor_incr = %i\n", lcd->DDRAM_addr, lcd->cursor_incr);
	/* Same for display_lines = 1 or 2 */
	lcd->DDRAM_addr = ((lcd->DDRAM_addr + lcd->cursor_incr)
		% APOTERM_HD44780_DDRAM_SIZE);
}

static int apoterm_hd44780_print(struct hd44780 *lcd, int line, char* buff)
{
	int i;

	if (lcd->display_on != 1) {
		buff[0] = '\0';
		return 0;
	}

	if (line == 1) {
		memcpy(buff, &lcd->DDRAM[APOTERM_HD44780_DDRAM_LINE1_START],
			APOTERM_HD44780_DISP_COLS);
		for(i = 0; i < APOTERM_HD44780_DISP_COLS; i++)
			if(buff[i] < 0x20)
				buff[i] = ' ';
		buff[APOTERM_HD44780_DISP_COLS] = '\0';

		return APOTERM_HD44780_DISP_COLS;
	}

	if (line == 2) {
		memcpy(buff, &lcd->DDRAM[APOTERM_HD44780_DDRAM_LINE2_START],
			APOTERM_HD44780_DISP_COLS);
		for(i = 0; i < APOTERM_HD44780_DISP_COLS; i++)
			if(buff[i] < 0x20)
				buff[i] = ' ';
		buff[APOTERM_HD44780_DISP_COLS] = '\0';

		return APOTERM_HD44780_DISP_COLS;
	}

	buff[0] = '\0';
	return 0;
}

static int apoterm_hd44780(uint8_t data, int rw, int rs, struct hd44780 *lcd)
{
	uint8_t ret;
	//printf("apoterm_hd44780 invoked. data = %i; rw = %i; rs = %i\n", data, rw, rs);

	if (rw == APOTERM_HD44780_WR) {
		if (rs == APOTERM_HD44780_INSTREG) {
			DEBUG_PRINT("APOTERM_HD44780_WR INSTREG data = 0x%x\n", data);

			/* Be aware of the sequence of checked conditions --
			we must start from the mast with the "more significant" bits */
			if (data & APOTERM_HD44780_SET_DDRAM_ADDR_m) {
				lcd->DDRAM_addr = ((uint8_t)(data & 0x7F) %
					APOTERM_HD44780_DDRAM_SIZE);

			} else if (data & APOTERM_HD44780_SET_CGRAM_ADDR_m) {

			} else if (data & APOTERM_HD44780_FUNCTION_SET_m) {
				if (data & (1 << 4)) /* Sets interface data length */
					{} // FIXME

				if (data & (1 << 3)) /* Number of display lines */
					lcd->display_lines = 2;
				else
					lcd->display_lines = 1;

				if (data & (1 << 2)) /* Character font */
					{} // FIXME

			} else if (data & APOTERM_HD44780_CURSOR_DISPLAY_SHIFT_m) {
				/* Cursor or Display Shift */

			} else if (data & APOTERM_HD44780_DISPLAY_ON_OFF_m) {
				if (data & (1 << 2)) { /* Display on */
					lcd->display_on = 1;
				} else {
					lcd->display_on = 0;
				}

				if (data & (1 << 1)) /* The cursor is displayed */
					{} // FIXME
				if (data & (1 << 0)) /* The character indicated by the cursor blinks */
					{} // FIXME

			} else if (data & APOTERM_HD44780_ENTRY_MODE_SET_m) {
				if (data & (1 << 1)) /* Increment */
					lcd->cursor_incr = -1;
				else
					lcd->cursor_incr = 1;

				if (data & (1 << 0)) { /* Shifts the entire display ... */
					// FIXME
				}

			} else if (data & APOTERM_HD44780_RETURN_HOME_m) {
				lcd->DDRAM_addr = 0;

			} else if (data & APOTERM_HD44780_CLEAR_DISP_m) {
				lcd->DDRAM_addr = 0;
				memset(lcd->DDRAM, 0, APOTERM_HD44780_DDRAM_SIZE);
			}

		}

		if (rs == APOTERM_HD44780_DATREG) {
			DEBUG_PRINT("APOTERM_HD44780_WR addr = 0x%x data = 0x%x\n",
				lcd->DDRAM_addr, (uint8_t)data);

			lcd->DDRAM[lcd->DDRAM_addr] = (uint8_t)data;
			apoterm_hd44780_increment_DDRAM_addr(lcd);
		}

	} else if (rw == APOTERM_HD44780_RD) {
		if (rs == APOTERM_HD44780_DATREG) {
			DEBUG_PRINT("APOTERM_HD44780_RD lcd->DDRAM_addr = 0x%x, data = 0x%x\n",
				lcd->DDRAM_addr, (uint8_t)lcd->DDRAM[lcd->DDRAM_addr]);
			ret = lcd->DDRAM[lcd->DDRAM_addr];
			apoterm_hd44780_increment_DDRAM_addr(lcd);
			return ret;
		}

		if (rs == APOTERM_HD44780_INSTREG) {
			return lcd->busyflag | (lcd->DDRAM_addr & 0x7F);
		}
	}

	return 0;
}

static void apoterm_display_update(apohw_state_t *d)
{
	apoterm_state_t *apoterm = &d->apoterm;
	apoio_state_t *apoio = &d->apoio;
	char buff[STR_BUFF];
	int i;
	char lcd_buff[APOTERM_HD44780_DISP_COLS + 1];
	unsigned int addr;
  #ifdef APOIO_EMUL_BUS_ADDR
	addr = apoio->emul_bus_addr & APOTERM_ADDR_m;
  #else /*APOIO_EMUL_BUS_ADDR*/
	addr = apoio->emul_bus_ctrl_new & APOTERM_ADDR_m;
  #endif /*APOIO_EMUL_BUS_ADDR*/

	snprintf(buff, STR_BUFF, "---------------------------------------------------\n\r");
	socket_write(d, buff, strlen(buff));
	snprintf(buff, STR_BUFF, "ADDR: %u   !RD: %u   !WR: %u   !CS0: %u   PWR: %u\n\r",
		addr,
		(apoio->emul_bus_ctrl_new & APOTERM_RD_m) ? 1 : 0,
		(apoio->emul_bus_ctrl_new & APOTERM_WR_m) ? 1 : 0,
		(apoio->emul_bus_ctrl_new & APOTERM_CS0_m) ? 1 : 0,
		apoterm->poweron);

	socket_write(d, buff, strlen(buff));


	snprintf(buff, STR_BUFF, "LED:  [ ][ ][ ][ ][ ][ ][ ][ ]\n\r");
	for (i = 0; i < 8; i++) {
		if (apoterm->led & (1 << i))
			buff[strlen(buff) - 4 - (3*i)] = '*';
	}

	socket_write(d, buff, strlen(buff));

	apoterm_hd44780_print(&apoterm->lcd, 1, lcd_buff);
	snprintf(buff, STR_BUFF, "LCD1: %s\n\r", lcd_buff);
	socket_write(d, buff, strlen(buff));

	apoterm_hd44780_print(&apoterm->lcd, 2, lcd_buff);
	snprintf(buff, STR_BUFF, "LCD2: %s\n\r", lcd_buff);
	socket_write(d, buff, strlen(buff));
}


struct apoterm_kbd_table_row {int code; int line; int response;};

static const struct apoterm_kbd_table_row apoterm_kbd_table[] = {
  {'3', 0, 0},
  {'6', 0, 1},
  {'9', 0, 2},
  {'*', 0, 3},
  /*{'e', 0, 4},*/
  {'2', 1, 0},
  {'5', 1, 1},
  {'8', 1, 2},
  {'0', 1, 3},
  {0x8, 1, 4},
  {'1', 2, 0},
  {'4', 2, 1},
  {'7', 2, 2},
  {'.', 2, 3},
  {0xd, 2, 4},
  {0,   0, 0},
};

static void apoterm_kbd_process_char(apoterm_state_t *apoterm, int ch)
{
	int i;
	const struct apoterm_kbd_table_row *p;

	DEBUG_PRINT("apoterm_kbd_process_char 0x%02x\n", ch);

	if (ch < ' ')
		return;

	for (p  = apoterm_kbd_table; p->code; p++) {
		if (p->code == ch) {
			if (p->line >= APOTERM_KBD_SCAN_LINES)
				break;
			apoterm->kbd_scan_response[p->line]
				&= ~(1 << p->response);
			DEBUG_PRINT("recognized %d %d\n", p->line, p->response);
			return;
		}

	}

	/* Unknow or no key - release all keys */
	for (i = 0; i < APOTERM_KBD_SCAN_LINES; i++)
		apoterm->kbd_scan_response[i] = ~0;
}

static unsigned int apoterm_kbd_get_response(apoterm_state_t *apoterm)
{
	unsigned int response = ~0;
	unsigned int scan_select;
	int i;

	scan_select = apoterm->kbd_scan_select;

	for (i = 0; i < APOTERM_KBD_SCAN_LINES; i++, scan_select >>= 1)
		if (!(scan_select & 1))
			response &= apoterm->kbd_scan_response[i];

	return response;
}


static void upgrade_apoterm_state(apohw_state_t *d)
{
	apoterm_state_t *apoterm = &d->apoterm;
	apoio_state_t *apoio = &d->apoio;
	int wr_en_old;
	int wr_en_new;
	int rd_en_old;
	int rd_en_new;
	unsigned int addr;

	if (apoio->emul_bus_ctrl_new & APOTERM_PWR_m)
		apoterm->poweron = 1;

	if (!apoterm->poweron)
		return;

  #ifdef APOIO_EMUL_BUS_ADDR
	addr = apoio->emul_bus_addr & APOTERM_ADDR_m;
  #else /*APOIO_EMUL_BUS_ADDR*/
	addr = apoio->emul_bus_ctrl_new & APOTERM_ADDR_m;
  #endif /*APOIO_EMUL_BUS_ADDR*/

	wr_en_old = (apoio->emul_bus_ctrl_old & APOTERM_WR_m) |
		(apoio->emul_bus_ctrl_old & APOTERM_CS0_m);
	wr_en_new = (apoio->emul_bus_ctrl_new & APOTERM_WR_m) |
		(apoio->emul_bus_ctrl_new & APOTERM_CS0_m);
	rd_en_old = (apoio->emul_bus_ctrl_old & APOTERM_RD_m) |
		(apoio->emul_bus_ctrl_old & APOTERM_CS0_m);
	rd_en_new = (apoio->emul_bus_ctrl_new & APOTERM_RD_m) |
		(apoio->emul_bus_ctrl_new & APOTERM_CS0_m);

	/* (WR | CS0): H -> L */
	if (wr_en_old && !wr_en_new)
	{
		DEBUG_PRINT("APOTERM_WR addr = %d; val = 0x%02x\n",
			addr,
			apoio->emul_bus_data_out);

		switch (addr) {
		case APOTERM_WR_LCD_INST:
			apoterm_hd44780(apoio->emul_bus_data_out,
				APOTERM_HD44780_WR,
				APOTERM_HD44780_INSTREG,
				&apoterm->lcd);
			break;

		case APOTERM_WR_LCD_WDATA:
			apoterm_hd44780(apoio->emul_bus_data_out,
				APOTERM_HD44780_WR,
				APOTERM_HD44780_DATREG,
				&apoterm->lcd);
			break;

		case APOTERM_WR_LED_WR:
			apoterm->led = apoio->emul_bus_data_out;
			break;

		case APOTERM_WR_KBD_WR:
			apoterm->kbd_scan_select = apoio->emul_bus_data_out;
			break;
		}

		apoterm_display_update(d);

	} else if (rd_en_old && !rd_en_new) { /* (RD | CS0): H -> L */
		switch (addr) {
		case APOTERM_RD_LCD_STAT:
			apoio->emul_bus_data_in = apoterm_hd44780(0,
				APOTERM_HD44780_RD,
				APOTERM_HD44780_INSTREG,
				&apoterm->lcd);
			break;

		case APOTERM_RD_LCD_RDATA:
			apoio->emul_bus_data_in = apoterm_hd44780(0,
				APOTERM_HD44780_RD,
				APOTERM_HD44780_DATREG,
				&apoterm->lcd);
			break;

		case APOTERM_RD_KBD_RD:
			apoio->emul_bus_data_in =
				apoterm_kbd_get_response(apoterm);
			break;
		}

		DEBUG_PRINT("APOTERM_RD addr = %d; val = 0x%02x\n",
			addr,
			apoio->emul_bus_data_in);
	}

	/* (RD | WR | CS0): H -> L
		start_autodestruction();
	*/
}

/* ------------------------------------------------------------------------- */

static void
apohw_reset(apohw_state_t *d)
{
}

static uint64_t apoio_bar0_read(void *opaque, hwaddr addr, unsigned size)
{
	apohw_state_t *d = opaque;

	DEBUG_PRINT("read  addr = 0x%02x size = %u\n", (uint32_t)addr, size);

	if (addr < APOIO_RAM_SIZE) {
		return (uint8_t)d->apoio.ram[addr];
	} else if (addr == APOIO_EMUL_BUS_DATA_IN) {
		DEBUG_PRINT(" data = 0x%02x\n", (uint8_t)d->apoio.emul_bus_data_in);
		return (uint8_t)d->apoio.emul_bus_data_in;
	}

	return 0;
}

static void apoio_bar0_write(void *opaque, hwaddr addr, uint64_t data,
                             unsigned size)
{
	apohw_state_t *d = opaque;
	data = (uint8_t)data;

	DEBUG_PRINT("write addr = 0x%02x data = 0x%llx size = %u\n",
		(uint32_t)addr, (long long)data, size);

	if (addr < APOIO_RAM_SIZE) {
		d->apoio.ram[addr] = data;

	} else if (addr == APOIO_EMUL_BUS_CTRL) {
		if (d->apoio.emul_bus_ctrl_new != data) {
			d->apoio.emul_bus_ctrl_old = d->apoio.emul_bus_ctrl_new;
			d->apoio.emul_bus_ctrl_new = data;

			upgrade_apoterm_state(d);
		}
  #ifdef APOIO_EMUL_BUS_ADDR
	} else if (addr == APOIO_EMUL_BUS_ADDR) {
		d->apoio.emul_bus_addr = data;
		upgrade_apoterm_state(d);
  #endif /*APOIO_EMUL_BUS_ADDR*/
	} else if (addr == APOIO_EMUL_BUS_DATA_OUT) {
		d->apoio.emul_bus_data_out = data;
		upgrade_apoterm_state(d);

	} else if (addr == APOIO_LED_PIO) {
		d->apoio.led_pio = (unsigned int) data;
		printf(DEBUG_PREFIX "LED_PIO = 0x%X\n", d->apoio.led_pio);
	}

}

static const MemoryRegionOps apoio_bar0_ops = {
    .read = apoio_bar0_read,
    .write = apoio_bar0_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static int apohw_init(PCIDevice *pci_dev)
{
	apohw_state_t *d = APOHW_DEV(pci_dev);
	apoio_state_t *s;
	uint8_t *pci_conf;
	QemuThread socket_thread;

	DEBUG_PRINT("Apohw started.\n");

	if (d->port == DEFAULT_PORT) {
		d->port += instance; /* Each instance of the same device
					should have another port number */
		instance++;
	}

	pci_conf = pci_dev->config;
	pci_conf[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */

	s = &d->apoio;
	memory_region_init_io(&s->bar0, OBJECT(d), &apoio_bar0_ops, d,
	                  "apohw-bar0", 64*1024);

	pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar0);
	//s->irq = d->dev.irq[0];

	qemu_thread_create(&socket_thread, "apohw-worker", init_socket, (void*)d, 0);

	apoterm_init(d);

	return 0;
}

static void apohw_exit(PCIDevice *pci_dev)
{
	/*apohw_state_t *d = APOHW_DEV(pci_dev);*/
	/*apoio_state_t *s = &d->apoio;*/

	/* region s->bar0 is destroyed by QOM now */
	/* memory_region_destroy(&s->bar0); */
}

/* VMState is needed for live migration of Qemu images */
const VMStateDescription vmstate_apoio = {
	.name = "apohw",
	.version_id = 1,
	.minimum_version_id = 1,
	.minimum_version_id_old = 1,
	.fields = (VMStateField []) {
		VMSTATE_BUFFER(ram, apoio_state_t),
		//VMSTATE_UINT8(timer0, apoio_state_t),
		VMSTATE_UINT8(led_pio, apoio_state_t),
		VMSTATE_UINT8(emul_bus_ctrl_old, apoio_state_t),
		VMSTATE_UINT8(emul_bus_ctrl_new, apoio_state_t),
		VMSTATE_UINT8(emul_bus_addr,     apoio_state_t),
		VMSTATE_UINT8(emul_bus_data_out, apoio_state_t),
		VMSTATE_UINT8(emul_bus_data_in,  apoio_state_t),
		VMSTATE_END_OF_LIST()
	}
};

static const VMStateDescription vmstate_apohw = {
	.name = "apohw",
	.version_id = 1,
	.minimum_version_id = 1,
	.minimum_version_id_old = 1,
	.fields = (VMStateField[]) {
	    VMSTATE_PCI_DEVICE(dev, apohw_state_t),
	    VMSTATE_STRUCT(apoio, apohw_state_t, 0, vmstate_apoio, apoio_state_t),
	    //VMSTATE_STRUCT(apoterm, apoterm_state_t, 0, vmstate_apoterm, apoterm_state_t), // FIXME
	    VMSTATE_END_OF_LIST()
	}
};

static void qdev_apohw_reset(DeviceState *dev)
{
	apohw_state_t *d = APOHW_DEV(dev);
	apohw_reset(d);
}

static Property apohw_properties[] = {
	DEFINE_PROP_UINT32("port", apohw_state_t, port, DEFAULT_PORT),
	DEFINE_PROP_END_OF_LIST(),
};

static void apohw_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
	PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

	k->init = apohw_init;
	k->exit = apohw_exit;
	k->vendor_id = PCI_VENDOR_ID_ALTERA;
	k->device_id = PCI_DEVICE_ID_ALTERA_CORPORATION_DEVICE;
	k->revision = 0x00;
	k->class_id = PCI_CLASS_COMMUNICATION_OTHER;
	dc->desc = "CVUT A0B36APO Hardware";
	dc->props = apohw_properties;
	dc->vmsd = &vmstate_apohw;
	set_bit(DEVICE_CATEGORY_MISC, dc->categories);
	dc->reset = qdev_apohw_reset;
}

static const TypeInfo apohw_info = {
	.name          = TYPE_APOHW_DEV,
	.parent        = TYPE_PCI_DEVICE,
	.instance_size = sizeof(apohw_state_t),
	.class_init    = apohw_class_init,
};

static void apohw_register_types(void)
{
	type_register_static(&apohw_info);
}

type_init(apohw_register_types)
