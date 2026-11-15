/*
 * Driver for the Realtek RTL8238B/C (and RTL8239-family) PoE PSE controllers.
 * VERSION 1.0.0
 * This code is in the Public Domain.
 *
 * Ported from PR #265 (Tobiasz, "Add chip-agnostic PoE interface and RTL8238B
 * driver") to the machine/ board layout, for the RealHD PSW8-25MG /
 * ZX-AFGW-SWTG218ANS (PCB-AFGW SWTG218ANS). Changes from the PR:
 *   - lives in BANK4 (BANK2 is too full on current main),
 *   - the I2C master engine is not used at all: status reads are bit-banged too,
 *     so GPIO46/47 simply stay in GPIO mode after bring-up,
 *   - debug output trimmed, `poe status` added.
 *
 * Hardware (confirmed by disassembling OEM firmware V100.9.9):
 *   - two PSE controllers on a bit-banged I2C bus, SCL = GPIO46, SDA = GPIO47,
 *     7-bit addresses 0x20 (ports 1-4) and 0x21 (ports 5-8),
 *   - the controller MCU firmware is volatile and must be uploaded at every
 *     power-on: reset (reg 0x1a), disable ports (reg 0x10 on both), arm (reg
 *     0xec = 03 00 00 00, if not armed reg 0x18 = 00 00 20 39 then 0xec again),
 *     stream the image into reg 0xf4 of 0x20 in ONE transaction, check reg 0xec
 *     bit 5 (accepted). OEM V100.9.9 uploads a 0x52ac (21164) byte image.
 *
 * The image is the vendor's firmware, extracted from the user's own OEM file
 * (stock_fw/extract_pse_image.py) and embedded by the Makefile at PSE_IMG_ADDR.
 * It is self-describing: 16-byte header with magic 0x8239 at bytes [2:3] and
 * four LE section lengths at [6..13]; image_len = 16 + sum(seclen) + 4.
 */

#include <stdint.h>
#include <8051.h>		// EA (global interrupt enable)
#include "rtl837x_common.h"
#include "rtl837x_sfr.h"
#include "rtl837x_regs.h"
#include "rtl837x_flash.h"
#include "machine.h"
#include "poe.h"

#pragma codeseg BANK4
#pragma constseg BANK4

#ifdef POE_CHIP_RTL8238

extern __xdata uint8_t sfr_data[4];
extern __xdata struct flash_region_t flash_region;
extern __code const struct machine machine;

#define PSE_IMG_ADDR	0x60000UL	// must match PSE_IMAGE_LOCATION in the Makefile
#define PSE_IMG_MAX	0x8000		// sanity bound on the header-declared length
#define PSE_CHUNK	12		// image bytes read from flash per chunk

#define BB_SCL_BIT	14		// GPIO46 % 32, in the GPIO 32-63 registers
#define BB_SDA_BIT	15		// GPIO47 % 32
#define BB_PSE_DL_CMD	0xf4		// firmware-download data port

// Interface globals (poe.h)
__xdata uint8_t  poe_port_num;
__xdata uint8_t  poe_st_admin;
__xdata uint8_t  poe_st_on;
__xdata uint8_t  poe_st_class;
__xdata uint8_t  poe_st_volt;
__xdata uint16_t poe_st_ma;

// Scratch kept in xdata: the 8051 overlay segment is full on this firmware.
static __xdata uint8_t  poe_resp[8];	// bytes read back by bb_read_reg()
static __xdata uint8_t  poe_reg;	// register address
static __xdata uint8_t  poe_rd_len;	// bytes to read (1..8)
static __xdata uint8_t  poe_rdslave;	// slave targeted by bb_read_reg()/bb_write_resp4()
static __xdata uint8_t  poe_bbcmd;	// 0xec or 0x18 for bb_cmd4()
static __xdata uint8_t  poe_ack_cmd;	// ACK of the command byte in the last bb_cmd4()
static __xdata uint8_t  poe_fbuf[16];	// image bytes read from flash
static __xdata uint16_t poe_ilen;
static __xdata uint16_t poe_off;
static __xdata uint8_t  poe_n;
static __xdata uint8_t  poe_j;
static __xdata uint8_t  poe_i;
static __xdata uint8_t  poe_try;
static __xdata uint8_t  poe_local;
static __xdata uint8_t  poe_set;	// bb_ports_cfg(): 0 = disable fields, 1 = enable
static __xdata uint8_t  poe_img_ok;
static __xdata uint8_t  poe_nack;
static __xdata uint8_t  poe_prep1;	// reg 0xec after first arm
static __xdata uint8_t  poe_prep2;	// reg 0xec just before the image
static __xdata uint8_t  poe_sent18;
static __xdata uint8_t  poe_ack_f4;
static __xdata uint32_t poe_calc;

static __xdata uint8_t poe_arm1[4] = { 0x03, 0x00, 0x00, 0x00 };	// reg 0xec arm payload
static __xdata uint8_t poe_arm2[4] = { 0x00, 0x00, 0x20, 0x39 };	// reg 0x18 alternate arm

static __xdata struct {
	uint8_t  bit;
	uint8_t  ack;		// 1 = target ACKed the last written byte
	uint16_t spin;
	uint8_t  out;		// byte to shift out
	uint8_t  in;		// byte shifted in
	uint8_t  n;
	uint8_t  ackout;	// master ACK (1) or NACK (0) after a read byte
} bbs;

/* ---------------- bit-banged I2C on GPIO46 (SCL) / GPIO47 (SDA) ---------------- */

static void bb_scl(uint8_t hi)
{
	if (hi)
		reg_bit_set(RTL837X_REG_GPIO_32_63_OUTPUT, BB_SCL_BIT);
	else
		reg_bit_clear(RTL837X_REG_GPIO_32_63_OUTPUT, BB_SCL_BIT);
}

static void bb_sda(uint8_t hi)
{
	if (hi)
		reg_bit_set(RTL837X_REG_GPIO_32_63_OUTPUT, BB_SDA_BIT);
	else
		reg_bit_clear(RTL837X_REG_GPIO_32_63_OUTPUT, BB_SDA_BIT);
}

static uint8_t bb_sda_read(void)
{
	reg_read_m(RTL837X_REG_GPIO_32_63_INPUT);
	return (sfr_data[2] & 0x80) ? 1 : 0;	// GPIO47 = bit 15 = sfr_data[2] bit 7
}

/* Route GPIO46/47 to plain GPIO, both push-pull outputs, idle high. */
static void bb_setup(void)
{
	reg_read_m(RTL837X_PIN_MUX_1);
	sfr_mask_data(0, 0x80, 0x00);	// SCL0 function bits -> GPIO
	sfr_mask_data(1, 0x01, 0x00);
	sfr_mask_data(1, 0x06, 0x00);	// SDA0 function bits -> GPIO
	reg_write_m(RTL837X_PIN_MUX_1);

	reg_bit_set(RTL837X_REG_GPIO_32_63_OUTPUT, BB_SDA_BIT);
	reg_bit_set(RTL837X_REG_GPIO_32_63_OUTPUT, BB_SCL_BIT);
	reg_bit_set(RTL837X_REG_GPIO_32_63_DIRECTION, BB_SDA_BIT);
	reg_bit_set(RTL837X_REG_GPIO_32_63_DIRECTION, BB_SCL_BIT);
}

static void bb_start(void)
{
	bb_sda(1);
	bb_scl(1);
	bb_sda(0);
	bb_scl(0);
}

static void bb_stop(void)
{
	bb_sda(0);
	bb_scl(1);
	bb_sda(1);
}

/* Clock out bbs.out MSB first, then one ACK clock (result in bbs.ack). */
static void bb_write_byte(void)
{
	for (bbs.bit = 0; bbs.bit < 8; bbs.bit++) {
		bb_sda(bbs.out & 0x80);
		bb_scl(1);
		bb_scl(0);
		bbs.out <<= 1;
	}
	bb_sda(1);
	reg_bit_clear(RTL837X_REG_GPIO_32_63_DIRECTION, BB_SDA_BIT);	// release SDA
	bb_scl(1);
	bbs.ack = bb_sda_read() ? 0 : 1;
	bb_scl(0);
	reg_bit_set(RTL837X_REG_GPIO_32_63_DIRECTION, BB_SDA_BIT);
}

/* Read one byte MSB first into bbs.in, then ACK (bbs.ackout) or NACK. */
static void bb_read_byte(void)
{
	reg_bit_clear(RTL837X_REG_GPIO_32_63_DIRECTION, BB_SDA_BIT);
	bbs.in = 0;
	for (bbs.bit = 0; bbs.bit < 8; bbs.bit++) {
		bb_scl(1);
		bbs.in <<= 1;
		if (bb_sda_read())
			bbs.in |= 1;
		bb_scl(0);
	}
	bb_sda(bbs.ackout ? 0 : 1);
	reg_bit_set(RTL837X_REG_GPIO_32_63_DIRECTION, BB_SDA_BIT);
	bb_scl(1);
	bb_scl(0);
}

/* START | slave<<1 | reg | reSTART | slave<<1|1 | poe_rd_len bytes | STOP */
static void bb_read_reg(void)
{
	bb_start();
	bbs.out = poe_rdslave << 1;		bb_write_byte();
	bbs.out = poe_reg;			bb_write_byte();
	bb_start();
	bbs.out = (poe_rdslave << 1) | 1;	bb_write_byte();
	for (bbs.n = 0; bbs.n < poe_rd_len; bbs.n++) {
		bbs.ackout = (bbs.n < (poe_rd_len - 1)) ? 1 : 0;
		bb_read_byte();
		poe_resp[bbs.n] = bbs.in;
	}
	bb_stop();
}

/* Write poe_resp[0..3] to register poe_reg of poe_rdslave. */
static void bb_write_resp4(void)
{
	bb_start();
	bbs.out = poe_rdslave << 1;	bb_write_byte();
	bbs.out = poe_reg;		bb_write_byte();
	bbs.out = poe_resp[0];		bb_write_byte();
	bbs.out = poe_resp[1];		bb_write_byte();
	bbs.out = poe_resp[2];		bb_write_byte();
	bbs.out = poe_resp[3];		bb_write_byte();
	bb_stop();
}

/* Write a 4-byte value to reg 0x10 = 0 on slave poe_rdslave (disable all its ports). */
static void bb_ports_off(void)
{
	poe_resp[0] = poe_resp[1] = poe_resp[2] = poe_resp[3] = 0;
	poe_reg = 0x10;
	bb_write_resp4();
}

/* START | 0x20<<1 | poe_bbcmd | 4-byte arm payload | STOP */
static void bb_cmd4(void)
{
	bb_start();
	bbs.out = machine.poe.addr0 << 1;	bb_write_byte();
	bbs.out = poe_bbcmd;			bb_write_byte();
	poe_ack_cmd = bbs.ack;
	for (poe_i = 0; poe_i < 4; poe_i++) {
		bbs.out = (poe_bbcmd == 0xec) ? poe_arm1[poe_i] : poe_arm2[poe_i];
		bb_write_byte();
	}
	bb_stop();
}

/* Release SDA and pulse SCL 9 times, then STOP: frees a slave stuck mid-byte. */
static void bb_busrecover(void)
{
	reg_bit_clear(RTL837X_REG_GPIO_32_63_DIRECTION, BB_SDA_BIT);
	for (bbs.bit = 0; bbs.bit < 9; bbs.bit++) {
		bb_scl(1);
		bb_scl(0);
	}
	reg_bit_set(RTL837X_REG_GPIO_32_63_DIRECTION, BB_SDA_BIT);
	bb_stop();
}

/* Map 0-based port poe_n -> controller (poe_rdslave) and channel (poe_local).
 * Per PR #265: ports 0-3 on addr0 as channels 3,2,1,0; ports 4-7 on addr1 as 0..3.
 * NOT yet verified on the PSW8-25MG: check with `poe status` and a PD. */
static void port_map(void)
{
	if (poe_n < 4) {
		poe_rdslave = machine.poe.addr0;
		poe_local = 3 - poe_n;
	} else {
		poe_rdslave = machine.poe.addr1;
		poe_local = poe_n - 4;
	}
}

/* Read-modify-write the admin fields of port poe_n: 2-bit mode in reg 0x12
 * (byte 2 of the 0x10 block, 3 = auto-detect on) and 1-bit enable in reg 0x14. */
static void port_admin(void)
{
	port_map();
	poe_reg = 0x10;	poe_rd_len = 4;	bb_read_reg();
	if (poe_set)
		poe_resp[2] |= (0x03 << (poe_local << 1));
	else
		poe_resp[2] &= ~(0x03 << (poe_local << 1));
	poe_reg = 0x10;	bb_write_resp4();

	poe_reg = 0x14;	poe_rd_len = 4;	bb_read_reg();
	if (poe_set)
		poe_resp[0] |= (0x01 << poe_local);
	else
		poe_resp[0] &= ~(0x01 << poe_local);
	poe_reg = 0x14;	bb_write_resp4();
}

static void ports_admin_all(void)
{
	for (poe_n = 0; poe_n < machine.poe.n_ports; poe_n++)
		port_admin();
}

/* ---------------- bring-up ---------------- */

static void poe_upload(void)
{
	poe_nack = 0;

	// Validate the embedded image and take its length from the header.
	flash_region.addr = PSE_IMG_ADDR;
	flash_region.len = 16;
	flash_read_bulk(poe_fbuf);
	poe_ilen = 16 + 4;
	poe_ilen += poe_fbuf[6]  | ((uint16_t)poe_fbuf[7]  << 8);
	poe_ilen += poe_fbuf[8]  | ((uint16_t)poe_fbuf[9]  << 8);
	poe_ilen += poe_fbuf[10] | ((uint16_t)poe_fbuf[11] << 8);
	poe_ilen += poe_fbuf[12] | ((uint16_t)poe_fbuf[13] << 8);
	poe_img_ok = (poe_fbuf[2] == 0x39 && poe_fbuf[3] == 0x82
		      && poe_ilen > 16 + 4 && poe_ilen <= PSE_IMG_MAX);

	print_string("PoE: PSE image ");
	itoa_short(poe_ilen);
	print_string(" bytes, ");
	if (!poe_img_ok) {
		print_string("NOT FOUND or invalid at 0x60000 - PoE stays off\n");
		return;
	}
	print_string("OK, uploading (takes a few seconds)\n");

	bb_setup();
	bb_busrecover();

	// Step 0: whole-chip reset, reg 0x1a = 0x20.
	bb_start();
	bbs.out = machine.poe.addr0 << 1;	bb_write_byte();
	if (!bbs.ack)
		poe_nack = 1;
	bbs.out = 0x1a;				bb_write_byte();
	bbs.out = 0x20;				bb_write_byte();
	bb_stop();
	delay(100);

	// Step 1: disable all ports on both controllers, clear per-port admin fields.
	poe_rdslave = machine.poe.addr0;	bb_ports_off();
	if (machine.poe.addr1) {
		poe_rdslave = machine.poe.addr1;
		bb_ports_off();
	}
	poe_set = 0;
	ports_admin_all();

	// Step 2: arm + stream, interrupts off (no printing, no delay() in here).
	EA = 0;
	poe_rdslave = machine.poe.addr0;
	poe_bbcmd = 0xec;	bb_cmd4();
	poe_reg = 0xec;	poe_rd_len = 1;	bb_read_reg();
	poe_prep1 = poe_resp[0];
	poe_sent18 = 0;
	if (!(poe_resp[0] & 0x04)) {		// bit 2 = armed
		poe_bbcmd = 0x18;	bb_cmd4();
		for (bbs.spin = 0; bbs.spin < 1000; bbs.spin++)
			reg_read_m(RTL837X_REG_GPIO_32_63_INPUT);
		poe_bbcmd = 0xec;	bb_cmd4();
		poe_sent18 = 1;
	}
	poe_reg = 0xec;	poe_rd_len = 1;	bb_read_reg();
	poe_prep2 = poe_resp[0];

	bb_start();
	bbs.out = machine.poe.addr0 << 1;	bb_write_byte();
	if (!bbs.ack)
		poe_nack = 1;
	bbs.out = BB_PSE_DL_CMD;		bb_write_byte();
	poe_ack_f4 = bbs.ack;
	for (poe_off = 0; poe_off < poe_ilen; poe_off += PSE_CHUNK) {
		poe_n = (poe_ilen - poe_off > PSE_CHUNK) ? PSE_CHUNK : (uint8_t)(poe_ilen - poe_off);
		flash_region.addr = PSE_IMG_ADDR + poe_off;
		flash_region.len = poe_n;
		flash_read_bulk(poe_fbuf);
		for (poe_j = 0; poe_j < poe_n; poe_j++) {
			bbs.out = poe_fbuf[poe_j];
			bb_write_byte();
		}
	}
	bb_stop();
	EA = 1;

	print_string("PoE: arm status 0x");	print_byte(poe_prep1);
	if (poe_sent18)
		print_string(" (sent 0x18 + 0xec)");
	print_string(", pre-image 0x");	print_byte(poe_prep2);
	print_string(", 0xf4 ");	print_string(poe_ack_f4 ? "ACK" : "NACK");
	if (poe_nack)
		print_string(", WARNING: no ACK from PSE address");
	write_char('\n');

	// Step 3: wait for the chip to validate the image (reg 0xec bit 5).
	for (poe_try = 0; poe_try < 25; poe_try++) {
		delay(8);
		EA = 0;
		poe_reg = 0xec;	poe_rd_len = 4;	bb_read_reg();
		EA = 1;
		if (poe_resp[0] & 0x20)
			break;
	}
	print_string("PoE: download status ");
	for (poe_j = 0; poe_j < 4; poe_j++) {
		print_byte(poe_resp[poe_j]);
		write_char(' ');
	}
	print_string((poe_resp[0] & 0x20) ? "=> ACCEPTED\n" : "=> REJECTED\n");

	delay(100);

	// Step 4: enable all ports (auto-detect). The OEM restores saved per-port
	// config here; we have none, so every port is enabled.
	poe_set = 1;
	ports_admin_all();
	print_string("PoE: all ports enabled\n");
}

void poe_bringup(void) __banked
{
	if (machine.poe.chip == POE_NONE)
		return;
	poe_upload();
}

/* ---------------- runtime control and status ---------------- */

void poe_port_set(uint8_t port_num, uint8_t port_on) __banked
{
	bb_setup();
	poe_n = port_num;
	poe_set = port_on;
	port_admin();
	print_string("PoE: port ");
	itoa(port_num + 1);
	print_string(port_on ? " enabled\n" : " disabled\n");
}

void poe_global_set(uint8_t on) __banked
{
	bb_setup();
	poe_set = on;
	ports_admin_all();
	print_string(on ? "PoE: all ports enabled\n" : "PoE: all ports disabled\n");
}

/*
 * Decode one port (poe_port_num) into the normalized status, per PR #265:
 *   admin = reg 0x12 mode field != 0, on = reg 0x10 byte 0 bit (4 + channel)
 *   class = high nibble of reg 0x0c + channel (0xc = class 0)
 *   volt  = top byte of reg 0x30 + 4*channel, 0.5 V/LSB
 *   mA    = low 16 bits of the same register * 125 / 4096
 */
void poe_get_port(void) __banked
{
	bb_setup();
	poe_n = poe_port_num;
	port_map();

	poe_reg = 0x10;	poe_rd_len = 4;	bb_read_reg();
	poe_st_admin = ((poe_resp[2] >> (poe_local << 1)) & 0x03) ? 1 : 0;
	poe_st_on = (poe_resp[0] >> (poe_local + 4)) & 0x01;

	poe_reg = 0x30 + (poe_local << 2);	poe_rd_len = 4;	bb_read_reg();
	poe_st_volt = poe_resp[3] >> 1;
	poe_calc = (uint32_t)(((uint16_t)poe_resp[1] << 8) | poe_resp[0]) * 125UL;
	poe_st_ma = (uint16_t)(poe_calc >> 12);

	poe_reg = 0x0c;	poe_rd_len = 4;	bb_read_reg();
	poe_st_class = (poe_resp[poe_local] >> 4) & 0x0f;
	if (poe_st_class == 0x0c)
		poe_st_class = 0;
	else if (poe_st_class < 1 || poe_st_class > 8)
		poe_st_class = 0xff;

	// The chip latches power-on/class/voltage after a PD is unplugged: only a
	// port drawing current counts as delivering.
	if (!(poe_st_on && poe_st_ma > 0)) {
		poe_st_on = 0;
		poe_st_class = 0xff;
	}
}

void poe_status_print(void) __banked
{
	print_string("Port Admin Power Class   V    mA\n");
	for (poe_port_num = 0; poe_port_num < machine.poe.n_ports; poe_port_num++) {
		poe_get_port();
		write_char(' ');
		itoa(poe_port_num + 1);
		print_string("    ");
		print_string(poe_st_admin ? "on " : "off");
		print_string("   ");
		print_string(poe_st_on ? "yes" : "no ");
		print_string("   ");
		if (poe_st_class == 0xff)
			write_char('-');
		else
			itoa(poe_st_class);
		print_string("    ");
		itoa(poe_st_volt);
		print_string("  ");
		itoa_short(poe_st_ma);
		write_char('\n');
	}
}

#endif /* POE_CHIP_RTL8238 */
