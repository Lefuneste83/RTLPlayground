/*
 * Driver for the Realtek RTL8238B/C (and RTL8239-family) PoE PSE controllers.
 * VERSION 1.5.1 - the running-firmware check is also done at the late boot hook
 *                 (the early one may not reach the PSE); boot decisions are
 *                 recorded and shown by `poe info`
 * VERSION 1.5.0 - no PoE power cycle on reboot: if the PSE firmware is already
 *                 running (warm reboot) it is kept and nothing is reset; on a
 *                 cold start the upload now runs right after flash init
 *                 instead of after the PHY init. `poe load` still forces it.
 * VERSION 1.4.0 - link watchdog: `poe watchdog N <minutes|off>` power-cycles a
 *                 port that delivers power but has had no link for that long
 * VERSION 1.3.0 - web UI support: /poe.json (send_poe), per-port admin mask
 *                 applied after the startup config (poe_apply, no flicker on
 *                 ports saved as off), `poe port N reset`, energy counters and
 *                 PD connect/remove events (poe_tick)
 * VERSION 1.2.0 - poe_hold(): ports off right after flash init, so devices are
 *                 powered once (after the upload) instead of twice at boot;
 *                 host poll default off (no PSE watchdog seen); `poe info` shows both
 * VERSION 1.1.0 - periodic host poll (poe_heartbeat), `poe info`
 * VERSION 1.0.1 - port->channel direction per board (machine.poe.ch_rev)
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
__xdata uint8_t  poe_poll_enabled = 0;
static __xdata uint8_t poe_up;		// 1 once an image was uploaded and accepted
static __xdata uint8_t poe_k;
static __xdata uint8_t poe_c;
static __xdata uint8_t poe_admin = 0xff;	// per-port admin state, bit n = port n (0-based)
static __xdata uint8_t poe_live;	// 1 once the startup config ran: ports may be enabled
static __xdata uint8_t poe_prev_on;	// ports delivering power at the last sample
static __xdata uint8_t poe_secs;	// poe_tick() second counter
static __xdata uint8_t poe_v2;		// last port voltage in 0.5 V units
static __xdata uint32_t poe_energy[8];	// per port, in units of mW x 10 s
static __xdata uint8_t poe_dig[10];
// Boot diagnostics for `poe info`: [0..2] early hook, [3..5] late hook:
// address ACK, reg 0xec, action (0 none, 1 kept running firmware, 2 uploaded, 3 deferred)
static __xdata uint8_t poe_dbg[6];
static __xdata uint8_t poe_wd_min[8];	// link watchdog per port, minutes (0 = off)
static __xdata uint8_t poe_wd_cnt[8];	// samples (POE_SAMPLE_SECS each) powered without link
static __xdata uint8_t poe_wd_pending;	// ports switched off by the watchdog, to re-enable
static __xdata uint8_t poe_wd_wait;	// seconds since the watchdog switched them off
#define POE_WD_MAX_MIN 40		// 40 min * 6 samples/min fits the 8-bit counter
#define POE_WD_OFF_SECS 3

extern __xdata uint8_t outbuf[TCP_OUTBUF_SIZE];
extern __xdata uint16_t slen;
void char_to_html(char c);
void str_code_to_html(__code const char *s);
#define POE_SAMPLE_SECS 10

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
 * Ports 0-3 on addr0, 4-7 on addr1; channel order per controller from
 * machine.poe.ch_rev (PR #265's keepLink has addr0 reversed: 0x01; the
 * PSW8-25MG V1.2 runs both straight: 0x00). */
static void port_map(void)
{
	if (poe_n < 4) {
		poe_rdslave = machine.poe.addr0;
		poe_local = (machine.poe.ch_rev & 0x01) ? 3 - poe_n : poe_n;
	} else {
		poe_rdslave = machine.poe.addr1;
		poe_local = (machine.poe.ch_rev & 0x02) ? 7 - poe_n : poe_n - 4;
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

/* Enable/disable every port according to the poe_admin mask. */
static void ports_admin_mask(void)
{
	for (poe_n = 0; poe_n < machine.poe.n_ports; poe_n++) {
		poe_set = (poe_admin >> poe_n) & 1;
		port_admin();
	}
	print_string("PoE: ports enabled, mask 0x");
	print_byte(poe_admin);
	write_char('\n');
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
	poe_up = (poe_resp[0] & 0x20) ? 1 : 0;

	delay(100);

	// Step 4: enable the ports (auto-detect) that are administratively on. At
	// boot this waits for poe_apply(), after the startup config has run, so a
	// port saved as off is never powered.
	if (poe_live)
		ports_admin_mask();
}

/* 1 if a PSE controller ACKs its address. */
static uint8_t pse_probe(void)
{
	bb_setup();
	bb_busrecover();
	bb_start();
	bbs.out = machine.poe.addr0 << 1;	bb_write_byte();
	bb_stop();
	return bbs.ack;
}

/* 1 if the PSE application image is loaded and running (reg 0xec bit 5). */
static uint8_t pse_running(void)
{
	poe_rdslave = machine.poe.addr0;
	poe_reg = 0xec;	poe_rd_len = 4;	bb_read_reg();
	return (poe_resp[0] != 0xff && (poe_resp[0] & 0x20)) ? 1 : 0;
}

/*
 * Early boot hook, right after flash_init.
 *  - Warm reboot: the PSE was not reset and still runs its firmware and powers
 *    its PDs. Keep it: no reset, no upload, so devices are not power-cycled.
 *  - Cold start: upload now rather than after the PHY init, so PDs are powered
 *    once (when the startup config has run) instead of being powered by the
 *    PSE bootloader and then cut by a late upload.
 * If the PSE does not answer yet, poe_bringup() retries later.
 */
void poe_hold(void) __banked
{
	if (machine.poe.chip == POE_NONE)
		return;
	poe_up = 0;
	poe_prev_on = 0;
	poe_dbg[0] = pse_probe();
	if (!poe_dbg[0]) {
		poe_dbg[2] = 3;
		print_string("PoE: PSE not answering yet, upload deferred\n");
		return;
	}
	poe_k = pse_running();
	poe_dbg[1] = poe_resp[0];
	if (poe_k) {
		poe_up = 1;
		poe_dbg[2] = 1;
		print_string("PoE: PSE firmware already running, kept (no power cycle)\n");
		return;
	}
	poe_dbg[2] = 2;
	poe_upload();
}

/* Late boot hook (after PHY init): only acts if the early one could not. */
void poe_bringup(void) __banked
{
	if (machine.poe.chip == POE_NONE || poe_up)
		return;
	poe_prev_on = 0;
	poe_dbg[3] = pse_probe();
	if (poe_dbg[3]) {
		poe_k = pse_running();
		poe_dbg[4] = poe_resp[0];
		if (poe_k) {
			poe_up = 1;
			poe_dbg[5] = 1;
			print_string("PoE: PSE firmware already running, kept (no power cycle)\n");
			return;
		}
	}
	poe_dbg[5] = 2;
	poe_upload();
}

/* `poe load`: force a PSE reset and a fresh upload (power-cycles all PDs). */
void poe_load(void) __banked
{
	if (machine.poe.chip == POE_NONE)
		return;
	poe_up = 0;
	poe_prev_on = 0;
	poe_upload();
}

/* Called once after the startup config has been executed: from now on ports
 * follow the admin mask (which the config may have changed with `poe port`). */
void poe_apply(void) __banked
{
	poe_live = 1;
	if (machine.poe.chip == POE_NONE || !poe_up)
		return;
	bb_setup();
	ports_admin_mask();
}

/* Host poll: read the reg 0x10 status block of each controller, the same
 * kind of status access the OEM web UI does continuously. */
void poe_heartbeat(void) __banked
{
	if (!poe_up || !poe_poll_enabled)
		return;
	bb_setup();
	poe_rdslave = machine.poe.addr0;
	poe_reg = 0x10;	poe_rd_len = 4;	bb_read_reg();
	if (machine.poe.addr1) {
		poe_rdslave = machine.poe.addr1;
		poe_reg = 0x10;	poe_rd_len = 4;	bb_read_reg();
	}
}

static void info_reg(void)
{
	bb_read_reg();
	print_string(" ");
	print_byte(poe_reg);
	write_char('=');
	for (poe_k = 0; poe_k < poe_rd_len; poe_k++)
		print_byte(poe_resp[poe_k]);
}

void poe_info_print(void) __banked
{
	print_string("PoE: image ");
	print_string(poe_up ? "accepted" : "not loaded");
	print_string(", host poll ");
	print_string(poe_poll_enabled ? "on\n" : "off\n");
	print_string("boot: early ack/ec/act, late ack/ec/act =");
	for (poe_k = 0; poe_k < 6; poe_k++) {
		write_char(' ');
		print_byte(poe_dbg[poe_k]);
	}
	write_char('\n');
	bb_setup();
	for (poe_c = 0; poe_c < 2; poe_c++) {
		poe_rdslave = poe_c ? machine.poe.addr1 : machine.poe.addr0;
		if (!poe_rdslave)
			break;
		print_string("0x");
		print_byte(poe_rdslave);
		write_char(':');
		poe_reg = 0xec;	poe_rd_len = 4;	info_reg();
		poe_reg = 0x1b;	poe_rd_len = 1;	info_reg();
		poe_reg = 0xe8;	poe_rd_len = 1;	info_reg();
		poe_reg = 0x0b;	poe_rd_len = 2;	info_reg();
		poe_reg = 0x10;	poe_rd_len = 4;	info_reg();
		poe_reg = 0x14;	poe_rd_len = 4;	info_reg();
		write_char('\n');
	}
}

/* ---------------- runtime control and status ---------------- */

void poe_port_set(uint8_t port_num, uint8_t port_on) __banked
{
	if (port_on)
		poe_admin |= (1 << port_num);
	else
		poe_admin &= ~(1 << port_num);
	if (!poe_live || !poe_up)
		return;		// startup config: applied later by poe_apply()
	bb_setup();
	poe_n = port_num;
	poe_set = port_on;
	port_admin();
	print_string("PoE: port ");
	itoa(port_num + 1);
	print_string(port_on ? " enabled\n" : " disabled\n");
}

/* Power-cycle one port: off, ~2 s, back on (only if administratively on). */
void poe_port_reset(uint8_t port_num) __banked
{
	if (!poe_live || !poe_up || !((poe_admin >> port_num) & 1)) {
		print_string("PoE: port not enabled\n");
		return;
	}
	bb_setup();
	poe_n = port_num;
	poe_set = 0;
	port_admin();
	delay(400);
	poe_n = port_num;
	poe_set = 1;
	port_admin();
	print_string("PoE: port ");
	itoa(port_num + 1);
	print_string(" power-cycled\n");
}

void poe_global_set(uint8_t on) __banked
{
	poe_admin = on ? 0xff : 0x00;
	if (!poe_live || !poe_up)
		return;
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
	if (!poe_st_on) {
		// Not powered: skip the measurement reads
		poe_st_class = 0xff;
		poe_st_volt = 0;
		poe_v2 = 0;
		poe_st_ma = 0;
		return;
	}

	poe_reg = 0x30 + (poe_local << 2);	poe_rd_len = 4;	bb_read_reg();
	poe_v2 = poe_resp[3];
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

/*
 * Once-a-second hook from the main loop. Every POE_SAMPLE_SECS: measure all
 * ports, accumulate energy and report PDs appearing/disappearing (console and
 * syslog). Also runs the optional host poll (`poe poll on`).
 */
void poe_tick(void) __banked
{
	if (machine.poe.chip == POE_NONE || !poe_up || !poe_live)
		return;
	// Watchdog second half: power the ports it switched off back on
	if (poe_wd_pending && ++poe_wd_wait >= POE_WD_OFF_SECS) {
		bb_setup();
		for (poe_n = 0; poe_n < machine.poe.n_ports; poe_n++) {
			if (((poe_wd_pending & poe_admin) >> poe_n) & 1) {
				poe_set = 1;
				port_admin();
			}
		}
		poe_wd_pending = 0;
	}
	poe_secs++;
	if (poe_poll_enabled && !(poe_secs & 1))
		poe_heartbeat();
	if (poe_secs < POE_SAMPLE_SECS)
		return;
	poe_secs = 0;
	for (poe_port_num = 0; poe_port_num < machine.poe.n_ports; poe_port_num++) {
		poe_get_port();
		poe_c = 1 << poe_port_num;
		if (poe_st_on) {
			// mW = (0.5 V units * mA) / 2
			poe_energy[poe_port_num] += ((uint32_t)poe_v2 * poe_st_ma) >> 1;
			if (!(poe_prev_on & poe_c)) {
				poe_prev_on |= poe_c;
				print_string("PoE: port ");
				itoa(poe_port_num + 1);
				print_string(" powered, class ");
				itoa(poe_st_class);
				write_char('\n');
			}
		} else if (poe_prev_on & poe_c) {
			poe_prev_on &= ~poe_c;
			print_string("PoE: port ");
			itoa(poe_port_num + 1);
			print_string(" power off\n");
		}
		// Link watchdog: powered but no Ethernet link for poe_wd_min minutes.
		// PoE port n is physical port n on these boards (link bit n).
		if (poe_wd_min[poe_port_num] && poe_st_on) {
			reg_read_m(RTL837X_REG_LINKS_STS);
			if ((sfr_data[1] >> poe_port_num) & 1) {
				poe_wd_cnt[poe_port_num] = 0;
			} else if (++poe_wd_cnt[poe_port_num] >= poe_wd_min[poe_port_num] * 6) {
				poe_wd_cnt[poe_port_num] = 0;
				print_string("PoE: port ");
				itoa(poe_port_num + 1);
				print_string(" watchdog: no link, power-cycling\n");
				poe_n = poe_port_num;
				poe_set = 0;
				port_admin();
				poe_wd_pending |= poe_c;
				poe_wd_wait = 0;
			}
		} else {
			poe_wd_cnt[poe_port_num] = 0;
		}
	}
}

/* Link watchdog for one port (0-based): minutes without link while powered
 * before the port is power-cycled, 0 = off. */
void poe_wd_set(uint8_t port_num, uint8_t minutes) __banked
{
	if (minutes > POE_WD_MAX_MIN)
		minutes = POE_WD_MAX_MIN;
	poe_wd_min[port_num] = minutes;
	poe_wd_cnt[port_num] = 0;
}

/* Decimal output of poe_calc into the HTTP buffer. */
static void out_dec(void)
{
	poe_k = 0;
	do {
		poe_dig[poe_k++] = '0' + (uint8_t)(poe_calc % 10);
		poe_calc /= 10;
	} while (poe_calc);
	while (poe_k)
		char_to_html(poe_dig[--poe_k]);
}

/*
 * GET /poe.json:
 * {"up":true,"budget":150,"ports":[{"port":1,"admin":true,"on":true,
 *   "class":4,"v2":104,"ma":73,"e":1234,"wd":0}, ...]}
 * v2 = voltage in 0.5 V units, e = energy since boot in mW x 10 s (Wh = e / 360000),
 * class 255 = none, wd = link watchdog in minutes (0 = off).
 */
void send_poe(void) __banked
{
	slen = strtox(outbuf, "HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Type: application/json\r\n\r\n");
	str_code_to_html("{\"up\":");
	str_code_to_html(poe_up ? "true" : "false");
	str_code_to_html(",\"budget\":");
	poe_calc = machine.poe.budget_w;	out_dec();
	str_code_to_html(",\"ports\":[");
	for (poe_port_num = 0; poe_port_num < machine.poe.n_ports; poe_port_num++) {
		if (poe_up) {
			poe_get_port();
		} else {
			poe_st_on = 0; poe_st_class = 0xff; poe_v2 = 0; poe_st_ma = 0;
		}
		if (poe_port_num)
			char_to_html(',');
		str_code_to_html("{\"port\":");
		poe_calc = poe_port_num + 1;	out_dec();
		str_code_to_html(",\"admin\":");
		str_code_to_html(((poe_admin >> poe_port_num) & 1) ? "true" : "false");
		str_code_to_html(",\"on\":");
		str_code_to_html(poe_st_on ? "true" : "false");
		str_code_to_html(",\"class\":");
		poe_calc = poe_st_class;	out_dec();
		str_code_to_html(",\"v2\":");
		poe_calc = poe_v2;		out_dec();
		str_code_to_html(",\"ma\":");
		poe_calc = poe_st_ma;		out_dec();
		str_code_to_html(",\"e\":");
		poe_calc = poe_energy[poe_port_num];	out_dec();
		str_code_to_html(",\"wd\":");
		poe_calc = poe_wd_min[poe_port_num];	out_dec();
		char_to_html('}');
	}
	str_code_to_html("]}");
}

#endif /* POE_CHIP_RTL8238 */
