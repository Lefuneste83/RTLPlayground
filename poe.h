#ifndef _POE_H_
#define _POE_H_

/*
 * Generic, chip-agnostic PoE interface.
 * VERSION 1.5.0 - ported from PR #265 (Tobiasz) to the machine/ board layout.
 *
 * A per-chip driver implements these operations (poe_rtl8238.c, selected by a
 * board's POE_CHIP_* macro in its PCB source). The console (`poe ...`)
 * and the boot bring-up call only this interface. Multi-field status is
 * returned in xdata globals (never function locals) to keep the 8051 overlay
 * segment small.
 */

#include <stdint.h>

/* Early boot hook (right after flash_init): keeps a PSE whose firmware is
 * already running (warm reboot, no power cycle), otherwise uploads it. */
void poe_hold(void) __banked;

/* Late boot hook (after PHY init): uploads only if the early hook could not. */
void poe_bringup(void) __banked;

/* `poe load`: force a PSE reset and fresh upload (power-cycles all PDs). */
void poe_load(void) __banked;

/* Enable/disable PoE on a single port. port_num is 0-based. */
void poe_port_set(uint8_t port_num, uint8_t port_on) __banked;

/* Enable/disable all ports at once. */
void poe_global_set(uint8_t on) __banked;

/* Print a per-port status table on the console (`poe status`). */
void poe_status_print(void) __banked;

/* Raw controller state dump (`poe info`), for diagnostics. */
void poe_info_print(void) __banked;

/* Apply the per-port admin state; called once after the startup config ran.
 * Until then `poe port` only records the state and ports stay off. */
void poe_apply(void) __banked;

/* Power-cycle one port (0-based): off, ~2 s, on. */
void poe_port_reset(uint8_t port_num) __banked;

/* Link watchdog for one port (0-based): power-cycle it when it delivers power
 * but has had no link for `minutes` (1..40), 0 = off. */
void poe_wd_set(uint8_t port_num, uint8_t minutes) __banked;

/* Once-a-second hook from the main loop: energy counters, PD events, host poll. */
void poe_tick(void) __banked;

/* GET /poe.json */
void send_poe(void) __banked;

/* Periodic host poll, called every ~2 s from the main loop once PoE is up.
 * The PSE application appears to reset itself if the host stays silent after
 * the upload (the OEM firmware polls it continuously); this keeps it alive. */
void poe_heartbeat(void) __banked;
extern __xdata uint8_t poe_poll_enabled;	// `poe poll on|off`, default off

/*
 * Normalized per-port status: set poe_port_num (0-based), call poe_get_port(),
 * read back the fields below.
 */
extern __xdata uint8_t  poe_port_num;	// input: port 0-based
extern __xdata uint8_t  poe_st_admin;	// 1 = administratively enabled
extern __xdata uint8_t  poe_st_on;	// 1 = delivering power to a PD
extern __xdata uint8_t  poe_st_class;	// PD class 0..8 (0xff = none / unknown)
extern __xdata uint8_t  poe_st_volt;	// port voltage, volts
extern __xdata uint16_t poe_st_ma;	// port current, mA
void poe_get_port(void) __banked;

#endif
