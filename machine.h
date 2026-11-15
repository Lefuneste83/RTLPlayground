#ifndef _MACHINE_H_
#define _MACHINE_H_

#include <stdint.h>

#define MACHINE_STR_HELPER(x)  #x
#define MACHINE_STR(x) MACHINE_STR_HELPER(x)

#ifndef MACHINE_PCB_NAME
#define MACHINE_PCB_NAME unknown
#endif
#define MACHINE_PCB_INFO MACHINE_STR(MACHINE_PCB_NAME)


#define MACHINE_DEF_DIR machine

#ifndef MACHINE_NAME
	#define MACHINE_NAME default_machine
#endif

#define MACHINE_INC(dir,file) MACHINE_STR(dir/file)

#define RTL8370 0x70
#define RTL8372 0x72
#define RTL8373 0x73

#include  MACHINE_INC(MACHINE_DEF_DIR,MACHINE_NAME.h)

/*
 * PoE: a board with a software-controlled PSE defines POE_CHIP_<chip> in its
 * machine/<MACHINE>.h. That selects the driver (poe_rtl8238.c) and implies
 * POE_PRESENT, which gates the `poe` command, the boot bring-up and the PSE
 * image embedding in the Makefile. Boards without it compile no PoE code.
 */
#if defined(POE_CHIP_RTL8238)
#define POE_PRESENT
#endif


#ifndef MACHINE_CHIPSET
#error "MACHINE_CHIPSET shall be defined in <machine>.h file"
#endif

#ifndef MACHINE_MIN_PORT
#if MACHINE_CHIPSET == RTL8372
#define MACHINE_MIN_PORT 3
#else
#define MACHINE_MIN_PORT 0
#endif
#endif

#ifndef MACHINE_MAX_PORT
#if MACHINE_CHIPSET == RTL8370
#define MACHINE_MAX_PORT 7
#else
#define MACHINE_MAX_PORT 8
#endif
#endif

#define MACHINE_PORT_VALID(p) \
	((uint8_t)((p) - MACHINE_MIN_PORT) <= (MACHINE_MAX_PORT - MACHINE_MIN_PORT))

// auto derive SOC_FAMILY
#define RTL836X 6
#define RTL837X 7

#ifdef SOC_FAMILY
#error "SOC_FAMILY can not be manually defined, shall be auto derived"
#endif
#if (MACHINE_CHIPSET == RTL8372) || (MACHINE_CHIPSET == RTL8373)
#define SOC_FAMILY RTL837X
#elif (MACHINE_CHIPSET == RTL8370 )
#define SOC_FAMILY RTL836X
#else
#error "Unsupported MACHINE_CHIPSET is specified!"
#endif

#define LED_27 1
// SYSTEM LED
#define LED_28_SYS 2
#define LED_29 4

struct high_leds {
	// Defines MUX and LED enabling for pins 27-29
	uint8_t mux : 3;
	uint8_t enable : 3;
	uint8_t reserved : 2;
};

// PoE PSE chip type for struct machine.poe (POE_NONE = no software-controlled PoE)
enum { POE_NONE = 0, POE_RTL8238 = 1 };

struct poe_config {
	uint8_t chip;		// POE_NONE / POE_RTL8238
	uint8_t addr0;		// first PSE controller I2C address (ports 1-4)
	uint8_t addr1;		// second controller (ports 5-8), 0 = none
	uint8_t n_ports;	// number of PoE ports
};

struct sfp_port
{
	uint8_t pin_detect; // gpio number 0-63, 0xFF = don't have it?
	uint8_t pin_los; // gpio number 0-63, 0xFF = don't have it?
	uint8_t pin_tx_disable; // gpio number 0-63, 0xFF = not present
	uint8_t sds;
	uint8_t i2c;
};

struct machine {
	char machine_name[30];
	uint8_t isRTL8373;
	uint8_t n_sfp;
	uint8_t n_10g;
	uint8_t log_to_phys_port[9];
	uint8_t phys_to_log_port[9]; // Starts at 0 for port 1
	uint8_t is_sfp[9];  // 0 for non-SFP ports 1 or 2 for the I2C port number
	// sfp_port[0] is the first SFP-port from the left on the device, sfp_port[1] the next if present 
	struct sfp_port sfp_port[2];
	uint8_t reset_pin;
	struct high_leds high_leds;
	// Defines which led-set (0-3) will be used for given logical port
	// led-set is physical group of LEDs that can be configured to show different port status combinations (see port_led_set below)
	uint8_t port_led_set[9];
	// Defines led-set configuration, applied to all ports using particular led-set
	// Each led-set can have 4 different hardware LED configurations. Which one should be used, depends how LED is wired on the board
	// See stock RTL837X_REG_LED3_2_SETx and RTL837X_REG_LED1_0_SETx registers for reference configuration
	uint32_t led_sets[4][4];
	uint8_t led_mux_custom;
	uint8_t led_mux[28];
	uint32_t mac_flash_offset;
	// Read the flash with single-IO (CMD_FREAD) instead of DIO for MMIO/code
	// fetch. Set for boards whose flash chip is not usable in DIO mode, which
	// shows up as a board that does not boot at all, garbled console output or
	// random resets. Costs management-plane speed only, the ASIC keeps
	// forwarding in hardware.
	uint8_t flash_sio;
	// PoE PSE descriptor; chip = POE_NONE (all zero) on boards without PoE
	struct poe_config poe;
};

struct machine_runtime
{
	uint8_t isRTL8373 : 1;
	uint8_t isN : 1;
};

void machine_custom_init(void) __banked;

#endif
