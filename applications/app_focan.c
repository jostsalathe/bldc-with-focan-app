/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "log.h"
#include "mempools.h"
#include "utils_math.h"
#include "terminal.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdio.h>


// Threads
static THD_FUNCTION(focan_protocol_thread, arg);
static THD_WORKING_AREA(focan_protocol_thread_wa, 1024);

// Private functions
static void terminalCallback(int argc, const char **argv);

// processing function prototypes
static void processByte(uint8_t data);
static void checkMsgTimeout(void);
static void interpreteRxData(void);
static void checkBreaksReleased(void);
static bool crcValid(void);
static void sendResponse(void);
static void logSendField(int fieldIndex, float fieldValue);
static void logMcData(void);
static void setErpmLimited(bool limited);


// Private variables
static volatile bool stop_now = TRUE;
static volatile bool is_running = FALSE;
static volatile bool enablePrintf = FALSE;
#define MSG_TIMEOUT_MS 1100
static volatile systime_t timeLastValidMessage;


#define BREAKS_RELEASED_PORT	HW_ADC_EXT_GPIO
#define BREAKS_RELEASED_PIN		HW_ADC_EXT_PIN
static volatile bool breaksReleased;

#define KMH_MIN			03.0f
#define KMH_LIMITED		22.0f
#define KMH_FREE		42.0f
#define KMH_TO_ERPM(KMH) KMH * 319.69f


#define LOG_CAN_ID 2
enum {
	// items that get regular updates first for easy indexing
	LOG_INDEX_V_BAT,		// [V]
	LOG_INDEX_I_BAT,		// [A]
	LOG_INDEX_I_MOT,		// [A]
	LOG_INDEX_E,			// [Wh]
	LOG_INDEX_E_CHG,		// [Wh]
	LOG_INDEX_DUTY,			// []
	LOG_INDEX_SPEED,		// [km/h]
	LOG_INDEX_TRIP,			// [km]
	LOG_INDEX_TEMP_FET,		// [°C]
	LOG_INDEX_FAULT,		// []
	// items after this get updated on change
	LOG_INDEX_THROTTLE,		// []
	LOG_INDEX_BREAKING,		// []
	LOG_INDEX_SPEED_LIMIT,	// [km/h]
	LOG_N_FIELDS
};
#define LOG_N_FIELDS_MC LOG_INDEX_THROTTLE


static SerialConfig uart_cfg = {
	9600,
	0,
	0,
	0
};

static SerialDriver * TxSerialPortDriver = &HW_UART_DEV;
static SerialDriver * RxSerialPortDriver = &HW_UART_DEV;
static stm32_gpio_t * TxGpioPort = HW_UART_TX_PORT;
static stm32_gpio_t * RxGpioPort = HW_UART_RX_PORT;
static uint8_t TxGpioPin = HW_UART_TX_PIN;
static uint8_t RxGpioPin = HW_UART_RX_PIN;
static uint8_t gpioAF = HW_UART_GPIO_AF;


#define TX_BUFFER_SIZE 14
#define RX_BUFFER_SIZE 20
static uint8_t TxBuffer[TX_BUFFER_SIZE];
static uint8_t RxBuffer[RX_BUFFER_SIZE];
static uint8_t RxIndex;


// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	stop_now = FALSE;
	RxIndex = 0;
	breaksReleased = FALSE;

	log_config_field(LOG_CAN_ID, LOG_INDEX_V_BAT, "v_bat", "Battery Voltage", "V", 3, false, false);
	log_config_field(LOG_CAN_ID, LOG_INDEX_I_BAT, "i_bat", "Battery Current", "A", 3, false, false);
	log_config_field(LOG_CAN_ID, LOG_INDEX_I_MOT, "i_mot", "Motor Current", "A", 3, false, false);
	log_config_field(LOG_CAN_ID, LOG_INDEX_E, "e", "Energy Consumed", "Wh", 3, false, false);
	log_config_field(LOG_CAN_ID, LOG_INDEX_E_CHG, "e_chg", "Energy Charged", "Wh", 3, false, false);
	log_config_field(LOG_CAN_ID, LOG_INDEX_DUTY, "duty", "Duty Cycle", "", 3, false, false);
	log_config_field(LOG_CAN_ID, LOG_INDEX_SPEED, "spd", "Speed", "km/h", 3, false, false);
	log_config_field(LOG_CAN_ID, LOG_INDEX_TRIP, "trip", "Trip Distance", "km", 3, false, false);
	log_config_field(LOG_CAN_ID, LOG_INDEX_TEMP_FET, "tmp", "MOSFET Temperature", "°C", 2, false, false);
	log_config_field(LOG_CAN_ID, LOG_INDEX_FAULT, "flt", "Fault Code", "", 3, false, false);
	log_config_field(LOG_CAN_ID, LOG_INDEX_THROTTLE, "thr", "Throttle", "", 3, false, false);
	log_config_field(LOG_CAN_ID, LOG_INDEX_BREAKING, "br", "Break Indicator", "", 0, false, false);
	log_config_field(LOG_CAN_ID, LOG_INDEX_SPEED_LIMIT, "spd_lim", "Speed Limit", "km/h", 1, false, false);
	log_start(LOG_CAN_ID, LOG_N_FIELDS, 10.0f, true, false, false); //TODO as soon as GPS works: true, true);

	palSetPadMode(BREAKS_RELEASED_PORT, BREAKS_RELEASED_PIN, PAL_MODE_INPUT_PULLUP);

	sdStart(TxSerialPortDriver, &uart_cfg);
	sdStart(RxSerialPortDriver, &uart_cfg);
	palSetPadMode(TxGpioPort, TxGpioPin, PAL_MODE_ALTERNATE(gpioAF) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
	palSetPadMode(RxGpioPort, RxGpioPin, PAL_MODE_ALTERNATE(gpioAF) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
	
	terminal_register_command_callback(
		"focan",
		"toggle focan app terminal output",
		NULL,
		terminalCallback);

	chThdCreateStatic(focan_protocol_thread_wa, sizeof(focan_protocol_thread_wa),
			NORMALPRIO, focan_protocol_thread, NULL);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	stop_now = TRUE;

	palSetPadMode(BREAKS_RELEASED_PORT, BREAKS_RELEASED_PIN, PAL_MODE_INPUT_ANALOG);

	sdStop(RxSerialPortDriver);
	sdStop(TxSerialPortDriver);
	palSetPadMode(TxGpioPort, TxGpioPin, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(RxGpioPort, RxGpioPin, PAL_MODE_INPUT_PULLUP);
	terminal_unregister_callback(terminalCallback);
	log_stop(LOG_CAN_ID);

	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

static THD_FUNCTION(focan_protocol_thread, arg) {
	(void)arg;

	chRegSetThreadName("App Focan");

	event_listener_t el;
	chEvtRegisterMaskWithFlags(&(RxSerialPortDriver->event), &el, EVENT_MASK(0), CHN_INPUT_AVAILABLE);

	timeLastValidMessage = chVTGetSystemTime();

	setErpmLimited(FALSE);
	setErpmLimited(TRUE);

	is_running = TRUE;

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			chEvtUnregister(&(RxSerialPortDriver->event), &el);
			is_running = FALSE;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.

		// Run your logic here. A lot of functionality is available in mc_interface.h.
		checkMsgTimeout();

		logMcData();

		chEvtWaitAnyTimeout(ALL_EVENTS, 10);
		bool rx = TRUE;
		while (rx) {
			msg_t res = sdGetTimeout(RxSerialPortDriver, TIME_IMMEDIATE);
			if (res != MSG_TIMEOUT) {
				processByte(res);
			} else {
				rx = FALSE;
			}
		}
	}
}

void processByte(uint8_t data) {
	if (RxIndex >= RX_BUFFER_SIZE				// protect from overflow
		|| (RxIndex == 0 && data != 0x01)		// first byte should be a 0x01
		|| (RxIndex == 1 && data != 0x14)		// second byte should be a 0x14 (data frame length)
		|| (RxIndex == 2 && data != 0x01)) {	// third byte should be a 0x01
		RxIndex = 0;
		if (enablePrintf)
		commands_printf("ignored received byte - index reset");
		return;
	}

	RxBuffer[RxIndex++] = data;

	if (RxIndex >= RX_BUFFER_SIZE) {	// after receiving the last byte
		if (enablePrintf)
		commands_printf("received complete message: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
						RxBuffer[0], RxBuffer[1], RxBuffer[2], RxBuffer[3], RxBuffer[4],
						RxBuffer[5], RxBuffer[6], RxBuffer[7], RxBuffer[8], RxBuffer[9],
						RxBuffer[10], RxBuffer[11], RxBuffer[12], RxBuffer[13], RxBuffer[14],
						RxBuffer[15], RxBuffer[16], RxBuffer[17], RxBuffer[18], RxBuffer[19]);
		checkBreaksReleased();
		if (crcValid()) {
			timeLastValidMessage = chVTGetSystemTime();
			interpreteRxData();
			sendResponse();
		}
		RxIndex = 0;
	}
}

static void checkMsgTimeout(void) {
	if (chVTTimeElapsedSinceX(timeLastValidMessage) > MS2ST(MSG_TIMEOUT_MS)) {
		// shut off motor
		mc_interface_set_current_rel(0.0f);
		logSendField(LOG_INDEX_THROTTLE, -0.0f);

		// reset timeout
		timeLastValidMessage = chVTGetSystemTime();
		
		// reset RX algorithm
		RxIndex = 0;

		// report timeout occurrence
		if (enablePrintf)
		commands_printf("%6d TIMEOUT detected! Shutting off motor.", ST2MS(chVTGetSystemTime()));
	}
}

void interpreteRxData(void) {
	// gear (0: off, 1: eco, 2: mid, 3: high)
	// uint8_t gear = RxBuffer[4] & 0x03;

	// light enable bit
	bool lightEn = (RxBuffer[5] & 0x20) != 0;
	setErpmLimited(!lightEn);

	// wheel diameter in 1/10 inch (25...500)
	// uint16_t wheelDiameter = ((RxBuffer[7] & 0x01) << 8) + RxBuffer[8];

	// speed lever position (400...1000)
	uint16_t speedLever = ((RxBuffer[16] & 0x03) << 8) + RxBuffer[17];
	if (enablePrintf)
	commands_printf("%6d %1d %04d", ST2MS(chVTGetSystemTime()), breaksReleased, speedLever);

	float throttle = speedLever >= 400 ? (speedLever - 400) / 600.0f : 0.0f;
	logSendField(LOG_INDEX_THROTTLE, throttle);
	if (breaksReleased && throttle > 0.0f && mc_interface_get_speed() * 3.6f > KMH_MIN) {
		mc_interface_set_current_rel(throttle);
	} else {
		mc_interface_set_brake_current_rel(throttle);
	}
}

void checkBreaksReleased(void) {
	// variables for some crude debouncing:
	//  the pad needs to read the same value three cycles in a row before breaksReleased is touched
	static bool padReadReleasedLastTime = FALSE;
	//static bool padReadReleasedLastLastTime = FALSE;

	bool padReadsReleased = palReadPad(BREAKS_RELEASED_PORT, BREAKS_RELEASED_PIN);

	if (padReadsReleased == padReadReleasedLastTime) {
		//&& padReadReleasedLastTime == padReadReleasedLastLastTime) {
		breaksReleased = padReadsReleased;
		logSendField(LOG_INDEX_BREAKING, breaksReleased ? 0.0f : 1.0f);
	}

	if (enablePrintf)
	//commands_printf("Break released (%d%d%d)? %s",
	//	padReadsReleased, padReadReleasedLastTime, padReadReleasedLastLastTime,
	commands_printf("Break released (%d%d)? %s",
		padReadsReleased, padReadReleasedLastTime,
		breaksReleased ? "released" : "pulled");

	//padReadReleasedLastLastTime = padReadReleasedLastTime;
	padReadReleasedLastTime = padReadsReleased;
}

// reveng.exe: width=8  poly=0x01  init=0x00  refin=FALSE  refout=FALSE  xorout=0x00  check=0x31  residue=0x00  name=(none)
// see https://reveng.sourceforge.io/readme.htm
uint8_t crc8calc(uint8_t *buffer, uint8_t len) {
	uint8_t poly = 0x01;
	uint8_t crc = 0;

	for (size_t i=0; i<len; ++i) {
		crc ^= buffer[i];
		for (size_t j=0; j<8; ++j) {
			if ((crc & 0x80) != 0) {
				crc = (crc << 1) ^ poly;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

bool crcValid(void) {
	uint8_t crcReg = 0;
	uint8_t crcReceived = RxBuffer[RX_BUFFER_SIZE-1];
	
	//crc8ProcessMessage(&crcReg, RxBuffer, RX_BUFFER_SIZE-1);
	crcReg = crc8calc(RxBuffer, RX_BUFFER_SIZE-1);

	if (enablePrintf)
	commands_printf("Received CRC = %02X and calculated CRC = %02X (%s)",
					crcReceived, crcReg, crcReceived == crcReg ? "valid" : "invalid");

	return (crcReg == crcReceived);
}

void sendResponse(void) {
	float rpm = mc_interface_get_rpm() / 15; // divided by motor pole pairs
	uint16_t msPerRev = rpm <= 2.0f ? 31456 : 60000.0f/rpm;

	TxBuffer[0] = 2;
	TxBuffer[1] = 14;
	TxBuffer[2] = 1;

	TxBuffer[3] = 0;
	TxBuffer[4] = 0;
	TxBuffer[5] = 0;
	TxBuffer[6] = 0;
	TxBuffer[7] = 0;

	// wheel speed in ms/rev 0...31456
	TxBuffer[8] = msPerRev>>8;
	TxBuffer[9] = msPerRev&0xFF;

	TxBuffer[10] = 0;
	TxBuffer[11] = 0;
	TxBuffer[12] = 0;

	TxBuffer[13] = crc8calc(TxBuffer, TX_BUFFER_SIZE-1);

	if (enablePrintf)
	commands_printf("responding with message: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
					RxBuffer[0], RxBuffer[1], RxBuffer[2], RxBuffer[3], RxBuffer[4],
					RxBuffer[5], RxBuffer[6], RxBuffer[7], RxBuffer[8], RxBuffer[9],
					RxBuffer[10], RxBuffer[11], RxBuffer[12], RxBuffer[13]);

	sdWrite(TxSerialPortDriver, TxBuffer, TX_BUFFER_SIZE);
}

static void logSendField(int fieldIndex, float fieldValue) {
	log_send_samples_f32(LOG_CAN_ID, fieldIndex, &fieldValue, 1);
}

static void logMcData(void) {
	float samples[LOG_N_FIELDS_MC] = {0.0f};

	samples[LOG_INDEX_V_BAT]	= mc_interface_get_input_voltage_filtered();
	samples[LOG_INDEX_I_BAT]	= mc_interface_get_tot_current_in_filtered();
	samples[LOG_INDEX_I_MOT]	= mc_interface_get_tot_current_filtered();
	samples[LOG_INDEX_E]		= mc_interface_get_watt_hours(false);
	samples[LOG_INDEX_E_CHG]	= mc_interface_get_watt_hours_charged(false);
	samples[LOG_INDEX_DUTY]		= mc_interface_get_duty_cycle_now();
	samples[LOG_INDEX_SPEED]	= mc_interface_get_speed() * 3.6f;
	samples[LOG_INDEX_TRIP]		= mc_interface_get_distance() / 1000.0f;
	samples[LOG_INDEX_TEMP_FET]	= mc_interface_temp_fet_filtered();
	samples[LOG_INDEX_FAULT]	= mc_interface_get_fault();

	log_send_samples_f32(LOG_CAN_ID, 0, samples, LOG_N_FIELDS_MC);
}

static void setErpmLimited(bool limited) {
	static bool currentlyLimited = true;
	if (limited != currentlyLimited) {
		// only use "unlimited" ERPM limit if the brake is engaged while switching the "light" on
		float newMaxSpeed = limited || breaksReleased ? KMH_LIMITED : KMH_FREE;

		// TODO see comm/commands.c > commands_process_packet() case COMM_SET_MCCONF
		mc_configuration *mcconf = mempools_alloc_mcconf();
		*mcconf = *mc_interface_get_configuration();

		mcconf->l_max_erpm = KMH_TO_ERPM(newMaxSpeed);

		commands_apply_mcconf_hw_limits(mcconf);
		mc_interface_set_configuration(mcconf);

		logSendField(LOG_INDEX_SPEED_LIMIT, newMaxSpeed);

		if (enablePrintf)
		commands_printf("Updated speed limit to %4.1f (%slimited)", (double) newMaxSpeed, limited ? "" : "un");

		currentlyLimited = limited;

		mempools_free_mcconf(mcconf);
	}
}

static void terminalCallback(int argc, const char **argv) {
	(void) argc;
	(void) argv;
	enablePrintf = !enablePrintf;
}
