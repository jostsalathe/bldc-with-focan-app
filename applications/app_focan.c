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

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

#define BREAKS_RELEASED_PORT	HW_ADC_EXT_GPIO
#define BREAKS_RELEASED_PIN		HW_ADC_EXT_PIN

static SerialConfig uart_cfg = {
	9600,
	0,
	USART_CR2_LINEN,
	0
};


// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	stop_now = false;
	palSetPadMode(BREAKS_RELEASED_PORT, BREAKS_RELEASED_PIN, PAL_MODE_INPUT_PULLUP);
	chThdCreateStatic(focan_protocol_thread_wa, sizeof(focan_protocol_thread_wa),
			NORMALPRIO, focan_protocol_thread, NULL);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	palSetPadMode(BREAKS_RELEASED_PORT, BREAKS_RELEASED_PIN, PAL_MODE_INPUT_ANALOG);

	stop_now = true;
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

	is_running = true;

	// Example of using the experiment plot
//	chThdSleepMilliseconds(8000);
//	commands_init_plot("Sample", "Voltage");
//	commands_plot_add_graph("Temp Fet");
//	commands_plot_add_graph("Input Voltage");
//	float samp = 0.0;
//
//	for(;;) {
//		commands_plot_set_graph(0);
//		commands_send_plot_points(samp, mc_interface_temp_fet_filtered());
//		commands_plot_set_graph(1);
//		commands_send_plot_points(samp, GET_INPUT_VOLTAGE());
//		samp++;
//		chThdSleepMilliseconds(10);
//	}

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.

		// Run your logic here. A lot of functionality is available in mc_interface.h.
		commands_printf("Break state: %s\n", palReadPad(BREAKS_RELEASED_PORT, BREAKS_RELEASED_PIN) ? "released" : "pulled");

		chThdSleepMilliseconds(10);
	}
}
