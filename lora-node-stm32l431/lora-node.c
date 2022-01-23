/*
Copyright © 2022 Frank Kunz

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the “Software”), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <lmic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rtc.h>
#include <debug.h>
#include <stdio.h>

#include <stdint.h>
#include <inttypes.h>

#define TX_TIMEOUT 60

static osjob_t reportjob;

// application router ID (LSBF)
static const u1_t APPEUI[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// unique device ID (LSBF)
static const u1_t DEVEUI[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// device-specific AES key (derived from device EUI)
static const u1_t DEVKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
	memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
	memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
	memcpy(buf, DEVKEY, 16);
}

static void reportfunc (osjob_t* j) {
	ostime_t tm = os_getTime();
	printf("Call %s %lums\n", __func__, (unsigned long)osticks2ms(tm));

	if (!(LMIC.opmode & OP_TXRXPEND)) {
		unsigned char txt[5] = "LoRa";
		LMIC_setTxData2(1, txt, sizeof(txt), 0);
	}
	os_setTimedCallback(j, tm+sec2osticks(TX_TIMEOUT), reportfunc);
}

// initial job
static void initfunc (osjob_t* j __attribute__((unused))) {
	printf("Call initfunc\n");

	// do here the init for sensors, etc.


	// reset MAC state
	LMIC_reset();

	// depends on RTC accuracy
	//LMIC_setClockError(MAX_CLOCK_ERROR * 5 / 100);

	// start joining
	LMIC_startJoining();
	// init done - onEvent() callback will be invoked...
}

// Delay loop for the default 8 MHz CPU clock with optimizer enabled
static void delay(uint32_t msec)
{
	for (uint32_t j=0; j < 2000UL * msec; j++) {
		__asm__("nop");
	}
}

int main(void) {
	osjob_t initjob;

	/* small busy wait here to give the debugger the chance to connect before enter deep sleep */
	delay(300);

	debug_printf_init();
	printf("\nLoRa STM32 example node\n");

	// initialize runtime env
	os_init();
	// setup initial job
	os_setCallback(&initjob, initfunc);
	// execute scheduled jobs and events
	os_runloop();
	// (not reached)
	return 0;
}

DECL_ON_LMIC_EVENT {
	debug_print_event(e);
	switch(e) {
		// network joined, session established
		case EV_JOINED:
			// start TX loop
			reportfunc(&reportjob);
			break;
		case EV_TXCOMPLETE:
			if (LMIC.dataLen) {
				// downlink data have been received
				uint8_t i;
				printf("received %u bytes:\n", LMIC.dataLen);
				for(i=LMIC.dataBeg;i<LMIC.dataBeg+LMIC.dataLen;i++) {
					printf(" %02x", LMIC.frame[i]);
				}
				printf("\n");
			}
			break;
		default:
			break;
	}
}

void LMICOS_logEvent(const char *pMessage) {
	printf("%s\n", pMessage);
}

void LMICOS_logEventUint32(const char *pMessage, uint32_t datum) {
	printf("%s (0x%"PRIx32")\n", pMessage, datum);
}
