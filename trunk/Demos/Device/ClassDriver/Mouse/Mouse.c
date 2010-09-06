// TODO: Implement a median filter.
#define TS
#define MAPPER
// 32 bounces a little
// 128 is mostly ok.  worst test case: dotting
// 128 gives around 7-8 coords per USB period
#define TASK_TIME 128
// #define SWEEP

// How do we generate a "pen down" event?  From the pen or the button?
#define CONFIG_PEN_DOWN
//#define CONFIG_BUTTON_DOWN

// Does the LED show the button-down status?
//#define CONFIG_LED_SHOWS_DOWN

#ifdef CONFIG_BUTTON_DOWN
#  define CONFIG_BUTTON
#endif

#if defined(CONFIG_LED_SHOWS_DOWN)
#  define CONFIG_LED
#endif

#define TS_NORMAL
// #define TS_RAMP
// #define TS_BULLSEYE

// If, after mapping, an axis reading is outside this range,
// it's likely to be a pen-up condition.
#define AXIS_MAX 1023
#define AXIS_MIN 0

#ifdef CONFIG_BUTTON
// Set PB6 for input, with pullup enabled
#define BUTTON_CONFIG  do {DDRB &= ~_BV(DDB6); PORTB |= _BV(PB6);} while (0);
// Is button down?
#define BUTTON_DOWN  (PINB & _BV(PB6))
#endif

#ifdef CONFIG_LED
// PD6
#define LED_CONFIG  (DDRD |= _BV(DDD6))
#define LED_ON      (PORTD |= _BV(PD6))
#define LED_OFF     (PORTD &= ~_BV(PD6))
#define LED_TOGGLE  (PORTD ^= _BV(PD6))
#endif

/*
             LUFA Library
     Copyright (C) Dean Camera, 2010.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com
*/

/*
  Copyright 2010  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this 
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in 
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting 
  documentation, and that the name of the author not be used in 
  advertising or publicity pertaining to distribution of the 
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the Mouse demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 *
 * Touchscreen changes (c) 2010 by Mitch Davis (mjd+lufa-ts@afork.com).
 * Copyright assigned to Dean Camera.
 * 
 * pin 1 on TS is closest to corner
 *  On_TS Side Pin  ADC  X Y
 *  pin 1 top  PF0  ADC0 P 5    
 *  pin 2 rite PF1  ADC1 0 P
 *  pin 3 bott PF4  ADC4 A 0
 *  pin 4 left PF5  ADC5 5 A
 *  0=0V, 5=5V, P=pulled high via 10k to Vcc, A=ADC input 
 */

#include <LUFA/Drivers/Peripheral/ADC.h>

#include "Mouse.h"

#ifdef CONFIG_BUTTON
static void button_config(void)
{
	BUTTON_CONFIG;
}

static unsigned int button_down(void)
{
	return BUTTON_DOWN;
}
#endif

typedef struct
{
	int16_t x;
	int16_t y;
} Coords;

typedef struct
{
	uint8_t Button; /**< Button mask for pen-down indication */
	uint16_t  X; /**< Current absolute X position of pen press */
	uint16_t  Y; /**< Current absolute Y position of pen press */
} USB_TouchscreenReport_Data_t;

/** Buffer to hold the previously generated Mouse HID report, for comparison purposes inside the HID class driver. */
uint8_t PrevMouseHIDReportBuffer[sizeof(USB_TouchscreenReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Mouse_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = 0,

				.ReportINEndpointNumber       = MOUSE_EPNUM,
				.ReportINEndpointSize         = MOUSE_EPSIZE,
				.ReportINEndpointDoubleBank   = false,

				.PrevReportINBuffer           = PrevMouseHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevMouseHIDReportBuffer),
			},
	};

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{	
	SetupHardware();
	
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);

	sei();

	for (;;)
	{
		HID_Device_USBTask(&Mouse_HID_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	ADC_Init(ADC_SINGLE_CONVERSION | ADC_PRESCALE_32);

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();

#ifdef CONFIG_LED
	/* LED output */
	LED_CONFIG;
	LED_OFF;
#endif

#ifdef CONFIG_BUTTON
	button_config();
#endif

	// Set up timer
	TCNT1 = 0;
	TCCR1B |= (1 << WGM12); // Configure timer 1 for CTC mode 
	OCR1A = 20000;
	TCCR1B |= ((1 << CS10) | (1 << CS11)); // Set up timer at Fcpu/64 
	// TODO: Put timer into one-shot mode so it doesn't auto-repeat
	TIMSK1 |= (1 << OCIE1A); // Enable CTC interrupt
}

/** Event handler for the library USB WakeUp event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Suspend event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Mouse_HID_Interface);

	USB_Device_EnableSOFEvents();

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Unhandled Control Request event. */
void EVENT_USB_Device_UnhandledControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Mouse_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Mouse_HID_Interface);
}

// How many samples are waiting to be picked up.
// Could be zero, could be one, could be more than one.
static volatile uint8_t samplesReady = 0;
static Coords theSample;

// Return values for X and Y, by atomically retrieving
// the sample storage location from the timer interrupt.
static bool read_XY(Coords *c)
{
#ifdef SWEEP
#define LOWER_SWEEP 0
#define UPPER_SWEEP 119
	static uint16_t sweep = LOWER_SWEEP;
#endif
	register bool ret;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
#ifndef SWEEP
		if ((ret = samplesReady))
		{
			*c = theSample;
			samplesReady = false;
		}
#else
		if (++sweep >= UPPER_SWEEP)
		{
			sweep = LOWER_SWEEP;
		}

		ret = samplesReady > 0;
		if (ret)
		{
			c->x = sweep;
			c->y = samplesReady;
			samplesReady = 0;
		}
#endif
	}

	return ret;
}

// Map ADC readings (0-1023 but usually well within this, eg, 220-905)
// into logical 0-1023.
static void map(Coords *c)
{
#ifdef MAPPER
	// These values come from my magic spreadsheet
	c->x *= 25;
	c->x >>= 4;
	c->x += -342;
	c->y *= 25;
	c->y >>= 4;
	c->y += -364;
#endif
}

#ifdef CONFIG_PEN_DOWN
static bool inBox(Coords *c)
{
	return c->x >= 0 && c->x <= AXIS_MAX && c->y >= 0 && c->y <= AXIS_MAX;
}
#endif

typedef enum {
	eUp = 0,
	eDown = 1,
} StateType;

static StateType prevState = eUp;

typedef enum {
#ifdef TS_NORMAL
	eNormal,
#endif
#ifdef TS_RAMP
	eRamp,
#endif
#ifdef TS_BULLSEYE
	eBullseye,
#endif
} ModeType;

#ifdef TS_NORMAL
static uint16_t mode_NORMAL(USB_TouchscreenReport_Data_t *TSReport, const Coords * const rawVal)
{
	static Coords mappedVal;
	static Coords lastGood;

	// We can't change the raw value (see readXY())
	// so take a copy and change that.
	mappedVal = *rawVal;
	map(&mappedVal);

#ifdef CONFIG_BUTTON_DOWN
	// Check for button up/button down state
	StateType thisState = button_down() ? eDown : eUp;
#elif defined(CONFIG_PEN_DOWN)
	// Check for pen up/pen down state
	StateType thisState = inBox(&mappedVal) ? eDown : eUp;
#else
	StateType thisState = eUp;
#endif

#ifdef CONFIG_LED_SHOWS_DOWN
	if (thisState == eDown)
	{
		LED_ON;
	}
	else
	{
		LED_OFF;
	}
#endif

	TSReport->Button = thisState == eDown ? 1 : 0;

	// If we're in the down state, let's use these values, and keep them
	// for when we're up.  If we're up, use the last known good values
	// from when we were down.
	if (thisState == eDown)
	{
		lastGood = mappedVal;
	}
	else
	{
		mappedVal = lastGood;
	}

	// Pack the data into the report struct.  Note, when we're up,
	// we still report the last known good down position.
	TSReport->X = lastGood.x;
	TSReport->Y = lastGood.y;

	uint16_t ReportSize;
	if (prevState == thisState && thisState == eUp)
	{
		// if we're up now and we were up before, don't report it
		ReportSize = 0;
	}
	else
	{
		// if we're down now, or this is the first sample taken with
		// pen up after being down, report it.
		ReportSize = sizeof(USB_TouchscreenReport_Data_t);
	}

	// Remember pen state for next time
	prevState = thisState;

	return ReportSize;
}
#endif

#ifdef TS_RAMP
static void mode_RAMP(void)
{
#if 0
// Old sweep stuff kept around for debugging.
#define TICKS_FOR_ONE_DIR 125
#define LOWER_SWEEP 0
#define UPPER_SWEEP 255
static uint16_t sweep = LOWER_SWEEP;
#endif
#if 0
	if (++sweep == UPPER_SWEEP)
	{
		sweep = LOWER_SWEEP;
	}
#endif
#if 0
#  if defined(ADC_X) && !defined(ADC_Y)
	rawXVal = read_X();
	rawYVal = sweep;
	setup_X();
#  endif
#  if !defined(ADC_X) && defined(ADC_Y)
	rawXVal = sweep;
	rawYVal = read_Y();
	setup_Y();
#  endif
#endif
}
#endif

#ifdef TS_BULLSEYE
static void mode_BULLSEYE(void)
{
}
#endif

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either REPORT_ITEM_TYPE_In or REPORT_ITEM_TYPE_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent
 *
 *  \return Boolean true to force the sending of the report, false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	static ModeType currentMode = eNormal;
	static Coords rawVal;

	USB_TouchscreenReport_Data_t* TSReport = (USB_TouchscreenReport_Data_t *)ReportData;

	bool haveASample = read_XY(&rawVal);
	if (haveASample == false)
	{
		*ReportSize = 0;
		return false;
	}

	switch (currentMode)
	{
#ifdef TS_NORMAL
		case eNormal:
			*ReportSize = mode_NORMAL(TSReport, &rawVal);
			break;
#endif
#ifdef TS_RAMP
		case eRamp:
			break;
#endif

#ifdef TS_BULLSEYE
		case eBullseye:
			break;
#endif
	}

	return false;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either REPORT_ITEM_TYPE_Out or REPORT_ITEM_TYPE_Feature
 *  \param[in] ReportData  Pointer to a buffer where the created report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
	// Unused (but mandatory for the HID class driver) in this demo, since there are no Host->Device reports
}

#ifdef TS
static void setup_X(void)
{
	ADC_SetupChannel(4);

	// PF5 (left) and PF1 (right) are outputs
	DDRF |= _BV(DDF1) | _BV(DDF5);
	PORTF |= _BV(PF1); // set bit (hi)
	PORTF &= ~_BV(PF5); // clear bit (lo)

	// Let's make PF0 an input, but with a pull-up.
	// Coordinate reading without pulling other side
	// high makes readings much worse.
	DDRF &= ~_BV(DDF0);
	PORTF |= _BV(PF0);
}

static void setup_Y(void)
{
	ADC_SetupChannel(5);

	// PF0 (top) and PF4 (bottom) are outputs
	DDRF |= _BV(DDF0) | _BV(DDF4);
	PORTF &= ~_BV(PF0); // clear bit (lo)
	PORTF |= _BV(PF4); // set bit (hi)

	// Let's make PF1 an input, but with a pull-up.
	// Coordinate reading without pulling other side
	// high makes readings much worse.
	DDRF &= ~_BV(DDF1);
	PORTF |= _BV(PF1);
}

static int read_X(void)
{
	return ADC_GetChannelReading(4 | ADC_REFERENCE_AVCC | ADC_RIGHT_ADJUSTED);
}

static int read_Y(void)
{
	return ADC_GetChannelReading(5 | ADC_REFERENCE_AVCC | ADC_RIGHT_ADJUSTED);
}
#endif

typedef void (*NoArgsFn_t)(void);

typedef struct {
	NoArgsFn_t handler;
	unsigned int delay_until_next;
} ReadTSStateHandler_t;

static void readXandSetupForY(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// FIXME: This is not true atomic, as
		// this will make x available before Y has
		// been read.
		theSample.x = read_X();
	}
	setup_Y();
}

static void readYandSetupForX(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		theSample.y = read_Y();
		samplesReady++;
		if (samplesReady > 250)
			samplesReady = 250;
	}
	setup_X();
}

static int ReadTSState = 0;

static ReadTSStateHandler_t ReadTSStates[] = {
	{readXandSetupForY, TASK_TIME},
	{readYandSetupForX, TASK_TIME},
};

ISR(TIMER1_COMPA_vect)
{
	ReadTSStateHandler_t *state = &ReadTSStates[ReadTSState];
	state->handler();
	if (++ReadTSState == sizeof(ReadTSStates)/sizeof(ReadTSStates[0]))
		ReadTSState = 0;
	// TODO: Once the code has been changed to put the timer into
	// one-shot mode, kick off timer interrupt again
	OCR1A = state->delay_until_next;
}

// TODO: Start setup of other axis on ADC completion, not on timer.
// ADC_vect 

