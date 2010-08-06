#define CONFIG_PEN_DOWN
//#define CONFIG_BUTTON_DOWN
//#define CONFIG_LED
//#define CONFIG_LED_SHOWS_DOWN
//#define CONFIG_TIMER
//#define CONFIG_LED_SHOWS_TIMER
//#define CONFIG_BUTTON
//#define CONFIG_TIMER

// If the reading is past this, it's likely to be a pen-up condition.
#define AXIS_MAX 950

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

#ifdef CONFIG_TIMER
#define TIMER_CONFIG do {} while(0)
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

#ifdef CONFIG_TIMER
	TCNT1 = 0;
	TCCR1B |= ((1 << CS10) | (1 << CS11)); // Set up timer at Fcpu/64 
#endif

	for (;;)
	{
#ifdef CONFIG_TIMER
#if 0
		LED_ON;
		_delay_ms(500);
		LED_OFF;
		_delay_ms(500);
#else
		if (TCNT1 >= 10000)
		{
			LED_TOGGLE;
			TCNT1 = 0;
		}
#endif
#endif

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
	LEDs_SetAllLEDs(LEDMASK_USB_READY);

	if (!(HID_Device_ConfigureEndpoints(&Mouse_HID_Interface)))
	  LEDs_SetAllLEDs(LEDMASK_USB_ERROR);

	USB_Device_EnableSOFEvents();
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

// Map ADC readings (0-1023 but usually well within this, eg, 220-905)
// into logical 0-1023.
static void map(Coords *c)
{
	// These values come from my magic spreadsheet
	c->x *= 25;
	c->x >>= 4;
	c->x += -342;
	c->y *= 25;
	c->y >>= 4;
	c->y += -364;
}

static bool inBox(Coords *c)
{
	// The magic value of 9999 will be regarded as out-of-box
 	return c->x >= 0 && c->x <= AXIS_MAX && c->y >= 0 && c->y <= AXIS_MAX;
}

typedef enum {
	eX = 0,
	eY = 1,
} WhichAxisType;

static WhichAxisType xymode = 0;
#if 0
#define TICKS_FOR_ONE_DIR 125
#define LOWER_SWEEP 0
#define UPPER_SWEEP 255
static uint16_t sweep = LOWER_SWEEP;
#endif

static Coords rawVal = {0, 0};
static Coords mappedVal;
static Coords finalVal;

typedef enum {
	eUp = 0,
	eDown = 1,
} State;

static State prevState = eUp;

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
	USB_TouchscreenReport_Data_t* TSReport = (USB_TouchscreenReport_Data_t *)ReportData;

	// Touchscreen
	if (xymode == eX)
	{
		rawVal.x = read_X();
		setup_Y();
	}
	else
	{
		rawVal.y = read_Y();
		setup_X();
	}

	xymode = xymode == eX ? eY : eX;
#if 0
	if (++sweep == UPPER_SWEEP)
	{
		sweep = LOWER_SWEEP;
	}

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

	mappedVal = rawVal;
	map(&mappedVal);

	// Check for pen up/pen down state
	State thisState = inBox(&mappedVal) ? eDown : eUp;

	// If we're in the down state, let's use these values.
	// (If we're up, the old values don't get overwritten)
	if (thisState == eDown)
	{
		finalVal = mappedVal;
	}

	// Pack the data into the report struct
	TSReport->Button = thisState == eDown ? 1 : 0;
	TSReport->X = finalVal.x;
	TSReport->Y = finalVal.y;

	if (prevState == thisState && thisState == eUp)
	{
		// if we're up now and we were up before, don't report
		*ReportSize = 0;
	}
	else
	{
		// if we're down now, or this is the first sample taken with
		// pen up, report it.
		*ReportSize = sizeof(USB_TouchscreenReport_Data_t);
	}

	// Remember pen state for next time
	prevState = thisState;

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
