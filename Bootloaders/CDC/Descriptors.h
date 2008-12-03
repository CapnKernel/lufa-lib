/*
             LUFA Library
     Copyright (C) Dean Camera, 2008.
              
  dean [at] fourwalledcubicle [dot] com
      www.fourwalledcubicle.com
*/

/*
  Copyright 2008  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, and distribute this software
  and its documentation for any purpose and without fee is hereby
  granted, provided that the above copyright notice appear in all
  copies and that both that the copyright notice and this
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
 *  Header file for Descriptors.c.
 */

#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

	/* Includes: */
		#include <LUFA/Drivers/USB/USB.h>

	/* Macros: */
		/** Structure for a CDC class Functional descriptor, with a given data size. This is used instead of a
		 *  type define so that the same macro can be used for functional descriptors of varying data lengths,
		 *  while allowing the sizeof() operator to return correct results.
		 *
		 *  \param DataSize  Size of the functional descriptor's data payload, in bytes
		 */
		#define CDC_FUNCTIONAL_DESCRIPTOR(DataSize)        \
		     struct                                        \
		     {                                             \
		          USB_Descriptor_Header_t Header;          \
			      uint8_t                 SubType;         \
		          uint8_t                 Data[DataSize];  \
		     }

		/** Endpoint number for the CDC control interface event notification endpoint. */
		#define CDC_NOTIFICATION_EPNUM         3

		/** Size of the CDC control interface notification endpoint bank, in bytes */
		#define CDC_NOTIFICATION_EPSIZE        8

		/** Endpoint number for the CDC data interface TX (data IN) endpoint */
		#define CDC_TX_EPNUM                   1	

		/** Endpoint number for the CDC data interface RX (data OUT) endpoint */
		#define CDC_RX_EPNUM                   2	

		/** Size of the CDC data interface TX and RX data endpoint banks, in bytes */
		#define CDC_TXRX_EPSIZE                16

	/* Type Defines: */
		/** Type define for the device configuration descriptor structure. This must be defined in the
		 *  application code, as the configuration descriptor contains several sub-descriptors which
		 *  vary between devices, and which describe the device's usage to the host.
		 */
		typedef struct
		{
			USB_Descriptor_Configuration_Header_t    Config;
			USB_Descriptor_Interface_t               CCI_Interface;
			CDC_FUNCTIONAL_DESCRIPTOR(2)             CDC_Functional_IntHeader;
			CDC_FUNCTIONAL_DESCRIPTOR(2)             CDC_Functional_CallManagement;
			CDC_FUNCTIONAL_DESCRIPTOR(1)             CDC_Functional_AbstractControlManagement;
			CDC_FUNCTIONAL_DESCRIPTOR(2)             CDC_Functional_Union;
			USB_Descriptor_Endpoint_t                ManagementEndpoint;
			USB_Descriptor_Interface_t               DCI_Interface;
			USB_Descriptor_Endpoint_t                DataOutEndpoint;
			USB_Descriptor_Endpoint_t                DataInEndpoint;
		} USB_Descriptor_Configuration_t;

	/* Function Prototypes: */
		/** Prototype for the function to return the address and size of a given descriptor when requested by
		 *  the host. See StdDescriptors.h for more details.
		 */
		bool USB_GetDescriptor(const uint16_t wValue, const uint8_t wIndex,
		                       void** const DescriptorAddress, uint16_t* const DescriptorSize)
		                       ATTR_WARN_UNUSED_RESULT ATTR_WEAK ATTR_NON_NULL_PTR_ARG(3, 4);

#endif
