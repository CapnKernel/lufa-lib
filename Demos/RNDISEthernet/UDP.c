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

#define  INCLUDE_FROM_UDP_C
#include "UDP.h"

int16_t UDP_ProcessUDPPacket(void* IPHeaderInStart, void* UDPHeaderInStart, void* UDPHeaderOutStart)
{
	IP_Header_t*  IPHeaderIN   = (IP_Header_t*)IPHeaderInStart;
	UDP_Header_t* UDPHeaderIN  = (UDP_Header_t*)UDPHeaderInStart;
	UDP_Header_t* UDPHeaderOUT = (UDP_Header_t*)UDPHeaderOutStart;
	
	int16_t RetSize = NO_RESPONSE;
	
	DecodeUDPHeader(UDPHeaderInStart);
	
	/* Check to see if the UDP packet is a DHCP packet */
	if (SwapEndian_16(UDPHeaderIN->DestinationPort) == UDP_PORT_DHCP_REQUEST)
	{
		RetSize = DHCP_ProcessDHCPPacket(&((uint8_t*)UDPHeaderInStart)[sizeof(UDP_Header_t)],
	                                     &((uint8_t*)UDPHeaderOutStart)[sizeof(UDP_Header_t)]);
	}
	
	if (RetSize > 0)
	{
		UDPHeaderOUT->SourcePort = UDPHeaderOUT->DestinationPort;
		UDPHeaderOUT->DestinationPort = UDPHeaderOUT->SourcePort;
		UDPHeaderOUT->Checksum = 0;
		UDPHeaderOUT->Length = RetSize;
		
		UDPHeaderOUT->Checksum = UDP_Checksum16(UDPHeaderOUT, IPHeaderIN->DestinationAddress, 
			                                    IPHeaderIN->SourceAddress,
			                                    (sizeof(UDP_Header_t) + RetSize));

		/* Return the size of the response so far */
		return (sizeof(UDP_Header_t) + RetSize);
	}
	
	return NO_RESPONSE;
}

static uint16_t UDP_Checksum16(void* UDPHeaderOutStart, IP_Address_t SourceAddress,
                               IP_Address_t DestinationAddress, uint16_t UDPOutSize)
{
	uint32_t Checksum = 0;
	
	/* UDP/IP checksums are the addition of the one's compliment of each word including the IP psudo-header,
	   complimented */
	
	Checksum += ((uint16_t*)&SourceAddress)[0];
	Checksum += ((uint16_t*)&SourceAddress)[1];
	Checksum += ((uint16_t*)&DestinationAddress)[0];
	Checksum += ((uint16_t*)&DestinationAddress)[1];
	Checksum += SwapEndian_16(PROTOCOL_UDP);
	Checksum += SwapEndian_16(UDPOutSize);

	for (uint8_t CurrWord = 0; CurrWord < (UDPOutSize >> 1); CurrWord++)
	  Checksum += ((uint16_t*)UDPHeaderOutStart)[CurrWord];
	
	if (UDPOutSize & 0x01)
	  Checksum += (((uint16_t*)UDPHeaderOutStart)[UDPOutSize >> 1] & 0x00FF);
	  
	while (Checksum & 0xFFFF0000)
	  Checksum = ((Checksum & 0xFFFF) + (Checksum >> 16));
	
	return ~Checksum;
}
