/*
 * Copyright (c) 2001, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 * $Id: tapdev.c,v 1.8 2006/06/07 08:39:58 adam Exp $
 */	    
#include "tapdev.h"
#include "uip.h"
#include "uip_arp.h"
#include "enc28j60.h"

//MAC address
const uint8_t mymac[6]={0x04,0x02,0x35,0x00,0x00,0x01};	
																				  
//Setup both ethernet hardware and MAC address
//return: 0=success; 1=failed
uint8_t tapdev_init(void)
{   	 
	uint8_t i,res=0;					  
	res=ENC28J60_Init((uint8_t*)mymac);	//setup ENC28J60					  
	//write IP address and MAC address into buffer
 	for (i = 0; i < 6; i++) uip_ethaddr.addr[i]=mymac[i];  
    //RJ45 LED indicator setup    
	ENC28J60_PHY_Write(PHLCON,0x0476);
	return res;	
}
//read out a packet 
uint16_t tapdev_read(void)
{	
	return  ENC28J60_Packet_Receive(MAX_FRAMELEN,uip_buf);
}
//send a packet
void tapdev_send(void)
{
	ENC28J60_Packet_Send(uip_len,uip_buf);
}












/*---------------------------------------------------------------------------*/
