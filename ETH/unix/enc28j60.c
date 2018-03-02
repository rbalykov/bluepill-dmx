#include "spi2.h"
#include "timer6.h"
#include <stdio.h>
#include "enc28j60.h"	  

static u8 ENC28J60BANK;
static u32 NextPacketPtr;


//delay for ms unit
//suitable for crystal is 8MHz
//static function for ENC28J60
static void ENC28J60_delayms(u32 ms)
{
	u16 i=0;
	while(ms--)
	{
		for(i=0;i<8000;i++);
	}
}
//Reset ENC28J60
//Initialize SPI2 and related I/O for ENC28J60
static void ENC28J60_SPI2_Init(void)
{
   	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2,  ENABLE ); 	
   	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );//PORTB,D,G时钟使能 
 	//CS pin
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;				 
 	GPIO_Init(GPIOB, &GPIO_InitStructure);					 
 	GPIO_SetBits(GPIOB,GPIO_Pin_12);						 
	//SPI2 pin	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
 	//RST pin
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOA, GPIO_Pin_8);
	
 	//setup SPI2
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	
	SPI_InitStructure.SPI_CRCPolynomial = 7;	
	SPI_Init(SPI2, &SPI_InitStructure);  
 
	SPI_Cmd(SPI2, ENABLE); 
	
	SPI2_ReadWriteByte(0xff);
}
void ENC28J60_Reset(void)
{
 	 
	ENC28J60_SPI2_Init(); //re-init SPI2
//	SPI2_SetSpeed(SPI_BaudRatePrescaler_4);	//9MHz
	SPI2_SetSpeed(SPI_BaudRatePrescaler_4);	//9MHz
 	TIM6_Init(1000,719);//setup a 100kHz clock for ENC28J60
	ENC28J60_RST_CLEAR(); //reset ENC28J60		
	ENC28J60_delayms(10);	 
	ENC28J60_RST_SET();	//finish reset			    
	ENC28J60_delayms(10);	 
}

//Read ENC28J60 register
//op: command
//addr: register address
//return: read out data
u8 ENC28J60_Read_Op(u8 op,u8 addr)
{
	u8 dat=0;	 

	ENC28J60_SELECT();	 
	dat=op|(addr&ADDR_MASK);
	SPI2_ReadWriteByte(dat);
	dat=SPI2_ReadWriteByte(0xFF);
	//datasheet p.29, read two times to get MAC/MII register value
 	if(addr&0x80)dat=SPI2_ReadWriteByte(0xFF);
	ENC28J60_NO_SELECT();	
	return dat;
}
//Write ENC28J60 register
//op: command
//addr: register address
//data: parameter to write
void ENC28J60_Write_Op(u8 op,u8 addr,u8 data)
{
	u8 dat = 0;	    
	ENC28J60_SELECT();			   
	dat=op|(addr&ADDR_MASK);
	SPI2_ReadWriteByte(dat);	  
	SPI2_ReadWriteByte(data);
	ENC28J60_NO_SELECT();
}
//Read Rx buffer data from ENC28J60
//len: data length to read
//data: pointer to store data
void ENC28J60_Read_Buf(u32 len,u8* data)
{
	ENC28J60_SELECT();			 
	SPI2_ReadWriteByte(ENC28J60_READ_BUF_MEM);
	while(len)
	{
		len--;			  
		*data=(u8)SPI2_ReadWriteByte(0);
		data++;
	}
	*data='\0'; //add end char
	ENC28J60_NO_SELECT();
}
//Write data to send via ENC28J60
//len: data length to send
//data: data pointer
void ENC28J60_Write_Buf(u32 len,u8* data)
{
	ENC28J60_SELECT();			   
	SPI2_ReadWriteByte(ENC28J60_WRITE_BUF_MEM);		 
	while(len)
	{
		len--;
		SPI2_ReadWriteByte(*data);
		data++;
	}
	ENC28J60_NO_SELECT();
}
//Setup ENC28J60 register bank
//ban: Bank to be setup
void ENC28J60_Set_Bank(u8 bank)
{								    
	if((bank&BANK_MASK)!=ENC28J60BANK)
	{				  
		ENC28J60_Write_Op(ENC28J60_BIT_FIELD_CLR,ECON1,(ECON1_BSEL1|ECON1_BSEL0));
		ENC28J60_Write_Op(ENC28J60_BIT_FIELD_SET,ECON1,(bank&BANK_MASK)>>5);
		ENC28J60BANK=(bank&BANK_MASK);
	}
}
//Read ENC28J60 register
//addr: register address
//return: read out value
u8 ENC28J60_Read(u8 addr)
{						  
	ENC28J60_Set_Bank(addr);//select bank		 
	return ENC28J60_Read_Op(ENC28J60_READ_CTRL_REG,addr);
}
//Write ENC28J60 register
//addr: register address	 
void ENC28J60_Write(u8 addr,u8 data)
{					  
	ENC28J60_Set_Bank(addr);		 
	ENC28J60_Write_Op(ENC28J60_WRITE_CTRL_REG,addr,data);
}
//Write into PHY register of ENC28J60
//addr: register address
//data: parameter written into register	 
void ENC28J60_PHY_Write(u8 addr,u32 data)
{
	u16 retry=0;
	ENC28J60_Write(MIREGADR,addr);	
	ENC28J60_Write(MIWRL,data);		
	ENC28J60_Write(MIWRH,data>>8);		   
	while((ENC28J60_Read(MISTAT)&MISTAT_BUSY)&&retry<0XFFF)retry++;//wait until PHY writing finish  
}
//Setup ENC28J60
//macaddr: assigned MAC address
//return: 0=success, 1=failed
u8 ENC28J60_Init(u8* macaddr)
{		
	u16 retry=0;		  
	ENC28J60_Reset();
	ENC28J60_Write_Op(ENC28J60_SOFT_RESET,0,ENC28J60_SOFT_RESET); //software reset
	while(!(ENC28J60_Read(ESTAT)&ESTAT_CLKRDY)&&retry<500)	//wait until clock is stable
	{
		retry++;
		ENC28J60_delayms(1);
	};
	if(retry>=500)return 1;//initialization failed
	//set Rx buffer address with 8k capacity
	NextPacketPtr=RXSTART_INIT;
	// Rx start
	//setup Rx start byte
	ENC28J60_Write(ERXSTL,RXSTART_INIT&0xFF);	
	ENC28J60_Write(ERXSTH,RXSTART_INIT>>8);	  
	//setup read pointer byte
	ENC28J60_Write(ERXRDPTL,RXSTART_INIT&0xFF);
	ENC28J60_Write(ERXRDPTH,RXSTART_INIT>>8);
	//setup "end of rx" byte
	ENC28J60_Write(ERXNDL,RXSTOP_INIT&0xFF);
	ENC28J60_Write(ERXNDH,RXSTOP_INIT>>8);
	//setup "starting tx" byte 
	ENC28J60_Write(ETXSTL,TXSTART_INIT&0xFF);
	ENC28J60_Write(ETXSTH,TXSTART_INIT>>8);
	//setup "eno of tx" byte
	ENC28J60_Write(ETXNDL,TXSTOP_INIT&0xFF);
	ENC28J60_Write(ETXNDH,TXSTOP_INIT>>8);
	// do bank 1 stuff,packet filter
	ENC28J60_Write(ERXFCON,ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN);
	ENC28J60_Write(EPMM0,0x3f);
	ENC28J60_Write(EPMM1,0x30);
	ENC28J60_Write(EPMCSL,0xf9);
	ENC28J60_Write(EPMCSH,0xf7);
	// do bank 2 stuff
	ENC28J60_Write(MACON1,MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
	// bring MAC out of reset
	ENC28J60_Write(MACON2,0x00);
	// enable automatic padding to 60bytes and CRC operations
	ENC28J60_Write_Op(ENC28J60_BIT_FIELD_SET,MACON3,MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN|MACON3_FULDPX);
	// set inter-frame gap (non-back-to-back)
	ENC28J60_Write(MAIPGL,0x12);
	ENC28J60_Write(MAIPGH,0x0C);
	// set inter-frame gap (back-to-back)
	ENC28J60_Write(MABBIPG,0x15);
	// Set the maximum packet size which the controller will accept
	// Do not send packets longer than MAX_FRAMELEN:
	ENC28J60_Write(MAMXFLL,MAX_FRAMELEN&0xFF);	
	ENC28J60_Write(MAMXFLH,MAX_FRAMELEN>>8);
	// do bank 3 stuff
	// write MAC address
	// NOTE: MAC address in ENC28J60 is byte-backward
	ENC28J60_Write(MAADR5,macaddr[0]);	
	ENC28J60_Write(MAADR4,macaddr[1]);
	ENC28J60_Write(MAADR3,macaddr[2]);
	ENC28J60_Write(MAADR2,macaddr[3]);
	ENC28J60_Write(MAADR1,macaddr[4]);
	ENC28J60_Write(MAADR0,macaddr[5]);
	//setup PHY as Duplex
	ENC28J60_PHY_Write(PHCON1,PHCON1_PDPXMD);	 
	// no loopback of transmitted frames	 禁止环回
	//HDLDIS：PHY 半双工环回禁止位
	ENC28J60_PHY_Write(PHCON2,PHCON2_HDLDIS);
	// switch to bank 0  
	ENC28J60_Set_Bank(ECON1);
	// enable interrutps
	ENC28J60_Write_Op(ENC28J60_BIT_FIELD_SET,EIE,EIE_INTIE|EIE_PKTIE);
	// enable packet reception
	ENC28J60_Write_Op(ENC28J60_BIT_FIELD_SET,ECON1,ECON1_RXEN);
	if(ENC28J60_Read(MAADR5)== macaddr[0])return 0;//initialization success
	else return 1; 	  

}
//Read EREVID
u8 ENC28J60_Get_EREVID(void)
{
	return ENC28J60_Read(EREVID);
}

	#include "uip.h"
	
	//Send a packet to ethernet via ENC28J60
	//len: packet length
	//packet: data pointer 
	void ENC28J60_Packet_Send(u32 len,u8* packet)
	{
		ENC28J60_Write(EWRPTL,TXSTART_INIT&0xFF);
		ENC28J60_Write(EWRPTH,TXSTART_INIT>>8);
		//setup TXND pointer to fit packet length 
		ENC28J60_Write(ETXNDL,(TXSTART_INIT+len)&0xFF);
		ENC28J60_Write(ETXNDH,(TXSTART_INIT+len)>>8);
		//setup packet control byte
		ENC28J60_Write_Op(ENC28J60_WRITE_BUF_MEM,0,0x00);
		//copy packet data to tx buffer
	 	ENC28J60_Write_Buf(len,packet);
	 	//send data to ethernet
		ENC28J60_Write_Op(ENC28J60_BIT_FIELD_SET,ECON1,ECON1_TXRTS);
		if((ENC28J60_Read(EIR)&EIR_TXERIF))ENC28J60_Write_Op(ENC28J60_BIT_FIELD_CLR,ECON1,ECON1_TXRTS);
	}
	
	//Get a packet from ehternet
	//maxlen: maximum data length to receive
	//packet: buffer pointer to store data
	//return: length of rx data pakcet 								  
	u32 ENC28J60_Packet_Receive(u32 maxlen,u8* packet)
	{
		u32 rxstat;
		u32 len;    													 
		if(ENC28J60_Read(EPKTCNT)==0)return 0;  //if received a packet	   
		//setup rx buffer read out pointer
		ENC28J60_Write(ERDPTL,(NextPacketPtr));
		ENC28J60_Write(ERDPTH,(NextPacketPtr)>>8);	   
		// 读下一个包的指针
		NextPacketPtr=ENC28J60_Read_Op(ENC28J60_READ_BUF_MEM,0);
		NextPacketPtr|=ENC28J60_Read_Op(ENC28J60_READ_BUF_MEM,0)<<8;
		//read next package
		len=ENC28J60_Read_Op(ENC28J60_READ_BUF_MEM,0);
		len|=ENC28J60_Read_Op(ENC28J60_READ_BUF_MEM,0)<<8;
	 	len-=4; //remove CRC counting
		//rea rx status
		rxstat=ENC28J60_Read_Op(ENC28J60_READ_BUF_MEM,0);
		rxstat|=ENC28J60_Read_Op(ENC28J60_READ_BUF_MEM,0)<<8;
		//limit rx length	
		if (len>maxlen-1)len=maxlen-1;	
		//CRC check
		if((rxstat&0x80)==0)len=0;//invalid
		else ENC28J60_Read_Buf(len,packet);//copy data form rx packet    
		//Rx read out pointer move to next start address of packet
		//releas buffer
		ENC28J60_Write(ERXRDPTL,(NextPacketPtr));
		ENC28J60_Write(ERXRDPTH,(NextPacketPtr)>>8);
		//set the packet receive flag
	 	ENC28J60_Write_Op(ENC28J60_BIT_FIELD_SET,ECON2,ECON2_PKTDEC);
		return(len);
	}



