/********************************** (C) COPYRIGHT ******************************
* File Name          :Compound_Dev.C											
* Author             : WCH                                                      
* Version            : V1.0                                                     
* Date               : 2017/03/15                                               
* Description        : A demo for USB compound device created by CH554, support 
					   keyboard and mouse, and HID compatible device.           
********************************************************************************/
#include 	".\Public\CH554.H"
#include	"Compound.h"
#include 	".\Public\debug.h"
#include 	"stdio.h"
#include <stdio.h>
#include <string.h>
#include ".\Public\GPIO.H"
sbit check = P1^4;
UINT8 flag1_1 = 1;


extern UINT8 	FLAG;												  // Trans complete flag
extern UINT8 	EnumOK;												// Enum ok flag	

UINT8 Res;	
UINT8 D1_len=0,D2_len=0;
UINT8 Flag_3=0,Flag_4=0;
UINT8X USART_RX_BUF[66];     //?車那??o3?,℅?∩車USART_REC_LEN??℅??迆.
UINT8X USART3_RX_BUF[140];     //?車那??o3?,℅?∩車USART_REC_LEN??℅???.
//?車那?℅∩足?
//bit15㏒?	?車那?赤那3谷㊣那??
//bit14㏒?	?車那?米?0x0d
//bit13~0㏒?	?車那?米?米?車DD∫℅??迆那y??
UINT16 USART_RX_STA=0;       //?車那?℅∩足?㊣那??	  
UINT16 USART3_RX_STA=0;       //?車那?℅∩足?㊣那??	
UINT16 num_1 =0,num_2=0;  				

void DAP_Thread(void)
{
    UINT8I num;

    if (USB_RequestFlag)
    {
		CH554UART0SendByte(0x66);
		num_2 = 0;
		num_1= 63;
		while(1)
		{
			num_2 += Ep3Buffer[num_1];
			if(num_2)break;
			num_1--;
		}
		CH554UART0SendByte(num_1+1);
		for(num=0;num<num_1+1;num++)
		{	
			CH554UART0SendByte(Ep3Buffer[num]);
		}
		while((USART_RX_STA&0x8000)==0);		
		USART_RX_STA = 0;	
		for(num=0;num<64;num++)
		{
			Ep3Buffer2[num] = USART_RX_BUF[num];//???1????
		}
		UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK; //enable receive
	    UEP3_T_LEN = 64;
	    UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK; //enable send
		USB_RequestFlag = 0;
    }
}

void main( void )
{
	UINT8I num=0,lenth=0;
	CfgFsys();//奀笘恁寁饜离
	mDelaymS(5);//党蜊翋け脹渾囀窒儒淥恛隅,斛樓
	Port1Cfg(3,4);                                                             //P16扢离芢侺耀宒
	
	USBDeviceInit();//場宎趙USB扢掘 Configure the USB module,the Endpoint and the USB Interrupt
	//UART1Setup();
  	EA = 1;	
	UEP1_T_LEN = 0;//啎妏蚚楷冞酗僅珨隅猁ь諾
  	UEP2_T_LEN = 0;//啎妏蚚楷冞酗僅珨隅猁ь諾
	UEP3_T_LEN = 0;//啎妏蚚楷冞酗僅珨隅猁ь諾
    Endp2Busy = 0;
	while(flag1_1)
	{
		if(check == 0)
		{
			flag1_1=0;
		}
		mDelaymS(100);  
	}
	for(num=0;num<64;num++)
	{
		USART_RX_BUF[num]= 0;//???1????
	}
	mInitSTDIO();//揹諳0,褫眕蚚衾覃彸
	TI = 0;
//    while (!UsbConfig) {;};
	while(1)
	{
//		if(USART_RX_STA&0x8000)
//		{		
//			USART_RX_STA = 0;	
//			for(num=0;num<64;num++)
//			{
//				CH554UART0SendByte(USART_RX_BUF[num]);
//			}
//		}
//		if(UsbConfig)//奻弇儂湖羲揹諳
//		{
//			if(USBByteCount)   //USB諉彶傷萸衄杅擂
//			{
//				if(!Endp2Busy)   //傷萸祥楛疆ㄗ諾玿綴腔菴珨婦杅擂ㄛ硐蚚釬揖楷奻換ㄘ
//				{
//					lenth = USBByteCount;		
//					//迡奻換傷萸
//					memcpy(Ep2Buffer+MAX_PACKET_SIZE,Ep2Buffer,lenth);
//					UEP2_T_LEN = lenth;                                                    //啎妏蚚楷冞酗僅珨隅猁ь諾
//					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //茼湘ACK
//					Endp2Busy = 1;
//					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;//翋滲杅揭燴俇ㄛ蚕翋滲杅党蜊砒茼源宒
//					USBByteCount= 0;				
//				}				
//			}
//		}
		DAP_Thread();
       	if (USBByteCount)
        {
			lenth = USBByteCount;
			CH554UART0SendByte(0x67);
			CH554UART0SendByte(lenth);
			for(num=0;num<lenth;num++)
			{
				CH554UART0SendByte(Ep2Buffer[num]);
			}
//            CH554UART0SendByte(Ep2Buffer[USBBufOutPoint++]);
            USBByteCount=0;
            if (USBByteCount == 0)
                UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
        }
		if(USBD0)
		{
//			Ep2Buffer[64] =   USBD0;
//			UEP2_T_LEN = 1;                                                    //啎妏蚚楷冞酗僅珨隅猁ь諾
//			UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //茼湘ACK
//			Endp2Busy = 1;
			
			CH554UART0SendByte(0x68);
			CH554UART0SendByte(1);
			CH554UART0SendByte(USBD0);
			USBD0 = 0;
		}
		if(USART3_RX_STA&0x8000)
		{
			if (!Endp2Busy)
			{
				lenth = USART3_RX_STA&0X0FFF;
				if(lenth<64)
				{
					memcpy(Ep2Buffer+64,&USART3_RX_BUF[0],lenth);
					UEP2_T_LEN = lenth;                                                    //啎妏蚚楷冞酗僅珨隅猁ь諾
					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //茼湘ACK
					Endp2Busy = 1;
					USART3_RX_STA = 0;
				}else if(lenth==64)
				{
					memcpy(Ep2Buffer+64,&USART3_RX_BUF[0],lenth);
					UEP2_T_LEN = lenth;                                                    //啎妏蚚楷冞酗僅珨隅猁ь諾
					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //茼湘ACK
					Endp2Busy = 1;
					while(Endp2Busy);	
					UEP2_T_LEN = 0;                                                    //啎妏蚚楷冞酗僅珨隅猁ь諾
					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //茼湘ACK
					Endp2Busy = 1;
					USART3_RX_STA = 0;
				}else if(lenth>64)
				{
					for(num=0;num<(lenth/64);num++)
					{
						memcpy(Ep2Buffer+64,&USART3_RX_BUF[num*64],64);
						UEP2_T_LEN = 64;                                                    //啎妏蚚楷冞酗僅珨隅猁ь諾
						UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //茼湘ACK
						Endp2Busy = 1;
						while(Endp2Busy);	
						UEP2_T_LEN = 0;                                                    //啎妏蚚楷冞酗僅珨隅猁ь諾
						lenth-=64;
					}
					memcpy(Ep2Buffer+64,&USART3_RX_BUF[num*64],lenth);
					UEP2_T_LEN = lenth;                                                    //啎妏蚚楷冞酗僅珨隅猁ь諾
					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //茼湘ACK
					Endp2Busy = 1;
					USART3_RX_STA = 0;
				}
			}
		}		
			
//		if(UartByteCount)
//				Uart_Timeout++;
//		if(!Endp2Busy)   //傷萸祥楛疆ㄗ諾玿綴腔菴珨婦杅擂ㄛ硐蚚釬揖楷奻換ㄘ
//		{
//			lenth = UartByteCount;
//			if(lenth>0)
//			{
//				if(lenth>39 || Uart_Timeout>100)
//				{		
//					Uart_Timeout = 0;
//					UartByteCount -= lenth;			
//					//迡奻換傷萸
//					Uart_Output_Point+=lenth;
//					if(Uart_Output_Point>=64)
//						Uart_Output_Point = 0;						
//					UEP2_T_LEN = lenth;                                                    //啎妏蚚楷冞酗僅珨隅猁ь諾
//					UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //茼湘ACK
//					Endp2Busy = 1;
//				}
//			}
//		}
//        if (USBByteCount)
//        { 
//			if (!Endp2Busy)
//			{
//				memcpy(Ep2Buffer+64,&Ep3Buffer[0],64);
//				Uart_Output_Point+=64;
//				UEP2_T_LEN = 64;                                                    //啎妏蚚楷冞酗僅珨隅猁ь諾
//				UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;            //茼湘ACK
//				Endp2Busy = 1;
//			}
//        }		
    }
}

void Uart0_ISR(void) interrupt INT_NO_UART0
{
	if(RI)  //?車那??D??(?車那?米?米?那y?Y㊣?D?那?0x0d 0x0a?芍?2)
	{
		Res = SBUF;	//?芍豕??車那?米?米?那y?Y
		if(((USART_RX_STA&0x8000)||(USART3_RX_STA&0x8000))==0)
		{
			if(((USART_RX_STA&0x4000)||(USART3_RX_STA&0x4000))==0)
			{
				if(Res==0x66)
				{
					USART_RX_STA|=0x4000;//?車那?米???赤﹞
					Flag_3 = 1;
				}else if(Res==0x67)
				{
					USART3_RX_STA|=0x4000;//?車那?米???赤﹞
					Flag_4 = 1;
				}else
				{
					USART_RX_STA = 0;
					USART3_RX_STA = 0;
				}
			}else if((((USART_RX_STA&0x4000))&&(~(USART3_RX_STA&0x4000))))
			{
				if(Flag_3)
				{
					D1_len = Res;
					Flag_3 = 0;
				}else
				{
					USART_RX_BUF[USART_RX_STA&0X0FFF]=Res ;
					USART_RX_STA++;
					if((USART_RX_STA&0x0FFF)==D1_len)USART_RX_STA|=0x8000;	//?車那?赤那3谷芍? 
				}
			}else if((((~USART_RX_STA&0x4000))&&((USART3_RX_STA&0x4000))))
			{
				if(Flag_4)
				{
					D2_len = Res;
					Flag_4 = 0;
				}else
				{
					USART3_RX_BUF[USART3_RX_STA&0X0FFF]=Res ;
					USART3_RX_STA++;
					if((USART3_RX_STA&0x0FFF)==D2_len)USART3_RX_STA|=0x8000;	//?車那?赤那3谷芍? 
				}
			}
		}else if((((USART_RX_STA&0x8000))&&(~(USART3_RX_STA&0x8000))))
		{
			if((USART3_RX_STA&0x4000)==0)
			{
				if(Res==0x67)
				{
					USART3_RX_STA|=0x4000;//?車那?米???赤﹞
					Flag_4 = 1;
				}else
				{
					USART3_RX_STA = 0;
				}
			}else
			{
				if(Flag_4)
				{
					D2_len = Res;
					Flag_4 = 0;
				}else
				{
					USART3_RX_BUF[USART3_RX_STA&0X0FFF]=Res ;
					USART3_RX_STA++;
					if((USART3_RX_STA&0x0FFF)==D2_len)USART3_RX_STA|=0x8000;	//?車那?赤那3谷芍? 
				}
			}
		}else if(((~(USART_RX_STA&0x8000))&&((USART3_RX_STA&0x8000))))
		{
			if((USART_RX_STA&0x4000)==0)
			{
				if(Res==0x66)
				{
					USART_RX_STA|=0x4000;//?車那?米???赤﹞
					Flag_3 = 1;
				}else
				{
					USART_RX_STA = 0;
				}
			}else
			{
				if(Flag_3)
				{
					D1_len = Res;
					Flag_3 = 0;
				}else
				{
					USART_RX_BUF[USART_RX_STA&0X0FFF]=Res ;
					USART_RX_STA++;
					if((USART_RX_STA&0x0FFF)==D1_len)USART_RX_STA|=0x8000;	//?車那?赤那3谷芍? 
				}
			}
		}
		RI = 0;
	} 
}

/**************************** END *************************************/
