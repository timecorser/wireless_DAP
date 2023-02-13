/********************************** (C) COPYRIGHT *******************************
* File Name          :Compound_Dev.C											*	
* Author             : WCH                                                      *
* Version            : V1.2                                                     *
* Date               : 2017/02/24                                               *
* Description        : A demo for USB compound device created by CH554, support *
					   keyboard , and HID-compliant device.                     *
********************************************************************************/


#ifndef	__COMPOUND_H__
#define __COMPOUND_H__

extern UINT8I 	UsbConfig;	
extern UINT16  TouchKeyButton;
extern UINT8I Endp1Busy;
extern UINT8I USBD0;
extern UINT8I USBByteCount;       //代表USB端点接收到的数据
extern UINT8I USBBufOutPoint;     //取数据指针
extern UINT8I UartByteCount;      //当前缓冲区剩余待取字节数
extern UINT8I Uart_Input_Point;   //循环缓冲区写入指针，总线复位需要初始化为0
extern UINT8I Uart_Output_Point;  //循环缓冲区取出指针，总线复位需要初始化为0
extern UINT8X  Ep2Buffer[2*64]; 
extern UINT8X  Ep3Buffer[64];  
extern UINT8X  Ep3Buffer2[64]; 
extern UINT8I Endp2Busy;
extern UINT8  USB_RequestFlag;

extern	void 	USBDeviceInit();
extern	void 	HIDValueHandle();
	
#endif
/**************************** END *************************************/
