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
extern UINT8I USBByteCount;       //����USB�˵���յ�������
extern UINT8I USBBufOutPoint;     //ȡ����ָ��
extern UINT8I UartByteCount;      //��ǰ������ʣ���ȡ�ֽ���
extern UINT8I Uart_Input_Point;   //ѭ��������д��ָ�룬���߸�λ��Ҫ��ʼ��Ϊ0
extern UINT8I Uart_Output_Point;  //ѭ��������ȡ��ָ�룬���߸�λ��Ҫ��ʼ��Ϊ0
extern UINT8X  Ep2Buffer[2*64]; 
extern UINT8X  Ep3Buffer[64];  
extern UINT8X  Ep3Buffer2[64]; 
extern UINT8I Endp2Busy;
extern UINT8  USB_RequestFlag;

extern	void 	USBDeviceInit();
extern	void 	HIDValueHandle();
	
#endif
/**************************** END *************************************/
