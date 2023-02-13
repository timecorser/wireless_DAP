/********************************** (C) COPYRIGHT ******************************
* File Name          :Compound_Dev.C												
* Author             : WCH                                                      
* Version            : V1.2                                                     
* Date               : 2017/02/24                                               
* Description        : A demo for USB compound device created by CH554, support 
					   keyboard , and HID-compliant device.                     
********************************************************************************/

#include 	".\Public\CH554.H"
#include 	".\Public\DEBUG.H"
#include 	"compound.h"
#include 	<stdio.h>
#include 	<stdlib.h>
#include 	<string.h>


#define 	THIS_ENDP0_SIZE         DEFAULT_ENDP0_SIZE
#define		BUFFER_SIZE				64
#define 	DUAL_BUFFER_SIZE		128
#define 	UsbSetupBuf     		((PUSB_SETUP_REQ)Ep0Buffer)
#define		L_WIN 					0X08
#define 	L_ALT 					0X04
#define		L_SHIFT					0X02
#define 	L_CTL					0X01
#define 	R_WIN 					0X80
#define 	R_ALT 					0X40
#define 	R_SHIFT					0X20
#define 	R_CTL					0X10
#define 	SPACE					0X2C
#define		ENTER					0X28

#define MOUSE 0

#pragma  NOAREGS

//UINT8X  	Ep0Buffer[THIS_ENDP0_SIZE]  _at_ 0x0000;  								// Endpoint 0, buffer OUT/OUT，the address must be even.
//UINT8X  	Ep1Buffer[BUFFER_SIZE] 		_at_ 0x000A;  								// Endpoint 1, buffer IN，the address must be even.
//UINT8X 		Ep2Buffer[DUAL_BUFFER_SIZE]	_at_ 0x0050;  								// Endpoint 2, buffer OUT[64]+IN[64]，the address must be even.

UINT8X  Ep0Buffer[64] _at_ 0x0000;                                 //端点0 发送和接收公用缓冲区，必须是偶地址
UINT8X  Ep1Buffer[64] _at_ 0x0040;                                                  //端点1上传缓冲区
UINT8X  Ep2Buffer[2*64] _at_ 0x0080;                                  //端点2 接收和发送双缓冲区,必须是偶地址
UINT8X  Ep3Buffer[64*1] _at_ 0x0100; //端点2 OUT双缓冲区,必须是偶地址
UINT8X  Ep3Buffer2[64*1] _at_ 0x0140; //端点2 OUT双缓冲区,必须是偶地址



/**************************** Global variable ********************************/	
PUINT8  	pDescr;                                                                	// USB enumerate complete flag.
USB_SETUP_REQ   					SetupReqBuf;                                   	// A buffer for Setup package.

UINT8  USB_RequestFlag= 0;
PUINT8 pDescr; //USB配置标志
UINT8I Endp3Busy = 0;
UINT8I SetupReq, SetupLen, Ready, Count, UsbConfig;

UINT8I Endp2Busy = 0;
UINT8I Endp1Busy;
UINT8I USBD0 = 0;       //代表USB端点接收到的数据
UINT8I USBByteCount = 0;       //代表USB端点接收到的数据
UINT8I USBBufOutPoint = 0;     //取数据指针
UINT8I LineCoding[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //初始化波特率为57600，1停止位，无校验，8数据位。
UINT16I USB_STATUS = 0;
UINT8C MyLangDescr[] = {0x04, 0x03, 0x09, 0x04};
unsigned char  code String_1[]={0x12, 0x03,'y',0x00,'u',0x00,'l',0x00,'e',0x00,'i',0x00,'t',0x00,'a',0x00,'o',0x00};           //语言描述符
unsigned char  code String_2[]={0x14, 0x03,'C',0x00,'M',0x00,'S',0x00,'I',0x00,'S',0x00,'-',0x00,'D',0x00,'A',0x00,'P',0x00};     
unsigned char  code String_3[]={0x1a, 0x03,'4',0x00,'8',0x00,'E',0x00,'A',0x00,'8',0x00,'0',0x00,'6',0x00,'E',0x00,'3',0x00,'1',0x00,'3',0x00,'9',0x00};
unsigned char  code String_4[]={0x1c, 0x03,'C',0x00,'M',0x00,'S',0x00,'I',0x00,'S',0x00,'-',0x00,'D',0x00,'A',0x00,'P',0x00,' ',0x00,'C',0x00,'D',0x00,'C',0x00};
unsigned char  code String_5[]={0x1c, 0x03,'C',0x00,'M',0x00,'S',0x00,'I',0x00,'S',0x00,'-',0x00,'D',0x00,'A',0x00,'P',0x00,' ',0x00,'D',0x00,'C',0x00,'I',0x00};
unsigned char  code String_6[]={0x14, 0x03,'C',0x00,'M',0x00,'S',0x00,'I',0x00,'S',0x00,'-',0x00,'D',0x00,'A',0x00,'P',0x00};


/**************************** Device Descriptor设备描述符 *************************************/
UINT8C DevDesc[18] = {																// Device Descriptor
0x12,//描述符的字节长度
0x01,//描述符类型：设备描述符
0x00,0x02,//USB2.0
0xEF,//设备类代码：复合设备
0x02,//设备子类代码
0x01,//设备协议代码
0x40,//端点0最大包长64字节                      
0x51,0xC2,// Vendor ID   |  VID =  0X5131///413c
0x01,0xF0,// Product ID  |  PID = 0X2007 /// 2105
0x00,0x01,// bcdDevice		
0x01,//制造商的字符串描述符索引
0x02,//产品的字符串描述符索引
0x03,//设备序列号的字符串描述符索引
0x01//当前速度下能支持的配置数量
};
/**************************** HID Report Descriptor *********************************/

UINT8C USBD_HID_ReportDescriptor[33] = 															// Report Descriptor, Composite device
{
	0x06, 0x00, 0xff, 	// Usage page Vendor defined
	0x09, 0x01, 		// Usage keyboard
	0xa1, 0x01, 		// Collation Application
	0x15, 0x00, 		// Logical min ( 0H )
	0x26, 0xff, 0x00,	// Logical max ( FFH )
	0x75, 0x08,  		// Report size ( 08H )
	0x95, 0x40, 		// Report count ( 40H )
	0x09, 0x01, 		// Mouse
	0x81, 0x02,  		// Input ( Data, Relative, Wrap )
	0x95, 0x40,  		// Logical min ( 0H )
	0x09, 0x01,	// Logical max ( FFH )
	0x91, 0x02, 		// Report size ( 08H )
	0x95, 0x01, 		// Report count ( 40H )
	0x09, 0x01, 		// Output ( Data, Relative, Wrap )
	0xB1, 0x02,
	0xC0
};


UINT8C CfgDesc[] =
{
//配置描述符（两个接口）
0x09,//字节数长度
0x02,//描述符类型：配置描述符
0x6B,0x00,//此配置信息总长
0x03,//此配置所支持的接口个数
0x01,//在SetConfiguration请求中用作参数来选定此配置
0x00,//描述此配置的字串描述符索引值，在SetConfiguration请求中用作选定配置的参数
0x80,//配置特性，配置为无
0x32,//配置所需电流，单位2ma，配置为100ma               					
//接口关联描述符
0x08,//字节数长度
0x0B,//描述符类型：接口关联描述符
0x00,//第一个接口为0
0x02,//总共两个接口
0x02,//CDC
0x02,//虚拟串口
0x01,//Common AT commands
0x04,//USBD_CDC_ACM_CIF_STR_NUM             			
//CDC接口描述符
0x09,//字节数长度
0x04,//描述符类型：接口描述符
0x00,//Number of Interface
0x00,//Alternate setting
0x01,//One endpoint used
0x02,//CDC_COMMUNICATION_INTERFACE_CLASS
0x02,//CDC_ABSTRACT_CONTROL_MODEL
0x00,//no protocol used
0x04,//USBD_CDC_ACM_CIF_STR_NUM
//以下为功能描述符
//功能描述符(头)
0x05,//字节数长度
0x24,//描述符类型：CS_INTERFACE 
0x00,//Header Func Desc
0x10,0x01,//CDC版本号，为0x0110（低字节在先） 
//管理描述符(没有数据类接口)
0x05,//字节数长度
0x24,//描述符类型：CS_INTERFACE 
0x01,//Call Management Func Desc
0x01,//device handles call management
0x01,//CDC data IF ID
//CDC控制描述
0x04,//字节数长度
0x24,//描述符类型：CS_INTERFACE 
0x02,//Abstract Control Management desc
0x02,//支持Set_Line_Coding、Set_Control_Line_State、Get_Line_Coding请求和Serial_State通知
//CDC功能描述
0x05,//字节数长度
0x24,//描述符类型：CS_INTERFACE 
0x06,//Union func desc
0x00,//这里为前面编号为0的CDC接口
0x01,//这里为接下来编号为1的数据类接口
//中断上传端点描述符
0x07,//字节数长度
0x05,//描述符类型：端点描述符
0x81,//端点的地址
0x03,//端点的属性，中断传输
0x40,0x00,//端点支持的最大包长度64字节
0x02,//端口查询的帧间隔数 
//以下为接口1（数据接口）描述符
//数据接口描述符
0x09,//字节数长度
0x04,//描述符类型：接口描述符
0x01,//接口号
0x00,//可设置的索引值
0x02,//此接口用的端点数量
0x0A,//接口所属的类值，配置为CDC数据类
0x00,//子类码
0x00,//协议码
0x05,//描述此接口的字串描述表的索引值 
//CDC输出端点
0x07,//字节数长度
0x05,//描述符类型：端点描述符
0x02,//端点的OUT地址
0x02,//端点的属性，批量传输
0x40,0x00,//端点支持的最大包长度64字节
0x00,//端口的查询时间
//CDC输入端点 
0x07,//字节数长度
0x05,//描述符类型：端点描述符
0x82,//端点的IN地址
0x02,//端点的属性，批量传输
0x40,0x00,//端点支持的最大包长度64字节
0x00,//端口的查询时间
//以下为接口2（数据接口）描述符
//数据接口描述符
0x09,//字节数长度
0x04,//描述符类型：接口描述符
0x02,//接口号
0x00,//可设置的索引值
0x02,//此接口用的端点数量
0x03,//接口所属的类值，配置为HID类
0x00,//子类码
0x00,//协议码
0x06,//描述此接口的字串描述表的索引值 HID Descriptor
//HID类描述符 
0x09,//字节数长度
0x21,//描述符类型：HID描述符
0x00,0x01,//HID类规范发布号V1.0
0x00,//硬件目标国家
0x01,//下面HID报告描述符的数量
0x22,//报告描述符类型
0x21,0x00,//HID报告描述符长度
//HID中断输入端点 
0x07,//字节数长度
0x05,//描述符类型：端点描述符
0x83,//端点的IN地址
0x03,//端点的属性，中断传输
0x40,0x00,//端点支持的最大包长度64字节
0x01,//端口查询的帧间隔数
//HID中断输出端点 
0x07,//字节数长度
0x05,//描述符类型：端点描述符
0x03,//端点的OUT地址
0x03,//端点的属性，中断传输
0x40,0x00,//端点支持的最大包长度64字节
0x01//端口查询的帧间隔数
};


void Config_Uart1(UINT8 *cfg_uart)
{
    UINT32 uart1_buad = 0;
	UINT8I num=0,lenth=0;
    *((UINT8 *)&uart1_buad) = cfg_uart[3];
    *((UINT8 *)&uart1_buad + 1) = cfg_uart[2];
    *((UINT8 *)&uart1_buad + 2) = cfg_uart[1];
    *((UINT8 *)&uart1_buad + 3) = cfg_uart[0];
	CH554UART0SendByte(0x68);
	CH554UART0SendByte(7);
	CH554UART0SendByte(uart1_buad/1000000+'0');
	CH554UART0SendByte(uart1_buad%1000000/100000+'0');
	CH554UART0SendByte(uart1_buad%100000/10000+'0');
	CH554UART0SendByte(uart1_buad%10000/1000+'0');
	CH554UART0SendByte(uart1_buad%1000/100+'0');
	CH554UART0SendByte(uart1_buad%100/10+'0');
	CH554UART0SendByte(uart1_buad%10+'0');
}

/*******************************************************************************
* Function Name  : USBDeviceInit()
* Description    : Configure USB mode ，USB device init configure.Configure tie Endpoint, compound device, 
				           Endpoint 0 control trans, Endpoint 1/2 interrupt(IN).
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBDeviceInit()
{
  IE_USB = 0;
  USB_CTRL = 0x00;        // 先设定USB设备模式
  UDEV_CTRL = bUD_PD_DIS; // 禁止DP/DM下拉电阻

  UDEV_CTRL &= ~bUD_LOW_SPEED; //选择全速12M模式，默认方式
  USB_CTRL &= ~bUC_LOW_SPEED;


	UEP0_DMA = Ep0Buffer;                                                      //端点0数据传输地址                                                //端点1上传缓冲区；端点0单64字节收发缓冲区
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                 //端点0手动翻转，OUT事务返回ACK，IN事务返回NAK

	UEP1_DMA = Ep1Buffer;  
	UEP4_1_MOD &= ~(bUEP4_RX_EN | bUEP4_TX_EN);                //端点0单64字节收发缓冲区                                                            //端点1 发送数据传输地址
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                 //端点1自动翻转同步标志位，IN事务返回NAK	
	
	UEP2_DMA = Ep2Buffer; 
	UEP3_DMA = Ep3Buffer;                                                     //端点2 IN数据传输地址	
	//使能端点2发送（IN），使能端点2接收（OUT），双64字节，接收（OUT）在前
	UEP2_3_MOD = UEP2_3_MOD | bUEP2_TX_EN | bUEP2_RX_EN | bUEP3_TX_EN | bUEP3_RX_EN & ~bUEP2_BUF_MOD;      // Endpoint 2 sigle 64 byte send buffer OUT[64]+IN[64] (OUT first)
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK; //端点2自动翻转同步标志位，IN事务返回NAK，OUT返回ACK
    UEP3_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_NAK;//端点3自动翻转同步标志位，IN事务返回NAK，OUT返回NACK


  USB_DEV_AD = 0x00;
  USB_CTRL |= bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN; // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
  UDEV_CTRL |= bUD_PORT_EN;                              // 允许USB端口
  USB_INT_FG = 0xFF;                                     // 清中断标志
  USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
  IE_USB = 1;
}


void DeviceInterrupt(void) interrupt INT_NO_USB using 1 //USB中断服务程序,使用寄存器组1
{
    UINT8 len,temp,num_s;
    if (UIF_TRANSFER) //USB传输完成标志
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
		case UIS_TOKEN_IN | 1: //endpoint 1# 端点批量上传
            UEP1_T_LEN = 0;      //预使用发送长度一定要清空
            Endp1Busy = 0;
            UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; //默认应答NAK
            break;

        case UIS_TOKEN_OUT | 2: //endpoint 2# 端点批量下传
            if (U_TOG_OK)         // 不同步的数据包将丢弃
            {
                USBByteCount = USB_RX_LEN;
                USBBufOutPoint = 0;                                             //取数据指针复位
                UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;       //收到一包数据就NAK，主函数处理完，由主函数修改响应方式
            }
            break;

		case UIS_TOKEN_IN | 2: //endpoint 2# 端点批量上传
            UEP2_T_LEN = 0;      //预使用发送长度一定要清空
            UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; //默认应答NAK
			Endp2Busy = 0;
            break;

		case UIS_TOKEN_OUT | 3: //endpoint 2# 端点批量下传
            if (U_TOG_OK)         // 不同步的数据包将丢弃
            {
				//UEP3_T_LEN = 0;
				if(USB_RX_LEN)
				{
					USB_RequestFlag = 1;;
//					UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;
				}
            }
            break;

        case UIS_TOKEN_IN | 3: //endpoint 3# 端点批量上传
            Endp3Busy = 0;
            UEP3_T_LEN = 0;      //预使用发送长度一定要清空
            UEP3_CTRL = UEP3_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; //默认应答NAK
            break;

        
//        case UIS_TOKEN_OUT | 1: //endpoint 1# 端点批量下传
//            if (U_TOG_OK)         // 不同步的数据包将丢弃
//            {
//                USBByteCount = USB_RX_LEN;
//                USBBufOutPoint = 0;                                             //取数据指针复位
//                UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;       //收到一包数据就NAK，主函数处理完，由主函数修改响应方式
//            }
//            break;

        case UIS_TOKEN_SETUP | 0: //SETUP事务
            len = USB_RX_LEN;
            if (len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = UsbSetupBuf->wLengthL;
                if(UsbSetupBuf->wLengthH || SetupLen > 0x7F )
                {
                    SetupLen = 0x7F;    // 限制总长度
                }
                len = 0;           // 默认为成功并且上传0长度
                SetupReq = UsbSetupBuf->bRequest;
                switch (UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK)
                {
                case USB_REQ_TYP_STANDARD:
                    switch (SetupReq) //请求码
                    {
                    case USB_GET_DESCRIPTOR:
                        switch (UsbSetupBuf->wValueH)
                        {
                        case 1:             //设备描述符
                            pDescr = DevDesc; //把设备描述符送到要发送的缓冲区
                            len = sizeof(DevDesc);
                            break;
                        case 2:             //配置描述符
                            pDescr = CfgDesc; //把设备描述符送到要发送的缓冲区
                            len = sizeof(CfgDesc);
                            break;
                        case 3: // 字符串描述符
                            switch (UsbSetupBuf->wValueL)
                            {
                            case 0:
                                pDescr = (PUINT8)(&MyLangDescr[0]);
                                len = sizeof(MyLangDescr);
                                break;
                            case 1:
                                pDescr = (PUINT8)(&String_1[0]);
                                len = sizeof(String_1);
                                break;
                            case 2:
                                pDescr = (PUINT8)(&String_2[0]);
                                len = sizeof(String_2);
                                break;
                            case 3:
                                pDescr = (PUINT8)(&String_3[0]);
                                len = sizeof(String_3);
                                break;
                            case 4:
                                pDescr = (PUINT8)(&String_4[0]);
                                len = sizeof(String_4);
                                break;
                            case 5:
                                pDescr = (PUINT8)(&String_5[0]);
                                len = sizeof(String_5);
                                break;
							case 6:
                                pDescr = (PUINT8)(&String_6[0]);
                                len = sizeof(String_6);
                                break;
                            default:
                                len = 0xFF; // 不支持的字符串描述符
                                break;
                            }
                            break;
                        case 0x22:                                                    	// HID report descriptor											
							pDescr = USBD_HID_ReportDescriptor;                                  	// Write to buffer
							len = sizeof( USBD_HID_ReportDescriptor );	
							Ready = 1;
							break;
                        default:
                            len = 0xff; //不支持的命令或者出错
                            break;
                        }
                        break;
                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL; //暂存USB设备地址
                        break;
                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if (SetupLen >= 1)
                        {
                            len = 1;
                        }
                        break;
                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
//                        if (UsbConfig)
//                        {
//                            Ready = 1; //set config命令一般代表usb枚举完成的标志
//                        }
                        break;
                    case 0x0A:
                        break;
                    case USB_CLEAR_FEATURE:                                                       //Clear Feature
                        if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) // 端点
                        {
                            switch (UsbSetupBuf->wIndexL)
                            {
							case 0x81:
                                UEP1_CTRL = UEP1_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            case 0x82:
                                UEP2_CTRL = UEP2_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            case 0x02:
                                UEP2_CTRL = UEP2_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                                break;
							case 0x83:
                                UEP3_CTRL = UEP3_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
                                break;
                            case 0x03:
                                UEP3_CTRL = UEP3_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
                                break;
                            default:
                                len = 0xFF; // 不支持的端点
                                break;
                            }
                        }
                        else
                        {
                            len = 0xFF; // 不是端点不支持
                        }
                        break;
                    case USB_SET_FEATURE:                             /* Set Feature */
                        if ((UsbSetupBuf->bRequestType & 0x1F) == 0x00) /* 设置设备 */
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x01)
                            {
                                if (CfgDesc[7] & 0x20)
                                {
                                    /* 设置唤醒使能标志 */
                                }
                                else
                                {
                                    len = 0xFF; /* 操作失败 */
                                }
                            }
                            else
                            {
                                len = 0xFF; /* 操作失败 */
                            }
                        }
                        else if ((UsbSetupBuf->bRequestType & 0x1F) == 0x02) /* 设置端点 */
                        {
                            if ((((UINT16)UsbSetupBuf->wValueH << 8) | UsbSetupBuf->wValueL) == 0x00)
                            {
                                switch (((UINT16)UsbSetupBuf->wIndexH << 8) | UsbSetupBuf->wIndexL)
                                {
								case 0x81:
                                    UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点1 IN STALL */
                                    break;
                                case 0x82:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点2 IN STALL */
                                    break;
                                case 0x02:
                                    UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点2 OUT Stall */
                                    break;
                                case 0x83:
                                    UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点2 IN STALL */
                                    break;
                                case 0x03:
                                    UEP3_CTRL = UEP3_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点2 OUT Stall */
                                    break;
                                default:
                                    len = 0xFF; /* 操作失败 */
                                    break;
                                }
                            }
                            else
                            {
                                len = 0xFF; /* 操作失败 */
                            }
                        }
                        else
                        {
                            len = 0xFF; /* 操作失败 */
                        }
                        break;
                    case USB_GET_STATUS:
                        //pDescr = (PUINT8)&USB_STATUS;
						Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        if (SetupLen >= 2)
                        {
                            len = 2;
                        }
                        else
                        {
                            len = SetupLen;
                        }
                        break;
                    default:
                        len = 0xff; //操作失败
                        break;
                    }

                    break;
                case USB_REQ_TYP_CLASS: /*HID类命令*/
                    if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_INTERF)
                    {
                        switch (SetupReq)
                        {
                        case 0x20://Configure
                            break;
                        case 0x21://currently configured
                            pDescr = LineCoding;
                            len = sizeof(LineCoding);
                            break;
                        case 0x22://generates RS-232/V.24 style control signals
							USBD0 = Ep0Buffer[2]+1;
                            break;
                        default:
                            len = 0xFF; /*命令不支持*/
                            break;
                        }
                    }
                    break;
                case USB_REQ_TYP_VENDOR:
                    if ((UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE)
                    {
                        switch (SetupReq)
                        {
                        case 0x20:                         //GetReport
                            if (UsbSetupBuf->wIndexL == 0x07)
                            {

                            }
                            break;
                        default:
                            len = 0xFF; /*命令不支持*/
                            break;
                        }
                    }
                    break;
                default:
                    len = 0xFF;
                    break;
                }
                if (len != 0 && len != 0xFF)
                {
                    if (SetupLen > len)
                    {
                        SetupLen = len; //限制总长度
                    }
                    len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //本次传输长度
                    memcpy(Ep0Buffer, pDescr, len);                                 //加载上传数据
                    SetupLen -= len;
                    pDescr += len;
                }
            }
            else
            {
                len = 0xff; //包长度错误
            }
            if (len == 0xff)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; //STALL
            }
            else if (len <= THIS_ENDP0_SIZE) //上传数据或者状态阶段返回0长度包
            {
                UEP0_T_LEN = len;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; //默认数据包是DATA1，返回应答ACK
            }
            else
            {
                UEP0_T_LEN = 0;                                                      //虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; //默认数据包是DATA1,返回应答ACK
            }
            break;
        case UIS_TOKEN_IN | 0: //endpoint0 IN
            switch (SetupReq)
            {
            case USB_GET_DESCRIPTOR:
            case 0x20:
                len = SetupLen >= THIS_ENDP0_SIZE ? THIS_ENDP0_SIZE : SetupLen; //本次传输长度
                memcpy(Ep0Buffer, pDescr, len);                                 //加载上传数据
                SetupLen -= len;
                pDescr += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG; //同步标志位翻转
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0; //状态阶段完成中断或者是强制上传0长度数据包结束控制传输
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;
        case UIS_TOKEN_OUT | 0: // endpoint0 OUT
            len = USB_RX_LEN;
            if (SetupReq == 0x20) //设置串口属性
            {
                if (U_TOG_OK)
                {
					num_s = 0;
					for(temp=0;temp<4;temp++)
					{
						if((Ep0Buffer[temp])!=(LineCoding[temp]))
							num_s++;
					}
					if(num_s)
					{
						memcpy(LineCoding, UsbSetupBuf, USB_RX_LEN);
						Config_Uart1(LineCoding);
					}

                    UEP0_T_LEN = 0;
                    UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;  // 准备上传0包
                }
            }
            else if (SetupReq == 0x09)
            {
                if (Ep0Buffer[0])
                {
                }
                else if (Ep0Buffer[0] == 0)
                {
                }
            }
            UEP0_CTRL ^= bUEP_R_TOG; //同步标志位翻转
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0; //写0清空中断
    }
    if (UIF_BUS_RST) //设备模式USB总线复位中断
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
		UEP3_CTRL = bUEP_AUTO_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
        USB_DEV_AD = 0x00;
        UIF_SUSPEND = 0;
        UIF_TRANSFER = 0;
		UIF_BUS_RST = 0;                                                             //清中断标志
		USBByteCount = 0;       //USB端点收到的长度
		UsbConfig = 0;          //清除配置值
        Endp3Busy = 0;
		Endp2Busy = 0;
		Endp1Busy = 0;
    }
    if (UIF_SUSPEND) //USB总线挂起/唤醒完成
    {
        UIF_SUSPEND = 0;
        if (USB_MIS_ST & bUMS_SUSPEND) //挂起
        {
        }
    }
    else
    {
        //意外的中断,不可能发生的情况
        USB_INT_FG = 0xFF; //清中断标志
    }
}


/*******************************************************************************
* Function Name  : static void UploadData(void)
* Description    : Upload the HID code
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

/*******************************************************************************
* Function Name  : extern HIDValueHandle( void )
* Description    : Upload the HID code
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


/**************************** END *************************************/
