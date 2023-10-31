#include "can.h"

#include "can.h"
#include "delay.h"
#include "usart.h"
#include "sys.h"
#include "math.h"
#include "stdlib.h"
#include "includes.h"

// CAN初始化
// tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
// tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
// tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
// brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
// mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
// Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//则波特率为:42M/((6+7+1)*6)=500Kbps
//返回值:0,初始化OK;
//其他,初始化失败;

//根据ODIVE规则 其基础ID 为 can_id = axis_id << 5 | cmd_id ： 例如，0x01带有  命令ID 0x0C  将导致 0x2C
// ODRIVE axis0 的节点ID 为 4 = 0000 0100 左移五位后为 000 1010 0000  =  0x0A0
// ODRIVE axis1 的节点ID 为 5 = 0000 0101 左移五位后为 000 1000 0000  =  0x080
//所以用屏蔽模式只要是开头为 0x0A0 和 0x080 以及 0x0C0 的 ID都能通过, 将前4位ID完全匹配位 000 1xxx xxxx

void FOC_CAN1_Mode_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE
	NVIC_InitTypeDef NVIC_InitStructure;
#endif
	//使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //使能PORTA时钟

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); //使能CAN1时钟

	//初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   //复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);			   //初始化PA11,PA12

	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1); // GPIOA11复用为CAN1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1); // GPIOA12复用为CAN1

	// CAN单元设置
	CAN_InitStructure.CAN_TTCM = DISABLE;		  //非时间触发通信模式
	CAN_InitStructure.CAN_ABOM = DISABLE;		  //软件自动离线管理
	CAN_InitStructure.CAN_AWUM = DISABLE;		  //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART = ENABLE;		  //禁止报文自动传送
	CAN_InitStructure.CAN_RFLM = DISABLE;		  //报文不锁定,新的覆盖旧的
	CAN_InitStructure.CAN_TXFP = DISABLE;		  //优先级由报文标识符决定
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; //模式设置
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;	  //重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1 = CAN_BS1_7tq;	  // Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2 = CAN_BS2_6tq;	  // Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler = 3;		  //分频系数(Fdiv)为brp+1
	CAN_Init(CAN1, &CAN_InitStructure);			  // 初始化CAN1

	//配置过滤器
	CAN_FilterInitStructure.CAN_FilterNumber = 0;					 //过滤器0
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;	 //屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; // 32位
	CAN_FilterInitStructure.CAN_FilterIdLow = 0;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x1000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xF000; // 32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0002;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; //过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;				 //激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);							 //滤波器初始化

	CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE); // FIFO0消息挂号中断暂时不允许.

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 主优先级为2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

#if CAN1_RX0_INT_ENABLE //使能RX0中断
//中断服务函数
void CAN1_RX0_IRQHandler(void)
{
	//	OS_ERR err;
	//	OSIntEnter();

	CanRxMsg rx_message;
	// if( CAN_MessagePending(CAN1,CAN_FIFO0)==0) 	return;
	CAN_Receive(CAN1, CAN_FIFO0, &rx_message); //读取邮箱数据
	printf("%x\r\n", rx_message.StdId);
	//	OSIntExit();//告诉操作系统退出中断
}
#endif

u8 FOC_CAN1_Send_Msg(u32 ID, u8 *msg, u8 len)
{
	u8 mbox;
	u16 i = 0;
	CanTxMsg TxMessage;
	TxMessage.StdId = ID; // 标准标识符为0
	TxMessage.ExtId = ID; // 设置扩展标示符（29位）
	TxMessage.IDE = 0;	  // 使用扩展标识符
	TxMessage.RTR = 0;	  // 消息类型为数据帧，一帧8位
	TxMessage.DLC = len;  // 发送两帧信息
	for (i = 0; i < len; i++)
		TxMessage.Data[i] = msg[i]; // 第一帧信息
	mbox = CAN_Transmit(CAN1, &TxMessage);
	i = 0;
	while ((CAN_TransmitStatus(CAN1, mbox) == CAN_TxStatus_Failed) && (i < 0XFFF))
		i++; //等待发送结束
	if (i >= 0XFFF)
		return 1;
	return 0;
}

u8 FOC_CAN1_Receive_Msg(u8 *buf)
{
	u32 i;
	CanRxMsg RxMessage;
	if (CAN_MessagePending(CAN1, CAN_FIFO0) == 0)
		return 0;							  //没有接收到数据,直接退出
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage); //读取数据
	for (i = 0; i < RxMessage.DLC; i++)
		buf[i] = RxMessage.Data[i];
	return RxMessage.DLC;
}
