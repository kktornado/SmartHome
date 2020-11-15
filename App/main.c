#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x_it.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_conf.h"
//#include "misc.h"
#include "stm32f10x_rcc.h"

#include <stdio.h>

#define  START_TASK_PRIO            1       //任务优先级
#define  START_STK_SIZE   				  128     //任务堆栈大小

#define  LED0_TASK_PRIO            2       //任务优先级
#define  LED0_STK_SIZE   				  50      //任务堆栈大小

#define  LED1_TASK_PRIO            3       //任务优先级
#define  LED1_STK_SIZE   				  50      //任务堆栈大小

#define   SYSTEM_SUPPORT_OS
//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define LED0 PBout(5)// PB5
#define LED1 PEout(5)// PE5	

void  start_task(void *pvPatameters);      //任务函数
void  led0_task(void *p_arg);              //任务函数
void  led1_task(void *p_arg);      				//任务函数

TaskHandle_t StartTask_Handle;             //任务句柄
TaskHandle_t LED0Task_Handle;             //任务句柄
TaskHandle_t LED1Task_Handle;             //任务句柄
 
 
static u8  fac_us=0;							//us延时倍乘数			   
static u16 fac_ms=0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数
	
//初始化PB5和PE5为输出口.并使能这两个口的时钟		    
//LED IO初始化
//void LED_Init(void)
//{
// 
// GPIO_InitTypeDef  GPIO_InitStructure;
// 	
// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);	 //使能PB,PE端口时钟
//	
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //LED0-->PB.5 端口配置
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
// GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
// GPIO_SetBits(GPIOB,GPIO_Pin_5);						 //PB.5 输出高

// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    		 //LED1-->PE.5 端口配置, 推挽输出
// GPIO_Init(GPIOE, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
// GPIO_SetBits(GPIOE,GPIO_Pin_5); 						 //PE.5 输出高 
//}

//初始化延迟函数
//当使用OS的时候,此函数会初始化OS的时钟节拍
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void delay_init()
{

	u32 reload;

	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);	//选择外部时钟  HCLK
	fac_us=SystemCoreClock/1000000;				//为系统时钟的1/8  

	reload=SystemCoreClock/1000000;				//每秒钟的计数次数 单位为K	   
	reload*=1000000/configTICK_RATE_HZ;		//根据configTICK_RATE_HZ设定溢出时间
												//reload为24位寄存器,最大值:16777216,在72M下,约合1.86s左右	
	fac_ms=1000/configTICK_RATE_HZ;			//代表OS可以延时的最少单位	   

	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//开启SYSTICK中断
	SysTick->LOAD=reload; 						//每1/delay_ostickspersec秒中断一次	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	//开启SYSTICK    

}				

int main()
{
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);   //设置中断优先级为4
	 delay_init();                    //延时函数初始化
//	 uart_init(115200);               //初始化串口
//	 LED_Init();                      //初始化LED
		 //创建开始任务
	 xTaskCreate((TaskFunction_t) start_task,         //任务函数
		           (const char*)"start_task",           //任务名称
							 (uint16_t)START_STK_SIZE,            //任务堆栈大小
							 (void*)NULL,                         //传递给任务函数的参数
							 (UBaseType_t)START_TASK_PRIO,        //任务优先级
							 (TaskHandle_t*)&StartTask_Handle);   //任务句柄
	 vTaskStartScheduler();  
}


void start_task(void *Pvparameters)
{
	taskENTER_CRITICAL();            //进入临界区
	
	//创建LED0任务
	 xTaskCreate((TaskFunction_t) led0_task,         //任务函数
						 (const char*)"led0_task",           //任务名称
						 (uint16_t)LED0_STK_SIZE,            //任务堆栈大小
						 (void*)NULL,                         //传递给任务函数的参数
						 (UBaseType_t)LED0_TASK_PRIO,        //任务优先级
						 (TaskHandle_t*)&LED0Task_Handle);   //任务句柄
						 
	//创建LED1任务
	 xTaskCreate((TaskFunction_t) led1_task,         //任务函数
						 (const char*)"led1_task",           //任务名称
						 (uint16_t)LED1_STK_SIZE,            //任务堆栈大小
						 (void*)NULL,                         //传递给任务函数的参数
						 (UBaseType_t)LED1_TASK_PRIO,        //任务优先级
						 (TaskHandle_t*)&LED1Task_Handle);   //任务句柄
						 
	 vTaskDelete(StartTask_Handle);                 //删除开始任务
	 taskEXIT_CRITICAL();                           //退出临界区						 
}

//LED0任务函数
void led0_task(void *pvParameters) 
{
//	while(1)
//	{
//		LED0=~LED0;
//		vTaskDelay(500);
//	}
	printf("led0\n");
}


//LED1任务函数
void led1_task(void *pvParameters)
{
//	while(1)
//	{
//		LED1=0;
//		vTaskDelay(200);
//		LED1=1;
//		vTaskDelay(800);
//	}
	printf("led1\n");
}