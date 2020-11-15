#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x_it.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
//#include "stm32f10x_conf.h"

//#include "stm32f10x_rcc.h"

#include <stdio.h>

#define  START_TASK_PRIO            1       //任务优先级
#define  START_STK_SIZE   				  128     //任务堆栈大小

#define  LED0_TASK_PRIO            2       //任务优先级
#define  LED0_STK_SIZE   				  50      //任务堆栈大小

#define  LED1_TASK_PRIO            3       //任务优先级
#define  LED1_STK_SIZE   				  50      //任务堆栈大小


void  start_task(void *pvPatameters);      //任务函数
void  led0_task(void *p_arg);              //任务函数
void  led1_task(void *p_arg);      				//任务函数

TaskHandle_t StartTask_Handle;             //任务句柄
TaskHandle_t LED0Task_Handle;             //任务句柄
TaskHandle_t LED1Task_Handle;             //任务句柄
 
 
static u8  fac_us=0;							//us延时倍乘数			   
static u16 fac_ms=0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数
	
int main()
{
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);   //设置中断优先级为4


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
	printf("led0\n");
}


//LED1任务函数
void led1_task(void *pvParameters)
{
	printf("led1\n");
}