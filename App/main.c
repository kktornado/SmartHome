#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x_it.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
//#include "stm32f10x_conf.h"

//#include "stm32f10x_rcc.h"

#include <stdio.h>

#define  START_TASK_PRIO            1       //�������ȼ�
#define  START_STK_SIZE   				  128     //�����ջ��С

#define  LED0_TASK_PRIO            2       //�������ȼ�
#define  LED0_STK_SIZE   				  50      //�����ջ��С

#define  LED1_TASK_PRIO            3       //�������ȼ�
#define  LED1_STK_SIZE   				  50      //�����ջ��С


void  start_task(void *pvPatameters);      //������
void  led0_task(void *p_arg);              //������
void  led1_task(void *p_arg);      				//������

TaskHandle_t StartTask_Handle;             //������
TaskHandle_t LED0Task_Handle;             //������
TaskHandle_t LED1Task_Handle;             //������
 
 
static u8  fac_us=0;							//us��ʱ������			   
static u16 fac_ms=0;							//ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��
	
int main()
{
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);   //�����ж����ȼ�Ϊ4


	 //������ʼ����
	 xTaskCreate((TaskFunction_t) start_task,         //������
		           (const char*)"start_task",           //��������
							 (uint16_t)START_STK_SIZE,            //�����ջ��С
							 (void*)NULL,                         //���ݸ��������Ĳ���
							 (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
							 (TaskHandle_t*)&StartTask_Handle);   //������
	 vTaskStartScheduler();  
}


void start_task(void *Pvparameters)
{
	taskENTER_CRITICAL();            //�����ٽ���
	
	//����LED0����
	 xTaskCreate((TaskFunction_t) led0_task,         //������
						 (const char*)"led0_task",           //��������
						 (uint16_t)LED0_STK_SIZE,            //�����ջ��С
						 (void*)NULL,                         //���ݸ��������Ĳ���
						 (UBaseType_t)LED0_TASK_PRIO,        //�������ȼ�
						 (TaskHandle_t*)&LED0Task_Handle);   //������
						 
	//����LED1����
	 xTaskCreate((TaskFunction_t) led1_task,         //������
						 (const char*)"led1_task",           //��������
						 (uint16_t)LED1_STK_SIZE,            //�����ջ��С
						 (void*)NULL,                         //���ݸ��������Ĳ���
						 (UBaseType_t)LED1_TASK_PRIO,        //�������ȼ�
						 (TaskHandle_t*)&LED1Task_Handle);   //������
						 
	 vTaskDelete(StartTask_Handle);                 //ɾ����ʼ����
	 taskEXIT_CRITICAL();                           //�˳��ٽ���						 
}

//LED0������
void led0_task(void *pvParameters) 
{
	printf("led0\n");
}


//LED1������
void led1_task(void *pvParameters)
{
	printf("led1\n");
}