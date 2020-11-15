#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x_it.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_conf.h"
//#include "misc.h"
#include "stm32f10x_rcc.h"

#include <stdio.h>

#define  START_TASK_PRIO            1       //�������ȼ�
#define  START_STK_SIZE   				  128     //�����ջ��С

#define  LED0_TASK_PRIO            2       //�������ȼ�
#define  LED0_STK_SIZE   				  50      //�����ջ��С

#define  LED1_TASK_PRIO            3       //�������ȼ�
#define  LED1_STK_SIZE   				  50      //�����ջ��С

#define   SYSTEM_SUPPORT_OS
//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
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
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

#define LED0 PBout(5)// PB5
#define LED1 PEout(5)// PE5	

void  start_task(void *pvPatameters);      //������
void  led0_task(void *p_arg);              //������
void  led1_task(void *p_arg);      				//������

TaskHandle_t StartTask_Handle;             //������
TaskHandle_t LED0Task_Handle;             //������
TaskHandle_t LED1Task_Handle;             //������
 
 
static u8  fac_us=0;							//us��ʱ������			   
static u16 fac_ms=0;							//ms��ʱ������,��ucos��,����ÿ�����ĵ�ms��
	
//��ʼ��PB5��PE5Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
//void LED_Init(void)
//{
// 
// GPIO_InitTypeDef  GPIO_InitStructure;
// 	
// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
//	
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 //LED0-->PB.5 �˿�����
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
// GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.5
// GPIO_SetBits(GPIOB,GPIO_Pin_5);						 //PB.5 �����

// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	    		 //LED1-->PE.5 �˿�����, �������
// GPIO_Init(GPIOE, &GPIO_InitStructure);	  				 //������� ��IO���ٶ�Ϊ50MHz
// GPIO_SetBits(GPIOE,GPIO_Pin_5); 						 //PE.5 ����� 
//}

//��ʼ���ӳٺ���
//��ʹ��OS��ʱ��,�˺������ʼ��OS��ʱ�ӽ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void delay_init()
{

	u32 reload;

	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);	//ѡ���ⲿʱ��  HCLK
	fac_us=SystemCoreClock/1000000;				//Ϊϵͳʱ�ӵ�1/8  

	reload=SystemCoreClock/1000000;				//ÿ���ӵļ������� ��λΪK	   
	reload*=1000000/configTICK_RATE_HZ;		//����configTICK_RATE_HZ�趨���ʱ��
												//reloadΪ24λ�Ĵ���,���ֵ:16777216,��72M��,Լ��1.86s����	
	fac_ms=1000/configTICK_RATE_HZ;			//����OS������ʱ�����ٵ�λ	   

	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//����SYSTICK�ж�
	SysTick->LOAD=reload; 						//ÿ1/delay_ostickspersec���ж�һ��	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	//����SYSTICK    

}				

int main()
{
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);   //�����ж����ȼ�Ϊ4
	 delay_init();                    //��ʱ������ʼ��
//	 uart_init(115200);               //��ʼ������
//	 LED_Init();                      //��ʼ��LED
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
//	while(1)
//	{
//		LED0=~LED0;
//		vTaskDelay(500);
//	}
	printf("led0\n");
}


//LED1������
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