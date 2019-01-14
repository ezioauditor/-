/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
void enter_stop_rtc(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	/******************************************************************************************************/
	// LM75BD�����ּ��Ĵ���
	/******************************************************************************************************/
	//char str[20];
	//�������¶�����
	uint8_t buffer_receive[2] = {0x01,0x01};
	//�¶ȼĴ�����ַ
	uint8_t temp_regist=0x00;
	//���üĴ�����ַ
	uint8_t control_regist=0x01;
	//������
	uint8_t control_byte[2]={0x01,0x02};
	//�¶����޼Ĵ�����ַ
	uint8_t thyst_regist=0x02;
	//�¶����޳�ʼֵ
	uint8_t test=0;
	//�¶����޼Ĵ�����ַ
	uint8_t tos_regist=0x03;
	//�¶ȳ��������ж��ź�
	uint8_t over_tem;
	//�¶�����
	uint8_t tem_thyst;
	//�¶�����
	//uint8_t tem_tos;
	char tem_status[10];
	//��ǰ�¶�û����������
	char safe[]="Safe";
	//��ǰ�¶ȳ������� ��
	char danger[]="Danger";
	//���
	signed int MS;
	signed int ML;
	signed int result;
	signed int result_thyst;
	signed int result_tos;
	
	/******************************************************************************************************/
	//	NRF24L01ָ����
	/******************************************************************************************************/
	//���Ĵ���ָ��
	uint8_t READ_REG=0x00;
	//д�Ĵ���ָ��
	uint8_t WRITE_REG=0x20;
	//��ȡ��������ָ��
	uint8_t RD_RX_PLOAD=0x61;
	//��������������
	uint8_t W_TX_PAYLOAD=0xA0;
	//���TX FIFOָ��
	uint8_t FLUSH_TX=0xE1;
	//���RX FIFOָ��
	uint8_t FLUSH_RX=0xE2;
	//�����ϴη����غ�
	uint8_t REUSE_TX_PL=0xE3;
	//�޲��������ڶ�ȡ״̬�Ĵ���
	uint8_t NOP=0xFF;

	/******************************************************************************************************/
	//	NRF24L01�Ĵ�����ַ
	/******************************************************************************************************/
	//�����ּĴ�����ַ
	uint8_t nRF_CONFIG=0x00;
	//�Զ�Ӧ�������ã�01
	uint8_t EN_AA=0x01;
	//�����ŵ����ã�02
	uint8_t EN_ADDR=0x02;
	//�Զ��ط��������ã�04
	uint8_t SETUP_RETR=0x04;
	//RF_CH
	uint8_t RF_CH=0x05;
	//��Ƶ����
	 uint8_t RF_SETUP=0x06;
	//״̬�Ĵ�����ַ
	uint8_t status_addr=0x07;
	//���ͼ��
	uint8_t tran_dec=0x08;
	//���չ��ʼ��
	uint8_t receive_pwr=0x09;
	//Ƶ��0�������ݵ�ַ
	uint8_t regist_addr=0x0A;
	//���͵�ַ�Ĵ���
	uint8_t TX_ADDR=0x10;
	//�������ݴ�С���üĴ���
	uint8_t receive_byte_reg=0x11;
	
	//д�Ĵ���ָ��+RF_CH�Ĵ�����ַ=0x20+0x08
	uint8_t Wr_trandec=WRITE_REG+tran_dec;
	//д�Ĵ���ָ��+RF_CH�Ĵ�����ַ=0x20+0x05
	uint8_t Re_pwr=WRITE_REG+receive_pwr;
	//д�Ĵ���ָ��+RF_CH�Ĵ�����ַ=0x20+0x05
	uint8_t Wr_RFCH=WRITE_REG+RF_CH;
	//д�Ĵ���ָ��+RF_SETUP�Ĵ�����ַ=0x20+0x06
	uint8_t Wr_RFsetup=WRITE_REG+RF_SETUP;
	//д�Ĵ���ָ��+ENAA�Ĵ�����ַ=0x20+0x01
	uint8_t Wr_ENAA=WRITE_REG+EN_AA;
	//д�Ĵ���ָ��+ENAA�Ĵ�����ַ=0x20+0x02
	uint8_t Wr_ENADDR=WRITE_REG+EN_ADDR;
	//д�Ĵ���ָ��+���üĴ�����ַ=0x20+0x00
	uint8_t COM_CONFIG=WRITE_REG+nRF_CONFIG;
	//д�Ĵ���ָ��+�Զ��ط��������üĴ�����ַ=0x20+0x04
	uint8_t COM_SETUP=WRITE_REG+SETUP_RETR;
	//д�Ĵ���ָ��+���͵�ַ�Ĵ���=0x20+0x10
	uint8_t ADDR_CONFIG=WRITE_REG+TX_ADDR;
	//д�Ĵ���ָ��+���ն˵�ַ=0x20+0x0A
	uint8_t RX_ADDR=WRITE_REG+regist_addr;
	//��״̬�Ĵ���ָ��+״̬�Ĵ�����ַ=0x00+0x07
	uint8_t Read_Status=READ_REG+status_addr;
	//д״̬�Ĵ���ָ��+״̬�Ĵ�����ַ=0x20+0x07
	uint8_t Wr_Status=WRITE_REG+status_addr;
	//д�Ĵ���ָ��+״̬�Ĵ�����ַ=0x20+0x07
	uint8_t Wr_Stuts=WRITE_REG+status_addr;
	//д�Ĵ���ָ��+���ý�����������=0x20+0x11
	uint8_t Wr_receivebyte=WRITE_REG+receive_byte_reg;
	//д�Ĵ���ָ��+���ý�����������=0x20+0x11
	uint8_t Rd_receivebyte=READ_REG+receive_byte_reg;
	
	//Feature�Ĵ�����ַ
	uint8_t Feature_config=0x1D;
	//����ָ��
	uint8_t Wr_feature=WRITE_REG+Feature_config;
	//������
	uint8_t Feature_com=0x00;
	
	//����
	uint8_t Rd_REG=READ_REG+0x17;
	//�������ļĴ���ֵ
	uint8_t test_config;
	uint8_t test_addr[5];

	
	//�ش������Ĵ�������
	uint8_t SETUP_NUM=0xFF;
	//RD_CH
	uint8_t RFCH_COM=0x00;
	//RF_SETUP����
	uint8_t RF_COM=0x07;
	//���ص�ַ
	uint8_t TX_ADDRESS0[5]={0x01,0x01,0x01,0x01,0x01};
	//���յ�ַ
	uint8_t RX_ADDRESS0[5]={0x10,0x10,0x10,0x10,0x10};                                          
	//����ģʽ������
	uint8_t TX_CONFIG =0x0E;
	//����ģʽ������
	uint8_t RX_CONFIG=0x0F;
	//״̬�Ĵ�������
	uint8_t status; 
	//���״̬�Ĵ�������
	uint8_t status_clear=0x7E;
	//���ն˽������ݴ�С
	uint8_t receive_byte=0x02;
	//���Ͷ˽������ݴ�С
	uint8_t tran_byte=0x02;
	//ENAAֵ
	uint8_t EN_A=0x01;
	//EN_ADDR
	uint8_t EN_ADD=0x01;
	
	//���չ���
	uint8_t pwr;
	
	int receive;
	int ready;
	int end;
	char tran_ok[8]="OK\r\n";
	char tran_fail[10]="failed\r\n";
	char ready_receive[23]="Ready for Receive\r\n";
	char ready_tran[23]="\r\n Ready to transmit\r\n";
	//���ն�����
	uint8_t data_receive[2];

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */	
	/******************************************************************************************************/
	/* �������� */
	/******************************************************************************************************/
	while(1){
	//����CE��
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_3,  GPIO_PIN_RESET);
		
	//���ص�ַ����
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	//д�Ĵ���ָ��+���͵�ַ�Ĵ���=0x20+0x10
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&ADDR_CONFIG,  1,  1000)!=HAL_OK);
	
	//��ַд��:0x10,0x10,0x10,0x10,0x10
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&RX_ADDRESS0,  5,  1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);
	/******************************************************************************************************/
	//���յ�ַ����
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	//д�Ĵ���ָ��+���͵�ַ�Ĵ���=0x20+0x0A
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&RX_ADDR,  1,  1000)!=HAL_OK);
	
	//��ַд�룺0x01,0x01,0x01,0x01,0x01
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&TX_ADDRESS0,  5,  1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);	
	
	/******************************************************************************************************/
	//EN_AA
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	//д�Ĵ���ָ��+EN_AA�Ĵ���=0x20+0x01
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&Wr_ENAA,  1,  1000)!=HAL_OK);
	
	//ָ�0x01
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&EN_A,  1,  1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);
	
	/******************************************************************************************************/
	//EN_RXADDR
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&Wr_ENADDR,  1,  1000)!=HAL_OK);
	
	//ָ�0x01
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&EN_ADD,  1,  1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);
	
	/******************************************************************************************************/
	//ͨ��Ƶ��
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&Wr_RFCH,  1,  1000)!=HAL_OK);
	
	//����д��
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&RFCH_COM,  1,  1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);
	/******************************************************************************************************/
	//�������
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	//д�Ĵ���ָ��+�ش�����=0x20+0x06
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&Wr_RFsetup,  1,  1000)!=HAL_OK);
	
	//����д��
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&RF_COM,  1,  1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);
	
	/******************************************************************************************************/
	//������Ч���
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	//д�Ĵ���ָ��+���ý�����������=0x20+0x11
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&Wr_receivebyte,  1,  1000)!=HAL_OK);
	
	//ָ�0000_0010
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&receive_byte,  1,  1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);
	
	/******************************************************************************************************/
	//���RX_DRλ
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	//дָ��+״̬�Ĵ�����ַ��0x20+0x07
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&Wr_Status,  1,  1000)!=HAL_OK);
	
	//����
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&status_clear,  1,  1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);
	
	//����״̬��ӦΪ0x0E
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	//���Ĵ���ָ��+״̬�Ĵ���
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&Read_Status,  1,  1000)!=HAL_OK);
	
	//��״̬�Ĵ�������
	while(HAL_SPI_Receive(&hspi1, (uint8_t*)&status, 1, 1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);
	
	//�鿴״̬�Ĵ�����ӦΪ0x40
	//HAL_UART_Transmit(&huart1,(uint8_t*)&status,1,1000);
	
	/******************************************************************************************************/
	//��RX FIFO
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	//��ϴFIFOָ��
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&FLUSH_RX,  1,  1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);

	/******************************************************************************************************/
	//CONFIG�Ĵ�������
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	//д�Ĵ���ָ��+���üĴ�����ַ=0x20+0x00
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&COM_CONFIG,  1,  1000)!=HAL_OK);
	
	//ָ�00001111
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&RX_CONFIG,  1,  1000)!=HAL_OK);
	
	//д������CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);

	/******************************************************************************************************/
	//����
	//receive=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
	//HAL_UART_Transmit(&huart1,(uint8_t*)&receive,1,1000);
	
	//����CE��
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_3,  GPIO_PIN_SET);
	
	HAL_Delay(10);
	
	//����CE��
	/*HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_3,  GPIO_PIN_RESET);*/
	
	//׼������
	//HAL_UART_Transmit(&huart1,(uint8_t*)ready_receive,23,1000);
	
	/******************************************************************************************************/
	
	//IRQ����Ч
	while((receive=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2))){
		
	}
	
	//HAL_UART_Transmit(&huart1,(uint8_t*)tran_ok,8,1000);
	
	//����CE��
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_3,  GPIO_PIN_RESET);
	
	/******************************************************************************************************/
	/*
	//����״̬��ӦΪ0x40
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	//���Ĵ���ָ��+״̬�Ĵ���
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&Read_Status,  1,  1000)!=HAL_OK);
	
	//��״̬�Ĵ�������
	while(HAL_SPI_Receive(&hspi1, (uint8_t*)&status, 1, 1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);
	
	//�鿴״̬�Ĵ�����ӦΪ0x40
	//HAL_UART_Transmit(&huart1,(uint8_t*)&status,1,1000);
	*/
	
	/******************************************************************************************************/
	/*
	//����FIFO��ӦΪ0x10
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&Rd_REG,  1,  1000)!=HAL_OK);
	
	while(HAL_SPI_Receive(&hspi1, (uint8_t*)&test_config,  1,  1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);
	
	HAL_UART_Transmit(&huart1,(uint8_t*)&test_config,1,1000);
	*/
	
	/******************************************************************************************************/
	//���RX_DRλ
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	//дָ��+״̬�Ĵ�����ַ��0x20+0x07
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&Wr_Status,  1,  1000)!=HAL_OK);
	
	//����
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&status_clear,  1,  1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);
	
	/******************************************************************************************************/
	/*
	//RX_DR=1
	if(status&0x40)
		//���ճɹ�
		HAL_UART_Transmit(&huart1,(uint8_t*)tran_ok,8,0xffff);
	else 
		//����ʧ��
		HAL_UART_Transmit(&huart1,(uint8_t*)tran_fail,10,0xffff);
	*/
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	//��ȡ��������������
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&RD_RX_PLOAD,  1,  1000)!=HAL_OK);
	
	//��������
	while(HAL_SPI_Receive(&hspi1, (uint8_t*)data_receive, 2, 1000) != HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);
	
	//�����ܵ����ݣ�δ�����뷢�Ͷ˱ȶ�
	//HAL_UART_Transmit(&huart1,(uint8_t*)data_receive,2,0xffff);
	
	/******************************************************************************************************/
	/*
	//����FIFO:����ӦΪ0x11
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&Rd_REG,  1,  1000)!=HAL_OK);
	
	while(HAL_SPI_Receive(&hspi1, (uint8_t*)&test_config,  1,  1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);
	
	HAL_UART_Transmit(&huart1,(uint8_t*)&test_config,1,1000);*/

	/******************************************************************************************************/
	//��RX FIFO
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_RESET);
	
	//��ϴFIFOָ��
	while(HAL_SPI_Transmit(&hspi1, (uint8_t*)&FLUSH_RX,  1,  1000)!=HAL_OK);
	
	//����CSN����
	HAL_GPIO_WritePin(GPIOA,  GPIO_PIN_4,  GPIO_PIN_SET);
	
	/******************************************************************************************************/

	/******************************************************************************************************/
	// ��λ���������� 
	/******************************************************************************************************/
	//δ��������
	HAL_UART_Transmit(&huart1,(uint8_t*)data_receive,2,0xffff);
	/*
	//ƴ�ӳ�16λ����
	//��λ
	MS=data_receive[0];
	//��λ
	ML=data_receive[1];
	result=result | MS;
	result=result<<8;
	result=result|ML;
	//�¶ȼĴ�����5λ��Ч����
	uint8_t result_a=result>>5;
	
	//HAL_UART_Transmit(&huart1,(uint8_t*) &MS,1,1000);
	//HAL_UART_Transmit(&huart1,(uint8_t*) &ML,1,1000);
	
	//uart�����ȵ�λ���λ
	//HAL_UART_Transmit(&huart1,(uint8_t*) &result_a,2,1000);*/
	}



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //while (1)
  //{
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
		
	//}
  /* USER CODE END 3 */

}

void enter_stop_rtc(){
	
	/* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower(); 
	/* Enable Fast WakeUP */
	HAL_PWREx_EnableFastWakeUp();
	
	/* Disable Wakeup Counter */
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

	/*To configure the wake up timer to 4s the WakeUpCounter is set to 0x242B:
	RTC_WAKEUPCLOCK_RTCCLK_DIV = RTCCLK_Div16 = 16 
	Wakeup Time Base = 16 /(37KHz) = ~0.432 ms
	Wakeup Time = ~5s = 0.432ms  * WakeUpCounter
	==> WakeUpCounter = ~5s/0.432ms = 11562 */
	
	//��ֵ=ʱ��*2312
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 11560, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
	 
	//system_power_config();
	
	//MX_RTC_Init();
	
	//����ģʽ
	HAL_PWR_EnterSTANDBYMode();
	
	//HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	
	/* Clear all related wakeup flags */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU); 

	__HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG(); //�����־������ڶ����Ժ��޷���������
	
	SystemClock_Config();

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 0x17;
  sTime.Minutes = 0x13;
  sTime.Seconds = 0x00;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BCD);

  sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x11;
  sDate.Year = 0x19;
  HAL_RTC_SetDate(&hrtc, &sDate, FORMAT_BCD);

    /**Enable the WakeUp 
    */
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 11560, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA0 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
