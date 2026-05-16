/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* Copyright (c) 2024 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
******************************************************************************
*/
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/apps/httpd.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "api.h"
#include "string.h"
#include "lwip/netif.h"
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/udp.h"
#include "lwip/netbuf.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include "err.h"
#include "lwip/ip.h"
#include "lwip/ip4.h"
#include "lwip/ip4_addr.h"
#include "lwip/ip_addr.h"
#include "lwip/sockets.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mbap_conf.h"
#include "mbap_user.h"
#include "tcp1.h"


#include <float.h>
#include <math.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "printf.h"

#include "modbus_master.h"

#include "w25qxx.h"
#include "w25qxxConf.h"

#include "pid.h"


#include <stdio.h>
#include <string.h>
#include "eeprom.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

/* Definitions for ModbusTCPTask */
osThreadId_t ModbusTCPTaskHandle;
const osThreadAttr_t ModbusTCPTask_attributes = {
  .name = "ModbusTCPTask",
  .stack_size = 350 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal5,
};
/* Definitions for MotorControlTas */
osThreadId_t MotorControlTasHandle;
const osThreadAttr_t MotorControlTas_attributes = {
  .name = "MotorControlTas",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal5,
};
/* Definitions for ModbusRTUTask */
osThreadId_t ModbusRTUTaskHandle;
const osThreadAttr_t ModbusRTUTask_attributes = {
  .name = "ModbusRTUTask",
  .stack_size = 168 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal5,
};
/* Definitions for GovernerTask */
osThreadId_t GovernerTaskHandle;
const osThreadAttr_t GovernerTask_attributes = {
  .name = "GovernerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal5,
};
/* USER CODE BEGIN PV */
uint16_t VirtAddVarTab[64] = {0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x80,0x90,0xA0,0xB0,0xC0,0xD0,0xE,0xF0,
0x100,0x110,0x120,0x130,0x140,0x150,0x160,0x170,0x180,0x190,0x1A0,0x1B0,0x1C0,
0x1D0,0x1E0,0x1F0,0x200,0x210,0x220,0x230,0x240,0x250,0x260,0x270,0x280,0x290,0x2A0,0x2B0,
0x2C0,0x2D0,0x2E0,0x2F0,0x300,0x310,0x320,0x330,0x340,0x350,0x360,0x370,0x380,0x390,0x3A0,0x3B0,0x3C0,0x3D0,0x3E0,0x3F0,0x400
};

int spiflash_ID = 0;

uint8_t state_modbus=0;
uint8_t result_modbus1;
uint16_t discreet_input_result_modbus;
uint8_t result_modbus2;
int plc_relay1 = 0;
int plc_relay2 = 0;
int plc_relay3 = 0;
int plc_relay4 = 0;
int plc_relay5 = 0;
int plc_relay6 = 0;
int plc_input1 = 0;
int plc_input2 = 0;
int plc_input3 = 0;
int plc_input4 = 0;
int plc_input5 = 0;
int plc_input6 = 0;
int plc_input7 = 0;
int plc_input8 = 0;

//modbus rtu vars
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))
uint8_t usart_rx[128];
size_t pos;
uint16_t old_pos;
uint16_t old_pos_uart1;
uint16_t lenght_uart1;
uint16_t position_uart1;
uint16_t lenght_uart1;

//dip switch and addressing variables
int dip1 = 0;
int dip2 = 0;
int dip3 = 0;
int dip4 = 0;
int address = 0;
int addressNew = 0;
int bootFlag = 0;
uint8_t ipLastPart = 2 ;
uint8_t MACAddrUSER ;
int booted = 0;
int RTUMaster = 0;

int pulse_count = 0;
int modbusStart = 0;
uint8_t discreet_input_status = 0;
int modbus_tcp_blinker = 0;

//motor variables 
int hardware_reduction1 = 0;
int hardware_reduction2 = 0;
int hardware_reduction3 = 0;
int hardware_reduction4 = 0;
int hardware_reduction5 = 0;
int hardware_reduction6 = 0;

int current_hardware_reduction = 0;

int pulse_count1 = 0;
int motor_start1 = 0;
int motor_pulse1 = 0;
int motor1ActionBit = 0;
int PWM_NUM1 = 2000;

int motor1ActionBit2 = 0;

int joint1UpperT = 175;
int joint1LowerT = -175;

int pulse_count1TIM = 0;

int pulse_rotation1 = 0;

int motor1DirBit = 0;

int motor1Rotation = 0;

int homeSwState1 = 0;

int commandUpdateBit = 0;

uint8_t command1_str[6];

uint8_t command1Sign_str[2];

int commandManualUpdateBit = 0 ;

int command1 = 0;

float command1F = 0;

int sign1 = 0;

int posReset = 0;
int gripperCommand = 0;

int actionBit = 0;
int actionBit2 = 0;
int motorRotationConstant = 20000;
int motor1GearRatio = 0; ;
int address1GearRatio = 100;
int address2GearRatio = 150;
int address3GearRatio = 100;
int address4GearRatio = 100;
int address5GearRatio = 100;
int address6GearRatio = 50; 
int motor1PulseCom = 0;

int manualControlBit = 0;

uint16_t virginity_bit;
int flash_state = 0;
int flash_page_change = 0;
uint16_t motor1CurrentPosEE = 0;

uint16_t motor1CurrentsignEE = 0;

float goalDegree1 = 0;

float degreeDifference1 = 0;

int signDifference1 = 0;

float currentDegree1 = 0;

uint8_t first_byte[2];
int eepromResetBit = 0;

int actionDone = 0;

int manualMotorNum = 0;
int manualMotorSign = 0;
int manualMotorPulse = 0;
int manualEepromReset = 0;
double PWMFrequency = 0;
double PWMSetPoint = 0;

int commandDegreeInput1 = 0;

int command1OK = 0;

int actionDone2 = 0;
int actionDone3 = 0;

int controllerComandBit = 0;

typedef struct{
  int16_t velocity;
  int64_t position;
  uint32_t last_counter_value;
  int direction;
  int isMoving;
  int64_t realPosition;
}encoder_instance;

encoder_instance enc_instance_mot;

PID_TypeDef MPID;

double frequency,frequency_out, frequencySetpoint;
int pwmARR = 0;
int pwmPSC = 0;

double PID_P = 0.8 ;

int encoder_counter = 0;

int encoderValueP; 

int encoder_position = 0;
int encoder_velocity = 0;
int encoder_direction = 0;
int encoderPositionReal = 0;
int motorEN = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_OTG_FS_USB_Init(void);
void ModbusTCPTaskFunc(void *argument);
void MotorControlTaskFunc(void *argument);
void ModbusRTUTaskFunc(void *argument);
void GovernerTaskFunc(void *argument);

/* USER CODE BEGIN PFP */
void setARR1(TIM_HandleTypeDef timer, uint32_t channel,uint16_t pulse){
  
  __HAL_TIM_SetAutoreload(&timer,pulse);
  __HAL_TIM_SetCompare(&timer,channel,pulse/2);
  // __HAL_TIM_URS_ENABLE(&timer);
  TIM5->EGR |= TIM_EGR_UG;
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
  if(htim -> Instance == TIM5)
  {
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      pulse_count1++;
      
    }
  }
  ////////////////////////////
  
  ////////////////////////////
}

void usart_transmit_data(const void* data, uint16_t len) {
  const uint8_t* b = data;
  while (len--) {
    LL_USART_TransmitData8(USART1, *b++);
    while (!LL_USART_IsActiveFlag_TXE(USART1));
  }
}

/**
* \brief           Process received data over UART
* \note            Either process them directly or copy to other bigger buffer
* \param[in]       data: Data to process
* \param[in]       len: Length in units of bytes
*/
void usart_process_data(const void* data, size_t len) {
  const uint8_t* d = data;
  while (len--) {
    LL_USART_TransmitData8(USART1, *d++);
    while (!LL_USART_IsActiveFlag_TXE(USART1));
  }
  while (!LL_USART_IsActiveFlag_TC(USART1));
}

/**
* \brief           Send string to USART
* \param[in]       str: String to send
*/
void usart_send_string(const char* str) {
  usart_process_data(str, strlen(str));
}

int extractBit(
               int num,
               int bitPosition)
{
  // Create a mask with only the 5th bit set to 1
  unsigned int mask = 1 << bitPosition;
  
  // Extract the bit using AND and right shift
  unsigned int extracted_bit
    = (num & mask) >> bitPosition;
  
  // Print the result
  
  
  return extracted_bit;
}

int setBit(
           int N,
           int K)
{
  
  // Bitwise OR with the mask
  return (N | (1 << (K - 1)));
}

int clearBit(
             int N,
             int K)
{
  
  // Bitwise AND with the mask
  return (N & (~(1 << (K - 1))));
}

int toggleBit(
              int N,
              int K)
{
  
  // Bitwise XOR with the mask
  return (N ^ (1 << (K - 1)));
}

uint8_t insert_bit(uint8_t n,   // The integer we are going to insert into
                   int position, // position is the position of the new bit to be inserted
                   int new_bit) // whether the newly inserted bit is true or false
{
  uint8_t x = n;
  uint8_t y = x;
  x <<= 1;
  if (new_bit)
    x |= (((uint8_t) 1) << position);
  else
    x &= ~(((uint8_t) 1) << position);
  x &= ((~((uint8_t) 0)) << position);
  y &= ~((~((uint8_t) 0)) << position);
  x |= y;
  return x;
}

/**
* @brief USART1 Initialization Function
* @param None
* @retval None
*/
static void MX_USART1_UART_Init(void)
{
  
  /* USER CODE BEGIN USART1_Init 0 */
  
  /* USER CODE END USART1_Init 0 */
  
  LL_USART_InitTypeDef USART_InitStruct = {0};
  
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PB7   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /* USART1 DMA Init */
  
  /* USART1_RX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_4);
  
  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  
  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_LOW);
  
  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_CIRCULAR);
  
  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);
  
  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);
  
  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);
  
  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);
  
  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_2);
  
  
  LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)&USART1->DR);
  LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)usart_rx);
  LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, ARRAY_LEN(usart_rx));
  
  
  
  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(USART1_IRQn);
  
  /* USER CODE BEGIN USART1_Init 1 */
  
  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_EnableDMAReq_RX(USART1);
  
  LL_USART_EnableIT_IDLE(USART1);
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
  
  
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */
  
  /* USER CODE END USART1_Init 2 */
  
}

void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim)             
{
  uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
  static uint8_t first_time = 0;
  if(!first_time)
  {
    encoder_value ->velocity = 0;
    first_time = 1;
  }
  else
  {
    if(temp_counter == encoder_value ->last_counter_value)
    {
      encoder_value ->velocity = 0;
      
    }
    else if(temp_counter > encoder_value ->last_counter_value)
    {
      if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
      {
        
        encoder_value ->direction = -1;
        encoder_value ->velocity = -encoder_value ->last_counter_value -
          (__HAL_TIM_GET_AUTORELOAD(htim)-temp_counter);
      }
      else
      {
        encoder_value ->direction = 1;
        encoder_value ->velocity = temp_counter -                             
          encoder_value ->last_counter_value;
      }
    }
    else
    {
      if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
      {
        encoder_value ->direction = -1;
	encoder_value ->velocity = temp_counter -                           
          encoder_value ->last_counter_value;
      }
      else
      {
        encoder_value ->direction = 1;
	encoder_value ->velocity = temp_counter +
          (__HAL_TIM_GET_AUTORELOAD(htim) -                                   
           encoder_value ->last_counter_value);
      }
    }
  }
  encoder_value ->position += encoder_value ->velocity;
  encoder_value ->last_counter_value = temp_counter;                                         
}


//array to integer function
int myAtoi(char *str){
  int res = 0; // Initialize result
  
  // Iterate through all characters of input string and
  // update result
  for (int i = 0; str[i] != '\0'; ++i)
    res = res*10 + str[i] - '0';
  
  // return result.
  return res;
}

int degree2pulse(float degree,int gearRatio,int motorSingleRotation){
  int pulse = 0;
  pulse = (int)((((double)degree)/360.00)* ((double)motorSingleRotation) * (double)gearRatio );
  
  return pulse;
}

float degreeCommandCal(float command,int sign){
  float realCommand = 0;
  if(sign == 1){
    realCommand = command * -1;
    return realCommand;
  }else{
    realCommand = command;
    return realCommand;
    
    
  }
  
  
}

//device and eeprom init function
void Device_Init(void){
  EE_ReadVariable(VirtAddVarTab[0], &virginity_bit);//reading fisrt boot
  
  if(virginity_bit!=1 ){
    
    EE_WriteVariable(VirtAddVarTab[0],1 );//virginity_bit
    EE_WriteVariable(VirtAddVarTab[1],0 );//motor 1 sign
    EE_WriteVariable(VirtAddVarTab[2],0 );//motor1 angle
    
  }
  ///////////////////////////////////////////
  
  EE_ReadVariable(VirtAddVarTab[0],&virginity_bit);
  EE_ReadVariable(VirtAddVarTab[1],&motor1CurrentsignEE);
  EE_ReadVariable(VirtAddVarTab[2],&motor1CurrentPosEE);
  
}


//eeprom update
void EEPROM_UPDATE(void){
  
  
  EE_WriteVariable(VirtAddVarTab[0],virginity_bit);
  EE_ReadVariable(VirtAddVarTab[0], &virginity_bit);
  EE_WriteVariable(VirtAddVarTab[1],motor1CurrentsignEE);
  EE_ReadVariable(VirtAddVarTab[1],&motor1CurrentsignEE);
  EE_WriteVariable(VirtAddVarTab[2],motor1CurrentPosEE);
  EE_ReadVariable(VirtAddVarTab[2],&motor1CurrentPosEE);
  
  
  
  
}
//eeprom update END

//calculate pulse difference
float calculatingDifferencePulse(float currentdegree,float goaldegree,int currentSign,int goalSign){
  float degreeDifference = 0;
  //make the numbers correct with their sign 
  //subtract goal position form current position 
  //return the value
  
  if(currentSign == 1){
    
    currentdegree = currentdegree * -1;
    
  }
  if(goalSign == 1){
    goaldegree = goaldegree * -1;
  }
  
  degreeDifference = goaldegree - currentdegree;
  
  
  return degreeDifference;
}
//calculate pulse difference END

int calculatePWMFrequency(int PSC,int ARR,int TimerClock){
  int PWMFrequency = 0;
  PWMFrequency = TimerClock / ((PSC + 1) * (ARR + 1));
  
  
  return PWMFrequency ; 
}

int calculatePWMARR(int PSC,int InputFrequency,int TimerClock){
  int ARR = 0;
  ARR = TimerClock / ((PSC + 1) * InputFrequency);
  
  
  return ARR ; 
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
* @brief  The application entry point.
* @retval int
*/

int main(void)
{
  
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */
  
  /* MCU Configuration--------------------------------------------------------*/
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */
  
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_USB_Init();
  /* USER CODE BEGIN 2 */
  HAL_FLASH_Unlock();
  flash_state = EE_Init();
  Device_Init();
  /* USER CODE END 2 */
  
  /* Init scheduler */
  osKernelInitialize();
  
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */
  
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */
  
  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  
  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  
  /* Create the thread(s) */
  /* creation of ModbusTCPTask */
  ModbusTCPTaskHandle = osThreadNew(ModbusTCPTaskFunc, NULL, &ModbusTCPTask_attributes);
  
  /* creation of MotorControlTas */
  MotorControlTasHandle = osThreadNew(MotorControlTaskFunc, NULL, &MotorControlTas_attributes);
  
  /* creation of ModbusRTUTask */
  ModbusRTUTaskHandle = osThreadNew(ModbusRTUTaskFunc, NULL, &ModbusRTUTask_attributes);
  
  /* creation of GovernerTask */
  GovernerTaskHandle = osThreadNew(GovernerTaskFunc, NULL, &GovernerTask_attributes);
  
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
  
  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
  
  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
* @brief System Clock Configuration
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
* @brief SPI1 Initialization Function
* @param None
* @retval None
*/
static void MX_SPI1_Init(void)
{
  
  /* USER CODE BEGIN SPI1_Init 0 */
  
  /* USER CODE END SPI1_Init 0 */
  
  /* USER CODE BEGIN SPI1_Init 1 */
  
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  
  /* USER CODE END SPI1_Init 2 */
  
}

/**
* @brief TIM2 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM2_Init(void)
{
  
  /* USER CODE BEGIN TIM2_Init 0 */
  
  /* USER CODE END TIM2_Init 0 */
  
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  
  /* USER CODE BEGIN TIM2_Init 1 */
  
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  
  /* USER CODE END TIM2_Init 2 */
  
}

/**
* @brief TIM5 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM5_Init(void)
{
  
  /* USER CODE BEGIN TIM5_Init 0 */
  
  /* USER CODE END TIM5_Init 0 */
  
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  
  /* USER CODE BEGIN TIM5_Init 1 */
  
  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  
  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);
  
}


/**
* @brief USB_OTG_FS Initialization Function
* @param None
* @retval None
*/
static void MX_USB_OTG_FS_USB_Init(void)
{
  
  /* USER CODE BEGIN USB_OTG_FS_Init 0 */
  
  /* USER CODE END USB_OTG_FS_Init 0 */
  
  /* USER CODE BEGIN USB_OTG_FS_Init 1 */
  
  /* USER CODE END USB_OTG_FS_Init 1 */
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */
  
  /* USER CODE END USB_OTG_FS_Init 2 */
  
}

/**
* Enable DMA controller clock
*/
static void MX_DMA_Init(void)
{
  
  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  
  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),5, 0));
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  
}

/**
* @brief GPIO Initialization Function
* @param None
* @retval None
*/
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MOTOR_EN_Pin|MOTOR_DIR_Pin|OUTPUT_RELAY_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FLASH_CS_Pin_Pin|ETH_RST_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_COM_Pin|LED_ACK_Pin, GPIO_PIN_SET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_SET);
  
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pins : MOTOR_EN_Pin MOTOR_DIR_Pin OUTPUT_RELAY_Pin */
  GPIO_InitStruct.Pin = MOTOR_EN_Pin|MOTOR_DIR_Pin|OUTPUT_RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  /*Configure GPIO pins : PE5 PE6 PE7 PE8
  PE9 PE10 PE11 PE12
  PE13 PE14 PE15 PE0
  PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
    |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
      |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0
        |GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  
  /*Configure GPIO pins : FLASH_CS_Pin_Pin ETH_RST_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin_Pin|ETH_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /*Configure GPIO pins : PC14 PC15 PC3 PC6
  PC7 PC8 PC9 PC10
  PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_6
    |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
      |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /*Configure GPIO pin : INPUT_ALARM_Pin */
  GPIO_InitStruct.Pin = INPUT_ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INPUT_ALARM_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pins : PB0 PB1 PB10 PB14
  PB15 PB4 PB6 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_14
    |GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  /*Configure GPIO pin : INPUT3_Pin */
  GPIO_InitStruct.Pin = INPUT3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INPUT3_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : DIP1_Pin DIP2_Pin DIP3_Pin DIP4_Pin */
  GPIO_InitStruct.Pin = DIP1_Pin|DIP2_Pin|DIP3_Pin|DIP4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  /*Configure GPIO pins : PD12 PD13 PD0 PD1
  PD2 PD3 PD4 PD5
  PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1
    |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
      |GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  /*Configure GPIO pins : LED_COM_Pin LED_ACK_Pin */
  GPIO_InitStruct.Pin = LED_COM_Pin|LED_ACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  /*Configure GPIO pin : LED_FAULT_Pin */
  GPIO_InitStruct.Pin = LED_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_FAULT_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pins : PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  /*Configure GPIO pin : USART1_DIR_Pin */
  GPIO_InitStruct.Pin = USART1_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USART1_DIR_GPIO_Port, &GPIO_InitStruct);
  
  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ModbusTCPTaskFunc */
/**
* @brief  Function implementing the ModbusTCPTask thread.
* @param  argument: Not used
* @retval None
*/
/* USER CODE END Header_ModbusTCPTaskFunc */
void ModbusTCPTaskFunc(void *argument)
{
  /* init code for LWIP */
  /* USER CODE BEGIN 5 */
  
  /* Infinite loop */
  for(;;)
  {
    if(bootFlag == 1 && booted == 0 ){
      HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_RESET);
      osDelay(500);
      HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_SET);
      osDelay(500);
      
      MX_LWIP_Init();
      osDelay(100);
      booted = 1 ;
      
    }
    if(booted == 1){
      mu_Init();
      tcp_Init();
      
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_MotorControlTaskFunc */
/**
* @brief Function implementing the MotorControlTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorControlTaskFunc */
void MotorControlTaskFunc(void *argument)
{
  /* USER CODE BEGIN MotorControlTaskFunc */
  PID(&MPID, &frequency, &frequency_out, &PWMSetPoint, 0.5, 0.5, 0, _PID_P_ON_E, _PID_CD_DIRECT);
  PID_SetMode(&MPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&MPID, 5);
  PID_SetOutputLimits(&MPID, 1, 200000);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  
  
  
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    if(RTUMaster == 0){
      HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port,MOTOR_EN_Pin,motorEN);
      
      if(actionBit == 1){
        
        if(motor_start1 == 1){
          if(pulse_count1 < motor_pulse1 * 0.35){
            PWMSetPoint = PWMFrequency;
            
          }else if(pulse_count1 >= (motor_pulse1 * 0.75) && pulse_count1 < (motor_pulse1 * 0.96) ){
            PWMSetPoint = PWMFrequency * 0.7 ; 
            
          }
          else if(pulse_count1 >= (motor_pulse1 * 0.96) ){
            PWMSetPoint = PWMFrequency * 0.05 ; 
            
          }
          PID_Compute(&MPID);
          pwmARR = calculatePWMARR(0,(int)frequency_out,84000000);
          frequency = calculatePWMFrequency(0,pwmARR,84000000);
          
          
          if(pulse_count1 < motor_pulse1){
            motor1ActionBit = 1;
            setARR1(htim5, TIM_CHANNEL_1, pwmARR);  
            HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_1);
            HAL_GPIO_TogglePin(LED_COM_GPIO_Port,LED_COM_Pin);
          }
          else{
            HAL_TIM_PWM_Stop_IT(&htim5,TIM_CHANNEL_1);
            setARR1(htim5, TIM_CHANNEL_1, 0); 
            motor1ActionBit = 0;
            if(manualControlBit == 1){
              motor1ActionBit2 = 0;
              HAL_GPIO_WritePin(LED_COM_GPIO_Port,LED_COM_Pin,1);
              
              
              
            }
            
            
          }
          
          //TIM1->CNT = 0;
        }else{
          HAL_TIM_PWM_Stop_IT(&htim5,TIM_CHANNEL_1);
          
          pulse_count1 = 0;
          motor1ActionBit = 0;
          
        }
        
        
        
        
        if( manualControlBit == 1){
          
          if(motor1ActionBit2 == 0){
            manualMotorSign = 0;
            motor_pulse1 = 0;
            actionBit2 = 0;
            manualControlBit = 0;
            actionBit = 0;
            motor_start1 = 0;
            HAL_TIM_PWM_Stop_IT(&htim5,TIM_CHANNEL_1);//motor1
            actionDone2 = 1;
            g_sHoldingRegsBuf[16] = actionDone2;
            
            pulse_count1 = 0;
            manualMotorPulse = 0;
            manualMotorSign = 0;
            pwmARR = 0;
            frequency_out = 0;
            frequency = 0;
            PWMFrequency = 0;
            setARR1(htim5,TIM_CHANNEL_1,0);
            //PID_CLear(&MPID);
            
            
            
          }
        }
        else if(manualControlBit == 0){
          
          if(actionBit2 == 0){
            motor_start1 = 1;
            motor1ActionBit = 1;
            actionBit2 =1 ;
            pulse_count1 = 0 ;
          } 
          if(motor1ActionBit == 0){
            actionBit2 = 0;
            actionBit = 0;
            command1F = 0;
            motor_start1 = 0;
            HAL_TIM_PWM_Stop_IT(&htim5,TIM_CHANNEL_1);//motor1
            actionDone = 1;
            g_sHoldingRegsBuf[4] = actionDone;
            pulse_count1 = 0;
            manualMotorPulse = 0;
            manualMotorSign = 0;
            pwmARR = 0;
            frequency_out = 0;
            frequency = 0;
            PWMFrequency = 0;
            setARR1(htim5,TIM_CHANNEL_1,0);
            EE_WriteVariable(VirtAddVarTab[1],(uint16_t)sign1 );//motor 1 sign
            EE_WriteVariable(VirtAddVarTab[2],(uint16_t)command1 );//motor1 angle
            
            EE_ReadVariable(VirtAddVarTab[1],&motor1CurrentsignEE);
            EE_ReadVariable(VirtAddVarTab[2],&motor1CurrentPosEE);
            
            
            
            currentDegree1 = ((float)motor1CurrentPosEE)/100;
          }
        }
        
      } 
    }
  }
  /* USER CODE END MotorControlTaskFunc */
}

/* USER CODE BEGIN Header_ModbusRTUTaskFunc */
/**
* @brief Function implementing the ModbusRTUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ModbusRTUTaskFunc */
void ModbusRTUTaskFunc(void *argument)
{
  /* USER CODE BEGIN ModbusRTUTaskFunc */
  /* Infinite loop */
  for(;;)
  {
    
    if(RTUMaster == 1){
      if(modbusStart == 0 && booted == 1){
        ModbusMaster_begin();
        osDelay(1000);
        modbusStart = 1;
      }
      
      if(modbusStart == 1){
        
        plc_relay1 = g_sHoldingRegsBuf[9];
        plc_relay2 = g_sHoldingRegsBuf[10];
        plc_relay3 = g_sHoldingRegsBuf[11];
        plc_relay4 = g_sHoldingRegsBuf[12];
        plc_relay5 = g_sHoldingRegsBuf[13];
        plc_relay6 = g_sHoldingRegsBuf[14];
        
        result_modbus1 = ModbusMaster_readDiscreteInputs(0x02,1024,8);
        osDelay(150);
        if (result_modbus1 == 0x00)
        {
          HAL_GPIO_WritePin(LED_FAULT_GPIO_Port,LED_FAULT_Pin,(GPIO_PinState)1);
          
          discreet_input_result_modbus = ModbusMaster_getResponseBuffer(0x00);
          plc_input1 = extractBit(discreet_input_result_modbus,0);
          plc_input2 = extractBit(discreet_input_result_modbus,1);
          plc_input3 = extractBit(discreet_input_result_modbus,2);
          plc_input4 = extractBit(discreet_input_result_modbus,3);
          plc_input5 = extractBit(discreet_input_result_modbus,4);
          plc_input6 = extractBit(discreet_input_result_modbus,5);
          plc_input7 = extractBit(discreet_input_result_modbus,6);
          plc_input8 = extractBit(discreet_input_result_modbus,7);
          //g_ucDiscreteInputsBuf[0] = discreet_input_result_modbus;
          g_sHoldingRegsBuf[0] = plc_input1 ;
          g_sHoldingRegsBuf[1] = plc_input2 ;
          g_sHoldingRegsBuf[2] = plc_input3 ;
          g_sHoldingRegsBuf[3] = plc_input4 ;
          g_sHoldingRegsBuf[4] = plc_input5 ;
          g_sHoldingRegsBuf[5] = plc_input6 ;
          g_sHoldingRegsBuf[6] = plc_input7 ;
          g_sHoldingRegsBuf[7] = plc_input8 ;
          ModbusMaster_writeSingleCoil(0x02,1280,plc_relay1);
          osDelay(150);
          ModbusMaster_writeSingleCoil(0x02,1281,plc_relay2);
          osDelay(150);
          ModbusMaster_writeSingleCoil(0x02,1282,plc_relay3);
          osDelay(150);
          ModbusMaster_writeSingleCoil(0x02,1283,plc_relay4);
          osDelay(150);
          ModbusMaster_writeSingleCoil(0x02,1284,plc_relay5);
          osDelay(150);
          ModbusMaster_writeSingleCoil(0x02,1285,plc_relay6);
          osDelay(150);
          
        }else{
          HAL_GPIO_WritePin(LED_FAULT_GPIO_Port,LED_FAULT_Pin,(GPIO_PinState)0);
          
        }
        osDelay(150);
        
        
        
        
      }
      
      
    }
    osDelay(5);
  }
  /* USER CODE END ModbusRTUTaskFunc */
}

/* USER CODE BEGIN Header_GovernerTaskFunc */
/**
* @brief Function implementing the GovernerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GovernerTaskFunc */
void GovernerTaskFunc(void *argument)
{
  /* USER CODE BEGIN GovernerTaskFunc */
  W25qxx_Init();
  osDelay(200);
  spiflash_ID = W25qxx_ReadID();
  if(bootFlag == 0){
    osDelay(1000);
    dip1 = HAL_GPIO_ReadPin(DIP1_GPIO_Port,DIP1_Pin);
    dip2 = HAL_GPIO_ReadPin(DIP2_GPIO_Port,DIP2_Pin);
    dip3 = HAL_GPIO_ReadPin(DIP3_GPIO_Port,DIP3_Pin);
    dip4 = HAL_GPIO_ReadPin(DIP4_GPIO_Port,DIP4_Pin);
    address = dip1|(dip2<<1)|(dip3<<2)|(dip4<<3);
    ipLastPart = address + 100;
    MACAddrUSER = address ;
    bootFlag = 1;
    if(address == 15){
      RTUMaster = 1;
    }else 
    {
      RTUMaster = 0 ;
    }
  }  
  /* Infinite loop */
  
  
  /* list of registers used in the process of moving motor / reading plc input and outputs
  if address is between 0 to 8
  holding register 0 is command Update Bit //this is read/write , computer writes to activate and device write 0 to end
  holding register 1 is sign of the command 1 is negative and 0 is positive
  holding register 2 is degree of command multiplied to 100 max is 180
  holding register 3 is speed of the command multiplied by 10 in hz max is 20000
  holding register 4 is command is done should be zeroed by the upper controller 
  holding register 5 is homing sesnsor state
  holding register 6 is Drive Fault state
  holding register 7 is hw reduction value between 1 and 200 
  holding register 8 is encoder sign
  holding register 9 is encoder value in degree multiplied by 100
  holding register 10 is manual command update bit
  holding register 11 is manual command sign
  holding register 12 is manual command pulse 1
  holding register 13 is manual command pulse 2
  holding register 14 is manual command speed
  holding register 15 is manual command EEPROM RESET
  holding register 16 is manual command Done
  
  holding register 18 is current sign eeprom
  holding register 19 is current position eeprom
  
  
  
  
  
  
  if address is 15
  holding register 0 is PLC Input 0
  holding register 1 is PLC Input 1
  holding register 2 is PLC Input 2
  holding register 3 is PLC Input 3
  holding register 4 is PLC Input 4
  holding register 5 is PLC Input 5
  holding register 6 is PLC Input 6
  holding register 7 is PLC Input 7
  
  holding register 9  is PLC Output 0 
  holding register 10 is PLC Output 1
  holding register 11 is PLC Output 2
  holding register 12 is PLC Output 3
  holding register 13 is PLC Output 4
  holding register 14 is PLC Output 5
  
  
  
  
  list of registers used in the process of moving motor / reading plc input and outputs END*/
  
  for(;;)
  {
    if(booted == 1){
      
      ///////reading dip switches in case of address change
      dip1 = HAL_GPIO_ReadPin(DIP1_GPIO_Port,DIP1_Pin);
      dip2 = HAL_GPIO_ReadPin(DIP2_GPIO_Port,DIP2_Pin);
      dip3 = HAL_GPIO_ReadPin(DIP3_GPIO_Port,DIP3_Pin);
      dip4 = HAL_GPIO_ReadPin(DIP4_GPIO_Port,DIP4_Pin);
      addressNew = dip1|(dip2<<1)|(dip3<<2)|(dip4<<3);
      if(addressNew != address){
        HAL_NVIC_SystemReset();
      }
      
      ///////reading dip switches in case of address change
      
      //updating current values in modbus
      g_sHoldingRegsBuf[18] = motor1CurrentsignEE;
      g_sHoldingRegsBuf[19] = motor1CurrentPosEE;
      //updating current values in modbus END
      
      //manual command 
      commandManualUpdateBit = g_sHoldingRegsBuf[10];
      if(commandManualUpdateBit == 1){
        commandManualUpdateBit = 0;
        g_sHoldingRegsBuf[10] = commandManualUpdateBit ;
        manualMotorSign = g_sHoldingRegsBuf[11];
        manualMotorPulse = ((g_sHoldingRegsBuf[13]<<16) | g_sHoldingRegsBuf[12]);
        manualEepromReset = g_sHoldingRegsBuf[15];
        PWMFrequency = g_sHoldingRegsBuf[14] * 10;
        if(PWMFrequency == 0 ){
          PWMFrequency = PWM_NUM1 * 10 ;
          
        }
        osDelay(10);
        
        
        if(manualEepromReset == 1){
          manualEepromReset = 0;
          eepromResetBit = 1;
          
        }
        
        
        HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port,MOTOR_DIR_Pin,(GPIO_PinState)manualMotorSign); // motor1
        osDelay(2);
        motor_pulse1 = manualMotorPulse ;
        motor_start1 = 1;
        pulse_count1 = 0;
        motor1ActionBit2 = 1;
        manualControlBit = 1;
        actionBit = 1;
        
      }
      //manual command   END
      
      //command update from Upper controller        
      commandUpdateBit = g_sHoldingRegsBuf[0]; //command update bit
      if(commandUpdateBit ==1 ){ 
        if(actionBit == 0){
          commandUpdateBit = 0;
          g_sHoldingRegsBuf[0] = commandUpdateBit; //command update bit
          EE_ReadVariable(VirtAddVarTab[1],&motor1CurrentsignEE);
          EE_ReadVariable(VirtAddVarTab[2],&motor1CurrentPosEE);
          osDelay(1);
          currentDegree1 = ((float)motor1CurrentPosEE)/100;
          command1 = g_sHoldingRegsBuf[2];
          command1F = (((command1 )/ 100.00));
          sign1 = g_sHoldingRegsBuf[1];
          PWMFrequency = g_sHoldingRegsBuf[3]*10;
          if(PWMFrequency == 0){
            PWMFrequency = PWM_NUM1 * 10 ;
          }
          commandDegreeInput1 = (int)degreeCommandCal(command1F,sign1);
          if(commandDegreeInput1<joint1UpperT || commandDegreeInput1>joint1LowerT ){
            command1OK = 1;
          }else{
            command1OK = 0;
          }
          if(command1OK == 1 ){
            degreeDifference1 = calculatingDifferencePulse(currentDegree1,command1F,motor1CurrentsignEE,sign1);
            if(degreeDifference1> 0 ){
              signDifference1 = 0;
            }else if(degreeDifference1< 0){
              signDifference1 = 1;
            }
            
            //somehow update the eeprom variables to save current robot position 
            osDelay(5);
            motor1GearRatio = g_sHoldingRegsBuf[7];
            if(motor1GearRatio == 0){
              //setting the variables based on the address    
              if(address == 0){
                motor1GearRatio = address1GearRatio;
                
              }else if(address == 1){
                motor1GearRatio = address2GearRatio;
                
              }else if(address == 2){
                motor1GearRatio = address3GearRatio; 
                
              }else if(address == 3){
                motor1GearRatio = address4GearRatio;
                
              }else if(address == 4){
                motor1GearRatio = address5GearRatio;
                
              }else if(address == 5){
                motor1GearRatio = address6GearRatio;
                
              }
              
              //setting the variables based on the address END 
              
            }
            
            motor1PulseCom = degree2pulse((fabs((double)degreeDifference1)),motor1GearRatio,motorRotationConstant);
            motor_pulse1 = motor1PulseCom ;
            HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port,MOTOR_DIR_Pin,(GPIO_PinState)signDifference1); // motor1
            osDelay(2);
            actionBit = 1 ;
          }else{
            actionBit = 0 ;
            actionBit2 = 0;
            actionBit = 0;
            command1F = 0;
            posReset = 0;
            motor_start1 = 0;
            actionDone = 1;
          }
        }   
      }
      
      //command update from Upper controller END
      
      //reset eeprom if upper controller commands
      if(eepromResetBit == 1){
        EE_WriteVariable(VirtAddVarTab[0],0 );//virginity_bit
        osDelay(50);
        Device_Init();
        eepromResetBit = 0;
        
      }
      //reset eeprom if upper controller commands END
      
      //handles flash page changes 
      if(flash_page_change == 1){
        flash_page_change = 0;
        EEPROM_UPDATE();  
      }
      //handles flash page changes END
    }
    
    osDelay(1);  
  }
  /* USER CODE END GovernerTaskFunc */
}

/**
* @brief  Period elapsed callback in non blocking mode
* @note   This function is called  when TIM6 interrupt took place, inside
* HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
* a global variable "uwTick" used as application time base.
* @param  htim : TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
    if(encoder_counter < 100){
      encoder_counter ++;
    }else if(encoder_counter >= 100){
      update_encoder(&enc_instance_mot,&htim2);
      encoder_direction = enc_instance_mot.direction;
      if(enc_instance_mot.position < 0 ){
        
        encoderPositionReal =(int)enc_instance_mot.position; 
        encoder_position = abs((int)enc_instance_mot.position);
        
      }else if(enc_instance_mot.position >=0){
        encoderPositionReal =(int)enc_instance_mot.position; 
        encoder_position = enc_instance_mot.position; 
      }
      encoder_velocity = enc_instance_mot.velocity ;
      
    }
  }
  /* USER CODE BEGIN Callback 1 */
  
  /* USER CODE END Callback 1 */
}

/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_NVIC_SystemReset();
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
