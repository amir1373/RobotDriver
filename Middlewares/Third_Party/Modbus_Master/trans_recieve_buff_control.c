#include "trans_recieve_buff_control.h"
#include "ringbuffer.h"
#include "main.h"

RingBuffer  m_Modbus_Master_RX_RingBuff;
uint8_t     m_Modbus_Master_RX_Buff[128];

//extern uint16_t old_pos_uart2;
//#define DMA_RX_BUFF_SIZE_UART2           2048   /* DMA buffer size, make it aligned to 2 bytes */
extern uint8_t modbus_connect_buffer[128];
//extern void usart_transmit_data1(const void* data, uint16_t len);
//extern UART_HandleTypeDef huart2;

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))
extern uint8_t usart_rx[128];
extern size_t pos;

extern uint16_t old_pos_uart1;
extern uint16_t position_uart1;
extern void usart_send_string(const char* str) ;
extern void usart_process_data(const void* data, size_t len); 




/**
* @brief  初始化中断接收的的ringbuffer环形队列配置,中断接收的字节都用m_Modbus_Master_RX_RingBuff该结构体指针进行管理
* @param
* @note
* @retval void
* @author xiaodaqi
*/
uint8_t Modbus_Master_RB_Initialize(void)
{
  
  rbInitialize(&m_Modbus_Master_RX_RingBuff, m_Modbus_Master_RX_Buff, sizeof(m_Modbus_Master_RX_Buff));
  return 1 ;
}


/**
* @brief  清除环形队列
* @param
* @note
* @retval void
* @author xiaodaqi
*/
uint8_t Modbus_Master_Rece_Flush(void)
{
  rbClear(&m_Modbus_Master_RX_RingBuff);
  return 0;
}
/**
* @brief  判断GPS的ringbuffer里面是否有尚未处理的字节
* @param
* @note
* @retval void
* @author xiaodaqi
*/
uint8_t Modbus_Master_Rece_Available(void)
{
  
  if(m_Modbus_Master_RX_RingBuff.flagOverflow==1)
  {
    rbClear(&m_Modbus_Master_RX_RingBuff);
  }
  return !rbIsEmpty(&m_Modbus_Master_RX_RingBuff);
}

/****************************************************************************************************/

/**
* @brief  获取接收寄存器里面的数值
* @param
* @note
* @retval void
* @author xiaodaqi
*/
//uint8_t data_buff_test[90];
//uint8_t i_add_test = 0 ;
uint8_t Modbus_Master_GetByte(uint8_t  *getbyte)
{
 // uint8_t data;
  /*
//  if(HAL_UART_Receive (&huart2 ,(uint8_t *)getbyte,1,0x01) != HAL_OK )
    if(HAL_UART_Receive (&huart2 ,(uint8_t *)getbyte,1,0x01) != HAL_OK )
  {
    return HAL_ERROR;
  }
  else
  {
    return HAL_OK;
  }
*/
//data = LL_USART_ReceiveData8(USART2);
//getbyte = &data;
 // i_add_test ++ ;
  //data_buff_test[i_add_test] = LL_USART_ReceiveData8(USART2);
//*getbyte = LL_USART_ReceiveData8(USART2);
//getbyte = &data;
    
 return 0;
}

/**
* @brief  中断处理函数，在串口接收中断中调用：将寄存器的数值压人缓冲区
* @param
* @note
* @retval void
* @author xiaodaqi
*/
uint8_t Modbus_Master_Rece_Handler(void)
{

  if(position_uart1 >=6 &&  modbus_connect_buffer[0] != 0){
  for(int i = 0; i < position_uart1; ++i)
  {
     rbPush(&m_Modbus_Master_RX_RingBuff, (uint8_t)(modbus_connect_buffer[i] & (uint8_t)0xFFU));
  }
  }
  

  
  return 0;
}

/**
* @brief  读出缓冲区的数据
* @param
* @note
* @retval void
* @author xiaodaqi
*/
uint8_t Modbus_Master_Read(void)
{
  uint8_t cur =0xff;
  if( !rbIsEmpty(&m_Modbus_Master_RX_RingBuff))
  {
    cur = rbPop(&m_Modbus_Master_RX_RingBuff);
  }
  return cur;
}

/**
* @brief  将数据包发送出去
* @param
* @note
* @retval void
* @author xiaodaqi
*/
uint8_t Modbus_Master_Write(uint8_t *buf,uint8_t length)
{
  //  HAL_GPIO_WritePin(GPIOA, USART2_DIR_Pin, GPIO_PIN_SET);
  /*
 // if(HAL_UART_Transmit(&huart2 ,(uint8_t *)buf,length,0xff))
  {
    //  HAL_GPIO_WritePin(GPIOA, USART2_DIR_Pin, GPIO_PIN_RESET);
    return HAL_ERROR;
    
  }
  else
  {
    //HAL_GPIO_WritePin(GPIOA, USART2_DIR_Pin, GPIO_PIN_RESET);
    return HAL_OK;
    
  }
*/
  
 // usart_transmit_data1((uint8_t *)buf, length);
  //usart_send_string((uint8_t *)buf);
 
  usart_process_data((uint8_t *)buf, length);
  return HAL_OK;
}

/**
* @brief  1ms周期的定时器
* @param
* @note
* @retval void
* @author xiaodaqi
*/
uint32_t Modbus_Master_Millis(void)
{
  return HAL_GetTick();
}
