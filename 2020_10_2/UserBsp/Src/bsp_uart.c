#include "bsp_uart.h"
#include "usart.h"
#include "stdio.h"
#include "remote_info.h"
#include "pc_info.h"
#include "pid.h"
#include "judgement_info.h"
#include "sys_config.h"

/*-----------------------      允许使用printf        -----------------------*/
/*-----------------------      允许使用printf        -----------------------*/

#if 1
#pragma import(__use_no_semihosting)             
//                
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//   
void _sys_exit(int x) 
{ 
	x = x; 
} 
//
int fputc(int ch, FILE *f)
{ 	
//	while((USART6->SR&0X40)==0);//Ñ­»··¢ËÍ,Ö±µ½·¢ËÍÍê±Ï   
//	USART6->DR = (uint8_t) ch; 
  HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1,0xffff);
	return ch;
	
}
#endif 
/*---------------------------------------------------------------------------*/

/**
  * @brief   clear idle it flag after uart receive a frame data
  * @param   uart IRQHandler id
  * @usage   call in uart_receive_handler() function
  */
static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
  /* clear idle it flag avoid idle interrupt all the time */
  __HAL_UART_CLEAR_IDLEFLAG(huart);
  
  /* handle received data in idle interrupt */
  if (huart == &DBUS_HUART)
  {
    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);
    
    /* handle dbus data dbus_buf from DMA */
    if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
    {
      rc_callback_handler(&rc, dbus_buf);
      //err_detector_hook(REMOTE_CTRL_OFFLINE);
    }

    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);

  }
  else if (huart == &COMPUTER_HUART)
  {
		__HAL_DMA_DISABLE(huart->hdmarx);
//		pc_data_unpack();
		debug_pid(&pid_yaw);
		__HAL_DMA_SET_COUNTER(huart->hdmarx, 1024);
    __HAL_DMA_ENABLE(huart->hdmarx);
  }
  else if (huart == &JUDGE_HUART)
	{
		uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx); 
		__HAL_DMA_DISABLE(huart->hdmarx);
		judgementDataHandler();
		__HAL_DMA_CLEAR_FLAG(huart->hdmarx, DMA_FLAGS);
		__HAL_DMA_SET_COUNTER(huart->hdmarx, 1024);
    __HAL_DMA_ENABLE(huart->hdmarx);
		
	}
	else if (huart == &DEBUG_HUART)
  {
		__HAL_DMA_DISABLE(huart->hdmarx);
//		debug_pid(&pid_pit);
		__HAL_DMA_SET_COUNTER(huart->hdmarx, 1024);
    __HAL_DMA_ENABLE(huart->hdmarx);
  }
	else
  {
  }
}

/**
  * @brief   callback this function when uart interrupt 
  * @param   uart IRQHandler id
  * @usage   call in uart handler function USARTx_IRQHandler()
  */
void uart_receive_handler(UART_HandleTypeDef *huart)
{  
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && 
      __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
  {
    uart_rx_idle_callback(huart);
  }
}
/**
  * @brief   enable global uart it and do not use DMA transfer done it
  * @param   uart IRQHandler id, receive buff, buff size
  * @retval  set success or fail
  */
static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
  uint32_t tmp1 = 0;

  tmp1 = huart->RxState;
  if (tmp1 == HAL_UART_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0))
    {
        return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(huart);

    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;
    
    huart->ErrorCode  = HAL_UART_ERROR_NONE;

    /* Enable the DMA Stream */
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,
                  (uint32_t)pData, Size);

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit
    in the UART CR3 register */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
/**
  * @brief   initialize uart device 
  */
void dbus_uart_init(void)
{
  //open uart idle it
  __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
  __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);

  UART_Receive_DMA_No_IT(&DBUS_HUART, dbus_buf, DBUS_MAX_LEN);
}
void judgement_uart_init(void)
{
  //open uart idle it
  __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
  __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
  UART_Receive_DMA_No_IT(&JUDGE_HUART, judge_buf, 1024);
}
void computer_uart_init(void)
{
  //open uart idle it
  __HAL_UART_CLEAR_IDLEFLAG(&COMPUTER_HUART);
  __HAL_UART_ENABLE_IT(&COMPUTER_HUART, UART_IT_IDLE);
  UART_Receive_DMA_No_IT(&COMPUTER_HUART, pc_buf, 1024);
}
void debug_uart_init(void)
{
  //open uart idle it
  __HAL_UART_CLEAR_IDLEFLAG(&DEBUG_HUART);
  __HAL_UART_ENABLE_IT(&DEBUG_HUART, UART_IT_IDLE);
  UART_Receive_DMA_No_IT(&DEBUG_HUART, debug_buf, 1024);
}
/**
  * @brief  Returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param  dma_stream: where y can be 1 or 2 to select the DMA and x can be 0
  *          to 7 to select the DMA Stream.
  * @retval The number of remaining data units in the current DMAy Streamx transfer.
  */
uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{
  /* Return the number of remaining data units for DMAy Streamx */
  return ((uint16_t)(dma_stream->NDTR));
}

