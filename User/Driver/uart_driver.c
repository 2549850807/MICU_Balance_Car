#include "uart_driver.h"

int Uart_Printf(UART_HandleTypeDef *huart, const char *format, ...)
{
	char buffer[512]; // 临时存储格式化后的字符串
	va_list arg;      // 处理可变参数
	int len;          // 最终字符串长度

	va_start(arg, format);
	// 安全地格式化字符串到 buffer
	len = vsnprintf(buffer, sizeof(buffer), format, arg);
	va_end(arg);

	// 通过 HAL 库发送 buffer 中的内容
	HAL_UART_Transmit(huart, (uint8_t *)buffer, (uint16_t)len, 0xFF);
	return len;
}

/* 串口 1 */
uint8_t uart1_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

uint8_t uart1_ring_buffer_input[BUFFER_SIZE]; // 环形缓冲区对应的线性数组
struct rt_ringbuffer uart1_ring_buffer; // 环形缓冲区

uint8_t uart1_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区

/* 串口 2 */
uint8_t uart2_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

uint8_t uart2_ring_buffer_input[BUFFER_SIZE]; // 环形缓冲区对应的线性数组
struct rt_ringbuffer uart2_ring_buffer; // 环形缓冲区

uint8_t uart2_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区

/* 串口 4 */
uint8_t uart4_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

uint8_t uart4_ring_buffer_input[BUFFER_SIZE]; // 环形缓冲区对应的线性数组
struct rt_ringbuffer uart4_ring_buffer; // 环形缓冲区

uint8_t uart4_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区

/* 串口 5 */
uint8_t uart5_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

uint8_t uart5_ring_buffer_input[BUFFER_SIZE]; // 环形缓冲区对应的线性数组
struct rt_ringbuffer uart5_ring_buffer; // 环形缓冲区

uint8_t uart5_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区

/* 串口 6 */
uint8_t uart6_rx_dma_buffer[BUFFER_SIZE]; // DMA 读取缓冲区

uint8_t uart6_ring_buffer_input[BUFFER_SIZE]; // 环形缓冲区对应的线性数组
struct rt_ringbuffer uart6_ring_buffer; // 环形缓冲区

uint8_t uart6_data_buffer[BUFFER_SIZE]; // 数据处理缓冲区

/* 串口空闲中断 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* 串口 1 */
    if (huart->Instance == USART1)
    {
        HAL_UART_DMAStop(huart);

        rt_ringbuffer_put(&uart1_ring_buffer, uart1_rx_dma_buffer, Size);

        memset(uart1_rx_dma_buffer, 0, sizeof(uart1_rx_dma_buffer));

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma_buffer, sizeof(uart1_rx_dma_buffer));
        
         __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
    
    /* 串口 2 */
    if (huart->Instance == USART2)
    {
        HAL_UART_DMAStop(huart);

        rt_ringbuffer_put(&uart2_ring_buffer, uart2_rx_dma_buffer, Size);

        memset(uart2_rx_dma_buffer, 0, sizeof(uart2_rx_dma_buffer));

        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_dma_buffer, sizeof(uart2_rx_dma_buffer));
        
         __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
    
    /* 串口 4 */
    if (huart->Instance == UART4)
    {
        HAL_UART_DMAStop(huart);

        rt_ringbuffer_put(&uart4_ring_buffer, uart4_rx_dma_buffer, Size);

        memset(uart4_rx_dma_buffer, 0, sizeof(uart4_rx_dma_buffer));

        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_dma_buffer, sizeof(uart4_rx_dma_buffer));
        
         __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
    }
    
    /* 串口 5 */
    if (huart->Instance == UART5)
    {
        HAL_UART_DMAStop(huart);

        rt_ringbuffer_put(&uart5_ring_buffer, uart5_rx_dma_buffer, Size);

        memset(uart5_rx_dma_buffer, 0, sizeof(uart5_rx_dma_buffer));

        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5_rx_dma_buffer, sizeof(uart5_rx_dma_buffer));
        
         __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
    }
    
    /* 串口 6 */
    if (huart->Instance == USART6)
    {
        HAL_UART_DMAStop(huart);

        rt_ringbuffer_put(&uart6_ring_buffer, uart6_rx_dma_buffer, Size);

        memset(uart6_rx_dma_buffer, 0, sizeof(uart6_rx_dma_buffer));

        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, uart6_rx_dma_buffer, sizeof(uart6_rx_dma_buffer));
        
         __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
    }
}
