#include "stm32f4xx_hal.h"
#include "wirless.h"
#include <string.h>
#include <stdio.h>
#include "main.h"

extern UART_HandleTypeDef huart3;
void MX_USART3_UART_Init(void);

// 初始化无线串口
void Wirless_Init(UART_HandleTypeDef *huart)
{
    MX_USART3_UART_Init();
}

// 发送字符串到无线串口（阻塞式）
Wirless_Status_t Wirless_SendString(UART_HandleTypeDef *huart, const char *str)
{
    if (HAL_UART_Transmit(huart, (uint8_t *)str, strlen(str), 100) == HAL_OK)
        return WIRLESS_OK;
    else
        return WIRLESS_ERROR;
}

// 重定向printf到串口
#ifdef __GNUC__
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 100);
    return ch;
}
#else
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 100);
    return ch;
}
#endif

