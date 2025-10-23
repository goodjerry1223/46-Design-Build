#ifndef WIRLESS_H
#define WIRLESS_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// 初始化无线串口（USART3，PC10/PC11）
void Wirless_Init(UART_HandleTypeDef *huart);

// 发送字符串到无线串口
typedef enum {
    WIRLESS_OK = 0,
    WIRLESS_ERROR
} Wirless_Status_t;

Wirless_Status_t Wirless_SendString(UART_HandleTypeDef *huart, const char *str);

#ifdef __cplusplus
}
#endif

#endif // WIRLESS_H 
