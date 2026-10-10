#ifndef TEST_BSP_UART_H
#define TEST_BSP_UART_H
#include <stdint.h>
typedef struct { int unused; } UART_HandleTypeDef;
typedef void (*UartRxCallback)(uint8_t *, uint16_t);
void UartInit(UART_HandleTypeDef *uart, UartRxCallback callback);
void UartTransmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
#endif
