#ifndef UART2_H
#define UART2_H

#include <stdint.h>

#define UART2_SUCCESS       (0)

#define UART2_ERROR         (0xFF00)    // you can proof any error occurred: if (UART2_xxx() & UART2_ERROR) ...
#define UART2_PARERROR      (0x0F00)    // general parameter error (UART2_xxx() & UART2_PARERROR) --> param error
#define UART2_ERRNULLPTR    (0x0100)
#define UART2_ERRBAUD       (0x0200)
#define UART2_BUFFULL       (0x1000)
#define UART2_BUFEMPTY      (0x2000)


#define UART2_RX_BUFFER_SIZE    128
#define UART2_TX_BUFFER_SIZE    128


uint16_t UART2_Init(uint32_t Baudrate);

uint16_t UART2_SendByte(uint8_t Data);
uint16_t UART2_SendBytes(uint8_t *Data, uint32_t Length);
uint16_t UART2_FlushTxBuffer();
void UART2_WaitUntilAllBytesTransmitted();

// you can test for errors in the upper nibble of return value!
uint16_t UART2_GetByte();
uint16_t UART2_FlushRxBuffer();

// Interrupt routine
void USART2_IRQHandler();


#endif // UART2_H
