#ifndef UART3_H
#define UART3_H

#include <stdint.h>

#define UART3_SUCCESS       (0)

#define UART3_ERROR         (0xFF00)    // you can proof any error occurred: if (UART3_xxx() & UART3_ERROR) ...
#define UART3_PARERROR      (0x0F00)    // general parameter error (UART3_xxx() & UART3_PARERROR) --> param error
#define UART3_ERRNULLPTR    (0x0100)
#define UART3_ERRBAUD       (0x0200)
#define UART3_BUFFULL       (0x1000)
#define UART3_BUFEMPTY      (0x2000)


#define UART3_RX_BUFFER_SIZE    256
#define UART3_TX_BUFFER_SIZE    256


uint16_t UART3_Init(uint32_t Baudrate);

uint16_t UART3_SendByte(uint8_t Data);
uint16_t UART3_SendBytes(uint8_t *Data, uint32_t Length);
uint16_t UART3_FlushTxBuffer();
void UART3_WaitUntilAllBytesTransmitted();

// you can test for errors in the upper nibble of return value!
uint16_t UART3_GetByte();
uint16_t UART3_FlushRxBuffer();

// Interrupt routine
void USART2_IRQHandler();


#endif // UART3_H
