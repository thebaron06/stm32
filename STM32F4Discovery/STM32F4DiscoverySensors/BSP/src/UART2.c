#include "UART2.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"

typedef struct {
    uint32_t First;
    uint32_t Last;
    uint32_t Length;
    volatile uint8_t *Data;
} RingBuffer;

static volatile uint8_t UART2_RxBufferData[UART2_RX_BUFFER_SIZE];
static volatile uint8_t UART2_TxBufferData[UART2_TX_BUFFER_SIZE];

static volatile RingBuffer UART2_RxBuffer = {0, 0, UART2_RX_BUFFER_SIZE, 0};
static volatile RingBuffer UART2_TxBuffer = {0, 0, UART2_TX_BUFFER_SIZE, 0};


uint16_t UART2_Init(uint32_t Baudrate) {
	//____ User-Cfg__________________________________________________
	USART_InitTypeDef const uis = {
        .USART_BaudRate              = Baudrate,
        .USART_HardwareFlowControl   = USART_HardwareFlowControl_None,
        .USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx,
        .USART_Parity                = USART_Parity_No,
        .USART_StopBits              = USART_StopBits_1,
        .USART_WordLength            = USART_WordLength_8b
	};
	// configure NVIC
    NVIC_InitTypeDef const NVIC_InitStructure = {
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelPreemptionPriority = 0,   // highest prio within group
        .NVIC_IRQChannel = USART2_IRQn,
        .NVIC_IRQChannelCmd = ENABLE
    };
	// USART2: PA2: TX, PA3: RX
	GPIO_InitTypeDef const GPIO_InitStructure = {
        .GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3,
        .GPIO_Mode = GPIO_Mode_AF,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_PuPd = GPIO_PuPd_NOPULL
	};
    //______________________________________________________________

    if (Baudrate == 0) {
        return UART2_ERRBAUD;
    }

    UART2_RxBuffer.First = 0;
    UART2_RxBuffer.Last = 0;
    UART2_RxBuffer.Length = UART2_RX_BUFFER_SIZE;
    UART2_RxBuffer.Data = UART2_RxBufferData;

    UART2_TxBuffer.First = 0;
    UART2_TxBuffer.Last = 0;
    UART2_TxBuffer.Length = UART2_TX_BUFFER_SIZE;
    UART2_TxBuffer.Data = UART2_TxBufferData;

    // Configure NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_Init((NVIC_InitTypeDef*)&NVIC_InitStructure);

    // Enable clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // Uart GPIO configuration
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    // Config GPIOs
	GPIO_Init(GPIOA, (GPIO_InitTypeDef*)&GPIO_InitStructure);

    USART_Init(USART2, (USART_InitTypeDef*)&uis);
    USART_Cmd(USART2, ENABLE);
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    return UART2_SUCCESS;
}

uint16_t UART2_SendByte(uint8_t Data) {
    return UART2_SendBytes(&Data, 1);
}

uint16_t UART2_SendBytes(uint8_t *Data, uint32_t Length) {
    uint32_t i = 0;
    uint16_t ret = UART2_SUCCESS;

    if (!Data) return UART2_ERRNULLPTR;
    if (!Length) return UART2_PARERROR;
    if (Length > UART2_TX_BUFFER_SIZE) return UART2_PARERROR;

    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);

    while (i < Length) {
        uint32_t first = UART2_TxBuffer.First;
        uint32_t last = UART2_TxBuffer.Last;
        uint32_t len = UART2_TxBuffer.Length;
        uint32_t idx = (first + 1) % len;

        if (idx == last) {
            ret = UART2_BUFFULL;
            break;
        }

        UART2_TxBuffer.Data[first] = Data[i++];
        UART2_TxBuffer.First = idx;
    }

    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    return ret;
}

uint16_t UART2_FlushTxBuffer() {
    NVIC_DisableIRQ(USART2_IRQn);
    UART2_TxBuffer.First = UART2_TxBuffer.Last = 0;
    NVIC_EnableIRQ(USART2_IRQn);
    return UART2_SUCCESS;
}

void UART2_WaitUntilAllBytesTransmitted() {
    while (USART_GetITStatus(USART2, USART_IT_TXE) != RESET);
}

// you can test for errors in the upper nibble of return value!
uint16_t UART2_GetByte() {
    uint16_t ret = UART2_SUCCESS;

    NVIC_DisableIRQ(USART2_IRQn);
    if (UART2_RxBuffer.First == UART2_RxBuffer.Last) {
        ret = UART2_BUFEMPTY;
        goto Error;
    }

    ret = UART2_RxBuffer.Data[UART2_RxBuffer.Last];
    UART2_RxBuffer.Last = (UART2_RxBuffer.Last + 1) % UART2_RxBuffer.Length;

Error:
    NVIC_EnableIRQ(USART2_IRQn);
    return ret;
}

uint16_t UART2_FlushRxBuffer() {
    NVIC_DisableIRQ(USART2_IRQn);
    UART2_RxBuffer.First = UART2_RxBuffer.Last = 0;
    NVIC_EnableIRQ(USART2_IRQn);
    return UART2_SUCCESS;
}

// override usart2 irq handler
void USART2_IRQHandler() {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        // byte received
        uint16_t buf = USART_ReceiveData(USART2);
        uint32_t first = UART2_RxBuffer.First;
        uint32_t last = UART2_RxBuffer.Last;
        uint32_t len = UART2_RxBuffer.Length;
        uint32_t idx = (first + 1) % len;

        if (idx != last) {
            UART2_RxBuffer.Data[first] = (uint8_t)(buf & 0x00FF);
            UART2_RxBuffer.First = idx;
        }
        // else drop the received byte

        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
    if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
        uint32_t first = UART2_TxBuffer.First;
        uint32_t last = UART2_TxBuffer.Last;
        uint32_t len = UART2_TxBuffer.Length;

        USART_SendData(USART2, (uint16_t)(UART2_TxBuffer.Data[last] & 0x00FF));

        UART2_TxBuffer.Last = (last + 1) % len;

        if  (UART2_TxBuffer.Last == first ) {
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
        //GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
        // must not be cleared
    }
}
