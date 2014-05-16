#include "UART3.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"

typedef struct {
    uint32_t First;
    uint32_t Last;
    uint32_t Length;
    volatile uint8_t *Data;
} RingBuffer;

static volatile uint8_t UART3_RxBufferData[UART3_RX_BUFFER_SIZE];
static volatile uint8_t UART3_TxBufferData[UART3_TX_BUFFER_SIZE];

static volatile RingBuffer UART3_RxBuffer = {0, 0, UART3_RX_BUFFER_SIZE, 0};
static volatile RingBuffer UART3_TxBuffer = {0, 0, UART3_TX_BUFFER_SIZE, 0};


uint16_t UART3_Init(uint32_t Baudrate) {
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
        .NVIC_IRQChannel = USART3_IRQn,
        .NVIC_IRQChannelCmd = ENABLE
    };
	// USART3: PD8: TX, PD9: RX
	GPIO_InitTypeDef const GPIO_InitStructure = {
        .GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9,
        .GPIO_Mode = GPIO_Mode_AF,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_Speed = GPIO_Speed_50MHz,
        .GPIO_PuPd = GPIO_PuPd_NOPULL
	};

	/* Be aware to change the PinSource_X and GPIOX variables/defines
	 * too. Those are located a few lines below.
	 */

    //______________________________________________________________

    if (Baudrate == 0) {
        return UART3_ERRBAUD;
    }

    UART3_RxBuffer.First = 0;
    UART3_RxBuffer.Last = 0;
    UART3_RxBuffer.Length = UART3_RX_BUFFER_SIZE;
    UART3_RxBuffer.Data = UART3_RxBufferData;

    UART3_TxBuffer.First = 0;
    UART3_TxBuffer.Last = 0;
    UART3_TxBuffer.Length = UART3_TX_BUFFER_SIZE;
    UART3_TxBuffer.Data = UART3_TxBufferData;

    // Configure NVIC
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_Init((NVIC_InitTypeDef*)&NVIC_InitStructure);

    // Enable clocks
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // Uart GPIO configuration
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

    // Config GPIOs
	GPIO_Init(GPIOD, (GPIO_InitTypeDef*)&GPIO_InitStructure);

    USART_Init(USART3, (USART_InitTypeDef*)&uis);
    USART_Cmd(USART3, ENABLE);
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    return UART3_SUCCESS;
}

uint16_t UART3_SendByte(uint8_t Data) {
    return UART3_SendBytes(&Data, 1);
}

uint16_t UART3_SendBytes(uint8_t *Data, uint32_t Length) {
    uint32_t i = 0;
    uint16_t ret = UART3_SUCCESS;

    if (!Data) return UART3_ERRNULLPTR;
    if (!Length) return UART3_PARERROR;
    if (Length > UART3_TX_BUFFER_SIZE) return UART3_PARERROR;

    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);

    while (i < Length) {
        uint32_t first = UART3_TxBuffer.First;
        uint32_t last = UART3_TxBuffer.Last;
        uint32_t len = UART3_TxBuffer.Length;
        uint32_t idx = (first + 1) % len;

        if (idx == last) {
            ret = UART3_BUFFULL;
            break;
        }

        UART3_TxBuffer.Data[first] = Data[i++];
        UART3_TxBuffer.First = idx;
    }

    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    return ret;
}

uint16_t UART3_FlushTxBuffer() {
    NVIC_DisableIRQ(USART3_IRQn);
    UART3_TxBuffer.First = UART3_TxBuffer.Last = 0;
    NVIC_EnableIRQ(USART3_IRQn);
    return UART3_SUCCESS;
}

void UART3_WaitUntilAllBytesTransmitted() {
    while (USART_GetITStatus(USART3, USART_IT_TXE) != RESET);
}

// you can test for errors in the upper nibble of return value!
uint16_t UART3_GetByte() {
    uint16_t ret = UART3_SUCCESS;

    NVIC_DisableIRQ(USART3_IRQn);
    if (UART3_RxBuffer.First == UART3_RxBuffer.Last) {
        ret = UART3_BUFEMPTY;
        goto Error;
    }

    ret = UART3_RxBuffer.Data[UART3_RxBuffer.Last];
    UART3_RxBuffer.Last = (UART3_RxBuffer.Last + 1) % UART3_RxBuffer.Length;

Error:
    NVIC_EnableIRQ(USART3_IRQn);
    return ret;
}

uint16_t UART3_FlushRxBuffer() {
    NVIC_DisableIRQ(USART3_IRQn);
    UART3_RxBuffer.First = UART3_RxBuffer.Last = 0;
    NVIC_EnableIRQ(USART3_IRQn);
    return UART3_SUCCESS;
}

// override USART3 irq handler
void USART3_IRQHandler() {
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        // byte received
        uint16_t buf = USART_ReceiveData(USART3);
        uint32_t first = UART3_RxBuffer.First;
        uint32_t last = UART3_RxBuffer.Last;
        uint32_t len = UART3_RxBuffer.Length;
        uint32_t idx = (first + 1) % len;

        if (idx != last) {
            UART3_RxBuffer.Data[first] = (uint8_t)(buf & 0x00FF);
            UART3_RxBuffer.First = idx;
        }
        // else drop the received byte

        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
    if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET) {
        uint32_t first = UART3_TxBuffer.First;
        uint32_t last = UART3_TxBuffer.Last;
        uint32_t len = UART3_TxBuffer.Length;

        USART_SendData(USART3, (uint16_t)(UART3_TxBuffer.Data[last] & 0x00FF));

        UART3_TxBuffer.Last = (last + 1) % len;

        if  (UART3_TxBuffer.Last == first ) {
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }
        //GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
        // must not be cleared
    }
}
