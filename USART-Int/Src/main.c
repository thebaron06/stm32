#include <stdint.h>
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"

#include "UART2.h"


#define MAX_BUF_SIZE    256

// Private variables
volatile uint32_t gSysTickCount = 0;

// Private function prototypes
static void Delay(volatile uint32_t nCount);
static void PeriphInit();


int main(void) {
    uint32_t idx = 0;
    uint16_t ret = 0;

    // BSP Init
    PeriphInit();
    UART2_Init(115200);

    UART2_SendBytes((uint8_t*)"Hallo!\r\n", 8);
    UART2_SendBytes((uint8_t*)"Hallo!\r\n", 8);
    UART2_SendBytes((uint8_t*)"Hallo!\r\n", 8);

    while (idx < 10) {
        UART2_SendByte('0'+idx++);
    }

    while (1) {
        if ( !((ret = UART2_GetByte()) & UART2_ERROR) ) {
            UART2_SendByte(ret);
        }
    }

	return 0;
}

static void PeriphInit() {
	GPIO_InitTypeDef  GPIO_InitStructure;

	// ---------- SysTick timer -------- //
	if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1){};
	}

    // ---------- LEDs -------- //
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    // Pull down is already assembled on board
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*
 * Called from systick handler
 */
void TimeDelay_Handler() {
	if (gSysTickCount) {
		gSysTickCount--;
	}
}

/*
 * Delay a number of systick cycles (1ms)
 */
static void Delay(volatile uint32_t nCount) {
	gSysTickCount = nCount;
	while(gSysTickCount);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
    while (1);
}
#endif


