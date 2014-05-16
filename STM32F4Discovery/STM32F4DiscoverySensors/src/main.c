#include "main.h"
#include "UART2.h"
#include "Debug_Interface.h"

volatile uint32_t gDelayTimer;
void Delay(volatile uint32_t TimeMs);
static void InitPeripherals();

uint8_t Buffer[4];
volatile int8_t XOffset;
volatile int8_t YOffset;
/* Click detection */
volatile uint8_t SingleClickDetect = 0x00;
extern uint8_t ClickReg;    /* Defined in stm32f4xx_it.c */

volatile LIS3DSH_OutXYZTypeDef axes, off;

int main(void) {
    InitPeripherals();
    debug_Init();
    gDelayTimer = 0x00;

    axes.x = axes.y = axes.z = 0;
    off.x = off.y = off.z = 0;

    STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);

    UART2_Init(460000);     // don't ask why...
    STM_EVAL_LEDOn(LED3);
    STM_EVAL_LEDOn(LED4);


    UART2_SendBytes((uint8_t*)"alive", 5);
    while (1) {
        if (STM_EVAL_PBGetState(BUTTON_USER)) {
            Delay(5);   // wait 50 ms
            if (STM_EVAL_PBGetState(BUTTON_USER))
                break;
        }
    }

    STM_EVAL_LEDOn(LED5);
    STM_EVAL_LEDOn(LED6);

    // first time read to get current sensor values
    // this will be the offset for detecting relative
    // changes only
    lis3dsh_ReadAxes((LIS3DSH_OutXYZTypeDef*)&off);

    while (1) {
        lis3dsh_ReadAxes((LIS3DSH_OutXYZTypeDef*)&axes);
        int x = axes.x - off.x;
        int y = axes.y - off.y;
        int z = axes.z - off.z;
        debug_printformatted("X = %8d\tY = %8d\tZ = %8d\n", x, y, z);

        Delay(8);  // wait for 8 * 10 = 80ms
        off = axes;     // update offsets
    }
}

static void InitPeripherals() {
    /* SysTick every 1 ms */
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);

    /* SysTick every 10 ms */
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);

    /* LEDs */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* Replaces the calls to 'STM_EVAL_LEDInit()' */
    GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    // Pull down is already assembled on board
	GPIO_Init(GPIOD, &GPIO_InitStructure);

    lis3dsh_init();
    lis3dsh_set_OutputDataRate(LIS3DSH_ODR_100_HZ);
    lis3dsh_AxesEnable(LIS3DSH_XYZ_ENABLE);
}

void Delay(volatile uint32_t TimeMs) {
    gDelayTimer = TimeMs;
    while(gDelayTimer != 0);
}

void TimingDelay_Handler() {
    if (gDelayTimer != 0x00) {
        gDelayTimer--;
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
    while (1) {}
}
#endif
