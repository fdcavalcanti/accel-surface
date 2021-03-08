#include "stdint.h"
#include "stdbool.h"
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#define LED1 GPIO_PIN_0

int main(void)
{
    uint32_t g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                        SYSCTL_OSC_MAIN |
                                        SYSCTL_USE_PLL |
                                        SYSCTL_CFG_VCO_480), 120000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED1);

    while(1){
        GPIOPinWrite(GPIO_PORTF_BASE, LED1, 0x01);
        SysCtlDelay(g_ui32SysClock/3);
        GPIOPinWrite(GPIO_PORTF_BASE, LED1, 0x00);
        SysCtlDelay(g_ui32SysClock/3);
    }

	return 0;
}
