#include "stdint.h"
#include "stdbool.h"
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/timer.h"

#define LED1 GPIO_PIN_0        // PF0
#define USER_BTN GPIO_PIN_1    // PF1
#define TIMER_100HZ GPIO_PIN_0 // PD0

#define ACCEL_SDA GPIO_PIN_3 // PB3
#define ACCEL_SCL GPIO_PIN_2 // PB2
#define ACCEL_ADDR 0x68      // GY521 Address
#define WHO_AM_I_ADDR 0x75   // MPU6050's register containing address
#define PWR_MGMT_ADDR 0x6B   // Power Management Register

int8_t data;
int32_t counter = 0;

void button_interrupt(void);
int8_t read_byte_I2C0(uint8_t addr);
void write_byte_I2C0(uint8_t reg, uint8_t data);

void timer100hz_interrupt(void){
    uint32_t status = TimerIntStatus(TIMER0_BASE, true);
    if ((status & TIMER_TIMA_TIMEOUT) == TIMER_TIMA_TIMEOUT){
        GPIOPinWrite(GPIO_PORTF_BASE, LED1, ~GPIOPinRead(GPIO_PORTF_BASE, LED1));
        data = read_byte_I2C0(WHO_AM_I_ADDR);
    }
    TimerIntClear(TIMER0_BASE, status);
}

int main(void)
{
    uint32_t g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                        SYSCTL_OSC_MAIN |
                                        SYSCTL_USE_PLL |
                                        SYSCTL_CFG_VCO_480), 120000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));

    // LED 1 Configuration
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED1);
    GPIOPinWrite(GPIO_PORTF_BASE, LED1, 0x00);

    // Interrupt button
    GPIOPadConfigSet(GPIO_PORTF_BASE, USER_BTN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(GPIO_PORTF_BASE, USER_BTN, GPIO_FALLING_EDGE);
    GPIOIntRegister(GPIO_PORTF_BASE, button_interrupt);
    GPIOIntEnable(GPIO_PORTF_BASE, USER_BTN);

    // Accelerometer I2C
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, ACCEL_SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, ACCEL_SCL);
    I2CMasterInitExpClk(I2C0_BASE, g_ui32SysClock, true);

    // Timer 100 Hz
    GPIOPinConfigure(GPIO_PD0_T0CCP0);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, TIMER_100HZ);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 12000000);  // 100 Hz = clock * periodo
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerIntRegister(TIMER0_BASE, TIMER_A, timer100hz_interrupt);
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    write_byte_I2C0(PWR_MGMT_ADDR, 0x01);
    SysCtlDelay(g_ui32SysClock/12000000);
    data = read_byte_I2C0(WHO_AM_I_ADDR);

    while(1){
    }

	return 0;
}

void button_interrupt(void){
    uint32_t status = GPIOIntStatus(GPIO_PORTF_BASE, true);
    if ((status & USER_BTN) == USER_BTN){
        GPIOPinWrite(GPIO_PORTF_BASE, LED1, ~GPIOPinRead(GPIO_PORTF_BASE, LED1));
    }
    data = read_byte_I2C0(WHO_AM_I_ADDR);
    GPIOIntClear(GPIO_PORTF_BASE,status);
}

int8_t read_byte_I2C0(uint8_t addr){
    I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, false);
    I2CMasterDataPut(I2C0_BASE, addr);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(I2CMasterBusBusy(I2C0_BASE) | I2CMasterBusy(I2C0_BASE));

    int i = 1000;
    do{
        i--;
    }while(i>0);

    if (I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE){
        return 0;
    }

    else
        return I2CMasterDataGet(I2C0_BASE);
}


void write_byte_I2C0(uint8_t reg, uint8_t data){
    I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, ACCEL_ADDR, false);
    I2CMasterDataPut(I2C0_BASE, data);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusBusy(I2C0_BASE));

}

