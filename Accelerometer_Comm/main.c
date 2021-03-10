#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/timer.h"

/*
 * https://jspicer.net/2018/07/27/solution-for-i2c-busy-status-latency/
 * https://e2e.ti.com/support/microcontrollers/other/f/other-microcontrollers-forum/343532/i2c-busy--and-error-flags-on-tm4c1292
 * I2CMasterBusBusy has a bug where there's a delay in setting the busy status bit.
 * Check through registers or do a complemented while loop: (!MasterBusBusy()); (MasterBusBusy());
 */

#define LED1 GPIO_PIN_0        // PF0
#define USER_BTN GPIO_PIN_1    // PF1
#define TIMER_100HZ GPIO_PIN_0 // PD0

#define MPU_SDA GPIO_PIN_3   // PB3
#define MPU_SCL GPIO_PIN_2   // PB2
#define MPU_ADDR 0x68        // GY521 Address
#define WHO_AM_I_ADDR 0x75   // MPU6050's register containing its address
#define PWR_MGMT_ADDR 0x6B   // Power Management Register
#define ACCEL_CONFIG_ADDR 0x1C  //Accelerometer Configuration (AFSEL)
#define TEMP_ADDR 0x41      // Temeperature sensor address
#define ACC_XOUT0 0x3B      // First register for Accelerometer X

uint8_t WHO_AM_I = 0x0;
float DATA_XYZ[3] = {0,0,0};

void button_interrupt(void);
int8_t read_byte_I2C0(uint8_t addr);
void write_byte_I2C0(uint8_t reg, uint8_t data);
void timer100hz_interrupt(void);
int8_t* burst_read_sequence_I2C0(uint8_t reg_start, int n);

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
    GPIOPinTypeI2C(GPIO_PORTB_BASE, MPU_SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, MPU_SCL);
    I2CMasterInitExpClk(I2C0_BASE, g_ui32SysClock, true);
    SysCtlDelay(g_ui32SysClock/120000);
    write_byte_I2C0(PWR_MGMT_ADDR, 0x00);
    SysCtlDelay(g_ui32SysClock/120000);
    write_byte_I2C0(ACCEL_CONFIG_ADDR, 0x10);
    SysCtlDelay(g_ui32SysClock/120000);
    WHO_AM_I = read_byte_I2C0(WHO_AM_I_ADDR);

    // Timer 100 Hz
    GPIOPinConfigure(GPIO_PD0_T0CCP0);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, TIMER_100HZ);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 1200000);  // 100 Hz = clock * periodo
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerIntRegister(TIMER0_BASE, TIMER_A, timer100hz_interrupt);
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    while(1){
    }

	return 0;
}

void timer100hz_interrupt(void){
    uint32_t status = TimerIntStatus(TIMER0_BASE, true);
    if ((status & TIMER_TIMA_TIMEOUT) == TIMER_TIMA_TIMEOUT){
        GPIOPinWrite(GPIO_PORTF_BASE, LED1, ~GPIOPinRead(GPIO_PORTF_BASE, LED1));
        WHO_AM_I = read_byte_I2C0(WHO_AM_I_ADDR);
        int8_t *p = burst_read_sequence_I2C0(ACC_XOUT0, 6);
        DATA_XYZ[0] = ((p[0] << 8) + p[1])/4096.0;
        DATA_XYZ[1] = ((p[2] << 8) + p[3])/4096.0;
        DATA_XYZ[2] = ((p[4] << 8) + p[5])/4096.0;

    }

    TimerIntClear(TIMER0_BASE, status);
}


int8_t read_byte_I2C0(uint8_t addr){
    I2CMasterSlaveAddrSet(I2C0_BASE, MPU_ADDR, false);
    I2CMasterDataPut(I2C0_BASE, addr);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(!I2CMasterBusy(I2C0_BASE));
    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, MPU_ADDR, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(!I2CMasterBusy(I2C0_BASE));
    while(I2CMasterBusy(I2C0_BASE));

    if (I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE)
        return I2CMasterDataGet(I2C0_BASE);
    else
        return 0xFF;
}

void write_byte_I2C0(uint8_t reg, uint8_t data){
    I2CMasterSlaveAddrSet(I2C0_BASE, MPU_ADDR, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(!I2CMasterBusy(I2C0_BASE));
    while(I2CMasterBusy(I2C0_BASE));

    // Need to wait here a bit. Issue with I2CMasterBusy().
    int i = 0;
    do{
        i++;
    }while(i<300);

    I2CMasterSlaveAddrSet(I2C0_BASE, MPU_ADDR, false);
    I2CMasterDataPut(I2C0_BASE, data);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusBusy(I2C0_BASE));

}

int8_t* burst_read_sequence_I2C0(uint8_t reg_start, int n){
    static int8_t temp[6] = {0}; // Initialize empty array. Need to be static or when leaving the function, it becomes garbage in memory
    uint8_t counter = 0;

    I2CMasterSlaveAddrSet(I2C0_BASE, MPU_ADDR, false);
    I2CMasterDataPut(I2C0_BASE, reg_start);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(!I2CMasterBusy(I2C0_BASE));
    while(I2CMasterBusy(I2C0_BASE));

    if (n == 1){
        I2CMasterSlaveAddrSet(I2C0_BASE, MPU_ADDR, true);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        while(!I2CMasterBusy(I2C0_BASE));
        while(I2CMasterBusy(I2C0_BASE));

        temp[counter] = I2CMasterDataGet(I2C0_BASE);
    }

    else if (n > 1){
        I2CMasterSlaveAddrSet(I2C0_BASE, MPU_ADDR, true);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        while(!I2CMasterBusy(I2C0_BASE));
        while(I2CMasterBusy(I2C0_BASE));
        temp[counter] = I2CMasterDataGet(I2C0_BASE);
        counter++;

        while(counter < n-1){
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            while(!I2CMasterBusy(I2C0_BASE));
            while(I2CMasterBusy(I2C0_BASE));
            temp[counter] = I2CMasterDataGet(I2C0_BASE);
            counter++;
        }

        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        while(!I2CMasterBusy(I2C0_BASE));
        while(I2CMasterBusy(I2C0_BASE));
        temp[counter] = I2CMasterDataGet(I2C0_BASE);
    }

    return temp;
}


void button_interrupt(void){
    uint32_t status = GPIOIntStatus(GPIO_PORTF_BASE, true);
    if ((status & USER_BTN) == USER_BTN){
        GPIOPinWrite(GPIO_PORTF_BASE, LED1, ~GPIOPinRead(GPIO_PORTF_BASE, LED1));
    }
    GPIOIntClear(GPIO_PORTF_BASE,status);
}
