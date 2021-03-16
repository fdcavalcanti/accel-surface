#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"

/*
 * https://jspicer.net/2018/07/27/solution-for-i2c-busy-status-latency/
 * https://e2e.ti.com/support/microcontrollers/other/f/other-microcontrollers-forum/343532/i2c-busy--and-error-flags-on-tm4c1292
 * I2CMasterBusBusy has a bug where there's a delay in setting the busy status bit.
 * Check through registers or do a complemented while loop: (!MasterBusBusy()); (MasterBusBusy());
 * Configuration for console print: https://software-dl.ti.com/ccs/esd/documents/sdto_cgt_tips_for_using_printf.html
 */

#define LED4 GPIO_PIN_0        // PF0
#define LED3 GPIO_PIN_4        // PF4
#define USER_BTN GPIO_PIN_1    // PF1
#define TIMER_100HZ GPIO_PIN_0 // PD0

#define MPU_SDA GPIO_PIN_3   // PB3
#define MPU_SCL GPIO_PIN_2   // PB2
#define MPU_ADDR 0x68        // GY521 Address
#define WHO_AM_I_ADDR 0x75   // MPU6050's register containing its address
#define PWR_MGMT_ADDR 0x6B   // Power Management Register
#define ACCEL_CONFIG_ADDR 0x1C  //Accelerometer Configuration (AFSEL)
#define TEMP_ADDR 0x41       // Temperature sensor address
#define ACC_XOUT0 0x3B       // First register for Accelerometer X

#define UART_RX_PIN GPIO_PIN_0  // UART0 Rx Pin
#define UART_TX_PIN GPIO_PIN_1  // UART0 Tx Pin

uint8_t WHO_AM_I = 0x0;          // I2C address of the slave.
bool RAW_OUTPUT = false;         // Select true to output raw data from accelerometer. False will ask for calibration.
bool CALIBRATION_STATUS = false; // Tells if the calibration process has been executed.
volatile float DATA_OUT[5];
float ACCEL_OFFSET[3] = {0,0,0};

void button_interrupt(void);
void write_byte_I2C0(uint8_t reg, uint8_t data);
void timer100hz_interrupt(void);
void send_data_uart(void);
bool calibrate_accelerometer(void);
int8_t read_byte_I2C0(uint8_t addr);
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

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    FPULazyStackingEnable();
    FPUEnable();

    // LED 1 Configuration
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED4);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED3);
    GPIOPinWrite(GPIO_PORTF_BASE, LED4 | LED3, LED4 | LED3);

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
    write_byte_I2C0(ACCEL_CONFIG_ADDR, 0x00);
    SysCtlDelay(g_ui32SysClock/120000);
    WHO_AM_I = read_byte_I2C0(WHO_AM_I_ADDR);

    // UART1
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, UART_TX_PIN | UART_RX_PIN);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
    UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 57600,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE| UART_CONFIG_PAR_NONE);
    UARTEnable(UART0_BASE);

    // Timer 100 Hz
    GPIOPinConfigure(GPIO_PD0_T0CCP0);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, TIMER_100HZ);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 12000000);  // 100 Hz = clock * periodo
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerIntRegister(TIMER0_BASE, TIMER_A, timer100hz_interrupt);

    if (RAW_OUTPUT){
        TimerEnable(TIMER0_BASE, TIMER_A);
        TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    }

    else{
        if (calibrate_accelerometer() == true){
            TimerEnable(TIMER0_BASE, TIMER_A);
            TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        }
        else{
            char error_msg[] = "Calibration error";
            GPIOPinWrite(GPIO_PORTF_BASE, LED4, LED4);
            UARTSend((uint8_t *)error_msg, strlen(error_msg));
        }
    }

    while(1){

    }

	return 0;
}

bool calibrate_accelerometer(void){
    uint32_t CAL_DATA_POINTS = 200, i;
    DATA_OUT[2] = 0.0;
    DATA_OUT[3] = 0.0;
    DATA_OUT[4] = 0.0;

    char error_msg[] = "Press button to start calibration\n";
    UARTSend((uint8_t *)error_msg, strlen(error_msg));

    do{
        UARTSend((uint8_t *)error_msg, strlen(error_msg));
        SysCtlDelay(120000000/2);
        GPIOPinWrite(GPIO_PORTF_BASE, LED4 | LED3, ~GPIOPinRead(GPIO_PORTF_BASE, LED4 | LED3));
    }while(CALIBRATION_STATUS == false);

    GPIOPinWrite(GPIO_PORTF_BASE, LED4 | LED3, LED4 | LED3);

    for (i = 0; i < CAL_DATA_POINTS; i++){
        int8_t *p = burst_read_sequence_I2C0(ACC_XOUT0, 6);
        DATA_OUT[2] += ((p[0] << 8) + p[1])/16384.0;
        DATA_OUT[3] += ((p[2] << 8) + p[3])/16384.0;
        DATA_OUT[4] += ((p[4] << 8) + p[5])/16384.0;
        SysCtlDelay(1000);
    }

    for (i = 0; i < 3; i++){
        ACCEL_OFFSET[i] = DATA_OUT[i+2] / CAL_DATA_POINTS;
    }

    GPIOPinWrite(GPIO_PORTF_BASE, LED4 | LED3, 0x00);

    return true;
}


void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{

    while(ui32Count--)
    {
        UARTCharPut(UART0_BASE, *pui8Buffer++);
    }
}


void send_data_uart(void){
    char stringout[50] = "";
    char conversion[9] = "";
    if (DATA_OUT[0] == 99999)
        DATA_OUT[0] = 0;
    else
        DATA_OUT[0]++;      // Sequence Counter


    int i=0, test=0;
    float temp=0;

    for (i = 0; i < 5; i++){
        temp = DATA_OUT[i];
        if (ceil(temp) == temp)
            test = snprintf(conversion, sizeof(conversion), "%.0f\t", temp);
        else
            test = snprintf(conversion, sizeof(conversion), "%.4f\t", temp);
        strcat(stringout,  conversion);
    }
    strcat(stringout, "\n");
    UARTSend((uint8_t *)stringout, strlen(stringout));
}


void timer100hz_interrupt(void){
    uint32_t status = TimerIntStatus(TIMER0_BASE, true);
    if ((status & TIMER_TIMA_TIMEOUT) == TIMER_TIMA_TIMEOUT){
        GPIOPinWrite(GPIO_PORTF_BASE, LED4, ~GPIOPinRead(GPIO_PORTF_BASE, LED4));
        WHO_AM_I = read_byte_I2C0(WHO_AM_I_ADDR);
        int8_t *p = burst_read_sequence_I2C0(ACC_XOUT0, 6);
        DATA_OUT[2] = ((p[0] << 8) + p[1])/16384.0 - ACCEL_OFFSET[0];
        DATA_OUT[3] = ((p[2] << 8) + p[3])/16384.0 - ACCEL_OFFSET[1];
        DATA_OUT[4] = ((p[4] << 8) + p[5])/16384.0 - ACCEL_OFFSET[2];
    }
    send_data_uart();
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
    char header[] = "sample\tsurface\tacc_x\tacc_y\tacc_z\n";
    uint32_t status = GPIOIntStatus(GPIO_PORTF_BASE, true);
    if ((status & USER_BTN) == USER_BTN){
        GPIOPinWrite(GPIO_PORTF_BASE, LED4, ~GPIOPinRead(GPIO_PORTF_BASE, LED4));
        DATA_OUT[0] = 0;
        if (CALIBRATION_STATUS == false)
            CALIBRATION_STATUS = true;
        UARTSend((uint8_t *)header, strlen(header));
    }
    GPIOIntClear(GPIO_PORTF_BASE,status);
}
