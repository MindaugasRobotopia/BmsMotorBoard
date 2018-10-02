
/* functions encoded in the Driverlib ROM in the MSP432 are faster and don't use any Flash memory. The prefix "MAP_" informs the compiler that the command is in ROM.
 *
 */


/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "Const.h"
#include "main.h"
#include "VCNL40x0.h"
#include <uartRs.h>
#include "board.h"

float dt;
volatile float Kp1 = 50;
volatile float Kd1= 0;
volatile float Ki1 = 90;
volatile float Kp2 = 30;
volatile float Kd2 = 0;
volatile float Ki2 = 80;
volatile float Kp3 = 50;
volatile float Kd3 = 0;
volatile float Ki3 = 90;
SERIAL_Obj fifoRx;
const int waitI2cSend = 960; // 960 = 20us
int ssIncrement = 10;
int startDuty = 150;
int stopBytes;
// Barcode scanner configuration messages
char set[] = {0x0A, 0x04, 0x31, 0x00, 0x24, 0x25, 0x53, 0x45, 0x54, 0xFF, 0xFD, 0x8D};
char end[] = {0x0A, 0x04, 0x31, 0x00, 0x24, 0x25, 0x45, 0x4E, 0x44, 0xFF, 0xFD, 0xA2, '\r'};
char stopScan[] = {0x08, 0x04, 0x31, 0x00, 0x27, 0x4C, 0x53, 0xFF, 0xFD, 0xFE};
char trigScan[] = {0x08, 0x04, 0x31, 0x00, 0x27, 0x4C, 0x54, 0xFF, 0xFD, 0xFE};
char continiousScan[] = {0x0B, 0x04, 0x31, 0x00, 0x50, 0x46, 0x30, 0x30, 0x30, 0x31, 0xFF, 0xFD, 0x6A};
char writeDefaultScanner[] = {0x08, 0x04, 0x31, 0x00, 0x2A, 0x57, 0x43, 0xFF, 0xFE, 0x00};
char restoreUserDefault[] = {0x08, 0x04, 0x31, 0x00, 0x29, 0x44, 0x43, 0xFF, 0xFE, 0x14};
char restoreFactoryDefault[] = {0x08, 0x04, 0x31, 0x00, 0x28, 0x44, 0x46, 0xFF, 0xFE, 0x12};
char lowDetectionMode2[] = {0x0B, 0x04, 0x31, 0x00, 0x50, 0x46, 0x30, 0x35, 0x30, 0x33, 0xFF, 0xFD, 0x63};
char baudRate[] = {0x31, 0x00, 0x50, 0x43, 0x30, 0x30, 0x30, 0x36, 0xFF, 0xFD, 0x77}; // 115200baud
char disableRepeatRead[] = {0x0B, 0x04, 0x31, 0x00, 0x50, 0x46, 0x30, 0x31, 0x30, 0x30, 0xFF, 0xFD, 0x6A};

float step1;
uint32_t stepCnt;
float distance;


// This is main function
void main(void)
{
    /* Stop WDT  */
    MAP_WDT_A_holdTimer();
    setGpio();
    initClock();
    initUart();
    initIrda();
    initI2c();
    initTimer32();

    /* Enabling MASTER interrupts */
    MAP_Interrupt_enableMaster();

    initialConditions();
    //dt = (float)TIMER1_COUNT/(float)HFXT_FREQ;
    dt = (float)PID_COUNT * MOTOR_TIMER_PERIOD/(float)HFXT_FREQ;
    serialId = SERIAL_ID;

    step1 = WHEEL_DIAMETER * 3.1416 / gearbox1 / encoderPpr;
    distance = DISTANCE;
    stepCnt = distance/step1;
    switchProxi = STATE_ON;
    stopBytes = 4;
    while (1)
    {

        toggleWdi();
        getStatusMotor3();
        proximitySensor();
    }
}

void proximitySensor(void)
{
    if(switchProxi == STATE_ON)
    {
        initProximity(); // Proximity sensor initialization
    }
    if(proxiCnt != 0) // Proximity interrupt indication
    {
    //SetProximityIntClear();
    __delay_cycles(waitI2cSend);
    ReadProxiValue();
    switchLed1(STATE_OFF);
    proxiCnt = 0;
    }
}

void stopTransmitRs485(void)
{
    if (txState == FINISHED) //(UCTXIFG == 1)
    {
        cntDisableTx++;
        if(cntDisableTx >= stopBytes)
        {
            switchTransmitterRs485(STATE_OFF);
            txState = 0;
            cntDisableTx = 0;
            clearOverrun();
            fifoRx.positionIn = 0;
        }
    }
}
/* Timer32 ISR Motors PID control */
__attribute__((ramfunc))
void irqTimer32_INT1_PID(void)
{
    MAP_Timer32_clearInterruptFlag(TIMER32_0_BASE);
    if(motor2status == STATE_ON)
    {
        if(ssMotor2 == STATE_OFF)
        {
            speedControl2();
        }
        else
        {
            softStartMotor2();
        }
    }
   if (motor1status == STATE_ON)
    {
        if(ssMotor1 == STATE_OFF)
        {
            speedControl1();
        }
        else
        {
            softStartMotor1();
        }
    }
     if (motor3status == STATE_ON)
    {
        if(ssMotor3 == STATE_OFF)
        {
            speedControl3();
        }
        else
        {
            softStartMotor3();
        }
    }
}

__attribute__((ramfunc))
void speedControl1(void)
{
    error1 = (float)speedSet1 - (float)speedMotor1;
    error_d = (error1-error_old1)/dt;
    output1 = Kp1*error1 + Ki1*error_i + Kd1*error_d;
    error_i = error_i + dt*1*error1;  // rectangular integration
    error_old1 = error1;

    //output1 = output1*1.0*MOTOR_TIMER_PERIOD/65535.0;
    output1 = ((uint32_t)output1*MOTOR_TIMER_PERIOD) >> 16;
    if(output1 <= 0) output1 = 0;
    if(output1 > MOTOR_TIMER_PERIOD) output1 = MOTOR_TIMER_PERIOD;
    setDutyMotor1(output1);
}
__attribute__((ramfunc))
void speedControl2(void)
{
    error2 = (float)speedSet2 - (float)speedMotor2;
    error_d = (error2-error_old2)/dt;
    output2 = Kp2*error2 + Ki2*error_i + Kd2*error_d;
    error_i = error_i + dt*1*error2;  // rectangular integration
    error_old2 = error2;

    output2 = ((uint32_t)output2*MOTOR_TIMER_PERIOD) >> 16;
    //output2 = output2*1.0*MOTOR_TIMER_PERIOD/65535.0;
    if(output2 <= 0) output2 = 0;
    if(output2 > MOTOR_TIMER_PERIOD) output2 = MOTOR_TIMER_PERIOD;
    setDutyMotor2(output2);
}
__attribute__((ramfunc))
void speedControl3(void)
{
    error3 = (float)speedSet3 - (float)speedMotor3;
    error_d = (error3-error_old3)/dt;
    output3 = Kp3*error3 + Ki3*error_i + Kd3*error_d;
    error_i = error_i + dt*1*error3;  // rectangular integration
    error_old3 = error3;

    output3 = ((uint32_t)output3*MOTOR_TIMER_PERIOD) >> 16;
    //output3 = output3*1.0*MOTOR_TIMER_PERIOD/65535.0;
    if(output3 <= 0) output3 = 0;
    if(output3 > MOTOR_TIMER_PERIOD) output3 = MOTOR_TIMER_PERIOD;
    setDutyMotor3(output3);
}


void initialConditions(void)
{
    switchMotor1(STATE_OFF);
    switchMotor3(STATE_OFF);
    distanceMode = STATE_OFF;
    SERIAL_setDefaults(&fifoRx);
    gearbox1 = GERBOX_1;
    gearbox2 = GERBOX_2;
    gearbox3 = GERBOX_3;
    encoderPpr = ENCODER;
    reduction1 = encoderPpr*gearbox1/60.0;
    reduction2 = encoderPpr*gearbox2/60.0;
    reduction3 = encoderPpr*gearbox3/60.0;
    motor1timer = 65535;
    motor2timer = 65535;
    motor3timer = 65535;
    motor1Duty = 0;
    motor2Duty = 0;
    motor3Duty = 0;
    switchLed1(STATE_OFF);
    switchLed2(STATE_OFF);
    setScannerTrig(STATE_ON);
    setScannerWake(STATE_OFF);
    switchSecondMotorBoard(STATE_ON);
    switchTransmitterRs485(STATE_OFF);
    switchIrda(STATE_OFF);
    setIrdaMode(MODE_1);
    motor1status = STATE_OFF;
    motor2status = STATE_OFF;
    motor3status = STATE_OFF;
    faultStatus = 0;
    ssMotor1 = STATE_OFF;
    ssMotor2 = STATE_OFF;
    ssMotor3 = STATE_OFF;
    switchSpeedControl(STATE_OFF);
    statusPid = STATE_OFF;
    speedMotor1 = 0;
    speedMotor2 = 0;
    speedMotor3 = 0;
    speedSet1 = SPEED_SET_1;
    speedSet2 = SPEED_SET_2;
    speedSet3 = SPEED_SET_3;
    proxiCnt = 0;
    bcCnt = 0;
    rsCnt = 0;
    txState = 0;
    cntDisableTx = 0;
    ssTimer = 0;
    memset(rx485, 0x00, RXLENGTH);
    rxCnt = 0;
}

void initMotor1(int direction)
{
    stepMotor1 = 0;
    setDirectionOfWheels(direction);
    initPwmMotor1();
    setDutyMotor1(motor1Duty);
    switchMotor1(STATE_ON);
    startCounterMotor1();
    motor1status = STATE_ON;
    ssMotor1 = STATE_ON;
    ssStopMotor1 = STATE_OFF;
    speedMotor1 = 0;
    speedMotor2 = 0;
    motor1timer = 65535;
    motor2timer = 65535;
}

void initMotor2(int direction)
{
    stepMotor2 = 0;
    setDirectionOfWheels(direction);
    initPwmMotor2();
    setDutyMotor2(motor2Duty);
    switchMotor1(STATE_ON);
    startCounterMotor2();
    motor2status = STATE_ON;
    ssMotor2 = STATE_ON;
    ssStopMotor2 = STATE_OFF;
    speedMotor1 = 0;
    speedMotor2 = 0;
}

void initMotor3(int direction)
{
    stepMotor3 = 0;
    setDirectionOfContacts(direction);
    initPwmMotor3();
    setDutyMotor3(motor3Duty);
    switchMotor3(STATE_ON);
    startCounterMotor3();
    motor3status = STATE_ON;
    ssMotor3 = STATE_ON;
    ssStopMotor3 = STATE_OFF;
}

void stopMotor1(void)
{

    setDutyMotor1(0);
    setDutyMotor2(0);
    switchSpeedControl(STATE_OFF);
    statusPid = STATE_OFF;
    switchMotor1(STATE_OFF);
    stopCounterMotor1();
    stopCounterMotor2();
    stopTimerA2();
    motor1status = STATE_OFF;
    motor2status = STATE_OFF;
    ssMotor1 = STATE_OFF;
    ssMotor2 = STATE_OFF;
    //MAP_Timer_A_disableCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    //MAP_Timer_A_disableCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
}

void initSoftStop(void)
{
    switchSpeedControl(STATE_OFF);
    statusPid = STATE_OFF;
    ssStopMotor1 = STATE_ON;
    ssStopMotor2 = STATE_ON;
}
void stopMotor01(void)
{
    setDutyMotor1(0);
    setDutyMotor2(0);
    switchMotor1(STATE_OFF);
    stopCounterMotor1();
    stopCounterMotor2();
    stopTimerA2();
    motor1status = STATE_OFF;
    motor2status = STATE_OFF;
    ssMotor1 = STATE_OFF;
    ssMotor2 = STATE_OFF;
    motor1timer = 65535;
    motor2timer = 65535;
    switchSpeedControl(STATE_OFF);
    statusPid = STATE_OFF;

    //MAP_Timer_A_disableCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    //MAP_Timer_A_disableCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
}


void stopMotor3(void)
{
    motor3Duty = 0;
    setDutyMotor3(0);
    switchMotor3(STATE_OFF);
    stopCounterMotor3();
    stopTimerA2();
    motor3timer = 65535;
    motor3status = STATE_OFF;
    ssMotor3 = STATE_OFF;
    switchSpeedControl(STATE_OFF);
    ssStopMotor3 = STATE_ON;
}


void increaseSpeed1(int speed)
{
    speedSet1 += speed;
    speedSet2 += speed;
}

void decreaseSpeed1(int speed)
{
    speedSet1 -= speed;
    speedSet2 -= speed;
}

void softStopMotor1(void)
{
    if (speedMotor1 > STOP_SPEED)
    {
        motor1Duty = motor1Duty - ssIncrement;
        setDutyMotor1(motor1Duty);
    }
    else
    {
        stopMotor1();

    }
}

void softStopMotor2(void)
{
    if (speedMotor2 > 100)
    //if (TA2CCR4 <= 1000)
    {
        motor2Duty = motor2Duty - ssIncrement;
        setDutyMotor2(motor2Duty);
    }
    else
    {
        stopMotor1();
    }
}

void softStopMotor3(void)
{
    if (speedMotor3 > 1)
    //if (TA2CCR4 <= 1000)
    {
        motor3Duty -= ssIncrement;
        setDutyMotor3(motor3Duty);
    }
    else
    {
        ssStopMotor3 = STATE_OFF;
    }
}

void softStartMotor1(void)
{
    if ((speedMotor1 >= speedSet1 + 50) || (speedMotor1 <= speedSet1))
    //if (TA2CCR4 <= 1000)
    {
        motor1Duty += ssIncrement;
        setDutyMotor1(motor1Duty);
    }
    else
    {
        ssMotor1 = STATE_OFF;
        ssMotor2 = STATE_OFF;
        switchSpeedControl(STATE_ON);
    }
}

void softStartMotor2(void)
{
    if((speedMotor2 >= speedSet2 + 50) || (speedMotor2 <= speedSet2))
    //if (TA2CCR1 <= 800)
    {
        motor2Duty += ssIncrement;
        setDutyMotor2(motor2Duty);
    }
    else
    {
        ssMotor1 = STATE_OFF;
        ssMotor2 = STATE_OFF;
        switchSpeedControl(STATE_ON);
    }
}

void softStartMotor3(void)
{
    if (speedMotor3 <= speedSet3)
    {
        motor3Duty += ssIncrement;
        setDutyMotor3(motor3Duty);
    }
    else
    {
        ssMotor3 = STATE_OFF;
    }
}

void driveMotor1(int duty)
{
    motor1Duty = duty;
    initMotor1(FORWARD);
}

void driveMotor2(int duty)
{
    motor2Duty = duty;
    initMotor2(FORWARD);
}
void driveMotor3(int duty)
{
    motor3Duty = duty;
    initMotor3(FORWARD);
}
void driveMotor1Forward(void)
{
    motor1Duty = startDuty;
    initMotor1(FORWARD);
}

void driveMotor2Forward(void)
{
    motor2Duty = startDuty;
    initMotor2(FORWARD);
}

void driveForward(void)
{
    motor1Duty = startDuty;
    motor2Duty = startDuty;
    initMotor1(FORWARD);
    initMotor2(FORWARD);
}

void driveBackward(void)
{
    motor1Duty = startDuty;
    motor2Duty = startDuty;
    initMotor1(BACKWARD);
    initMotor2(BACKWARD);
}
void driveMotor3Up(void)
{
    motor3Duty = startDuty;
    initMotor3(UP);
    switchSpeedControl(STATE_ON);
}

void driveMotor3Down(void)
{
    motor3Duty = startDuty;
    initMotor3(DOWN);
    switchSpeedControl(STATE_ON);
}

void speedSetMotor1(int speed)
{
    speedSet1 = speed;
}

void faultMotor1(void)
{
    stopMotor1();
    faultStatus = FAULT_MOTOR_1;
}

void faultMotor3(void)
{
    stopMotor3();
    faultStatus = FAULT_MOTOR_3;
}


//IRDA receive interrupt
__attribute__((ramfunc))
void irqSerialRxIrdaEUSCIA2(void) // EUSCIA2_IRQHandler
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);
    MAP_UART_clearInterruptFlag(EUSCI_A2_BASE, status);
    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        irdaReceive = UCA2RXBUF;
    }
}

//Barcode read interrupt
__attribute__((ramfunc))
void irqSerialRxBarcodeEUSCIA0(void) //EUSCIA0_IRQHandler
{
    int z;
    z = 0;
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
    MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, status);
    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        barCodeValue = 0;
        barCodeReceive[bcCnt] = UCA0RXBUF - 48;
        bcCnt++;
        if(bcCnt > 8)
        {
            for(z=0; z < BARCODELENGTH; z++)
            {
                barCodeValue += barCodeReceive[z];
                if (barCodeValue == 2)
                {
                    stopBytes = 3;
                    sendStop();
                }
            }
            bcCnt = 0;
        }
    }
}

// RS485 receive
__attribute__((ramfunc))
void irqSerialRxRs485EUSCIA1(void) // EUSCIA1_IRQHandler
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A1_BASE);

    MAP_UART_clearInterruptFlag(EUSCI_A1_BASE, status);
    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        //int byteRx;
        int tempHead;
        // get received byte
        byteRx = UCA1RXBUF;
        // if received character is lower case letter (a to z), make it upper case
        if((byteRx >= 'a')&&(byteRx <= 'z')) byteRx -= 0x20;

        rx485[rxCnt] = byteRx;
        rxCnt++;
        if(rx485[rxCnt-1] == '\r')
        {
            rxCnt = 0;
            rxParseCommand();
        }
    }
    /*{
    int tempHead;
    byteRx = UCA1RXBUF;
    // if received character is lower case letter (a to z), make it upper case
    if((byteRx >= 'a')&&(byteRx <= 'z')) byteRx -= 0x20;

    // make sure that buffer is not overloaded
    if(fifoRx.positionIn < fifoRx.lengthOfBuffer)
    {
        fifoRx.buffer[fifoRx.head][fifoRx.positionIn] = byteRx;
    }
    // if received character is "carriage return",
    // finish current buffer and increment fifo head position
    if(byteRx == '\r')
    {
        // reset position in current buffer to 0;
        fifoRx.positionIn = 0;
        tempHead = fifoRx.head;
        tempHead++;
        // if head gets out of range, return to the beginning of cyclic buffer
        if(tempHead >= fifoRx.noOfBuffers)
        {
            tempHead = 0;
        }
        // check if incremented head value will not overlap with tail
        // increment if OK
        if(tempHead != fifoRx.tail) fifoRx.head = tempHead;
        SERIAL_parseCommand(&fifoRx);
    }
    // if received character is not
    else
    {
        fifoRx.positionIn++;
    }
    }*/
    stopTransmitRs485();
}

__attribute__((ramfunc))
void irqEncoderPORT3(void) // PORT3_IRQHandler
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(POSITION_A_MOTOR1_PORT);
    MAP_GPIO_clearInterruptFlag(POSITION_A_MOTOR1_PORT, status);

    if(status & POSITION_A_MOTOR1_PIN)
    {
        motor1timerValue();
        setMotor1TimerValue(0);
        stepMotor1++;
        speedMotor1 = encoderTimerClk/motor1timer;
        if ((motor1timer >= 65000) || (speedMotor1 >= MAX_SPEED))
        {
            speedMotor1 = 0;
        }
        if (distanceMode == STATE_ON)
        {
            if (stepMotor1 >= stepCnt)
            {
                stopMotor1();
                distanceMode = STATE_OFF;
            }
        }

        //speedWheel1Rpm = speedMotor1/reduction1;
    }
    if(status & POSITION_A_MOTOR2_PIN)
    {
        motor2timerValue();
        setMotor2TimerValue(0);
        speedMotor2 = encoderTimerClk/motor2timer;
        stepMotor2++;
        if ((motor2timer >= 65000) || (speedMotor2 >= MAX_SPEED))
        {
            speedMotor2 = 0;
        }
        if (distanceMode == STATE_ON)
        {
            if (stepMotor2 >= stepCnt)
            {
                stopMotor1();
                distanceMode = STATE_OFF;
            }
        }
        //speedWheel2Rpm = speedMotor2/reduction2;
    }
    if(status & POSITION_A_MOTOR3_PIN)
    {
        motor3timerValue();
        setMotor3TimerValue(0);
        speedMotor3 = encoderTimerClk/motor3timer;
        stepMotor3++;
        if ((motor3timer >= 65000) || (speedMotor3 >= MAX_SPEED))
        {
            speedMotor3 = 0;
        }
        //speedWheel3Rpm = speedMotor3/reduction3;

    }
}

void irqMotorFaultPORT4(void) // PORT4_IRQHandler // MOTOR FAULT INTERRUPT
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(FAULT_MOTORS_PORT);
    MAP_GPIO_clearInterruptFlag(FAULT_MOTORS_PORT, status);

    if(motor1status == STATE_ON)
    {
        faultMotor1();
    }
    if(motor3status == STATE_ON)
    {
        faultMotor3();
    }
}

void irqMotor3positionPORT5 (void) // PORT5_IRQHandler // MOTOR3 position interrupt
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(POSITION_P1_MOTOR3_PORT);
    MAP_GPIO_clearInterruptFlag(POSITION_P1_MOTOR3_PORT, status);

    if(status & POSITION_P1_MOTOR3_PIN)
    {
        switchLed1(STATE_ON);
        stopMotor3();
        statusMotor3 = BOTTOM;
    }
    else
    {
        switchLed2(STATE_ON);
        stopMotor3();
        statusMotor3 = TOP;
    }
}

void irq_PORT1_proximity(void) //PORT1_IRQHandler // PROXIMITY THRESHOLD INTERRUPT
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(PROX_INT_PORT);
    MAP_GPIO_clearInterruptFlag(PROX_INT_PORT, status);
    switchLed1(STATE_ON);
    stopMotor1();
    sendStop();
    SetProximityIntClear();
    proxiCnt = 1;
}

void ScannerStop(void)
{
    scannerDataLength = sizeof(stopScan);
    transmitScanner(stopScan);
}

void ScannerTrig(void)
{
    scannerDataLength = sizeof(continiousScan);
    transmitScanner(continiousScan);
}

void rs485test(void)
{
    //dataLength = sizeof(end);
    transmitRs485(end);
}

void sendStart(void)
{
    char msg[] = {SERIAL_ID_SLAVE, DRIVE_FORWARD, '\r'};
    transmitRs485(msg);
}

void sendStop(void)
{
    char msg[] = {SERIAL_ID_SLAVE, STOP_DRIVE, '\r'};
    transmitRs485(msg);
}

void sendToSlave(char data)
{
    stopBytes = 4;
    char msg[] = {SERIAL_ID_SLAVE, data, '\r'};
    transmitRs485(msg);
}


void rxParseCommand(void)
{
    if (rx485[0] == serialId)
    {
        switch(rx485[1])
        {
            case STOP_DRIVE:
                stopMotor1();
                sendToSlave(STOP_DRIVE);
                break;
            case 'L':
                switchLed1(STATE_TOGGLE);
                break;
            case SOFT_STOP:
                initSoftStop();
                sendToSlave(rx485[1]);
                break;
            case 'N':
                driveMotor1(100);
                driveMotor2(100);
                break;
            case DRIVE_FORWARD:
                driveForward();
                sendToSlave(rx485[1]);
                break;
            case DRIVE_BACKWARD:
                driveBackward();
                sendToSlave(rx485[1]);
                break;
            case 'Z':
                sendStart();
                break;
            case DRIVE_FORWARD_DISTANCE:
                //stepCnt = DISTANCE/step1;
                driveForward();
                distanceMode = STATE_ON;
                sendToSlave(rx485[1]);
                break;
            case DRIVE_BACKWARD_DISTANCE:
                //stepCnt = DISTANCE/step1;
                driveBackward();
                distanceMode = STATE_ON;
                sendToSlave(rx485[1]);
                break;
            case DRIVE_FORWARD_10CM:
                distance = 100;
                stepCnt = distance/step1;
                driveForward();
                distanceMode = STATE_ON;
                sendToSlave(rx485[1]);
                break;
            case DRIVE_BACKWARD_10CM:
                distance = 100;
                stepCnt = distance/step1;
                driveBackward();
                distanceMode = STATE_ON;
                sendToSlave(rx485[1]);
                break;
            case 'Q':
                driveMotor3(500);
                break;
            case 'W':
                driveMotor3Up();
                break;
            case INCREASE_SPEED:
                increaseSpeed1(50);
                sendToSlave(rx485[1]);
                break;
            case DECREASE_SPEED:
                decreaseSpeed1(50);
                sendToSlave(rx485[1]);

                break;
            case 'T':
                ScannerTrig();
                break;
            case '9':
                SetProximityIntClear();
                break;
            case 'P':
                setProximity();
                break;
            default:
                break;
        }
    }
    memset(rx485, 0x00, RXLENGTH);
}
