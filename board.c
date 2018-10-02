#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "Const.h"
#include "board.h"
#include "VCNL40x0.h"


const Timer_A_ContinuousModeConfig continuousModeConfig =
{
     TIMER_A_CLOCKSOURCE_SMCLK,
     TIMER_A_CLOCKSOURCE_DIVIDER_8,
     TIMER_A_TAIE_INTERRUPT_DISABLE,
     TIMER_A_DO_CLEAR
};

/* Timer_A PWM Configuration Parameter */
Timer_A_PWMConfig pwmConfig1 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        MOTOR_TIMER_PERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_4,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0
};

Timer_A_PWMConfig pwmConfig2 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        MOTOR_TIMER_PERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_0,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0
};

Timer_A_PWMConfig pwmConfig21 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        MOTOR_TIMER_PERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0
};

Timer_A_PWMConfig pwmConfig3 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        MOTOR_TIMER_PERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_3,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0
};


/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 115200 baud rate.
 * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const eUSCI_UART_Config uartConfigRs485 =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        4,                                      // BRDIV = 13
        9,                                       // UCxBRF = 0
        34,                                      // UCxBRS = 37
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // MSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

const eUSCI_UART_Config uartConfigScanner =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        26,                                      // BRDIV = 13
        0,                                       // UCxBRF = 0
        111,                                      // UCxBRS = 37
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

const eUSCI_UART_Config uartConfigIrda =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        26,                                      // BRDIV = 13
        0,                                       // UCxBRF = 0
        111,                                      // UCxBRS = 37
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // MSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

/* I2C Master Configuration Parameter */
const eUSCI_I2C_MasterConfig i2cConfig1 =
{
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        HFXT_FREQ,                              // SMCLK = 48MHz
        EUSCI_B_I2C_SET_DATA_RATE_1MBPS,      // Desired I2C Clock of 1MHz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
};


void setGpio(void)
{
    //INPUT PINS
    MAP_GPIO_setAsInputPinWithPullUpResistor(EN_MOTORS_PORT, EN_MOTORS_PIN);
    MAP_GPIO_setAsInputPinWithPullUpResistor(EN_MOTOR3_PORT, EN_MOTOR3_PIN);
    MAP_GPIO_setAsInputPin(PROX_INT_PORT, PROX_INT_PIN);
    MAP_GPIO_setAsInputPin(POSITION_A_MOTOR2_PORT, POSITION_A_MOTOR2_PIN);
    MAP_GPIO_setAsInputPin(POSITION_A_MOTOR3_PORT, POSITION_A_MOTOR3_PIN);
    MAP_GPIO_setAsInputPin(POSITION_A_MOTOR1_PORT, POSITION_A_MOTOR1_PIN);
    MAP_GPIO_setAsInputPinWithPullDownResistor(POSITION_P1_MOTOR3_PORT, POSITION_P1_MOTOR3_PIN);
    MAP_GPIO_setAsInputPinWithPullDownResistor(POSITION_P2_MOTOR3_PORT, POSITION_P2_MOTOR3_PIN);
    MAP_GPIO_setAsInputPin(FAULT_MOTORS_PORT, FAULT_MOTORS_PIN);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(SCANNER_UARTRX_PORT, SCANNER_UARTRX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(SCANNER_UARTTX_PORT, SCANNER_UARTTX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(PROX_SDA_PORT, PROX_SDA_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(PROX_SCL_PORT, PROX_SCL_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(RS485_UARTRX_PORT, RS485_UARTRX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(RS485_UARTTX_PORT, RS485_UARTTX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(IRDA_UARTRX_PORT, IRDA_UARTRX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(IRDA_UARTTX_PORT, IRDA_UARTTX_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PWM3_PORT, PWM3_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PWM1_PORT, PWM1_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    //MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PWM2_PORT, PWM2_PIN, GPIO_SECONDARY_MODULE_FUNCTION);
    MAP_GPIO_setAsInputPinWithPullDownResistor(PWM1_PORT, PWM1_PIN);
    MAP_GPIO_setAsInputPinWithPullDownResistor(PWM2_PORT, PWM2_PIN);
    MAP_GPIO_setAsInputPinWithPullDownResistor(PWM3_PORT, PWM3_PIN);

    // UNUSED PINS - SET AS OUTPUT
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN4);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN4);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN7);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN6);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN7);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN1);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN2);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN3);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN1);
    //MAP_GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN4); // TDI WHEN UNUSED
    //MAP_GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_PIN5); //TDO WHEN UNUSED
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_PIN5);

    //OUTPUT PINS
    MAP_GPIO_setAsOutputPin(SCANNER_TRIG_PORT, SCANNER_TRIG_PIN);
    MAP_GPIO_setAsOutputPin(SCANNER_WAKE_PORT, SCANNER_WAKE_PIN);
    MAP_GPIO_setAsOutputPin(LED1_PORT, LED1_PIN);
    MAP_GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
    MAP_GPIO_setAsOutputPin(SD_IRDA_PORT, SD_IRDA_PIN);
    MAP_GPIO_setAsOutputPin(MODE_IRDA_PORT, MODE_IRDA_PIN);
    MAP_GPIO_setAsOutputPin(DIR_MOTOR1_PORT, DIR_MOTOR1_PIN);
    MAP_GPIO_setAsOutputPin(DIR_MOTOR3_PORT, DIR_MOTOR3_PIN);
    MAP_GPIO_setAsOutputPin(DE_RS485_PORT, DE_RS485_PIN);
    MAP_GPIO_setAsOutputPin(EN_MOTORBOARD2_PORT, EN_MOTORBOARD2_PIN);
    MAP_GPIO_setAsOutputPin(WDI_PORT, WDI_PIN);
    MAP_GPIO_setAsOutputPin(PWM1_PORT, PWM1_PIN);
    MAP_GPIO_setAsOutputPin(PWM2_PORT, PWM2_PIN);
    MAP_GPIO_setAsOutputPin(PWM3_PORT, PWM3_PIN);

    MAP_GPIO_setDriveStrengthHigh(LED1_PORT, LED1_PIN);
    MAP_GPIO_setDriveStrengthHigh(LED2_PORT, LED2_PIN);


    /* Enabling interrupts on A1, A2, A3 */
    MAP_GPIO_clearInterruptFlag(POSITION_A_MOTOR1_PORT, POSITION_A_MOTOR1_PIN);
    MAP_GPIO_enableInterrupt(POSITION_A_MOTOR1_PORT, POSITION_A_MOTOR1_PIN);
    MAP_GPIO_clearInterruptFlag(POSITION_A_MOTOR2_PORT, POSITION_A_MOTOR2_PIN);
    MAP_GPIO_enableInterrupt(POSITION_A_MOTOR2_PORT, POSITION_A_MOTOR2_PIN);
    MAP_GPIO_clearInterruptFlag(POSITION_A_MOTOR3_PORT, POSITION_A_MOTOR3_PIN);
    MAP_GPIO_enableInterrupt(POSITION_A_MOTOR3_PORT, POSITION_A_MOTOR3_PIN);
    MAP_GPIO_interruptEdgeSelect(POSITION_A_MOTOR1_PORT, POSITION_A_MOTOR1_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_GPIO_interruptEdgeSelect(POSITION_A_MOTOR2_PORT, POSITION_A_MOTOR2_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_GPIO_interruptEdgeSelect(POSITION_A_MOTOR3_PORT, POSITION_A_MOTOR3_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_Interrupt_enableInterrupt(INT_PORT3); // SAME PORT FOR ALL PINS

    /* Enabling interrupts on P1, P2 */
    MAP_GPIO_clearInterruptFlag(POSITION_P1_MOTOR3_PORT, POSITION_P1_MOTOR3_PIN);
    MAP_GPIO_enableInterrupt(POSITION_P1_MOTOR3_PORT, POSITION_P1_MOTOR3_PIN);
    MAP_GPIO_clearInterruptFlag(POSITION_P2_MOTOR3_PORT, POSITION_P2_MOTOR3_PIN);
    MAP_GPIO_enableInterrupt(POSITION_P2_MOTOR3_PORT, POSITION_P2_MOTOR3_PIN);
    MAP_GPIO_interruptEdgeSelect(POSITION_P1_MOTOR3_PORT, POSITION_P1_MOTOR3_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_GPIO_interruptEdgeSelect(POSITION_P2_MOTOR3_PORT, POSITION_P2_MOTOR3_PIN, GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_Interrupt_enableInterrupt(INT_PORT5); // SAME PORT FOR ALL PINS

    /* Enabling interrupt on FAULT */
    GPIO_clearInterruptFlag(FAULT_MOTORS_PORT, FAULT_MOTORS_PIN);
    GPIO_enableInterrupt(FAULT_MOTORS_PORT, FAULT_MOTORS_PIN);
    GPIO_interruptEdgeSelect(FAULT_MOTORS_PORT, FAULT_MOTORS_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
    Interrupt_enableInterrupt(INT_PORT4);

    /* Enabling interrupt on ProxInt */
    GPIO_clearInterruptFlag(PROX_INT_PORT, PROX_INT_PIN);
    GPIO_enableInterrupt(PROX_INT_PORT, PROX_INT_PIN);
    GPIO_interruptEdgeSelect(PROX_INT_PORT, PROX_INT_PIN, GPIO_HIGH_TO_LOW_TRANSITION);
    Interrupt_enableInterrupt(INT_PORT1);
}

void switchMotor1(int state)
{
    if (state == STATE_OFF)
    {
        //MAP_GPIO_setOutputHighOnPin(EN_MOTORS_PORT, EN_MOTORS_PIN);
        MAP_GPIO_setAsInputPinWithPullUpResistor(EN_MOTORS_PORT, EN_MOTORS_PIN);
        MAP_GPIO_setAsInputPinWithPullDownResistor(PWM1_PORT, PWM1_PIN);
        MAP_GPIO_setAsInputPinWithPullDownResistor(PWM2_PORT, PWM2_PIN);

    }
    else
    {
        MAP_GPIO_setAsOutputPin(PWM1_PORT, PWM1_PIN);
        MAP_GPIO_setAsOutputPin(PWM2_PORT, PWM2_PIN);
        MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PWM1_PORT, PWM1_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

        MAP_GPIO_setAsInputPinWithPullDownResistor(EN_MOTORS_PORT, EN_MOTORS_PIN);
        //MAP_GPIO_setOutputLowOnPin(EN_MOTORS_PORT, EN_MOTORS_PIN);
        //MAP_GPIO_setAsOutputPin(EN_MOTORS_PORT, EN_MOTORS_PIN);
    }
}

void switchMotor3(int state)
{
    if (state == STATE_OFF)
    {
        MAP_GPIO_setOutputHighOnPin(EN_MOTOR3_PORT, EN_MOTOR3_PIN);
        MAP_GPIO_setAsInputPinWithPullDownResistor(PWM3_PORT, PWM3_PIN);
    }
    else
    {
        MAP_GPIO_setAsOutputPin(PWM3_PORT, PWM3_PIN);
        MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(PWM3_PORT, PWM3_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
        MAP_GPIO_setOutputLowOnPin(EN_MOTOR3_PORT, EN_MOTOR3_PIN);
    }
}

void switchLed1(int state)
{
    if (state == STATE_ON)
    {
        MAP_GPIO_setOutputHighOnPin(LED1_PORT, LED1_PIN);
    }
    else if (state == STATE_TOGGLE)
    {
        MAP_GPIO_toggleOutputOnPin(LED1_PORT, LED1_PIN);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(LED1_PORT, LED1_PIN);
    }
}

void switchLed2(int state)
{
    if (state == STATE_ON)
    {
        MAP_GPIO_setOutputHighOnPin(LED2_PORT, LED2_PIN);
    }
    else if (state == STATE_TOGGLE)
    {
        MAP_GPIO_toggleOutputOnPin(LED2_PORT, LED2_PIN);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(LED2_PORT, LED2_PIN);
    }
}

void setDirectionOfWheels(int direction)
{
    if (direction == FORWARD)
    {
        MAP_GPIO_setOutputHighOnPin(DIR_MOTOR1_PORT, DIR_MOTOR1_PIN);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(DIR_MOTOR1_PORT, DIR_MOTOR1_PIN);
    }
}

void setDirectionOfContacts(int direction)
{
    if (direction == UP)
    {
        MAP_GPIO_setOutputHighOnPin(DIR_MOTOR3_PORT, DIR_MOTOR3_PIN);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(DIR_MOTOR3_PORT, DIR_MOTOR3_PIN);
    }
}

void toggleWdi(void)
{
    MAP_GPIO_toggleOutputOnPin(WDI_PORT, WDI_PIN);
}

void setIrdaMode(int mode)
{
    if (mode == MODE_1)
    {
        MAP_GPIO_setOutputLowOnPin(MODE_IRDA_PORT, MODE_IRDA_PIN);
    }
    else
    {
        MAP_GPIO_setOutputHighOnPin(MODE_IRDA_PORT, MODE_IRDA_PIN);
    }
}

void switchIrda(int state)
{
    if (state == STATE_OFF)
    {
        MAP_GPIO_setOutputHighOnPin(SD_IRDA_PORT, SD_IRDA_PIN);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(SD_IRDA_PORT, SD_IRDA_PIN);
    }
}

void setScannerTrig(int state)
{
    if (state == STATE_ON)
    {
        MAP_GPIO_setOutputHighOnPin(SCANNER_TRIG_PORT, SCANNER_TRIG_PIN);
    }
    else if (state == STATE_TOGGLE)
    {
        MAP_GPIO_toggleOutputOnPin(SCANNER_TRIG_PORT, SCANNER_TRIG_PIN);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(SCANNER_TRIG_PORT, SCANNER_TRIG_PIN);
    }
}

void setScannerWake(int state)
{
    if (state == STATE_ON)
    {
        MAP_GPIO_setOutputHighOnPin(SCANNER_WAKE_PORT, SCANNER_WAKE_PIN);
    }
    else if (state == STATE_TOGGLE)
    {
        MAP_GPIO_toggleOutputOnPin(SCANNER_WAKE_PORT, SCANNER_WAKE_PIN);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(SCANNER_WAKE_PORT, SCANNER_WAKE_PIN);
    }
}

void switchTransmitterRs485(int state)
{
    if (state == STATE_ON)
    {
        MAP_GPIO_setOutputHighOnPin(DE_RS485_PORT, DE_RS485_PIN);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(DE_RS485_PORT, DE_RS485_PIN);
    }
}

void switchSecondMotorBoard(int state)
{
    if (state == STATE_ON)
    {
        MAP_GPIO_setOutputHighOnPin(EN_MOTORBOARD2_PORT, EN_MOTORBOARD2_PIN);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(EN_MOTORBOARD2_PORT, EN_MOTORBOARD2_PIN);
    }
}


__attribute__((ramfunc))
void setDutyMotor1(int duty)
{
    TA2CCR4 = duty;
}

__attribute__((ramfunc))
void setDutyMotor2(int duty)
{
    TA2CCR1 = duty;
}

__attribute__((ramfunc))
void setDutyMotor3(int duty)
{
    TA2CCR3 = duty;
}


void initPwmMotor1(void)
{
    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig1);
    if(motor2status == STATE_ON)
    {
        MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
        MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
    }

}

void initPwmMotor2(void)
{
    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig21);
    // TA2.0 is not suitable for PWM. TA2.1 PWM interrupt toggles P8.1 GPIO.
    MAP_Interrupt_enableInterrupt(INT_TA2_0);
    MAP_Interrupt_enableInterrupt(INT_TA2_N);
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
}


__attribute__((ramfunc))
void irqPwm2LowTA2_N(void) //TA2_N_IRQHandler
{
    P8->OUT &= ~BIT1;  //MAP_GPIO_setOutputLowOnPin(PWM2_PORT, PWM2_PIN);
    TIMER_A2->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG; //MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
}

__attribute__((ramfunc))
void irqPwm2HighTA2_0(void) //TA2_0_IRQHandler
{
    P8->OUT |= BIT1; //MAP_GPIO_setOutputHighOnPin(PWM2_PORT, PWM2_PIN);
    TIMER_A2->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    TIMER_A2->CTL &= ~TIMER_A_CTL_IFG;
    if (ssMotor1 == STATE_ON)
    {
        ssTimer++;
        if (ssTimer >= START_PERIOD)
        {
            softStartMotor1();
            softStartMotor2();
            ssTimer = 0;
        }
    }
    if (statusPid == STATE_ON)
    {
        pidCnt++;
        if(pidCnt >= PID_COUNT)
        {
            speedControl1();
            speedControl2();
            pidCnt = 0;
        }
    }
    if(ssStopMotor1 == STATE_ON)
    {
        ssTimer++;
        if (ssTimer >= STOP_PERIOD)
        {
            softStopMotor1();
            softStopMotor2();
            ssTimer = 0;
        }
    }
}


/* Timer32 ISR */
void T32_INT2_IRQHandler(void)
{
    MAP_Timer32_clearInterruptFlag(TIMER32_1_BASE);
    //switchLed2(STATE_TOGGLE);
}

void initPwmMotor3(void)
{
    MAP_Timer_A_generatePWM(TIMER_A2_BASE, &pwmConfig3);
}

void stopTimerA2(void) //Motor PWM timer
{
    MAP_Timer_A_stopTimer(TIMER_A2_BASE);
}

void stopTimerA1(void) // Motor2 encoder timer
{
    MAP_Timer_A_stopTimer(TIMER_A1_BASE);
}

void stopTimerA0(void) // Motor1(3) encoder timer
{
    MAP_Timer_A_stopTimer(TIMER_A0_BASE);
}

void stopCounterMotor1(void)
{
    MAP_Timer_A_stopTimer(TIMER_A0_BASE);
    Timer_A_clearTimer(TIMER_A0_BASE);
}

void stopCounterMotor2(void)
{
    MAP_Timer_A_stopTimer(TIMER_A1_BASE);
    Timer_A_clearTimer(TIMER_A1_BASE);
}

void stopCounterMotor3(void)
{
    MAP_Timer_A_stopTimer(TIMER_A0_BASE);
    Timer_A_clearTimer(TIMER_A0_BASE);
}

// Counter for motor1 speed measurement
void startCounterMotor1(void)
{
    /* Configuring Continuous Mode */
    MAP_Timer_A_configureContinuousMode(TIMER_A0_BASE, &continuousModeConfig);
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
    encoderTimerClk = HFXT_FREQ/continuousModeConfig.clockSourceDivider;
}

// Counter for motor2 speed measurement
void startCounterMotor2(void)
{
    /* Configuring Continuous Mode */
    MAP_Timer_A_configureContinuousMode(TIMER_A1_BASE, &continuousModeConfig);
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_CONTINUOUS_MODE);
    encoderTimerClk = HFXT_FREQ/continuousModeConfig.clockSourceDivider;
}

// Counter for motor3 speed measurement
void startCounterMotor3(void)
{
    /* Configuring Continuous Mode */
    MAP_Timer_A_configureContinuousMode(TIMER_A0_BASE, &continuousModeConfig);
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
    encoderTimerClk = HFXT_FREQ/continuousModeConfig.clockSourceDivider;
}

// Enable/disable speed control PID
void switchSpeedControl(int state)
{
    if(state == STATE_ON)
    {
        //MAP_Timer32_startTimer(TIMER32_0_BASE, false);
        statusPid = STATE_ON;
    }
    else
    {
        //MAP_Timer32_haltTimer(TIMER32_0_BASE);
        statusPid = STATE_OFF;
    }
}

void getStatusMotor3(void)
{
    if(MAP_GPIO_getInputPinValue(POSITION_P1_MOTOR3_PORT, POSITION_P1_MOTOR3_PIN) == 1)
    {
        statusMotor3 = BOTTOM;
    }
    if(MAP_GPIO_getInputPinValue(POSITION_P2_MOTOR3_PORT, POSITION_P2_MOTOR3_PIN) == 1)
    {
        statusMotor3 = TOP;
    }
    if((MAP_GPIO_getInputPinValue(POSITION_P2_MOTOR3_PORT, POSITION_P2_MOTOR3_PIN) != 1) && (MAP_GPIO_getInputPinValue(POSITION_P1_MOTOR3_PORT, POSITION_P1_MOTOR3_PIN) != 1))
    {
        statusMotor3 = TRANSITION;
    }
}

int motor1timerValue(void)
{
    motor1timer = TA0R;
    return motor1timer;
}

void setMotor1TimerValue(int value)
{
    TA0R = value;
}

int motor2timerValue(void)
{
    motor2timer = TA1R;
    return motor2timer;
}

void setMotor2TimerValue(int value)
{
    TA1R = value;
}

int motor3timerValue(void)
{
    motor3timer = TA0R;
    return motor3timer;
}

void setMotor3TimerValue(int value)
{
    TA0R = value;
}

void transmitI2c(void)
{
    while (MAP_I2C_masterIsStopSent(EUSCI_B0_BASE) == EUSCI_B_I2C_SENDING_STOP);
    TXByteCtr = 1;
    q = 0;
    MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, i2cSend[0]);
}

void SetProximityCurrent(int value)
{
    i2cSend[0] = REGISTER_PROX_CURRENT;  // VCNL40x0 IR LED Current register
    i2cSend[1] = value; //0x01 - 0x14
    transmitI2c();
}

void SetProximityCommandRegisterSeflfTimed(void)
{

    i2cSend[0] = REGISTER_COMMAND;
    i2cSend[1] = COMMAND_SELFTIMED_MODE_ENABLE;
    transmitI2c();
}

void SetProximityCommandRegisterEn(void)
{

    i2cSend[0] = REGISTER_COMMAND;
    i2cSend[1] = COMMAND_PROX_ENABLE + COMMAND_SELFTIMED_MODE_ENABLE;
    //i2cSend[1] = COMMAND_PROX_ON_DEMAND;
    transmitI2c();
}

void SetProximityRate(void)
{
    i2cSend[0] = REGISTER_PROX_RATE;
    i2cSend[1] = PROX_MEASUREMENT_RATE_125;
    transmitI2c();
}

void SetProximityInt(void)
{
    i2cSend[0] = REGISTER_INTERRUPT_CONTROL;
    i2cSend[1] = INTERRUPT_THRES_ENABLE;
    transmitI2c();
}

void SetProximityIntClear(void)
{
    i2cSend[0] = REGISTER_INTERRUPT_STATUS;
    i2cSend[1] = INTERRUPT_STATUS_THRES_HI; //0x01
    transmitI2c();
}

void SetProximityIntTresH(uint_fast16_t value)
{
    unsigned char HiByte = 0;
    HiByte = (unsigned char)((value & 0xff00)>>8);
    i2cSend[0] = REGISTER_INTERRUPT_HIGH_THRES;
    i2cSend[1] = HiByte;
    transmitI2c();
}

void SetProximityIntTresL(uint_fast16_t value)
{
    unsigned char LoByte = 0;
    LoByte = (unsigned char)(value & 0x00ff);
    i2cSend[0] = REGISTER_INTERRUPT_HIGH_THRES + 1; // 0x8d;
    i2cSend[1] = LoByte;
    transmitI2c();
}

void SetProximityDisable(void)
{
    i2cSend[0] = REGISTER_COMMAND;
    i2cSend[1] = COMMAND_ALL_DISABLE;
    transmitI2c();
}

void ReadProxiId(void)
{
    i2cSend[0] = REGISTER_ID;
    transmitI2c();
    MAP_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
    MAP_I2C_masterReceiveSingleByte(EUSCI_B0_BASE);
    /* Enable RX interrupt */
    MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
}

__attribute__((ramfunc))
void ReadProxiValue(void)
{
    i2cSend[0] = REGISTER_PROX_VALUE;
    transmitI2c();
    MAP_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
    MAP_I2C_masterReceiveStart(EUSCI_B0_BASE);
    /* Enable RX interrupt */
    MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
}

void irq_EUSCIB0_i2c(void) //EUSCIB0_IRQHandler
{
    uint_fast16_t status;

    status = MAP_I2C_getEnabledInterruptStatus(EUSCI_B0_BASE);
    MAP_I2C_clearInterruptFlag(EUSCI_B0_BASE, status);

    if (status & EUSCI_B_I2C_NAK_INTERRUPT)
    {
        MAP_I2C_masterSendStart(EUSCI_B0_BASE);
    }
    if (status & EUSCI_B_I2C_TRANSMIT_INTERRUPT0)
    {
        if (TXByteCtr)
        {
            MAP_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, i2cSend[1]);
            TXByteCtr--;
        } else
        {
            MAP_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
        }
    }
    if (status & EUSCI_B_I2C_RECEIVE_INTERRUPT0)
        {
            i2cReceive[q] = MAP_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);
            q++;
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
            ProxiValue = (i2cReceive[0] << 8 | i2cReceive[1]);

            if(q > 1)
            {
                MAP_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
                while (MAP_I2C_masterIsStopSent(EUSCI_B0_BASE) == EUSCI_B_I2C_SENDING_STOP);
                SetProximityIntClear();
            }
        }
}

void initProximity(void)
{
    SetProximityDisable();
    __delay_cycles(I2CWAIT);
    SetProximityCurrent(PROXI_LED_CURRENT);
    __delay_cycles(I2CWAIT);
    SetProximityRate();
    __delay_cycles(I2CWAIT);
    SetProximityInt();
    __delay_cycles(I2CWAIT);
    SetProximityIntTresH(PROXI_TRESH_HIGH);
    __delay_cycles(I2CWAIT);
    SetProximityIntTresL(PROXI_TRESH_HIGH);
    __delay_cycles(I2CWAIT);
    SetProximityCommandRegisterSeflfTimed();
    __delay_cycles(I2CWAIT);
    SetProximityCommandRegisterEn();
    switchProxi = 0;
}

void calibrateProximity(void)
{
    ReadProxiValue();
    __delay_cycles(I2CWAIT);
    SetProximityIntTresH(ProxiValue);
    __delay_cycles(I2CWAIT);
    SetProximityIntTresL(ProxiValue);
    __delay_cycles(I2CWAIT);
}
void setProximity(void)
{
    switchProxi = STATE_ON;
}

void initUart(void)
{
    /* Configuring UART Modules */
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfigScanner);
    MAP_UART_initModule(EUSCI_A1_BASE, &uartConfigRs485);
    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfigIrda);

    /* Enable UART modules */
    MAP_UART_enableModule(EUSCI_A0_BASE);
    MAP_UART_enableModule(EUSCI_A1_BASE);
    MAP_UART_enableModule(EUSCI_A2_BASE);
    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);
}

void initI2c(void)
{
    MAP_I2C_initMaster(EUSCI_B0_BASE, &i2cConfig1);
    /* Specify slave address */
    MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, VCNL40x0_ADDRESS);
    /* Set Master in transmit mode */
    //MAP_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    /* Enable I2C Module to start operations */
    MAP_I2C_enableModule(EUSCI_B0_BASE);
    /* Enable and clear the interrupt flag */
    MAP_I2C_clearInterruptFlag(EUSCI_B0_BASE,
           EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT);
    /* Enable master transmit interrupt */
    MAP_I2C_enableInterrupt(EUSCI_B0_BASE,
           EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIB0);
}

void initTimer32(void)
{
    // TIMER32_0_BASE interrupt for motor PID control
    MAP_Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    MAP_Timer32_setCount(TIMER32_0_BASE, TIMER1_COUNT); //48000 = 1ms
    MAP_Interrupt_enableInterrupt(INT_T32_INT1);
/*
    MAP_Timer32_initModule(TIMER32_1_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    MAP_Timer32_setCount(TIMER32_1_BASE, timer1count); //48000 = 1ms
    MAP_Timer32_startTimer(TIMER32_1_BASE, false);
    MAP_Interrupt_enableInterrupt(INT_T32_INT2);
*/
}

void initIrda(void)
{
    EUSCI_A2->IRCTL |= EUSCI_A_IRCTL_IRTXCLK;
    EUSCI_A2->IRCTL |= EUSCI_A_IRCTL_IRRXPL;
    EUSCI_A2->IRCTL |= EUSCI_A_IRCTL_IREN;
}

void transmitRs485(char *data)
{
    switchTransmitterRs485(STATE_ON);
    while(*data != '\r')
    //while(*data)
    {
        MAP_UART_transmitData(EUSCI_A1_BASE, *data);
        data++;
    }
    MAP_UART_transmitData(EUSCI_A1_BASE, '\r');
    txState = FINISHED;
}
void clearOverrun(void)
{
    EUSCI_A1->STATW &= ~EUSCI_A_STATW_OE;

}

void transmitIrda(char *data)
{
    MAP_UART_transmitData(EUSCI_A2_BASE, *data);
}

void transmitScanner(char *data)
{
    int x = 0;
    for (x = 0; x < scannerDataLength; x++)
    {
        MAP_UART_transmitData(EUSCI_A0_BASE, data[x]);
    }
}

void initClock(void)
{
    /* Configuring pins for peripheral/crystal usage */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    /* Setting the external clock frequency. */
    CS_setExternalClockSourceFrequency(0, HFXT_FREQ);
    /* Starting HFXT in non-bypass mode without a timeout. Before start change VCORE to 1 to support the 48MHz frequency */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    CS_startHFXT(false);
    /* Initializing MCLK to HFXT (effectively 48MHz) */
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
}
