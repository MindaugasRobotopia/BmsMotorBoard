/*
 * board.h
 */


#ifndef BOARD_H_
#define BOARD_H_

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "Const.h"
//#include "main.h"
#include "uartRs.h"

#define MAX_SPEED                   ((int)3000)
#define STOP_SPEED                  ((int)100)
#define DISTANCE                    ((float)300)
#define DISTANCE_STOP               ((float)50)
#define WHEEL_DIAMETER              ((int)20) // wheel diameter in mm
#define GERBOX_1                    ((int)150);
#define GERBOX_2                    ((int)150);
#define GERBOX_3                    ((int)150);
#define ENCODER                     ((int)7);
#define LOW_SPEED                   ((uint32_t)400)
#define HFXT_FREQ                   ((uint32_t)48000000)
#define START_PERIOD                ((int)150)
#define STOP_PERIOD                 ((int)50)
#define TIMER1_COUNT                ((uint32_t)192000)//uint32_t timer1count = 192000; //1ms = 48000 counts (48MHz CPU)
#define MOTOR_TIMER_PERIOD          2000// 4000 // 12kHz (48MHz CPU)
#define PID_COUNT                   ((int)40)
#define SPEED_SET_1                 ((uint32_t)800)
#define SPEED_SET_2                 ((uint32_t)800)
#define SPEED_SET_3                 ((uint32_t)500)
#define SOFTSTART_INCREMENT         ((int)4)
//unsigned int HighThreshold;

#define FINISHED ((int)1)

#define SCANNER_TRIG_PIN        GPIO_PIN0
#define SCANNER_TRIG_PORT       GPIO_PORT_P1
#define SCANNER_WAKE_PIN        GPIO_PIN1
#define SCANNER_WAKE_PORT       GPIO_PORT_P1
#define SCANNER_UARTRX_PIN      GPIO_PIN2
#define SCANNER_UARTRX_PORT     GPIO_PORT_P1
#define SCANNER_UARTTX_PIN      GPIO_PIN3
#define SCANNER_UARTTX_PORT     GPIO_PORT_P1
#define PROX_INT_PIN            GPIO_PIN5
#define PROX_INT_PORT           GPIO_PORT_P1
#define PROX_SDA_PIN            GPIO_PIN6
#define PROX_SDA_PORT           GPIO_PORT_P1
#define PROX_SCL_PIN            GPIO_PIN7
#define PROX_SCL_PORT           GPIO_PORT_P1
#define LED1_PIN                GPIO_PIN0
#define LED1_PORT               GPIO_PORT_P2
#define LED2_PIN                GPIO_PIN1
#define LED2_PORT               GPIO_PORT_P2
#define RS485_UARTRX_PIN        GPIO_PIN2
#define RS485_UARTRX_PORT       GPIO_PORT_P2
#define RS485_UARTTX_PIN        GPIO_PIN3
#define RS485_UARTTX_PORT       GPIO_PORT_P2
#define SD_IRDA_PIN             GPIO_PIN0
#define SD_IRDA_PORT            GPIO_PORT_P3
#define MODE_IRDA_PIN           GPIO_PIN1
#define MODE_IRDA_PORT          GPIO_PORT_P3
#define IRDA_UARTRX_PIN         GPIO_PIN2
#define IRDA_UARTRX_PORT        GPIO_PORT_P3
#define IRDA_UARTTX_PIN         GPIO_PIN3
#define IRDA_UARTTX_PORT        GPIO_PORT_P3
#define POSITION_A_MOTOR2_PIN   GPIO_PIN5
#define POSITION_A_MOTOR2_PORT  GPIO_PORT_P3
#define POSITION_A_MOTOR3_PIN   GPIO_PIN6
#define POSITION_A_MOTOR3_PORT  GPIO_PORT_P3
#define POSITION_A_MOTOR1_PIN   GPIO_PIN7
#define POSITION_A_MOTOR1_PORT  GPIO_PORT_P3
#define EN_MOTOR3_PIN           GPIO_PIN2
#define EN_MOTOR3_PORT          GPIO_PORT_P4
#define EN_MOTORS_PIN           GPIO_PIN3
#define EN_MOTORS_PORT          GPIO_PORT_P4
#define DIR_MOTOR1_PIN          GPIO_PIN4
#define DIR_MOTOR1_PORT         GPIO_PORT_P4
#define DIR_MOTOR3_PIN          GPIO_PIN5
#define DIR_MOTOR3_PORT         GPIO_PORT_P4
#define FAULT_MOTORS_PIN        GPIO_PIN6
#define FAULT_MOTORS_PORT       GPIO_PORT_P4
#define DE_RS485_PIN            GPIO_PIN0
#define DE_RS485_PORT           GPIO_PORT_P5
#define POSITION_P1_MOTOR3_PIN  GPIO_PIN1
#define POSITION_P1_MOTOR3_PORT GPIO_PORT_P5
#define POSITION_P2_MOTOR3_PIN  GPIO_PIN2
#define POSITION_P2_MOTOR3_PORT GPIO_PORT_P5
#define EN_MOTORBOARD2_PIN      GPIO_PIN5
#define EN_MOTORBOARD2_PORT     GPIO_PORT_P5
#define PWM3_PIN                GPIO_PIN6
#define PWM3_PORT               GPIO_PORT_P6
#define PWM1_PIN                GPIO_PIN7
#define PWM1_PORT               GPIO_PORT_P6
#define PWM2_PIN                GPIO_PIN1
#define PWM2_PORT               GPIO_PORT_P8
#define WDI_PIN                 GPIO_PIN0
#define WDI_PORT                GPIO_PORT_P8

uint32_t stepMotor1;
uint32_t stepMotor2;
uint32_t stepMotor3;
int ssStopMotor1;
int ssStopMotor2;
int ssStopMotor3;
int ssMotor1;
int ssMotor2;
int ssMotor3;
int ssTimer;
int statusPid;
int pidCnt;
volatile int q;
int TXByteCtr;
char i2cSend[2];
unsigned char i2cValue[1];
unsigned char i2cReceive[2];
int switchProxi;
unsigned int ProxiValue;
int motor1status;
int motor2status;
int motor3status;
uint_fast16_t motor1timer;
uint_fast16_t motor2timer;
uint_fast16_t motor3timer;
int statusMotor3;
int scannerDataLength;
uint32_t encoderTimerClk;
int txState;
int cntDisableTx;
int dataLength;

void transmitI2c(void);
void initClock(void);
void speedControl1(void);
void speedControl2(void);
void softStartMotor1(void);
void softStartMotor2(void);
void softStartMotor3(void);
void softStopMotor1(void);
void softStopMotor2(void);
void softStopMotor3(void);
#endif /* BOARD_H_ */
