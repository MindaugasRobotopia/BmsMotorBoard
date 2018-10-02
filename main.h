/*
 * main.h
 *
 *  Created on: 2018-07-20
 *      Author: Mindaugas
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#define BARCODELENGTH ((int)7)
#define RXLENGTH ((int)4)
unsigned int rx485[RXLENGTH];
int rxCnt;
int serialId;

volatile uint32_t speedSet1;
volatile uint32_t speedSet2;
volatile uint32_t speedSet3;
volatile uint32_t speedMotor1;
volatile uint32_t speedMotor2;
volatile uint32_t speedMotor3;

int faultStatus;
uint_fast16_t motor1Duty;
uint_fast16_t motor2Duty;
uint_fast16_t motor3Duty;
int dutyMax;
float reduction1;
float reduction2;
float reduction3;
float speedWheel1Rpm;
float speedWheel2Rpm;
float speedWheel3Rpm;
int gearbox1;
int gearbox2;
int gearbox3;
int encoderPpr;
volatile float error1, error2, error3;
volatile float output1;
volatile float output2;
volatile float output3;
volatile uint32_t value1;
volatile float error_i,error_d,error_old1, error_old2, error_old3;
float s1;
int bcCnt;
int proxiCnt;
int byteRx;

unsigned int barCodeValue;
int distanceMode;
//unsigned char barCodeReceive[256];
char irdaReceive;
unsigned int barCodeReceive[BARCODELENGTH];
void rxParseCommand(void);
void sendToSlave(char data);
void sendStop(void);
void sendStart(void);
void stopMotor01(void);
void setGpio(void);
void switchMotor1(int state);
void switchMotor3(int state);
void switchLed1(int state);
void switchLed2(int state);
void setScannerTrig(int state);
void setScannerWake(int state);
void switchSecondMotorBoard(int state);
void switchTransmitterRs485(int state);
void toggleWdi(void);
void setDirectionOfContacts(int direction);
void setDirectionOfWheels(int direction);
void setDutyMotor1(int duty);
void setDutyMotor2(int duty);
void setDutyMotor3(int duty);
void initPwmMotor1(void);
void initPwmMotor2(void);
void initPwmMotor3(void);
void startCounterMotor1(void);
void startCounterMotor2(void);
void startCounterMotor3(void);
int motor1timerValue(void);
void setMotor1TimerValue(int value);
int motor2timerValue(void);
void setMotor2TimerValue(int value);
int motor3timerValue(void);
void setMotor3TimerValue(int value);
void switchIrda(int state);
void setIrdaMode(int mode);
void initialConditions(void);
void initMotor1(int direction);
void initMotor2(int direction);
void initMotor3(int direction);
void stopCounterMotor1(void);
void stopCounterMotor2(void);
void stopCounterMotor3(void);
void stopMotor1(void);
void stopMotor3(void);
void faultMotor1(void);
void faultMotor3(void);
void setPWM(float val);

void speedControl3(void);
void initSoftStop(void);
void initI2c(void);
void initPwmMotor0(void);
void driveMotor1(int duty);
void switchSpeedControl(int state);
void speedSetMotor1(int speed);
void increaseSpeed1(int speed);
void decreaseSpeed1(int speed);
void driveMotor1Forward(void);
void driveMotor2Forward(void);
void driveMotor2(int duty);
void driveMotor3Up(void);
void driveMotor3Down(void);
void driveMotor3(int duty);
void getStatusMotor3(void);
void driveForward(void);
void stopTimerA2(void);
void driveBackward(void);

void initUart(void);
void SetProximityCommandRegister(uint8_t* Command);
void SetProximityCommandRegisterSeflfTimed(void);
void SetProximityCommandRegisterEn(void);
void ReadProxiValue(void);
void ReadProxiId(void);
void i2cTransmitRead(void);
void i2cTransmit1(void);
void i2cTransmit2(void);
void SetProximityIntTresL(uint_fast16_t value);
void SetProximityIntTresH(uint_fast16_t value);
void SetProximityIntClear(void);
void SetProximityInt(void);
void SetProximityRate(void);
void SetProximityCurrent(int value);
void SetProximityDisable(void);
void calibrateProximity(void);
void proximitySensor(void);

void enable3V3M(void);


void transmitScanner(char *data);
void ScannerStop(void);
void ScannerTrig(void);
void initProximity(void);
void initIrda(void);
void transmitIrda(char *data);
void setProximity(void);
void initTimer32(void);

void clearOverrun(void);
void stopTransmitRs485(void);
void transmitRs485(char *data);
void rs485test(void);
const char *f2[4];
//const char * f1;
long int f3;
long int f5;
char serialData[2];
char serialRx[6];


int rsCnt;
//char * ffr[2];

#endif /* MAIN_H_ */
