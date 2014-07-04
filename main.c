/*
 * File:   main.c
 * Author: Tomasz Ciborski
 *
 * Created on February 7, 2014, 11:10 AM
 *
 * Extra information:
 * Fosc = 40 MHz => Tosc = 25 ns
 * 10 bit PWM resolution
 * Unipolar
 * Right motor - motor connected to I/O 1
 *
 * Calibration Mode takes precedence over Race Mode
 */

#define MAX_SPEED 30000 //32767 // can be from 0 to 32767
#define MIN_SPEED 0 // can be from 0 to 32767

#define K_P 120 //80
// #define K_I 1
// #define K_D 1

#define CORNERING_K_P 700 // 36 -> linear from 0 to 32k
#define CORNERING_UPPER_LIMIT MAX_SPEED
#define CORNERING_LOWER_LIMIT 27000
#define CORNERING_B CORNERING_K_P * 500 // type range 300

#define STOP_RANGE_TRESHOLD 300 // type range //can be from 0 to 900
#define LOST_LINE_SUM_COMPUTATIONS_START_TRESHOLD 1500 // 1500

//#define MOTORS_OFF

/*
+-----------------------+------------+-----------+--------------+-----------+-----------+------------+-----------+----------+
| School's setup        |            |           |              |           |           |            |           |          |
|                       | 0          | 1         | 2            | 3         | 4         | 5          | 6         | 7        |
| RE                    |            |           |              |           |           |            |           |          |
| RD                    |            |           |              |           |           |            |           |          |
| RJ                    |            |           |              |           |           | PB1        | PIEZO     |          |
| RB                    | PB2        |           |              |           |           |            |           |          |
| RG                    |            |           |              |           |           |            | N/A       | N/A      |
| RH                    | Q1         | Q2        |              |           | SW4/AN12  | SW5/AN13   | SW6/AN14  | SW7/AN15 |
| RF                    | LD1/AN5    | LD2/AN6   | LD3/AN7      | LD4/AN8   | LD5/AN9   | LD6/AN10   | LD7/AN11  | LD8      |
| RC                    |            |           | SW0          | SW1       | SW2       | SW3        | N/A       | N/A      |
| RA                    | POT/AN0    | TEMP/AN1  | LIGHT S./AN2 | AN3       | Q3        | AN4        | N/A       | N/A      |
|                       |            |           |              |           |           |            |           |          |
|                       |            |           |              |           |           |            |           |          |
| Final setup           |            |           |              |           |           |            |           |          |
|                       | 0          | 1         | 2            | 3         | 4         | 5          | 6         | 7        |
| RE                    | Bipolar 1  | Dir 1     | Bipolar 2    | Dir 2     | Enable    | LED CALIB. | LED RACE. |          |
| RD                    |            |           |              |           |           |            |           |          |
| RJ                    |            |           |              |           |           |            |           |          |
| RB                    | INT0/CALIB | INT1/RACE |              |           |           |            |           |          |
| RG                    |            |           | RX2/BatLan   | CCP4/PWM1 | CCP5/PWM2 |            | N/A       | N/A      |
| RH                    |            |           |              |           | AN12/S8   | AN13/S9    | AN14      | AN15     |
| RF                    | AN5/S1     | AN6/S2    | AN7/S3       | AN8/S4    | AN9/S5    | AN10/S6    | AN11/S7   |          |
| RC                    |            |           |              |           |           |            | N/A       | N/A      |
| RA                    | AN0/AA+    | AN1/AA-   | AN2/AB+      | AN3/AB-   |           | AN4/S0     | N/A       | N/A      |
|                       |            |           |              |           |           |            |           |          |
|                       |            |           |              |           |           |            |           |          |
| PID calibration setup |            |           |              |           |           |            |           |          |
|                       | 0          | 1         | 2            | 3         | 4         | 5          | 6         | 7        |
| RE                    | Bipolar 1  | Dir 1     | Bipolar 2    | Dir 2     | Enable    | LED CALIB. | LED RACE  |          |
| RD                    | Kp 0       | Kp 1      | Ki 0         | Ki 1      | Kd 0      | Kd 1       |           |          |
| RJ                    |            |           |              |           |           |            |           |          |
| RB                    | INT0/CALIB | INT1/RACE |              |           |           |            |           |          |
| RG                    |            |           |              | CCP4/PWM1 | CCP5/PWM2 |            | N/A       | N/A      |
| RH                    |            |           |              |           | AN12/S8   | AN13/S9    | AN14/Kdn  | AN15/Kdd |
| RF                    | AN5/S1     | AN6/S2    | AN7/S3       | AN8/S4    | AN9/S5    | AN10/S6    | AN11/S7   |          |
| RC                    |            |           |              |           |           |            | N/A       | N/A      |
| RA                    | AN0/Kpn    | AN1/Kpd   | AN2/Kin      | AN3/Kid   |           | AN4/S0     | N/A       | N/A      |
+-----------------------+------------+-----------+--------------+-----------+-----------+------------+-----------+----------+
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include <adc.h>
#include <delays.h>
#include <timers.h>
#include <pwm.h>
#include <EEP.h>

typedef unsigned int uint;
typedef unsigned char uchar;
typedef unsigned long ulong;



#define TRUE 1
#define FALSE 0
#define INT_MAX 32767
#define INT_MIN -32768
#define UINT_MAX 65535
#define UINT_MIN 0
#define SENSORS_COUNT 10
#define ADC_RESOLUTION 1024
#define ADC_MAX 1023
#define ADC_MIN 0
#define SENSOR_READINGS_MIDDLE_POINT (ADC_RESOLUTION*SENSORS_COUNT) / 2
#define MOTOR_RIGHT 0
#define MOTOR_LEFT 1
#define MOTOR_BOTH 2
#define READINGS_HISTORY_LENGTH 10
#define RIGHT 1
#define CENTER 0
#define LEFT -1

#define POWER_DIFFERENCE_MAX INT_MAX //INT_MAX + (-INT_MIN)
#define POWER_DIFFERENCE_MIN INT_MIN //-POWER_DIFFERENCE_MAX

#define KPN_ADDRESS 0x00
#define KPD_ADDRESS KPN_ADDRESS + sizeof(uint)
#define KIN_ADDRESS KPD_ADDRESS + sizeof(uint)
#define KID_ADDRESS KIN_ADDRESS + sizeof(uint)
#define KDN_ADDRESS KID_ADDRESS + sizeof(uint)
#define KDD_ADDRESS KDN_ADDRESS + sizeof(uint)
#define DRAG_RACE_ADJUSTMENT_ADDRESS KDD_ADDRESS + sizeof(uint)
#define SENSORS_CALIBRATED_MAX_ADDRESS DRAG_RACE_ADJUSTMENT_ADDRESS + sizeof(uint) * SENSORS_COUNT
#define SENSORS_CALIBRATED_MIN_ADDRESS SENSORS_CALIBRATED_MAX_ADDRESS + sizeof(uint) * SENSORS_COUNT

#pragma udata history
int ReadingsHistory[READINGS_HISTORY_LENGTH][SENSORS_COUNT + 2];
uint ReadingsHistoryPointer = 0;
#pragma udata
char ComputationsEnabled = TRUE;

int SensorsCalibratedMax[SENSORS_COUNT];
int SensorsCalibratedMin[SENSORS_COUNT];
char RaceModeEnabled = FALSE;
char CalibrationModeEnabled = FALSE;
char FreewheelsModeEnabled = FALSE;

// PROFILING:
int ProfilingTimer = 0;

// DEBUGGING:
int bp;

void RanInErrorState();
void DelayMiliseconds(char miliseconds);

char IsCalibrateButtonPressed();
char IsRaceButtonPressed();

void CalibrateSensors(char (*resolveRunCondition)());
void DelayHalfSecond();
void FreewheelsMode(char enable);

void DelayQuarterSecond();

/* ------------- INTERNAL FUNCTIONS ------------- */

char BitRead(char *byte, char bitNumber) {
    return (*byte) & (1 << bitNumber);
}

void BitSet(char *byte, char bitNumber) {
    (*byte) |= 1 << bitNumber;
}

void BitClear(char *byte, char bitNumber) {
    (*byte) &= ~(1 << bitNumber);
}

void ConfigureBipolar1() {
    TRISEbits.TRISE0 = 0;
}

void SetBipolar1(int value) {
    LATEbits.LATE0 = value;
}

void ConfigureBipolar2() {
    TRISEbits.TRISE2 = 0;
}

void SetBipolar2(int value) {
    LATEbits.LATE2 = value;
}

void ConfigureDirection1() {
    TRISEbits.TRISE1 = 0;
}

void SetDirection1(int value) {
    LATEbits.LATE1 = value;
}

void ConfigureDirection2() {
    TRISEbits.TRISE3 = 0;
}

void SetDirection2(int value) {
    LATEbits.LATE3 = value;
}

void ConfigureEnable() {
    TRISEbits.TRISE4 = 0;
}

void SetEnable(int value) {
    LATEbits.LATE4 = value;
}

void ConfigureCCP4() {
    TRISGbits.TRISG3 = 0;
}

void ConfigureCCP5() {
    TRISGbits.TRISG4 = 0;
}

void SelectSensor(uchar sensorNumber) {
    switch (sensorNumber) {
        case 0:
        {
            SetChanADC(ADC_CH4);
            break;
        }
        case 1:
        {
            SetChanADC(ADC_CH5);
            break;
        }
        case 2:
        {
            SetChanADC(ADC_CH6);
            break;
        }
        case 3:
        {
            SetChanADC(ADC_CH7);
            break;
        }
        case 4:
        {
            SetChanADC(ADC_CH8);
            break;
        }
        case 5:
        {
            SetChanADC(ADC_CH9);
            break;
        }
        case 6:
        {
            SetChanADC(ADC_CH10);
            break;
        }
        case 7:
        {
            SetChanADC(ADC_CH11);
            break;
        }
        case 8:
        {
            SetChanADC(ADC_CH12);
            break;
        }
        case 9:
        {
            SetChanADC(ADC_CH13);
            break;
        }
        default:
        {
            RanInErrorState();
            break;
        }
    }
}

int GetSensorValue(uchar sensorNumber) {
    int result = 0;

    switch (sensorNumber) {
        case 0:
        {
            result = 9;
            break;
        }
        case 1:
        {
            result = 7;
            break;
        }
        case 2:
        {
            result = 5;
            break;
        }
        case 3:
        {
            result = 3;
            break;
        }
        case 4:
        {
            result = 1;
            break;
        }
        case 5:
        {
            result = -1;
            break;
        }
        case 6:
        {
            result = -3;
            break;
        }
        case 7:
        {
            result = -5;
            break;
        }
        case 8:
        {
            result = -7;
            break;
        }
        case 9:
        {
            result = -9;
            break;
        }
        default:
        {
            RanInErrorState();
            break;
        }
    }

    result *= 100;
    return result;
}

void ConfigureGlobalInterrupts() {
    INTCONbits.GIE = 1;
}

void ConfigurePeripheralInterrupts() {
    INTCONbits.PEIE = 1;
}

void ConfigureInterruptRB0() {
    //INTCONbits.RBIE = 1;
    INTCONbits.INT0IE = 1;
}

void ConfigureInterruptRB1() {
    INTCON3bits.INT1IE = 1;
}

void WriteToEEPROM(uint value, uint address) {
    // little endian
    uint highByteAddress = address;
    uint lowByteAddress = address + 1;
    uchar highByte = (value & 0xFF00) >> 8;
    uchar lowByte = value & 0x00FF;
    Busy_eep();
    Write_b_eep(highByteAddress, highByte);
    DelayMiliseconds(1);
    Busy_eep();
    Write_b_eep(lowByteAddress, lowByte);
    DelayMiliseconds(1);
    Busy_eep();
}

uint ReadFromEEPROM(uint address) {
    uint value;
    uint highByteAddress = address;
    uint lowByteAddress = address + 1;
    uchar highByte;
    uchar lowByte;
    Busy_eep();
    highByte = Read_b_eep(highByteAddress);
    Busy_eep();
    lowByte = Read_b_eep(lowByteAddress);
    Busy_eep();
    value = (highByte << 8) | lowByte;
    return value;
}
//char (*resolveRunCondition)()

char DebounceValue(char (*functionWhichReturnsDigitalValue) ()) {
    char ones = 0, zeroes = 0, i;
    for (i = 0; i < 9; i++) {
        if (functionWhichReturnsDigitalValue()) {
            ones++;
        } else {
            zeroes++;
        }
        DelayMiliseconds(10);
    }
    return ones > zeroes;
}

void ConfigureCalibrationLED() {
    TRISEbits.TRISE5 = 0;
}

void SetCalibrationLED(char value) {
    LATEbits.LATE5 = value;
}

void ConfigureRaceLED() {
    TRISEbits.TRISE6 = 0;
}

void SetRaceLED(char value) {
    LATEbits.LATE6 = value;
}

char ReturnTrue() {
    return TRUE;
}

char ReturnFalse() {
    return FALSE;
}

/* ------------- EO INTERNAL FUNCTIONS ------------- */

uint CalculateDutyCycle(int speed) {
    uint uspeed;
    speed -= 32767; // reverse scale (AL to AH)
    if (speed < 0) {
        speed = -speed;
    }
    uspeed = (uint) speed; // convert +int to uint
    uspeed = uspeed << 1; // scale up uspeed to full uint
    uspeed = uspeed >> 6; //scale down uspeed (16 bits) down to 10 bits

    return uspeed;
}

// Speed can be any int value from max negative to max positive

//void MotorRight(int speed) {
//    SetDCPWM4(CalculateDutyCycle(speed));
//
//}
//
//void MotorLeft(int speed) {
//    SetDCPWM5(CalculateDutyCycle(speed));
//    SetDirection2(speed > 0);
//}

void FreewheelsMode(char enable) {
    if (enable) {
        SetEnable(FALSE);
        SetDCPWM4(UINT_MAX);
        SetDCPWM5(UINT_MAX);
        FreewheelsModeEnabled = TRUE;
    } else {
        SetEnable(TRUE);
        FreewheelsModeEnabled = FALSE;
    }
}

void MotorSpeed(int value, char motor) {
    uint dutyCycle;
    //    uchar directionBit = 0;

#ifdef MOTORS_OFF
    value = 0;
#endif

    dutyCycle = CalculateDutyCycle(value);

    switch (motor) {
        case MOTOR_RIGHT:
        {
            SetDCPWM4(dutyCycle);
            break;
        }
        case MOTOR_LEFT:
        {
            SetDCPWM5(dutyCycle);
            break;
        }
        case MOTOR_BOTH:
        {
            SetDCPWM4(dutyCycle);
            //            SetDirection1(directionBit);

            SetDCPWM5(dutyCycle);
            //            SetDirection2(directionBit);
            break;
        }
        default:
        {
            RanInErrorState();
            break;
        }
    }


}

int ReadCurrent(char motor) {
    int positive, negative;

    switch (motor) {
        case MOTOR_RIGHT:
        {
            SetChanADC(ADC_CH0);
            positive = ReadAdcChannel();
            SetChanADC(ADC_CH1);
            negative = ReadAdcChannel();
            break;
        }
        case MOTOR_LEFT:
        {
            SetChanADC(ADC_CH2);
            positive = ReadAdcChannel();
            SetChanADC(ADC_CH3);
            negative = ReadAdcChannel();
            break;
        }
        default:
        {
            RanInErrorState();
            break;
        }
    }

    return positive - negative;
}

void ProfilerReset() {
    WriteTimer0(0);
}

void ProfilerStart() {
    OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_2);
    INTCONbits.T0IE = 1; // enable interrupts for timer0 ??
    ProfilerReset();
}

uint ProfilerRead() {
    ProfilingTimer = ReadTimer0();
    return ProfilingTimer;
}

void ProfilerStop() {
    CloseTimer0();
}

void SetupPWM() {
    OpenTimer2(
            TIMER_INT_OFF &
            T2_PS_1_4);

    ConfigureCCP4();
    ConfigureCCP5();
    OpenPWM4(255); // 10 bit resolution
    OpenPWM5(255);

    MotorSpeed(0, MOTOR_BOTH);
}

void SetupEnable() {
    ConfigureEnable();
    SetEnable(1);
}

void SetupRightMotor() {
    ConfigureBipolar1();
    SetBipolar1(0); // disable
    ConfigureDirection1();
    SetDirection1(1);
}

void SetupLeftMotor() {
    ConfigureBipolar2();
    SetBipolar2(0); // disable
    ConfigureDirection2();
    SetDirection2(1);
}

void SetupSensors() {
    OpenADC(ADC_FOSC_64 &
            ADC_RIGHT_JUST &
            ADC_2_TAD,
            ADC_INT_OFF &
            ADC_VREFPLUS_VDD &
            ADC_VREFMINUS_VSS,
            0b0000); // all 16 analogue inputs
}

void SetupIOBoard() {
    // Configure ADC
    OpenADC(ADC_FOSC_64 &
            ADC_RIGHT_JUST &
            ADC_2_TAD,
            ADC_CH0 &
            ADC_INT_OFF &
            ADC_VREFPLUS_VDD &
            ADC_VREFMINUS_VSS,
            0b1100); // first 3 are Analogue Inputs

    // Configure LEDs as output
    TRISF = 0;
}

/* INTERRUPT SERVICING */



#pragma interrupt InterruptServiceRoutine

void InterruptServiceRoutine() {
    // Calibration button
    if (INTCONbits.INT0IF) {
        // if there was a press
        if (DebounceValue(IsCalibrateButtonPressed)) // make sure it was firm
        {
            // and toggle calibration flag
            if (CalibrationModeEnabled) {
                CalibrationModeEnabled = FALSE;
            } else {
                CalibrationModeEnabled = TRUE;
            }
            SetCalibrationLED(CalibrationModeEnabled);
        }

        // take down interrupt flag
        INTCONbits.INT0IF = 0;
    }

    // Race button
    if (INTCON3bits.INT1IF) {
        if (DebounceValue(IsRaceButtonPressed)) {
            if (RaceModeEnabled) {
                RaceModeEnabled = FALSE;
            } else {
                RaceModeEnabled = TRUE;
            }

            SetRaceLED(RaceModeEnabled);
        }

        INTCON3bits.INT1IF = 0;
    }

    if (INTCONbits.TMR0IF) {
        char swap = TRUE;
        // there was an OVERFLOW!! WARNING!

        MotorSpeed(0, MOTOR_BOTH);

        while (TRUE) {
            SetCalibrationLED(swap);
            SetRaceLED(!swap);

            DelayHalfSecond();
            swap = !swap;
        }

        // never reach this piece
        INTCONbits.TMR0IF = 0;
    }
}

#pragma code HighISR=0x08

void JumpToInterruptServiceRoutine() {
    _asm GOTO InterruptServiceRoutine _endasm
}
#pragma code

/* EO INTERRUPT SERVICING */

void RanInErrorState() {
    // TODO
}

void SetupButtons() {
    ConfigureInterruptRB0();
    ConfigureInterruptRB1();
    ConfigurePeripheralInterrupts(); //TODO Check if works w/o this line
    ConfigureGlobalInterrupts();
}

void SetupControlLEDs() {
    ConfigureCalibrationLED();
    SetCalibrationLED(0);

    ConfigureRaceLED();
    SetRaceLED(0);
}

void Setup() {
    SetupPWM();
    SetupRightMotor();
    SetupLeftMotor();
    SetupEnable();
    SetupControlLEDs();
    FreewheelsMode(FALSE);
    // Actuators setup done

    SetupButtons();
    SetupSensors();
    // Sensors setup done
}

int ReadAdcChannel() {
    int valueRead;
    ConvertADC();
    while (BusyADC());
    valueRead = ReadADC();
    return valueRead;
}

int ApplyCutOff(int array[], int percentAboveAverage) {
    uchar sensorCounter;
    int average;
    int sum = 0;
    int cutoff;

    for (sensorCounter = 0; sensorCounter < SENSORS_COUNT; sensorCounter++) {
        sum += array[sensorCounter];
    }
    average = sum / SENSORS_COUNT;

    cutoff = average + (int) ((long) ((long) percentAboveAverage * (long) 100) / (long) average);

    sum = 0;
    for (sensorCounter = 0; sensorCounter < SENSORS_COUNT; sensorCounter++) {
        if (array[sensorCounter] < cutoff) {
            array[sensorCounter] = 0;
        }
        sum += array[sensorCounter];
    }

    return sum;
}

// Returns position

int ReadSensors(int array[], char calibrated) { // timing = 330.2 us
    uchar sensorCounter;
    int readValue;
    int sum = 0;

    for (sensorCounter = 0; sensorCounter < SENSORS_COUNT; sensorCounter++) {
        int min = SensorsCalibratedMin[sensorCounter];
        int max = SensorsCalibratedMax[sensorCounter];

        SelectSensor(sensorCounter); // timing = 2.55 us
        readValue = ReadAdcChannel(); // max read is 1023 // timing = 7.25 us
        // above 2 timing = 9.35 us

        if (calibrated) { // timing = 23.3 us
            // means scaled
            // to be from 0 to 1023
            if (readValue > max) {
                readValue = max;
            } else if (readValue < min) {
                readValue = min;
            }
            readValue -= min;
            readValue = ((long) readValue * (long) ADC_MAX) / (max - min); // timing with ulongs = 22.85 us // timing with ints = 9.05 us
        }

        array[sensorCounter] = readValue; // timing = 0.85 us
        sum += readValue; // timing = 0.65 us
    }

    return sum;
}

// Returns position from -900 to +900

int CalculateSensorsWeightedAverage(int readings[]) {
    char sensorCounter;

    int sensorValue;
    int sensorWeight;

    long weightsSum = 0; // mianownik
    long weightTimesValue = 0; // liczebnik

    int weightedAverage = 0;

    for (sensorCounter = 0; sensorCounter < SENSORS_COUNT; sensorCounter++) {
        sensorValue = GetSensorValue(sensorCounter);
        sensorWeight = readings[sensorCounter];

        weightTimesValue += (long) sensorWeight * (long) sensorValue;
        weightsSum += sensorWeight;
    }

    weightedAverage = (int) (weightTimesValue / weightsSum);
    return weightedAverage;
}

void WriteArray(int array[], int address) {
    int i;

    for (i = 0; i < SENSORS_COUNT; i++) {
        WriteToEEPROM(array[i], address + i * sizeof (int));
    }
}

void ReadArray(int address, int destination[]) {
    int i;

    for (i = 0; i < SENSORS_COUNT; i++) {
        destination[i] = ReadFromEEPROM(address + i * sizeof (int));
    }
}

void SaveCalibratedValuesToEEPROM() {
    WriteArray(SensorsCalibratedMax, SENSORS_CALIBRATED_MAX_ADDRESS);
    WriteArray(SensorsCalibratedMin, SENSORS_CALIBRATED_MIN_ADDRESS);
}

void LoadCalibratedValuesFromEEPROM() {
    ReadArray(SENSORS_CALIBRATED_MAX_ADDRESS, SensorsCalibratedMax);
    ReadArray(SENSORS_CALIBRATED_MIN_ADDRESS, SensorsCalibratedMin);
}

char IsCalibrateButtonPressed() {
    return PORTBbits.RB0;
}

char IsRaceButtonPressed() {
    return PORTBbits.RB1;
}

/* DELAYS */

void DelayMiliseconds(char miliseconds) {
    Delay10KTCYx(miliseconds);
}

void DelayQuarterSecond() {
    Delay10KTCYx(250);
}

void DelayHalfSecond() {
    Delay10KTCYx(250);
    Delay10KTCYx(250);
}

void Delay1Second() {
    Delay10KTCYx(250);
    Delay10KTCYx(250);
    Delay10KTCYx(250);
    Delay10KTCYx(250);
}

void Delay2Second() {
    Delay1Second();
    Delay1Second();
}

void Delay3Second() {
    Delay1Second();
    Delay1Second();
    Delay1Second();
}

void Delay9Second() {
    Delay3Second();
    Delay3Second();
    Delay3Second();
}

/* EO DELAYS */

/* I/O BOARD FUNCTIONS */

int ReadPot() {
    int valueRead;
    SetChanADC(ADC_CH0);
    valueRead = ReadAdcChannel();
    return valueRead;
}

int ReadPotScaled() {
    int pot = ReadPot();
    pot = pot - 512;
    pot <<= 6;
    return pot;
}

int IsPb1Pressed() {
    return !PORTJbits.RJ5;
}

int IsPb2Pressed() {
    return !PORTBbits.RB0;
}

void WriteToLeds(char value) {
    LATF = value;
}

/* EO I/O BOARD FUNCTIONS */

void GoStraightAdjustment() {
    SetupPWM();
    SetupRightMotor();
    SetupLeftMotor();
    SetupEnable();
    SetupIOBoard();

    while (TRUE) {
        int pot = ReadPotScaled();
        int maxSpeedRight = INT_MAX / 9;
        int maxSpeedLeft = INT_MAX / 9 + pot / 9;

        WriteToEEPROM(pot, DRAG_RACE_ADJUSTMENT_ADDRESS);

        MotorSpeed(maxSpeedRight, MOTOR_RIGHT);
        MotorSpeed(maxSpeedLeft, MOTOR_LEFT);
    }
}

void GoStraight() {
    int pot = ReadFromEEPROM(DRAG_RACE_ADJUSTMENT_ADDRESS);
    Setup();

    while (TRUE) {
        int maxSpeedRight = INT_MAX / 4;
        int maxSpeedLeft = INT_MAX / 4 + pot / 4;

        MotorSpeed(maxSpeedRight, MOTOR_RIGHT);
        MotorSpeed(maxSpeedLeft, MOTOR_LEFT);
    }
}

void DoEightLoop() {
    int leftMotorAdjustment = ReadFromEEPROM(DRAG_RACE_ADJUSTMENT_ADDRESS);
    int motorRightSpeed = INT_MAX / 3;
    int motorLeftSpeed = INT_MAX / 3 + leftMotorAdjustment / 3;
    Setup();

    while (1) {
        MotorSpeed(0, MOTOR_BOTH);
        Delay1Second();

        MotorSpeed(motorRightSpeed, MOTOR_RIGHT);
        MotorSpeed(motorLeftSpeed, MOTOR_LEFT);
        Delay1Second();

        MotorSpeed(0, MOTOR_RIGHT);
        MotorSpeed(motorLeftSpeed, MOTOR_LEFT);
        Delay2Second();

        MotorSpeed(motorRightSpeed, MOTOR_RIGHT);
        MotorSpeed(motorLeftSpeed, MOTOR_LEFT);
        Delay1Second();

        MotorSpeed(motorRightSpeed, MOTOR_RIGHT);
        MotorSpeed(0, MOTOR_LEFT);
        Delay2Second();

        MotorSpeed(motorRightSpeed, MOTOR_RIGHT);
        MotorSpeed(motorLeftSpeed, MOTOR_LEFT);
        Delay1Second();
    }
}

void TechDemo1() {
    SetupPWM();
    SetupRightMotor();
    SetupLeftMotor();
    SetupEnable();
    SetupIOBoard();

    while (1) {
        int pb1 = IsPb1Pressed();
        int pb2 = IsPb2Pressed();
        int pot = ReadPotScaled();

        if (pb1 && !pb2) {// pb1 pressed only
            // right motor
            MotorSpeed(pot, MOTOR_RIGHT);
            MotorSpeed(0, MOTOR_LEFT);
            WriteToLeds(0x0F);
        } else if (!pb1 && pb2) {// pb2 only
            // left motor
            MotorSpeed(0, MOTOR_RIGHT);
            MotorSpeed(pot, MOTOR_LEFT);
            WriteToLeds(0xF0);
        } else if (pb1 && pb2) {// both
            // both motors reversed way
            MotorSpeed(pot, MOTOR_RIGHT);
            MotorSpeed(-pot, MOTOR_LEFT);
            WriteToLeds(0xFF);
        } else {// none
            // both motors same direction
            MotorSpeed(pot, MOTOR_RIGHT);
            MotorSpeed(pot, MOTOR_LEFT);
            WriteToLeds(0x00);
        }
    }
}

void EraseSensorsCalibrationData() {
    memset(SensorsCalibratedMax, 0, sizeof (uint) * SENSORS_COUNT);
    memset(SensorsCalibratedMin, 0xFF, sizeof (uint) * SENSORS_COUNT);
}

// min is 0, max is 1023

void FillSensorsCalibrationDataWithDefaultValues() {
    int i;
    int *valuePtr;
    for (i = 0; i < SENSORS_COUNT; i++) {
        valuePtr = SensorsCalibratedMax;
        *(valuePtr + i) = 1023;
    }

    memset(SensorsCalibratedMin, 0, sizeof (int) * SENSORS_COUNT);
}

void FillSensorsCalibrationDataWithPreCalibrationValues() {
    int i;
    int *valuePtr;

    for (i = 0; i < SENSORS_COUNT; i++) {
        valuePtr = SensorsCalibratedMax;
        *(valuePtr + i) = 0;
    }

    for (i = 0; i < SENSORS_COUNT; i++) {
        valuePtr = SensorsCalibratedMin;
        *(valuePtr + i) = 1023;
    }
}

char IsCalibrationMode() {
    return CalibrationModeEnabled;
}

char IsRaceMode() {
    return RaceModeEnabled;
}

void CalibrateSensors(char (*resolveRunCondition)()) {
    uchar sensorNumber;
    uint tempArray[SENSORS_COUNT];
    while ((resolveRunCondition) ()) { // timing = 146 us
        ReadSensors(tempArray, FALSE);

        // find max/min and store in global arrays
        for (sensorNumber = 0; sensorNumber < SENSORS_COUNT; sensorNumber++) {
            // find maximum
            if (tempArray[sensorNumber] > SensorsCalibratedMax[sensorNumber]) {
                SensorsCalibratedMax[sensorNumber] = tempArray[sensorNumber];
            }

            //find minimum
            if (tempArray[sensorNumber] < SensorsCalibratedMin[sensorNumber]) {
                SensorsCalibratedMin[sensorNumber] = tempArray[sensorNumber];
            }
        }
    }
}

void CapValueLong(long * value, long min, long max) {
    if (*value > max) {
        *value = max;
    } else if (*value < min) {
        *value = min;
    }
}

void CapValueInt(int * value, int min, int max) {
    if (*value > max) {
        *value = max;
    } else if (*value < min) {
        *value = min;
    }
}

ulong SumUintArray(uint array[], int count) {
    char counter;
    ulong sum = 0;

    for (counter = 0; counter < count; counter++) {
        sum += array[counter];
    }

    return sum;
}

// Uses history for that
// -1 -> lost on the left
//  0 -> end of line
// +1 -> lost on right

char FindLostLine() {
    int historyCounter;
    long sum = 0;
    int average;
    // spr czy wa jest bardziej po prawej czy lewej
    // srednia z wa
    for (historyCounter = 0; historyCounter < READINGS_HISTORY_LENGTH; historyCounter++) {
        sum += (ReadingsHistory[historyCounter][11]);
    }
    average = (int) (sum / (long) READINGS_HISTORY_LENGTH);

    // mamy srednia od -900 do +900
    if (average > STOP_RANGE_TRESHOLD) {// gites // from 400
        // byla po prawej ostatnio
        return RIGHT;
    } else if (average < -STOP_RANGE_TRESHOLD) {
        // byla po lewej
        return LEFT;
    } else {// gites
        // end of line
        return CENTER;
    }
}

char TrackHistory() {
    char readyToAnalyse = FALSE;
    // History save
    ReadingsHistoryPointer++;
    if (ReadingsHistoryPointer == READINGS_HISTORY_LENGTH) {
        readyToAnalyse = TRUE;
    }
    ReadingsHistoryPointer = ReadingsHistoryPointer % READINGS_HISTORY_LENGTH;
    // End of history save
    return readyToAnalyse;
}

void CopyArrayToArray(int arrSrc[], int arrDst[]) {
    char i;
    for (i = 0; i < SENSORS_COUNT; i++) {
        arrDst[i] = arrSrc[i];
    }
}

void FullBreak() {
//    SetDirection1(0);
//    SetDirection2(0);
//    MotorSpeed(INT_MAX, MOTOR_BOTH);
//
//    DelayMiliseconds(200);
//
//    MotorSpeed(0, MOTOR_BOTH);
//
//    SetDirection1(1);
//    SetDirection2(1);
}

// pos -900 to +900

int CalculateMaxSpeed(int position)//int upperLimit, int lowerLimit)
{
    long speedModifier;
    long maxSpeed;

        if (position < 0) {
        position = -position;
    }

    speedModifier = (long)position * CORNERING_K_P;

    maxSpeed = (MAX_SPEED - speedModifier);
    maxSpeed += CORNERING_B;

    CapValueLong(&maxSpeed, (long) CORNERING_LOWER_LIMIT, (long) CORNERING_UPPER_LIMIT);

    return (int) maxSpeed;
}

void Race() {
    uint tempSensorReadings[SENSORS_COUNT];
    int sensorReadingsSum;

    int maxSpeed = MAX_SPEED; // 80% //19660; // 60%
    int minSpeed = MIN_SPEED;

    int error, lastError = 0;
    int derivative;
    long long integral = 0;
    long powerDifference;

    int motorLeftSpeed;
    int motorRightSpeed;

    char canAnalyseHistory = FALSE;

    char eol = FALSE;

    Setup();
    FillSensorsCalibrationDataWithDefaultValues();

    while (TRUE) {
        FreewheelsMode(TRUE);
        while (IsCalibrationMode()) {
            FillSensorsCalibrationDataWithPreCalibrationValues();
            CalibrateSensors(IsCalibrationMode); // each inner loop timing ~= 146 us
            // runs as long as the IsCalibrationMode() returns true

            // save to EEPROM
            //            WriteArray(SensorsCalibratedMax, SENSORS_CALIBRATED_MAX_ADDRESS);
            //            WriteArray(SensorsCalibratedMin, SENSORS_CALIBRATED_MIN_ADDRESS);

            bp = 0;
        }

        while (IsRaceMode()) {
            // read from EEPROM
            bp = 0;

            //            ReadArray(SENSORS_CALIBRATED_MAX_ADDRESS, SensorsCalibratedMax);
            //            ReadArray(SENSORS_CALIBRATED_MIN_ADDRESS, SensorsCalibratedMin);

            bp = 0;

            while (IsRaceMode()) {
                if (FreewheelsModeEnabled) {
                    FreewheelsMode(FALSE);
                }

                sensorReadingsSum = ReadSensors(tempSensorReadings, TRUE);
                //sensorReadingsSum = ApplyCutOff(tempSensorReadings, 0);
                if (ComputationsEnabled) {
                    eol = FALSE;
                    if (TrackHistory()) {
                        canAnalyseHistory = TRUE;
                    }

                    CopyArrayToArray(tempSensorReadings, ReadingsHistory[ReadingsHistoryPointer]);
                    error = CalculateSensorsWeightedAverage(ReadingsHistory[ReadingsHistoryPointer]);

                    ReadingsHistory[ReadingsHistoryPointer][SENSORS_COUNT] = sensorReadingsSum; // save sum to 11th element
                    ReadingsHistory[ReadingsHistoryPointer][SENSORS_COUNT + 1] = error; // save WA to 12th element

                    maxSpeed = CalculateMaxSpeed(error);

                    bp = 0;

#ifdef K_D
                    derivative = error - lastError;
                    lastError = error;
#endif
#ifdef K_I
                    integral += error;
#endif

                    powerDifference =
                            -(long long) error * K_P
#ifdef K_I
                            - (long long) integral * K_I
#endif
#ifdef K_D
                            - (long long) derivative * K_D
#endif
                            ;

                    CapValueLong(&powerDifference, (long) POWER_DIFFERENCE_MIN, (long) POWER_DIFFERENCE_MAX);

                    if (powerDifference < 0) {
                        motorRightSpeed = maxSpeed + powerDifference;
                        motorLeftSpeed = maxSpeed;
                    } else {
                        motorRightSpeed = maxSpeed;
                        motorLeftSpeed = maxSpeed - powerDifference;
                    }
                }


                if (sensorReadingsSum > LOST_LINE_SUM_COMPUTATIONS_START_TRESHOLD) {
                    ComputationsEnabled = TRUE;
                } else {
                    ComputationsEnabled = FALSE;

                    if (canAnalyseHistory) {
                        char lastTimeSeenOn = FindLostLine();

                        maxSpeed = MAX_SPEED;

                        switch (lastTimeSeenOn) {
                            case(LEFT):
                            {
                                motorRightSpeed = maxSpeed;
                                motorLeftSpeed = 0;
                                break;
                            }
                            case(RIGHT):
                            {

                                motorRightSpeed = 0;
                                motorLeftSpeed = maxSpeed;
                                break;
                            }
                            case(CENTER):
                            {

                                if (!eol) {
                                    FullBreak();
                                }

                                eol = TRUE;
                                motorRightSpeed = 0;
                                motorLeftSpeed = 0;
                                break;
                            }
                        }
                    } else {
                        if (!eol) {
                            FullBreak();
                        }

                        eol = TRUE;

                        motorRightSpeed = 0;
                        motorLeftSpeed = 0;
                    }
                }

                CapValueInt(&motorRightSpeed, minSpeed, maxSpeed);
                CapValueInt(&motorLeftSpeed, minSpeed, maxSpeed);

                MotorSpeed(motorRightSpeed, MOTOR_RIGHT);
                MotorSpeed(motorLeftSpeed, MOTOR_LEFT);
            }
        }
        MotorSpeed(0, MOTOR_BOTH);
    }
}

void main() {
    Race();
}