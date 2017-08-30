/**
 * @file mian.cpp
 * @author Leon Stam
 * @date 17-8-2017
 * @brief File containing example to drive a square with the makeblock robot.
 */

//-----------------Includes-----------------
#include <Arduino.h>
#include <MeMegaPi.h>
#include "MeEEPROM.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MePort.h>

//-----------------Defines-----------------
//#define DEBUG_ENABLE 1

#define FORWARDS	130
#define BACKWARDS	-130
#define STOP 		0

// define all analog ports
#if defined(__AVR_ATmega1280__)|| defined(__AVR_ATmega2560__)
int16_t analogs[16] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12,
		A13, A14, A15 };
#endif

//-----------------Global variables-----------------
// define the encoders for the motors on the correct slot
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderOnBoard Encoder_3(SLOT3);

// define for the sensor that looks for a line
MeLineFollower line(PORT_8);

// define for DC motor
MeMegaPiDCMotor motor4(PORT4B);

// current state
uint8_t state = 0;

// variable to measure time
uint32_t time = 0;

// configure gyro sensor
MeGyro gyro;
double ZangleStart;
double ZangleNew;
double currentXangle;
double currentYangle;
double currentZangle;
double tmpAngle;

//-----------------Function prototypes-----------------
void initEncoderMotor(void);
void isr_process_encoder1(void);
void isr_process_encoder2(void);
void isr_process_encoder3(void);
void configureIO(void);
void startMeasureAngle(void);
double getMeasuredRightAngle(void);
double getMeasuredLeftAngle(void);
void startMeasureTime(void);
uint32_t getMeasuredTime(void);

//-----------------Setup-----------------
//Code that is performed one time
void setup() {
	initEncoderMotor();
	configureIO();

	// start serial communication over the USB
	Serial.begin(115200);

	//Serial2 is used to communicate to the raspberry pi
	//Serial3 is used to communicate with bluetooth

	gyro.begin();

	delay(1000);

	state = 0;
	startMeasureTime();
}

//-----------------Loop-----------------
//Code that is performed as long as the controller is running
void loop() {

	switch (state) {
	case 0:
		//drive

		Encoder_1.setTarPWM(FORWARDS);
		Encoder_2.setTarPWM(BACKWARDS);

		if (getMeasuredTime() > 1500) {
			Encoder_1.setTarPWM(STOP);
			Encoder_2.setTarPWM(STOP);
			startMeasureAngle();
			state = 1;
		}

		break;
	case 1:
		//turn left

		Encoder_1.setTarPWM(FORWARDS);
		Encoder_2.setTarPWM(FORWARDS);

		if (getMeasuredLeftAngle() >= 85 && getMeasuredLeftAngle() <= 95) {
			Encoder_1.setTarPWM(STOP);
			Encoder_2.setTarPWM(STOP);
			startMeasureTime();
			state = 2;
		}
		break;
	case 2:
		//stop

		if (getMeasuredTime() > 1000) {
			startMeasureTime();
			state = 0;
		}

		break;
	}

	//update gyro sensor
	gyro.update();

	// update the motor state
	Encoder_1.loop();
	Encoder_2.loop();
	Encoder_3.loop();
}

//-----------------Functions-----------------

double savedAngle = 0;

void startMeasureAngle(void) {
	savedAngle = gyro.getAngleZ();
}

double getMeasuredRightAngle(void) {
	double currentAngle = gyro.getAngleZ() + 180;
	double ret = 0;

	if (currentAngle >= savedAngle) {
		ret = currentAngle - savedAngle;
	} else {
		ret = 360 - savedAngle + currentAngle;
	}

	ret -= 180;

	if (ret < 0) {
		ret += 360;
	}

	return ret;
}

double getMeasuredLeftAngle(void) {
	double currentAngle = gyro.getAngleZ() + 180;
	double ret = 0;

	if (currentAngle <= savedAngle) {
		ret = savedAngle - currentAngle;
	} else {
		ret = 360 - currentAngle + savedAngle;
	}

	ret -= 180;

	if (ret < 0) {
		ret += 360;
	}

	return ret;
}

uint32_t savedTime = 0;

void startMeasureTime(void) {
	savedTime = millis();
}

uint32_t getMeasuredTime(void) {
	return (millis() - savedTime);
}

// Function to configure the pins of the controller
void configureIO(void) {
	// set pin 13 as output for a LED
	pinMode(13, OUTPUT);
}

// function to set the correct parameters of the motors
void initEncoderMotor(void) {
	// set the functions to use when a interrupt is called
	attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
	attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
	attachInterrupt(Encoder_3.getIntNum(), isr_process_encoder3, RISING);

	//Set Pwm 8KHz
	TCCR1A = _BV(WGM10);
	TCCR1B = _BV(CS11) | _BV(WGM12);

	TCCR2A = _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(CS21);

	// set the number of pulses from the encoder
	Encoder_1.setPulse(8);
	Encoder_2.setPulse(8);
	Encoder_3.setPulse(8);

	// set parameters for the position PID controller
	Encoder_1.setRatio(46);
	Encoder_2.setRatio(46);
	Encoder_3.setRatio(75);

	// set parameters for the position PID controller
	Encoder_1.setPosPid(1.8, 0, 1.2);
	Encoder_2.setPosPid(1.8, 0, 1.2);
	Encoder_3.setPosPid(1.8, 0, 1.2);

	// set parameters for the speed PID controller
	Encoder_1.setSpeedPid(0.18, 0, 0);
	Encoder_2.setSpeedPid(0.18, 0, 0);
	Encoder_3.setSpeedPid(0.18, 0, 0);

	// set the motion mode of the motor
	Encoder_1.setMotionMode(DIRECT_MODE);
	Encoder_2.setMotionMode(DIRECT_MODE);
	Encoder_3.setMotionMode(DIRECT_MODE);
}

/**
 * \par Function
 *    isr_process_encoder1
 * \par Description
 *    This function use to process the interrupt of encoder1 drvicer on board,
 *    used to calculate the number of pulses.
 */
void isr_process_encoder1(void) {
	if (digitalRead(Encoder_1.getPortB()) == 0) {
		Encoder_1.pulsePosMinus();
	} else {
		Encoder_1.pulsePosPlus();
	}
}

/**
 * \par Function
 *    isr_process_encoder2
 * \par Description
 *    This function use to process the interrupt of encoder2 drvicer on board,
 *    used to calculate the number of pulses.
 */
void isr_process_encoder2(void) {
	if (digitalRead(Encoder_2.getPortB()) == 0) {
		Encoder_2.pulsePosMinus();
	} else {
		Encoder_2.pulsePosPlus();
	}
}

/**
 * \par Function
 *    isr_process_encoder3
 * \par Description
 *    This function use to process the interrupt of encoder3 drvicer on board,
 *    used to calculate the number of pulses.
 */
void isr_process_encoder3(void) {
	if (digitalRead(Encoder_3.getPortB()) == 0) {
		Encoder_3.pulsePosMinus();
	} else {
		Encoder_3.pulsePosPlus();
	}
}
