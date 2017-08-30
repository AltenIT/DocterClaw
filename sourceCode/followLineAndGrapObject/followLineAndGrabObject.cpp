//-----------------Includes-----------------
#include <Arduino.h>
#include <MeMegaPi.h>
#include "MeEEPROM.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MePort.h>

//-----------------Defines-----------------
//#define DEBUG_ENABLE 1

#define LINE_BAND_LOST		-1
#define LINE_BAND_MIN		6
#define LINE_BAND_DOWN		13
#define LINE_BAND_MIDDEL	20
#define LINE_BAND_UP		27
#define LINE_BAND_MAX		34

#define TIME_TO_SEE_NO_LINE		15		//time in milliseconds to wait before a line is found
#define MOTOR_DEAD_BAND			80		// space from 0 to ... when the motor is not strong enough to move the robot
#define BAND_TO_COUNTER			10
#define MAX_PWM					200

#define FORWARDS		120
#define BACKWARDS		-120
#define STOP			0

// define all analog ports
#if defined(__AVR_ATmega1280__)|| defined(__AVR_ATmega2560__)
int16_t analogs[16] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12,
		A13, A14, A15 };
#endif

typedef enum {
	STATE_FOLLOW_LINE, STATE_TAKE_OBJECT, STATE_DONE, STATE_TURN, STATE_DROP
} State;

typedef enum {
	ARM_DOWN, GRAB, ARM_UP, FORWARD, ARM_DOWN_2, RELEASE, ARM_UP_2, DONE
} StateTakeObject;

//-----------------Global variables-----------------
// define the encoders for the motors
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderOnBoard Encoder_3(SLOT3);

// define for the sensor that looks for a line
MeLineFollower line(PORT_8);

// define for DC motor
MeMegaPiDCMotor motor4(PORT4B);

// speed for the vehicle
int16_t moveSpeed = 100;
int16_t rotateSpeed = 120;

// to check of the line is seen before
int8_t flagFoundLine = 0;

// count the time the line in seen to be sure of the line is seen
uint8_t timesSeenLine = 0;

// Global variable to measure time
uint32_t time;
uint32_t savedTime = 0;

// speed for the left and right motor
int16_t motorRight = 0;
int16_t motorLeft = 0;

// PD controller variables
int32_t kp = 25;
int32_t kd = 15;
int16_t baseSpeed = 100;
int32_t lastError = 0;

// states voor the current code
State currentState = STATE_FOLLOW_LINE;
StateTakeObject currentStateTakeObject = GRAB;

//-----------------Function prototypes-----------------
void isr_process_encoder1(void);
void isr_process_encoder2(void);
void isr_process_encoder3(void);
int8_t calculateSensorValue(void);
void initEncoderMotor(void);
void configureIO(void);
uint32_t getMeasuredTime(void);
void startMeasureTime(void);
int16_t limitMotorSpeed(int16_t speed);
void followLine(void);
void releaseObject(StateTakeObject stateWhenDone);
void grabObject(StateTakeObject stateWhenDone);
void armUp(StateTakeObject stateWhenDone);
void armDown(StateTakeObject stateWhenDone);

//-----------------Setup-----------------
void setup() {
	initEncoderMotor();
	configureIO();

	// start serial communication over the USB
	Serial.begin(115200);

	//Serial2 is used to communicate to the raspberry pi
	//Serial3 is used to communicate with bluetooth

	startMeasureTime();

}

//-----------------Loop-----------------

/**
 * \par Function
 *    loop
 * \par Description
 *    main function for arduino to follow a line
 */
void loop() {
	switch (currentState) {
	case STATE_FOLLOW_LINE:
		followLine();
		break;
	case STATE_TAKE_OBJECT:
		switch (currentStateTakeObject) {
		case ARM_DOWN:
			// arm down
			armDown(GRAB);
			break;
		case GRAB:
			// close hand
			grabObject(ARM_UP);
			break;
		case ARM_UP:
			// arm up
			armUp(FORWARD);
			startMeasureTime();
			break;
		case FORWARD:
			// move a little bit forward to prevent the robot of finding the line to fast before the turn
			Encoder_1.setMotorPwm(moveSpeed);
			Encoder_2.setMotorPwm(-moveSpeed);

			if (getMeasuredTime() > 100) {
				currentStateTakeObject = ARM_DOWN_2;
				currentState = STATE_TURN;
			}
			break;
		}
		break;
	case STATE_TURN:
		// rotate left until both light sensors found the line

		Encoder_1.setMotorPwm(rotateSpeed);
		Encoder_2.setMotorPwm(rotateSpeed);
		if (line.readSensors() == S1_IN_S2_IN) {
			timesSeenLine++;
		} else {
			if (timesSeenLine > 0) {
				timesSeenLine--;
			}
		}
		if (timesSeenLine > 3) {
			timesSeenLine = 0;
			flagFoundLine = 2;
			Encoder_1.setMotorPwm(0);
			Encoder_2.setMotorPwm(0);
			currentState = STATE_FOLLOW_LINE;
		}
		break;
	case STATE_DROP:
		switch (currentStateTakeObject) {
		case ARM_DOWN_2:
			// arm down
			armDown(RELEASE);
			break;
		case RELEASE:
			// open hand
			releaseObject(ARM_UP_2);
			break;
		case ARM_UP_2:
			// arm up
			armUp(DONE);
			break;
		case DONE:
			currentState = STATE_DONE;
		}
#if defined(DEBUG_ENABLE)
		Serial.print("State:\t");
		Serial.print(currentStateTakeObject);
		Serial.print("\tMotor 3:\t");
		Serial.print(Encoder_3.getCurPos());
		Serial.println();
#endif
		break;
	case STATE_DONE:
		//stop
		Encoder_1.setMotorPwm(0);
		Encoder_2.setMotorPwm(0);

		break;
	}

	// update the motor state
	Encoder_1.loop();
	Encoder_2.loop();
	Encoder_3.loop();
}

//-----------------Functions-----------------

void armUp(StateTakeObject stateWhenDone) {

	Encoder_3.moveTo(0, 100);

	if (Encoder_3.distanceToGo() < 10 && Encoder_3.distanceToGo() > -10) {
		currentStateTakeObject = stateWhenDone;
		Encoder_3.moveTo(0, 0);
	}
}

void armDown(StateTakeObject stateWhenDone) {

	Encoder_3.moveTo(550, 100);

	if (Encoder_3.distanceToGo() < 10 && Encoder_3.distanceToGo() > -5) {
		currentStateTakeObject = stateWhenDone;
		Encoder_3.moveTo(550, 0);
		startMeasureTime();
		if (stateWhenDone == RELEASE) {
			motor4.run(-100);
		} else if (stateWhenDone == GRAB) {
			motor4.run(100);
		}
	}
}

void grabObject(StateTakeObject stateWhenDone) {
	if (getMeasuredTime() > 2700) {
		motor4.run(0);
		startMeasureTime();
		currentStateTakeObject = stateWhenDone;
	}
}

void releaseObject(StateTakeObject stateWhenDone) {

	if (getMeasuredTime() > 2700) {
		motor4.run(0);
		startMeasureTime();
		currentStateTakeObject = stateWhenDone;
	}
}

void followLine(void) {
	if (getMeasuredTime() > 5) {
		startMeasureTime();

		// get error for the sensor
		int32_t error = calculateSensorValue() - LINE_BAND_MIDDEL;
		motorRight = 0;
		motorLeft = 0;

		// check of a line is found
		if (error > -LINE_BAND_MIDDEL + LINE_BAND_LOST) {

			// set flag that a line is found
			if (flagFoundLine < 1) {
				flagFoundLine = 1;
			}

			// pd-controler to move the robot over the line
			int32_t motorspeed = kp * error + kd * (lastError - error);
			lastError = error;
			motorRight = baseSpeed + motorspeed;
			motorLeft = (-1 * baseSpeed) + motorspeed;

		} else {
			if (flagFoundLine == 1) {
				// line is seen already so try to take object
				currentState = STATE_TAKE_OBJECT;
			} else if (flagFoundLine == 2) {
				// second time line is followed so drop object
				currentState = STATE_DROP;
			}
		}

	}

	// set the correct speed to the motors
	Encoder_1.setMotorPwm(limitMotorSpeed(motorRight));
	Encoder_2.setMotorPwm(limitMotorSpeed(motorLeft));
}

int16_t limitMotorSpeed(int16_t speed) {
	if (speed > MAX_PWM) {
		speed = MAX_PWM;
	} else if (speed > 0 && speed < MOTOR_DEAD_BAND) {
		speed = MOTOR_DEAD_BAND;
	} else if (speed < 0 && speed > -MOTOR_DEAD_BAND) {
		speed = -MOTOR_DEAD_BAND;
	} else if (speed < -MAX_PWM) {
		speed = -MAX_PWM;
	}

	return speed;
}

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

int8_t calculateSensorValue(void) {
	static int8_t ret2 = LINE_BAND_MIDDEL;

	uint8_t val = line.readSensors();

	switch (val) {
	case S1_IN_S2_IN:
		if (ret2 == LINE_BAND_LOST) {
			ret2 = LINE_BAND_MIDDEL;
		} else if (ret2 > LINE_BAND_MIDDEL) {
			ret2--;
		} else if (ret2 < LINE_BAND_MIDDEL) {
			ret2++;
		}
		break;
	case S1_IN_S2_OUT:
		if (ret2 == LINE_BAND_LOST) {
			ret2 = (LINE_BAND_MIDDEL + LINE_BAND_UP) / 2;
		} else if (ret2 < LINE_BAND_UP) {
			ret2++;
		} else if (ret2 > LINE_BAND_UP) {
			ret2--;
		}
		break;
	case S1_OUT_S2_IN:
		if (ret2 == LINE_BAND_LOST) {
			ret2 = (LINE_BAND_MIDDEL + LINE_BAND_DOWN) / 2;
		} else if (ret2 > LINE_BAND_DOWN) {
			ret2--;
		} else if (ret2 < LINE_BAND_DOWN) {
			ret2++;
		}

		break;
	case S1_OUT_S2_OUT:
		if (ret2 >= LINE_BAND_UP && ret2 < LINE_BAND_MAX) {
			ret2++;
		} else if (ret2 <= LINE_BAND_DOWN && ret2 > LINE_BAND_MIN) {
			ret2--;
		} else if (ret2 == LINE_BAND_MIDDEL) {
			ret2 = LINE_BAND_LOST;
		} else if (ret2 < LINE_BAND_UP && ret2 > LINE_BAND_MIDDEL) {
			ret2--;
		} else if (ret2 > LINE_BAND_DOWN && ret2 < LINE_BAND_MIDDEL) {
			ret2++;
		}

		break;
	}

	return ret2;
}
