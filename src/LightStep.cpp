
//      ******************************************************************
//      *                                                                *
//      *                         ESP-LightStep                          *
//      *                                                                *
//      *            Paul Kerspe                     4.6.2020            *
//      *       based on the concept of FlexyStepper by Stan Reifel      *
//      *                                                                *
//      *         Modified by Thomas Hettasch 2021-09-15                 *
//      *                                                                *
//      ******************************************************************

// MIT License
//
// Copyright (c) 2020 Paul Kerspe
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished
// to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

//
// This library is used to control one or more stepper motors.  It requires a
// stepper driver board that has a Step and Direction interface.  The motors are
// accelerated and decelerated as they travel to the final position.  This driver
// supports changing the target position, speed or rate of acceleration while a
// motion is in progress.
//
// for more details and a manual on how to use it, check the README.md on github and the provided examples
// https://github.com/pkerspe/ESP-FlexyStepper/blob/master/README.md
//
// This library is based on the works of Stan Reifel in his FlexyStepper library:
// https://github.com/Stan-Reifel/FlexyStepper
//

#include "LightStep.h"

//
// direction signal level for "step and direction"
//
#define POSITIVE_DIRECTION LOW
#define NEGATIVE_DIRECTION !POSITIVE_DIRECTION

// ---------------------------------------------------------------------------------
//                                  Setup functions
// ---------------------------------------------------------------------------------

//
// constructor for the stepper class
//
LightStep::LightStep()
{
	this->lastStepTime_InUS = 0L;
	this->stepsPerRevolution = 200L;
	this->stepsPerMillimeter = 25.0;
	this->directionOfMotion = 0;
	this->currentPosition_InSteps = 0L;
	this->targetPosition_InSteps = 0L;
	this->setSpeedInStepsPerSecond(200);
	this->setAccelerationInStepsPerSecondPerSecond(200.0);
	this->setDecelerationInStepsPerSecondPerSecond(200.0);
	this->currentStepPeriod_InUS = 0.0;
	this->nextStepPeriod_InUS = 0.0;
	this->emergencyStopActive = false;
	this->holdEmergencyStopUntilExplicitRelease = false;
	this->isCurrentlyHomed = false;
	this->directionTowardsHome = -1;
	this->disallowedDirection = 0;
	this->activeLimitSwitch = 0; //see LIMIT_SWITCH_BEGIN and LIMIT_SWITCH_END
	this->lastStepDirectionBeforeLimitSwitchTrigger = 0;
	this->limitSwitchCheckPeformed = false;
}

LightStep::~LightStep()
{
	if (this->xHandle != NULL)
	{
		this->stopService();
	}
}

//TODO: use https://github.com/nrwiersma/ESP8266Scheduler/blob/master/examples/simple/simple.ino for ESP8266
void LightStep::startAsService(void)
{
	disableCore1WDT(); // we have to disable the Watchdog timer to prevent it from rebooting the ESP all the time another option would be to add a vTaskDelay but it would slow down the stepper
	xTaskCreatePinnedToCore(
			LightStep::taskRunner, /* Task function. */
			"FlexyStepper",               /* String with name of task (by default max 16 characters long) */
			2000,                         /* Stack size in bytes. */
			this,                         /* Parameter passed as input of the task */
			1,                            /* Priority of the task, 1 seems to work just fine for us */
			&this->xHandle,               /* Task handle. */
			1 /* the cpu core to use */
	);
}

void LightStep::taskRunner(void *parameter)
{
	LightStep *stepperRef = static_cast<LightStep *>(parameter);
	for (;;)
	{
		stepperRef->processMovement();
		//vTaskDelay(1); // This would be a working solution to prevent the WDT to fire (if not disabled, yet it will cause noticeably less smooth stepper movements / lower frequencies)
	}
}

void LightStep::stopService(void)
{
	vTaskDelete(this->xHandle);
	this->xHandle = NULL;
}

bool LightStep::isStartedAsService()
{
	return (this->xHandle != NULL);
}

/**
 * get the overall max stack size since task creation (since the call to startAsService() )
 * This function is used to determine if the stacksize is large enough and has more of a debugging purpose.
 * Return the minimum amount of free bytes on the stack that has been measured so far.
 */
long LightStep::getTaskStackHighWaterMark()
{
	if (this->isStartedAsService())
	{
		return uxTaskGetStackHighWaterMark(this->xHandle);
	}
	return 0;
}

/**
 * get the distance in steps to the currently set target position.
 * 0 is returned if the stepper is already at the target position.
 * The returned value is signed, depending on the direction to move to reach the target
 */
long LightStep::getDistanceToTargetSigned()
{
	return (this->targetPosition_InSteps - this->currentPosition_InSteps);
}

/**
 * perform an emergency stop, causing all movements to be canceled instantly
 * the optional parameter 'holdUntilReleased' allows to define if the emergency stop shall only affect the current motion (if any) 
 * or if it should hold the emergency stop status (kind of a latching functionality) until the releaseEmergencyStop() function is called explicitly.
 * Default for holdUntilReleased is false (if paremter is ommitted)
 */
void LightStep::emergencyStop(bool holdUntilReleased)
{
	this->holdEmergencyStopUntilExplicitRelease = holdUntilReleased;
	this->emergencyStopActive = (!this->motionComplete() || this->holdEmergencyStopUntilExplicitRelease);
	if (this->_emergencyStopTriggeredCallback)
	{
		this->_emergencyStopTriggeredCallback();
	}
}

/**
 * releases an emergency stop that has previously been engaded using a call to emergencyStop(true)
 */
void LightStep::releaseEmergencyStop()
{
	this->emergencyStopActive = false;
	if (this->_emergencyStopReleasedCallback)
	{
		this->_emergencyStopReleasedCallback();
	}
}

/**
 *  configure the direction in which to move to reach the home position
 *  Accepts 1 or -1 as allowed values. Other values will be ignored
 */
void LightStep::setDirectionToHome(signed char directionTowardHome)
{
	if (directionTowardHome == -1 || directionTowardHome == 1)
	{
		this->directionTowardsHome = directionTowardHome;
	}
}

/**
 * Notification of an externaly detected limit switch activation
 * Accepts LIMIT_SWITCH_BEGIN (-1) or LIMIT_SWITCH_END (1) as parameter values to indicate 
 * whether the limit switch near the begin (direction of home position) or at the end of the movement has ben triggered.
 * It is strongly recommended to perform debouncing before calling this function to prevent issues when button is released and retriggering the limit switch function
 */
void LightStep::setLimitSwitchActive(byte limitSwitchType)
{
	if (limitSwitchType == LIMIT_SWITCH_BEGIN || limitSwitchType == LIMIT_SWITCH_END || limitSwitchType == LIMIT_SWITCH_COMBINED_BEGIN_AND_END)
	{
		this->activeLimitSwitch = limitSwitchType;
		this->limitSwitchCheckPeformed = false; //set flag for newly set limit switch trigger
		if (this->_limitTriggeredCallback)
		{
			this->_limitTriggeredCallback(); //TODO: this function is called from within a ISR in ESPStepperMotorServer thus we should try to delay calling of the callback to the backound task / process Steps function
		}
	}
}

/**
 * clear the limit switch flag to allow movement in both directions again
 */
void LightStep::clearLimitSwitchActive()
{
	this->activeLimitSwitch = 0;
}

/**
 * get the current direction of motion of the connected stepper motor
 * returns 1 for "forward" motion
 * returns -1 for "backward" motion
 * returns 0 if the stepper has reached its destination position and is not moving anymore
 */
int LightStep::getDirectionOfMotion(void)
{
	return this->directionOfMotion;
}

/**
 * returns true if the stepper is currently in motion and moving in the direction of the home position.
 * Depends on the settings of setDirectionToHome() which defines where "home" is...a rather philosophical question :-)
 */
bool LightStep::isMovingTowardsHome()
{
	return (this->directionOfMotion == this->directionTowardsHome);
}

/*
* connect the stepper object to the IO pins
* stepPinNumber = IO pin number for the Step signale
* directionPinNumber = IO pin number for the direction signal
*/
void LightStep::connectToPins(byte stepPinNumber, byte directionPinNumber)
{
	this->stepPin = stepPinNumber;
	this->directionPin = directionPinNumber;

	// configure the IO pins
	pinMode(stepPin, OUTPUT);
	digitalWrite(stepPin, LOW);
	
	if(directionPin<255){
	pinMode(directionPin, OUTPUT);
	}
	digitalWrite(directionPin, LOW);
}

/*
* setup an IO pin to trigger an external brake for the motor.
* This is an optional step, set to -1 to disable this function (which is default)
* the active state parameter defines if the external brake is configured in an active high (pin goes high to enable the brake) or active low (pin goes low to activate the brake) setup.
* active high = 1, active low = 2
* Will be set to ative high by default or if an invalid value is given
*/
void LightStep::setBrakePin(byte brakePin, byte activeState)
{
	this->brakePin = brakePin;
	if (activeState == LightStep::ACTIVE_HIGH || activeState == LightStep::ACTIVE_LOW)
	{
		this->brakePinActiveState = activeState;
	}
	else
	{
		this->brakePinActiveState = LightStep::ACTIVE_HIGH;
	}

	if (this->brakePin >= 0)
	{
		// configure the IO pins
		pinMode(this->brakePin, OUTPUT);
		this->deactivateBrake();
		_isBrakeConfigured = true;
	}
	else
	{
		_isBrakeConfigured = false;
	}
}

/**
 * set a delay in milliseconds between stopping the stepper motor and engaging the pyhsical brake (trigger the eternal pin configured via setBrakePin() ).
 * Default is 0, resulting in immediate triggering of the motor brake once the motor stops moving.
 * This value does NOT affect the triggering of the brake in cade of an emergency stop. In this case the brake will always get triggered without delay
 */
void LightStep::setBrakeEngageDelayMs(unsigned long delay)
{
	this->_brakeEngageDelayMs = delay;
}

/**
 * set a timeout in milliseconds after which the brake shall be released once triggered and no motion is performed by the stpper motor.
 * By default the value is -1 indicating, that the brake shall never be automatically released, as long as the stepper motor is not moving to a new position.
 * Value must be larger than 1 (Even though 1ms delay does probably not make any sense since physical brakes have a delay that is most likely higher than that just to engange)
 */
void LightStep::setBrakeReleaseDelayMs(signed long delay)
{
	if (delay < 0)
	{
		this->_brakeReleaseDelayMs = -1;
	}
	else
	{
		this->_brakeReleaseDelayMs = delay;
	}
}

/**
 * activate (engage) the motor brake (if any is configured, otherwise will do nothing) 
 */
void LightStep::activateBrake()
{
	if (this->_isBrakeConfigured)
	{
		digitalWrite(this->brakePin, (this->brakePinActiveState == LightStep::ACTIVE_HIGH) ? 1 : 0);
		this->_isBrakeActive = true;
		this->_timeToEngangeBrake = LONG_MAX;
	}
}

/**
 * deactivate (release) the motor brake (if any is configured, otherwise will do nothing) 
 */
void LightStep::deactivateBrake()
{
	if (this->_isBrakeConfigured)
	{
		digitalWrite(this->brakePin, (this->brakePinActiveState == LightStep::ACTIVE_HIGH) ? 0 : 1);
		this->_isBrakeActive = false;
		this->_timeToReleaseBrake = LONG_MAX;
		this->_hasMovementOccuredSinceLastBrakeRelease = false;

		//TODO: add delay here if configured as to https://github.com/pkerspe/ESP-StepperMotor-Server/issues/16
	}
}

bool LightStep::isBakeActive()
{
	return this->_isBrakeActive;
}

// ---------------------------------------------------------------------------------
//                     Public functions with units in millimeters
// ---------------------------------------------------------------------------------

//
// set the number of steps the motor has per millimeters
//
void LightStep::setStepsPerMillimeter(float motorStepsPerMillimeter)
{
	stepsPerMillimeter = motorStepsPerMillimeter;
}

//
// get the current position of the motor in millimeters, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in millimeters returned
//
float LightStep::getCurrentPositionInMillimeters()
{
	return ((float)getCurrentPositionInSteps() / stepsPerMillimeter);
}

//
// set the current position of the motor in millimeters, this does not move the
// motor
//
void LightStep::setCurrentPositionInMillimeters(
		float currentPositionInMillimeters)
{
	setCurrentPositionInSteps((long)round(currentPositionInMillimeters *
																				stepsPerMillimeter));
}

//
// set the maximum speed, units in millimeters/second, this is the maximum speed
// reached while accelerating
//  Enter:  speedInMillimetersPerSecond = speed to accelerate up to, units in
//            millimeters/second
//
void LightStep::setSpeedInMillimetersPerSecond(float speedInMillimetersPerSecond)
{
	setSpeedInStepsPerSecond(speedInMillimetersPerSecond * stepsPerMillimeter);
}

//
// set the rate of acceleration, units in millimeters/second/second
//  Enter:  accelerationInMillimetersPerSecondPerSecond = rate of acceleration,
//          units in millimeters/second/second
//
void LightStep::setAccelerationInMillimetersPerSecondPerSecond(
		float accelerationInMillimetersPerSecondPerSecond)
{
	setAccelerationInStepsPerSecondPerSecond(
			accelerationInMillimetersPerSecondPerSecond * stepsPerMillimeter);
}

//
// set the rate of deceleration, units in millimeters/second/second
//  Enter:  decelerationInMillimetersPerSecondPerSecond = rate of deceleration,
//          units in millimeters/second/second
//
void LightStep::setDecelerationInMillimetersPerSecondPerSecond(
		float decelerationInMillimetersPerSecondPerSecond)
{
	setDecelerationInStepsPerSecondPerSecond(
			decelerationInMillimetersPerSecondPerSecond * stepsPerMillimeter);
}

//
// home the motor by moving until the homing sensor is activated, then set the
// position to zero, with units in millimeters
//  Enter:  directionTowardHome = 1 to move in a positive direction, -1 to move
//            in a negative directions
//          speedInMillimetersPerSecond = speed to accelerate up to while moving
//            toward home, units in millimeters/second
//          maxDistanceToMoveInMillimeters = unsigned maximum distance to move
//            toward home before giving up
//          homeSwitchPin = pin number of the home switch, switch should be
//            configured to go low when at home
//  Exit:   true returned if successful, else false
//
bool LightStep::moveToHomeInMillimeters(signed char directionTowardHome,
																							 float speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters,
																							 int homeLimitSwitchPin)
{
	return (moveToHomeInSteps(directionTowardHome,
														speedInMillimetersPerSecond * stepsPerMillimeter,
														maxDistanceToMoveInMillimeters * stepsPerMillimeter,
														homeLimitSwitchPin));
}

//
// move relative to the current position, units are in millimeters, this function
// does not return until the move is complete
//  Enter:  distanceToMoveInMillimeters = signed distance to move relative to the
//          current position in millimeters
//
void LightStep::moveRelativeInMillimeters(float distanceToMoveInMillimeters)
{
	setTargetPositionRelativeInMillimeters(distanceToMoveInMillimeters);

	while (!processMovement())
		;
}

//
// setup a move relative to the current position, units are in millimeters, no
// motion occurs until processMove() is called
//  Enter:  distanceToMoveInMillimeters = signed distance to move relative to the
//          current position in millimeters
//
void LightStep::setTargetPositionRelativeInMillimeters(
		float distanceToMoveInMillimeters)
{
	setTargetPositionRelativeInSteps((long)round(distanceToMoveInMillimeters *
																							 stepsPerMillimeter));
}

//
// move to the given absolute position, units are in millimeters, this function
// does not return until the move is complete
//  Enter:  absolutePositionToMoveToInMillimeters = signed absolute position to
//          move to in units of millimeters
//
void LightStep::moveToPositionInMillimeters(
		float absolutePositionToMoveToInMillimeters)
{
	setTargetPositionInMillimeters(absolutePositionToMoveToInMillimeters);

	while (!processMovement())
		;
}

//
// setup a move, units are in millimeters, no motion occurs until processMove()
// is called
//  Enter:  absolutePositionToMoveToInMillimeters = signed absolute position to
//          move to in units of millimeters
//
void LightStep::setTargetPositionInMillimeters(
		float absolutePositionToMoveToInMillimeters)
{
	setTargetPositionInSteps((long)round(absolutePositionToMoveToInMillimeters *
																			 stepsPerMillimeter));
}

float LightStep::getTargetPositionInMillimeters()
{
	return getTargetPositionInSteps() / stepsPerMillimeter;
}

//
// Get the current velocity of the motor in millimeters/second.  This functions is
// updated while it accelerates up and down in speed.  This is not the desired
// speed, but the speed the motor should be moving at the time the function is
// called.  This is a signed value and is negative when the motor is moving
// backwards.  Note: This speed will be incorrect if the desired velocity is set
// faster than this library can generate steps, or if the load on the motor is too
// great for the amount of torque that it can generate.
//  Exit:  velocity speed in steps per second returned, signed
//
float LightStep::getCurrentVelocityInMillimetersPerSecond()
{
	return (getCurrentVelocityInStepsPerSecond() / stepsPerMillimeter);
}

// ---------------------------------------------------------------------------------
//                     Public functions with units in revolutions
// ---------------------------------------------------------------------------------

//
// set the number of steps the motor has per revolution
//
void LightStep::setStepsPerRevolution(float motorStepPerRevolution)
{
	stepsPerRevolution = motorStepPerRevolution;
}

//
// get the current position of the motor in revolutions, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in revolutions returned
//
float LightStep::getCurrentPositionInRevolutions()
{
	return ((float)getCurrentPositionInSteps() / stepsPerRevolution);
}

//
// set the current position of the motor in revolutions, this does not move the
// motor
//
void LightStep::setCurrentPositionInRevolutions(
		float currentPositionInRevolutions)
{
	setCurrentPositionInSteps((long)round(currentPositionInRevolutions *
																				stepsPerRevolution));
}

//
// set the maximum speed, units in revolutions/second, this is the maximum speed
// reached while accelerating
//  Enter:  speedInRevolutionsPerSecond = speed to accelerate up to, units in
//            revolutions/second
//
void LightStep::setSpeedInRevolutionsPerSecond(float speedInRevolutionsPerSecond)
{
	setSpeedInStepsPerSecond(speedInRevolutionsPerSecond * stepsPerRevolution);
}

//
// set the rate of acceleration, units in revolutions/second/second
//  Enter:  accelerationInRevolutionsPerSecondPerSecond = rate of acceleration,
//          units in revolutions/second/second
//
void LightStep::setAccelerationInRevolutionsPerSecondPerSecond(
		float accelerationInRevolutionsPerSecondPerSecond)
{
	setAccelerationInStepsPerSecondPerSecond(
			accelerationInRevolutionsPerSecondPerSecond * stepsPerRevolution);
}

//
// set the rate of deceleration, units in revolutions/second/second
//  Enter:  decelerationInRevolutionsPerSecondPerSecond = rate of deceleration,
//          units in revolutions/second/second
//
void LightStep::setDecelerationInRevolutionsPerSecondPerSecond(
		float decelerationInRevolutionsPerSecondPerSecond)
{
	setDecelerationInStepsPerSecondPerSecond(
			decelerationInRevolutionsPerSecondPerSecond * stepsPerRevolution);
}

//
// home the motor by moving until the homing sensor is activated, then set the
//  position to zero, with units in revolutions
//  Enter:  directionTowardHome = 1 to move in a positive direction, -1 to move in
//            a negative directions
//          speedInRevolutionsPerSecond = speed to accelerate up to while moving
//            toward home, units in revolutions/second
//          maxDistanceToMoveInRevolutions = unsigned maximum distance to move
//            toward home before giving up
//          homeSwitchPin = pin number of the home switch, switch should be
//            configured to go low when at home
//  Exit:   true returned if successful, else false
//
bool LightStep::moveToHomeInRevolutions(signed char directionTowardHome,
																							 float speedInRevolutionsPerSecond, long maxDistanceToMoveInRevolutions,
																							 int homeLimitSwitchPin)
{
	return (moveToHomeInSteps(directionTowardHome,
														speedInRevolutionsPerSecond * stepsPerRevolution,
														maxDistanceToMoveInRevolutions * stepsPerRevolution,
														homeLimitSwitchPin));
}

//
// move relative to the current position, units are in revolutions, this function
// does not return until the move is complete
//  Enter:  distanceToMoveInRevolutions = signed distance to move relative to the
//          current position in revolutions
//
void LightStep::moveRelativeInRevolutions(float distanceToMoveInRevolutions)
{
	setTargetPositionRelativeInRevolutions(distanceToMoveInRevolutions);

	while (!processMovement())
		;
}

//
// setup a move relative to the current position, units are in revolutions, no
// motion occurs until processMove() is called
//  Enter:  distanceToMoveInRevolutions = signed distance to move relative to the
//            currentposition in revolutions
//
void LightStep::setTargetPositionRelativeInRevolutions(
		float distanceToMoveInRevolutions)
{
	setTargetPositionRelativeInSteps((long)round(distanceToMoveInRevolutions *
																							 stepsPerRevolution));
}

//
// move to the given absolute position, units are in revolutions, this function
// does not return until the move is complete
//  Enter:  absolutePositionToMoveToInRevolutions = signed absolute position to
//            move to in units of revolutions
//
void LightStep::moveToPositionInRevolutions(
		float absolutePositionToMoveToInRevolutions)
{
	setTargetPositionInRevolutions(absolutePositionToMoveToInRevolutions);

	while (!processMovement())
		;
}

//
// setup a move, units are in revolutions, no motion occurs until processMove()
// is called
//  Enter:  absolutePositionToMoveToInRevolutions = signed absolute position to
//          move to in units of revolutions
//
void LightStep::setTargetPositionInRevolutions(
		float absolutePositionToMoveToInRevolutions)
{
	setTargetPositionInSteps((long)round(absolutePositionToMoveToInRevolutions *
																			 stepsPerRevolution));
}

float LightStep::getTargetPositionInRevolutions()
{
	return getTargetPositionInSteps() / stepsPerRevolution;
}

//
// Get the current velocity of the motor in revolutions/second.  This functions is
// updated while it accelerates up and down in speed.  This is not the desired
// speed, but the speed the motor should be moving at the time the function is
// called.  This is a signed value and is negative when the motor is moving
// backwards.  Note: This speed will be incorrect if the desired velocity is set
// faster than this library can generate steps, or if the load on the motor is too
// great for the amount of torque that it can generate.
//  Exit:  velocity speed in steps per second returned, signed
//
float LightStep::getCurrentVelocityInRevolutionsPerSecond()
{
	return (getCurrentVelocityInStepsPerSecond() / stepsPerRevolution);
}

// ---------------------------------------------------------------------------------
//                        Public functions with units in steps
// ---------------------------------------------------------------------------------

//
// set the current position of the motor in steps, this does not move the motor
// Note: This function should only be called when the motor is stopped
//    Enter:  currentPositionInSteps = the new position of the motor in steps
//
void LightStep::setCurrentPositionInSteps(long currentPositionInSteps)
{
	currentPosition_InSteps = currentPositionInSteps;
}

//
// get the current position of the motor in steps, this functions is updated
// while the motor moves
//  Exit:  a signed motor position in steps returned
//
long LightStep::getCurrentPositionInSteps()
{
	return (currentPosition_InSteps);
}

//
// set the maximum speed, units in steps/second, this is the maximum speed reached
// while accelerating
//  Enter:  speedInStepsPerSecond = speed to accelerate up to, units in steps/second
//
void LightStep::setSpeedInStepsPerSecond(float speedInStepsPerSecond)
{
	desiredSpeed_InStepsPerSecond = speedInStepsPerSecond;
	desiredPeriod_InUSPerStep = 1000000.0 / desiredSpeed_InStepsPerSecond;
}

//
// set the rate of acceleration, units in steps/second/second
//  Enter:  accelerationInStepsPerSecondPerSecond = rate of acceleration, units in
//          steps/second/second
//
void LightStep::setAccelerationInStepsPerSecondPerSecond(
		float accelerationInStepsPerSecondPerSecond)
{
	acceleration_InStepsPerSecondPerSecond = accelerationInStepsPerSecondPerSecond;
	acceleration_InStepsPerUSPerUS = acceleration_InStepsPerSecondPerSecond / 1E12;

	periodOfSlowestStep_InUS =
			1000000.0 / sqrt(2.0 * acceleration_InStepsPerSecondPerSecond);
	minimumPeriodForAStoppedMotion = periodOfSlowestStep_InUS / 2.8;
}

//
// set the rate of deceleration, units in steps/second/second
//  Enter:  decelerationInStepsPerSecondPerSecond = rate of deceleration, units in
//          steps/second/second
//
void LightStep::setDecelerationInStepsPerSecondPerSecond(
		float decelerationInStepsPerSecondPerSecond)
{
	deceleration_InStepsPerSecondPerSecond = decelerationInStepsPerSecondPerSecond;
	deceleration_InStepsPerUSPerUS = deceleration_InStepsPerSecondPerSecond / 1E12;
}

/**
 * set the current position as the home position (Step count = 0)
 */
void LightStep::setCurrentPositionAsHomeAndStop()
{
	this->isOnWayToHome = false;
	this->currentStepPeriod_InUS = 0.0;
	this->nextStepPeriod_InUS = 0.0;
	this->directionOfMotion = 0;
	this->currentPosition_InSteps = 0;
	this->targetPosition_InSteps = 0;
	this->isCurrentlyHomed = true;
}

/**
 * start jogging in the direction of home (use setDirectionToHome() to set the proper direction) until the limit switch is hit, then set the position as home
 * Warning: This function requires a limit switch to be configured otherwise the motor will never stop jogging.
 * This is a non blocking function, you need make sure LightStep is started as service (use startAsService() function) or need to call the processMovement function manually in your main loop.
 */
void LightStep::goToLimitAndSetAsHome(callbackFunction callbackFunctionForHome, long maxDistanceToMoveInSteps)
{
	if (callbackFunctionForHome)
	{
		this->_homeReachedCallback = callbackFunctionForHome;
	}
	//the second check basically utilizes the fact the the begin and end limit switch id is 1 respectively -1 so the values are the same as the direction of the movement when the steppers moves towards of of the limits
	if (this->activeLimitSwitch == 0 || this->activeLimitSwitch != this->directionTowardsHome) 
	{
		this->setTargetPositionInSteps(this->getCurrentPositionInSteps() + (this->directionTowardsHome * maxDistanceToMoveInSteps));
	}
	this->isOnWayToHome = true; //set as last action, since other functions might overwrite it
}

void LightStep::goToLimit(signed char direction, callbackFunction callbackFunctionForLimit)
{
	if (callbackFunctionForLimit)
	{
		this->_callbackFunctionForGoToLimit = callbackFunctionForLimit;
	}

	if (this->activeLimitSwitch == 0)
	{
		this->setTargetPositionInSteps(this->getCurrentPositionInSteps() + (this->directionTowardsHome * 2000000000));
	}
	this->isOnWayToLimit = true; //set as last action, since other functions might overwrite it
}

/**
 * register a callback function to be called whenever a movement to home has been completed (does not trigger when movement passes by the home position)
 */
void LightStep::registerHomeReachedCallback(callbackFunction newFunction)
{
	this->_homeReachedCallback = newFunction;
}

/**
 * register a callback function to be called whenever a
 */
void LightStep::registerLimitReachedCallback(callbackFunction limitSwitchTriggerdCallbackFunction)
{
	this->_limitTriggeredCallback = limitSwitchTriggerdCallbackFunction;
}

/**
 * register a callback function to be called whenever a target position has been reached
 */
void LightStep::registerTargetPositionReachedCallback(positionCallbackFunction targetPositionReachedCallbackFunction)
{
	this->_targetPositionReachedCallback = targetPositionReachedCallbackFunction;
}

/**
 * register a callback function to be called whenever a emergency stop is triggered 
 */
void LightStep::registerEmergencyStopTriggeredCallback(callbackFunction emergencyStopTriggerdCallbackFunction)
{
	this->_emergencyStopTriggeredCallback = emergencyStopTriggerdCallbackFunction;
}

/**
 * register a callback function to be called whenever the emergency stop switch is released
 */
void LightStep::registerEmergencyStopReleasedCallback(callbackFunction emergencyStopReleasedCallbackFunction)
{
	this->_emergencyStopReleasedCallback = emergencyStopReleasedCallbackFunction;
}

/**
 * start jogging (continous movement without a fixed target position)
 * uses the currently set speed and acceleration settings
 * to stop the motion call the stopJogging function.
 * Will also stop when the external limit switch has been triggered using setLimitSwitchActive() or when the emergencyStop function is triggered
 * Warning: This function requires either a limit switch to be configured otherwise or manual trigger of the stopJogging/setTargetPositionToStop or emergencyStop function, the motor will never stop jogging
 */
void LightStep::startJogging(signed char direction)
{
	this->setTargetPositionInSteps(direction * 2000000000);
}

/**
 * Stop jopgging, basically an alias function for setTargetPositionToStop()
 */
void LightStep::stopJogging()
{
	this->setTargetPositionToStop();
}

//
// home the motor by moving until the homing sensor is activated, then set the
// position to zero with units in steps
//  Enter:  directionTowardHome = 1 to move in a positive direction, -1 to move in
//            a negative directions
//          speedInStepsPerSecond = speed to accelerate up to while moving toward
//            home, units in steps/second
//          maxDistanceToMoveInSteps = unsigned maximum distance to move toward
//            home before giving up
//          homeSwitchPin = pin number of the home switch, switch should be
//            configured to go low when at home
//  Exit:   true returned if successful, else false
//
bool LightStep::moveToHomeInSteps(signed char directionTowardHome,
																				 float speedInStepsPerSecond, long maxDistanceToMoveInSteps,
																				 int homeLimitSwitchPin)
{
	float originalDesiredSpeed_InStepsPerSecond;
	bool limitSwitchFlag;

	//
	// setup the home switch input pin
	//
	pinMode(homeLimitSwitchPin, INPUT_PULLUP);

	//
	// remember the current speed setting
	//
	originalDesiredSpeed_InStepsPerSecond = desiredSpeed_InStepsPerSecond;

	//
	// if the home switch is not already set, move toward it
	//
	if (digitalRead(homeLimitSwitchPin) == HIGH)
	{
		//
		// move toward the home switch
		//
		setSpeedInStepsPerSecond(speedInStepsPerSecond);
		setTargetPositionRelativeInSteps(maxDistanceToMoveInSteps * directionTowardHome);
		limitSwitchFlag = false;
		while (!processMovement())
		{
			if (digitalRead(homeLimitSwitchPin) == LOW)
			{
				limitSwitchFlag = true;
				directionOfMotion = 0;
				break;
			}
		}

		//
		// check if switch never detected
		//
		if (limitSwitchFlag == false)
			return (false);

		if (this->_limitTriggeredCallback)
		{
			this->_limitTriggeredCallback();
		}
	}
	delay(25);

	//
	// the switch has been detected, now move away from the switch
	//
	setTargetPositionRelativeInSteps(maxDistanceToMoveInSteps *
																	 directionTowardHome * -1);
	limitSwitchFlag = false;
	while (!processMovement())
	{
		if (digitalRead(homeLimitSwitchPin) == HIGH)
		{
			limitSwitchFlag = true;
			directionOfMotion = 0;
			break;
		}
	}
	delay(25);

	//
	// check if switch never detected
	//
	if (limitSwitchFlag == false)
		return (false);

	//
	// have now moved off the switch, move toward it again but slower
	//
	setSpeedInStepsPerSecond(speedInStepsPerSecond / 8);
	setTargetPositionRelativeInSteps(maxDistanceToMoveInSteps * directionTowardHome);
	limitSwitchFlag = false;
	while (!processMovement())
	{
		if (digitalRead(homeLimitSwitchPin) == LOW)
		{
			limitSwitchFlag = true;
			directionOfMotion = 0;
			break;
		}
	}
	delay(25);

	//
	// check if switch never detected
	//
	if (limitSwitchFlag == false)
		return (false);

	//
	// successfully homed, set the current position to 0
	//
	setCurrentPositionInSteps(0L);

	setTargetPositionInSteps(0L);
	this->isCurrentlyHomed = true;
	this->disallowedDirection = directionTowardHome;
	//
	// restore original velocity
	//
	setSpeedInStepsPerSecond(originalDesiredSpeed_InStepsPerSecond);
	return (true);
}

//
// move relative to the current position, units are in steps, this function does
// not return until the move is complete
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current
//          position in steps
//
void LightStep::moveRelativeInSteps(long distanceToMoveInSteps)
{
	setTargetPositionRelativeInSteps(distanceToMoveInSteps);

	while (!processMovement())
		;
}

//
// setup a move relative to the current position, units are in steps, no motion
// occurs until processMove() is called
//  Enter:  distanceToMoveInSteps = signed distance to move relative to the current
//            positionin steps
//
void LightStep::setTargetPositionRelativeInSteps(long distanceToMoveInSteps)
{
	setTargetPositionInSteps(currentPosition_InSteps + distanceToMoveInSteps);
}

//
// move to the given absolute position, units are in steps, this function does not
// return until the move is complete
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to
//            in unitsof steps
//
void LightStep::moveToPositionInSteps(long absolutePositionToMoveToInSteps)
{
	setTargetPositionInSteps(absolutePositionToMoveToInSteps);

	while (!processMovement())
		;
}

//
// setup a move, units are in steps, no motion occurs until processMove() is called
//  Enter:  absolutePositionToMoveToInSteps = signed absolute position to move to
//            in units of steps
//
void LightStep::setTargetPositionInSteps(long absolutePositionToMoveToInSteps)
{
	//abort potentially running homing movement
	this->isOnWayToHome = false;
	this->isOnWayToLimit = false;
	targetPosition_InSteps = absolutePositionToMoveToInSteps;
	this->firstProcessingAfterTargetReached = true;
}

long LightStep::getTargetPositionInSteps()
{
	return targetPosition_InSteps;
}

//
// setup a "Stop" to begin the process of decelerating from the current velocity
// to zero, decelerating requires calls to processMove() until the move is complete
// Note: This function can be used to stop a motion initiated in units of steps
// or revolutions
//
void LightStep::setTargetPositionToStop()
{
	//abort potentially running homing movement
	this->isOnWayToHome = false;
	this->isOnWayToLimit = false;

	if(directionOfMotion == 0){
		return;
	}

	long decelerationDistance_InSteps;

	//
	// move the target position so that the motor will begin deceleration now
	//
	decelerationDistance_InSteps = (long)round(
			5E11 / (deceleration_InStepsPerSecondPerSecond * currentStepPeriod_InUS *
							currentStepPeriod_InUS));

	if (directionOfMotion > 0)
		setTargetPositionInSteps(currentPosition_InSteps + decelerationDistance_InSteps);
	else
		setTargetPositionInSteps(currentPosition_InSteps - decelerationDistance_InSteps);
}

//
// if it is time, move one step
//  Exit:  true returned if movement complete, false returned not a final target
//           position yet
//
bool LightStep::processMovement(void)
{
	long distanceToTarget_Signed;
	unsigned long currentTime_InUS;
	unsigned long periodSinceLastStep_InUS;

	//
	// check if currently stopped
	//
	if (directionOfMotion == 0) {
		distanceToTarget_Signed = targetPosition_InSteps - currentPosition_InSteps;
		// check if target position in a positive direction
		if (distanceToTarget_Signed > 0)
		{
			directionOfMotion = 1;
			digitalWrite(directionPin, POSITIVE_DIRECTION);
			nextStepPeriod_InUS = periodOfSlowestStep_InUS;
			lastStepTime_InUS = micros();
			lastStepDirectionBeforeLimitSwitchTrigger = directionOfMotion;
			return (false);
		}

		// check if target position in a negative direction
		else if (distanceToTarget_Signed < 0)
		{
			directionOfMotion = -1;
			digitalWrite(directionPin, NEGATIVE_DIRECTION);
			nextStepPeriod_InUS = periodOfSlowestStep_InUS;
			lastStepTime_InUS = micros();
			lastStepDirectionBeforeLimitSwitchTrigger = directionOfMotion;
			return (false);
		}
		else
		{
			this->lastStepDirectionBeforeLimitSwitchTrigger = 0;
			//activate brake since motor is stopped
			if (this->_isBrakeConfigured && !this->_isBrakeActive && this->_hasMovementOccuredSinceLastBrakeRelease)
			{
				this->triggerBrakeIfNeededOrSetTimeout();
			}
			return (true);
		}
	}

	// determine how much time has elapsed since the last step (Note 1: this method
	// works even if the time has wrapped. Note 2: all variables must be unsigned)
	currentTime_InUS = micros();
	periodSinceLastStep_InUS = currentTime_InUS - lastStepTime_InUS;
	// if it is not time for the next step, return
	if (periodSinceLastStep_InUS < (unsigned long)nextStepPeriod_InUS)
		return (false);

	//we have to move, so deactivate brake (if configured at all) immediately
	if (this->_isBrakeConfigured && this->_isBrakeActive)
	{
		this->deactivateBrake();
	}

	// execute the step on the rising edge
	digitalWrite(stepPin, HIGH);

	// update the current position and speed
	currentPosition_InSteps += directionOfMotion;
	currentStepPeriod_InUS = nextStepPeriod_InUS;

	// remember the time that this step occured
	lastStepTime_InUS = currentTime_InUS;

	// figure out how long before the next step
	DeterminePeriodOfNextStep();

	// return the step line low
	digitalWrite(stepPin, LOW);

	this->_hasMovementOccuredSinceLastBrakeRelease = true;

	// check if the move has reached its final target position, return true if all
	// done
	if (currentPosition_InSteps == targetPosition_InSteps)
	{
		// at final position, make sure the motor is not going too fast
		if (nextStepPeriod_InUS >= minimumPeriodForAStoppedMotion)
		{
			currentStepPeriod_InUS = 0.0;
			nextStepPeriod_InUS = 0.0;
			directionOfMotion = 0;
			this->lastStepDirectionBeforeLimitSwitchTrigger = 0;

			if (this->firstProcessingAfterTargetReached)
			{
				firstProcessingAfterTargetReached = false;
				if (this->_targetPositionReachedCallback)
				{
					this->_targetPositionReachedCallback(currentPosition_InSteps);
				}
				//activate brake since we reached the final position
				if (this->_isBrakeConfigured && !this->_isBrakeActive)
				{
					this->triggerBrakeIfNeededOrSetTimeout();
				}
			}
			return (true);
		}
	}
	return (false);
}

/**
 * internal helper to determine if brake shall be acitvated (if configured at all) or if a delay needs to be set
 */
void LightStep::triggerBrakeIfNeededOrSetTimeout()
{
	//check if break is already set or a timeout has already beend set
	if (this->_isBrakeConfigured && !this->_isBrakeActive && this->_timeToEngangeBrake == LONG_MAX)
	{
		if (this->_brakeReleaseDelayMs > 0 && this->_hasMovementOccuredSinceLastBrakeRelease)
		{
			this->_timeToReleaseBrake = millis() + this->_brakeReleaseDelayMs;
		}

		if (this->_brakeEngageDelayMs == 0)
		{
			this->activateBrake();
		}
		else
		{
			this->_timeToEngangeBrake = millis() + this->_brakeEngageDelayMs;
		}
	}
}

// Get the current velocity of the motor in steps/second.  This functions is
// updated while it accelerates up and down in speed.  This is not the desired
// speed, but the speed the motor should be moving at the time the function is
// called.  This is a signed value and is negative when the motor is moving
// backwards.  Note: This speed will be incorrect if the desired velocity is set
// faster than this library can generate steps, or if the load on the motor is too
// great for the amount of torque that it can generate.
//  Exit:  velocity speed in steps per second returned, signed
//
float LightStep::getCurrentVelocityInStepsPerSecond()
{
	if (currentStepPeriod_InUS == 0.0)
		return (0);
	else
	{
		if (directionOfMotion > 0)
			return (1000000.0 / currentStepPeriod_InUS);
		else
			return (-1000000.0 / currentStepPeriod_InUS);
	}
}

//
// check if the motor has competed its move to the target position
//  Exit:  true returned if the stepper is at the target position
//
bool LightStep::motionComplete()
{
	if ((directionOfMotion == 0) &&
			(currentPosition_InSteps == targetPosition_InSteps))
		return (true);
	else
		return (false);
}

//
// determine the period for the next step, either speed up a little, slow down a
// little or go the same speed
//
void LightStep::DeterminePeriodOfNextStep()
{
	long distanceToTarget_Signed;
	long distanceToTarget_Unsigned;
	long decelerationDistance_InSteps;
	float currentStepPeriodSquared;
	bool speedUpFlag = false;
	bool slowDownFlag = false;
	bool targetInPositiveDirectionFlag = false;
	bool targetInNegativeDirectionFlag = false;

	//
	// determine the distance to the target position
	//
	distanceToTarget_Signed = targetPosition_InSteps - currentPosition_InSteps;
	if (distanceToTarget_Signed >= 0L)
	{
		distanceToTarget_Unsigned = distanceToTarget_Signed;
		targetInPositiveDirectionFlag = true;
	}
	else
	{
		distanceToTarget_Unsigned = -distanceToTarget_Signed;
		targetInNegativeDirectionFlag = true;
	}

	//
	// determine the number of steps needed to go from the current speed down to a
	// velocity of 0, Steps = Velocity^2 / (2 * Deceleration)
	//
	currentStepPeriodSquared = currentStepPeriod_InUS * currentStepPeriod_InUS;
	decelerationDistance_InSteps = (long)round(
			5E11 / (deceleration_InStepsPerSecondPerSecond * currentStepPeriodSquared));

	//
	// check if: Moving in a positive direction & Moving toward the target
	//    (directionOfMotion == 1) && (distanceToTarget_Signed > 0)
	//
	if ((directionOfMotion == 1) && (targetInPositiveDirectionFlag))
	{
		//
		// check if need to start slowing down as we reach the target, or if we
		// need to slow down because we are going too fast
		//
		if ((distanceToTarget_Unsigned < decelerationDistance_InSteps) ||
				(nextStepPeriod_InUS < desiredPeriod_InUSPerStep))
			slowDownFlag = true;
		else
			speedUpFlag = true;
	}

	//
	// check if: Moving in a positive direction & Moving away from the target
	//    (directionOfMotion == 1) && (distanceToTarget_Signed < 0)
	//
	else if ((directionOfMotion == 1) && (targetInNegativeDirectionFlag))
	{
		//
		// need to slow down, then reverse direction
		//
		if (currentStepPeriod_InUS < periodOfSlowestStep_InUS)
		{
			slowDownFlag = true;
		}
		else
		{
			directionOfMotion = -1;
			digitalWrite(directionPin, NEGATIVE_DIRECTION);
		}
	}

	//
	// check if: Moving in a negative direction & Moving toward the target
	//    (directionOfMotion == -1) && (distanceToTarget_Signed < 0)
	//
	else if ((directionOfMotion == -1) && (targetInNegativeDirectionFlag))
	{
		//
		// check if need to start slowing down as we reach the target, or if we
		// need to slow down because we are going too fast
		//
		if ((distanceToTarget_Unsigned < decelerationDistance_InSteps) ||
				(nextStepPeriod_InUS < desiredPeriod_InUSPerStep))
			slowDownFlag = true;
		else
			speedUpFlag = true;
	}

	//
	// check if: Moving in a negative direction & Moving away from the target
	//    (directionOfMotion == -1) && (distanceToTarget_Signed > 0)
	//
	else if ((directionOfMotion == -1) && (targetInPositiveDirectionFlag))
	{
		//
		// need to slow down, then reverse direction
		//
		if (currentStepPeriod_InUS < periodOfSlowestStep_InUS)
		{
			slowDownFlag = true;
		}
		else
		{
			directionOfMotion = 1;
			digitalWrite(directionPin, POSITIVE_DIRECTION);
		}
	}

	//
	// check if accelerating
	//
	if (speedUpFlag)
	{
		//
		// StepPeriod = StepPeriod(1 - a * StepPeriod^2)
		//
		nextStepPeriod_InUS = currentStepPeriod_InUS - acceleration_InStepsPerUSPerUS *
																											 currentStepPeriodSquared * currentStepPeriod_InUS;

		if (nextStepPeriod_InUS < desiredPeriod_InUSPerStep)
			nextStepPeriod_InUS = desiredPeriod_InUSPerStep;
	}

	//
	// check if decelerating
	//
	if (slowDownFlag)
	{
		//
		// StepPeriod = StepPeriod(1 + a * StepPeriod^2)
		//
		nextStepPeriod_InUS = currentStepPeriod_InUS + deceleration_InStepsPerUSPerUS *
																											 currentStepPeriodSquared * currentStepPeriod_InUS;

		if (nextStepPeriod_InUS > periodOfSlowestStep_InUS)
			nextStepPeriod_InUS = periodOfSlowestStep_InUS;
	}
}

// -------------------------------------- End --------------------------------------
