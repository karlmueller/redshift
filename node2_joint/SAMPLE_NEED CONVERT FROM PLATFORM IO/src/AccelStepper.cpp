// AccelStepper.cpp
//
// Copyright (C) 2009-2013 Mike McCauley
// $Id: AccelStepper.cpp,v 1.24 2020/04/20 00:15:03 mikem Exp mikem $
#include <Arduino.h>
#include "AccelStepper.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <math.h>
#include "esp_timer.h"

#if 0
// Some debugging assistance
void dump(uint8_t* p, int l)
{
    int i;

    for (i = 0; i < l; i++)
    {
	Serial.print(p[i], HEX);
	Serial.print(" ");
    }
    Serial.println("");
}
#endif

void AccelStepper::moveTo(long absolute)
{
    if (_targetPos != absolute)
    {
        _targetPos = absolute;
        computeNewSpeed();
        // compute new n?
    }
}

//user defined constraint function to accomodate for lack of inbuilt arduino library
float Xconstrain(float _val, float _minVal, float _maxVal)
{
    float _constrained = _val;

    if (_val < _minVal)
    {
        _constrained = _minVal;
    }

    if (_val > _maxVal)
    {
        _constrained = _minVal;
    }

    return _constrained;
}

void AccelStepper::move(long relative)
{
    moveTo(_currentPos + relative);
    //std::cout << "moved by: " << relative << std::endl;
}

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
bool AccelStepper::runSpeed()
{
    // Dont do anything unless we actually have a step interval
    if (!_stepInterval)
    {
        //printf("stepint return false \n");
        return false;
    }
    unsigned long time = esp_timer_get_time(); //time in us since the esp32 timer init. 
    if (time - _lastStepTime >= _stepInterval)
    {
        if (_direction == DIRECTION_CW)
        {
            // Clockwise
            _currentPos += 1;
        }
        else
        {
            // Anticlockwise
            _currentPos -= 1;
        }
        
        step(_currentPos);

        _lastStepTime = time; // Caution: does not account for costs in step()

        return true;
    }
    else
    {
        return false;
    }
}

long AccelStepper::distanceToGo()
{
    return _targetPos - _currentPos;
}

long AccelStepper::targetPosition()
{
    return _targetPos;
}

long AccelStepper::currentPosition()
{
    return _currentPos;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void AccelStepper::setCurrentPosition(long position)
{
    _targetPos = _currentPos = position;
    _n = 0;
    _stepInterval = 0;
    _speed = 0.0;
}

void AccelStepper::computeNewSpeed()
{
    long distanceTo = distanceToGo(); // +ve is clockwise from curent location

    long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16

    if (distanceTo == 0 && stepsToStop <= 1)
    {
        // We are at the target and its time to stop
        _stepInterval = 0;
        _speed = 0.0;
        _n = 0;
        return;
    }

    if (distanceTo > 0)
    {
        // We are anticlockwise from the target
        // Need to go clockwise from here, maybe decelerate now
        if (_n > 0)
        {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
                _n = -stepsToStop; // Start deceleration
        }
        else if (_n < 0)
        {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
                _n = -_n; // Start accceleration
        }
    }
    else if (distanceTo < 0)
    {
        // We are clockwise from the target
        // Need to go anticlockwise from here, maybe decelerate
        if (_n > 0)
        {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
                _n = -stepsToStop; // Start deceleration
        }
        else if (_n < 0)
        {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
                _n = -_n; // Start accceleration
        }
    }

    // Need to accelerate or decelerate
    if (_n == 0)
    {
        // First step from stopped
        _cn = _c0;
        _direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    else
    {
        // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
        _cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
        _cn = fmax(_cn, _cmin);
    }
    _n++;
    _stepInterval = _cn;
    _speed = 1000000.0 / _cn;
    if (_direction == DIRECTION_CCW)
        _speed = -_speed;

#if 0
    Serial.println(_speed);
    Serial.println(_acceleration);
    Serial.println(_cn);
    Serial.println(_c0);
    Serial.println(_n);
    Serial.println(_stepInterval);
    Serial.println(distanceTo);
    Serial.println(stepsToStop);
    Serial.println("-----");
#endif
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if the motor is still running to the target position.
bool AccelStepper::run()
{
    if (runSpeed())
        computeNewSpeed();
    return _speed != 0.0 || distanceToGo() != 0;
}

//AccelStepper::AccelStepper(uint8_t interface, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, bool enable)
AccelStepper::AccelStepper(bool enable)
{
    esp_err_t esp_timer_early_init(void);

    #define STEP_GPIO GPIO_NUM_4
    #define MOTOR_ENABLE GPIO_NUM_21
    #define RST GPIO_NUM_22
    #define SLP GPIO_NUM_16
    #define DIR_PIN GPIO_NUM_0
    #define m0_PIN GPIO_NUM_17
    #define m1_PIN GPIO_NUM_18
    #define m2_PIN GPIO_NUM_19

    gpio_reset_pin(STEP_GPIO);
    gpio_reset_pin(MOTOR_ENABLE);
    gpio_reset_pin(RST);
    gpio_reset_pin(SLP);
    gpio_reset_pin(DIR_PIN);
    gpio_reset_pin(m0_PIN);
    gpio_reset_pin(m1_PIN);
    gpio_reset_pin(m2_PIN);

    gpio_set_direction(STEP_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_ENABLE, GPIO_MODE_OUTPUT);
    gpio_set_direction(RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(SLP, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(m0_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(m1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(m2_PIN, GPIO_MODE_OUTPUT);

    gpio_set_level(MOTOR_ENABLE, 0);
    gpio_set_level(RST, 1);
    gpio_set_level(SLP, 1);
    gpio_set_level(m0_PIN, 1); //microstepping control pins
    gpio_set_level(m1_PIN, 1);
    gpio_set_level(m2_PIN, 0);

    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 0.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 25; //min pulse up-time for step in us (at least I think this is in us?). 20us for DRV8825, maybe increase if unstable?
    _enablePin = 0xff; 
    _lastStepTime = 0;

    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;

    if (enable)
        enableOutputs();
    // Some reasonable default
    setAcceleration(1);
    printf("stepper init completed");
}

// /*Unsure if this code is required for proper function... this may be needed upon debug
AccelStepper::AccelStepper(void (*forward)(), void (*backward)())
{
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 0.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
    _enablePin = 0xff;
    _lastStepTime = 0;
    _forward = forward;
    _backward = backward;

    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;

    // Some reasonable default
    setAcceleration(1);
}
// */

void AccelStepper::setMaxSpeed(float speed)
{
    if (speed < 0.0)
        speed = -speed;
    if (_maxSpeed != speed)
    {
        _maxSpeed = speed;
        _cmin = 1000000.0 / speed;
        // Recompute _n from current speed and adjust speed if accelerating or cruising
        if (_n > 0)
        {
            _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
            computeNewSpeed();
        }
    }
}

float AccelStepper::maxSpeed()
{
    return _maxSpeed;
}

void AccelStepper::setAcceleration(float acceleration)
{
    if (acceleration == 0.0)
        return;
    if (acceleration < 0.0)
        acceleration = -acceleration;
    if (_acceleration != acceleration)
    {
        // Recompute _n per Equation 17
        _n = _n * (_acceleration / acceleration);
        // New c0 per Equation 7, with correction per Equation 15
        _c0 = 0.676 * sqrt(2.0 / acceleration) * 1000000.0; // Equation 15
        _acceleration = acceleration;
        computeNewSpeed();
    }
}

void AccelStepper::setSpeed(float speed)
{
    if (speed == _speed)
        return;
    speed = Xconstrain(speed, -_maxSpeed, _maxSpeed);
    if (speed == 0.0)
        _stepInterval = 0;
    else
    {
        _stepInterval = fabs(1000000.0 / speed);
        _direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    _speed = speed;
}

float AccelStepper::speed()
{
    return _speed;
}

// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step(long step)
{
    (void)(step); // Unused

    gpio_set_level(DIR_PIN, _direction);
    gpio_set_level(STEP_GPIO, 1); // pulse step pin
    vTaskDelay(_minPulseWidth / (1000 * portTICK_PERIOD_MS));
    gpio_set_level(STEP_GPIO, 0);
    //printf("stepped!!!");
    /*
    // _pin[0] is step, _pin[1] is direction
    setOutputPins(_direction ? 0b10 : 0b00); // Set direction first else get rogue pulses
    setOutputPins(_direction ? 0b11 : 0b01); // step HIGH
    // Caution 200ns setup time
    // Delay the minimum allowed pulse width
    vTaskDelay(_minPulseWidth / (1000 * portTICK_PERIOD_MS));
    //delayMicroseconds(_minPulseWidth);
    setOutputPins(_direction ? 0b10 : 0b00); // step LOW
    */
}

// Prevents power consumption on the outputs
void AccelStepper::disableOutputs()
{
    gpio_set_level(MOTOR_ENABLE, 1);
}

void AccelStepper::enableOutputs()
{
    gpio_set_level(MOTOR_ENABLE, 0);
}

void AccelStepper::setMinPulseWidth(unsigned int minWidth)
{
    _minPulseWidth = minWidth;
}

// Blocks until the target position is reached and stopped
void AccelStepper::runToPosition()
{
    while (run())
        YIELD; // Let system housekeeping occur
}

void AccelStepper::setTargetPos(long newTarget)
{
    _targetPos = newTarget;
}

bool AccelStepper::runSpeedToPosition()
{
    if (_targetPos == _currentPos)
        return false;
    if (_targetPos > _currentPos)
        _direction = DIRECTION_CW;
    else
        _direction = DIRECTION_CCW;
    return runSpeed();
}

// Blocks until the new target position is reached
void AccelStepper::runToNewPosition(long position)
{
    moveTo(position);
    runToPosition();
}

void AccelStepper::stop()
{
    if (_speed != 0.0)
    {
        long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)) + 1; // Equation 16 (+integer rounding)
        if (_speed > 0)
            move(stepsToStop);
        else
            move(-stepsToStop);
    }
}

bool AccelStepper::isRunning()
{
    return !(_speed == 0.0 && _targetPos == _currentPos);
}

