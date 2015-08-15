/*************************************************** 
  This is a library for the forthcoming Logos Electromechanical 8x8 Driver. 
  It is based on the library for the Adafruit 16-channel PWM & Servo driver.


  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <PWM8x8Driver.h>
#include <Wire.h>
#if defined(__AVR__)
 #define WIRE Wire
#elif defined(CORE_TEENSY) // Teensy boards
 #define WIRE Wire
#else // Arduino Due
 #define WIRE Wire1
#endif

// Set to true to print some debug messages, or false to disable them.
#define ENABLE_DEBUG_OUTPUT false

#define STEP_TABLE_HALF 	{{1, 0, 1, 0}, {1, 0, 0, 1}, {0, 1, 0, 1}, {0, 1, 1, 0}, {1, 0, 1, 0}, {1, 0, 0, 1}, {0, 1, 0, 1}, {0, 1, 1, 0}}
#define STEP_TABLE_FULL		{{1, 0, 1, 0}, {1, 0, 0, 0}, {1, 0, 0, 1}, {0, 0, 0, 1}, {0, 1, 0, 1}, {0, 1, 0, 0}, {0, 1, 1, 0}, {0, 0, 1, 0}}

PWM8x8Driver::PWM8x8Driver(uint8_t addr) {
  _i2caddr = addr;
}

void PWM8x8Driver::begin(void) {
 WIRE.begin();
 reset();
}


void PWM8x8Driver::reset(void) {
 write8(PCA9685_MODE1, 0x0);
}

void PWM8x8Driver::setPWMFreq(float freq) {
  //Serial.print("Attempting to set freq ");
  //Serial.println(freq);
  freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("Estimated pre-scale: "); Serial.println(prescaleval);
  }
  uint8_t prescale = floor(prescaleval + 0.5);
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("Final pre-scale: "); Serial.println(prescale);
  }
  
  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  write8(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
                                          // This is why the beginTransmission below was not working.
  //  Serial.print("Mode now 0x"); Serial.println(read8(PCA9685_MODE1), HEX);
}

void PWM8x8Driver::setPWM(uint8_t num, uint16_t on, uint16_t off) {
  //Serial.print("Setting PWM "); Serial.print(num); Serial.print(": "); Serial.print(on); Serial.print("->"); Serial.println(off);

  WIRE.beginTransmission(_i2caddr);
  WIRE.write(LED0_ON_L+4*num);
  WIRE.write(on);
  WIRE.write(on>>8);
  WIRE.write(off);
  WIRE.write(off>>8);
  WIRE.endTransmission();
}

// Sets pin without having to deal with on/off tick placement and properly handles
// a zero value as completely off.  Optional invert parameter supports inverting
// the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
void PWM8x8Driver::setPin(uint8_t num, uint16_t val, bool invert)
{
  // Clamp value between 0 and 4095 inclusive.
  val = min(val, PCA9685_MAX_VAL);
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      setPWM(num, PCA9685_FULL_ON, 0);
    }
    else if (val == PCA9685_MAX_VAL) {
      // Special value for signal fully off.
      setPWM(num, 0, PCA9685_FULL_ON);
    }
    else {
      setPWM(num, 0, PCA9685_MAX_VAL-val);
    }
  }
  else {
    if (val == PCA9685_MAX_VAL) {
      // Special value for signal fully on.
      setPWM(num, PCA9685_FULL_ON, 0);
    }
    else if (val == 0) {
      // Special value for signal fully off.
      setPWM(num, 0, PCA9685_FULL_ON);
    }
    else {
      setPWM(num, 0, val);
    }
  }
}

uint8_t PWM8x8Driver::read8(uint8_t addr) {
  WIRE.beginTransmission(_i2caddr);
  WIRE.write(addr);
  WIRE.endTransmission();

  WIRE.requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return WIRE.read();
}

void PWM8x8Driver::write8(uint8_t addr, uint8_t d) {
  WIRE.beginTransmission(_i2caddr);
  WIRE.write(addr);
  WIRE.write(d);
  WIRE.endTransmission();
}

PWM8x8Driver::PWM8x8Driver(PWM8x8Driver * board) {
	_board = board;
	_pins = {{0, 0}, {0, 0}};
	_dir = DIR_FWD;
	_speed = 0;*
}

void PWM8x8Driver::setPins(uint8_t high1, uint8_t low1, uint8_t high2, uint8_t low2) {
	_pins[0][0] = low1;
	_pins[0][1] = high1;
	_pins[1][0] = low2;
	_pins[1][1] = high2;
}

void PWM8x8Driver::getPins(uint8_t *high1, uint8_t *low1, uint8_t *high2, uint8_t *low2) {
	*low1 = _pins[0][0];
	*low2 = _pins[1][0];
	*high1 = _pins[0][1];
	*high2 = _pins[1][1];
}

void PWM8x8Driver::setDirection(uint8_t direction) {
	if ((direction > DIR_FWD) || (direction < DIR_REV)) return;
	_dir = direction;
	_writeMotor();
}

uint8_t	PWM8x8Driver::getDirection(void) {
	return _dir;
}

void PWM8x8Driver::setSpeed(uint16_t speed) {
	if (speed > PCA9685_FULL_ON) speed = PCA9685_FULL_ON;
	_speed = speed;
	_writeMotor();
}

uint16_t PWM8x8Driver::getSpeed(void) {
	return _speed;
}

void PWM8x8Driver::setVelocity(int16_t velocity) {
	_speed = abs(velocity);
	if (_speed > PCA9685_FULL_ON) _speed = PCA9685_FULL_ON;
	if (velocity >= 0) {
		_dir = DIR_FWD;
	} else {
		_dir = DIR_REV;
	}
	_writeMotor();
}

int16_t PWM8x8Driver::getVelocity(void) {
	if (_dir == DIR_FWD) return _speed;
	if (_dir == DIR_REV) return -_speed;
	else return 0;
}

void PWM8x8Driver::brake(bool state) {
	if (state) {
		// turn both high sides off
		_board->setPWM(_pins[0][1], 0, PCA9685_FULL_ON);
		_board->setPWM(_pins[1][1], 0, PCA9685_FULL_ON);
		// turn both low sides on
		_board->setPWM(_pins[0][0], PCA9685_FULL_ON, 0);
		_board->setPWM(_pins[1][0], PCA9685_FULL_ON, 0);
	} else {
		// turn everything off
		_board->setPWM(_pins[0][0], 0, PCA9685_FULL_ON);
		_board->setPWM(_pins[1][0], 0, PCA9685_FULL_ON);
		_board->setPWM(_pins[0][1], 0, PCA9685_FULL_ON);
		_board->setPWM(_pins[1][1], 0, PCA9685_FULL_ON);
	}
}

void PWM8x8Driver::_writeMotor(void) {
	if (_dir == DIR_FWD) {
		// turn everything off to prevent shoot-through
		_board->setPWM(_pins[0][0], 0, PCA9685_FULL_ON);
		_board->setPWM(_pins[1][0], 0, PCA9685_FULL_ON);
		_board->setPWM(_pins[0][1], 0, PCA9685_FULL_ON);
		_board->setPWM(_pins[1][1], 0, PCA9685_FULL_ON);
		// set side 1 high side and side 2 low side on to drive
		_board->setPWM(_pins[0][1], _speed);
		_board->setPWM(_pins[1][0], _speed);
	} else if (_dir == DIR_REV) {
		// turn everything off to prevent shoot-through
		_board->setPWM(_pins[0][0], 0, PCA9685_FULL_ON);
		_board->setPWM(_pins[1][0], 0, PCA9685_FULL_ON);
		_board->setPWM(_pins[0][1], 0, PCA9685_FULL_ON);
		_board->setPWM(_pins[1][1], 0, PCA9685_FULL_ON);
		// set side 2 high side and side 1 low side on to drive
		_board->setPWM(_pins[0][0], _speed);
		_board->setPWM(_pins[1][1], _speed);
	}
}

PWM8x8StepperMotorDriver::PWM8x8StepperMotorDriver(PWM8x8Driver * board, uint8_t mode) {
	_board = board;
	_pins = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
	_dir = DIR_FWD;
	_stepIndex = STEP_TABLE_SIZE;
	_current = PCA9685_MAX_VAL;
	if (mode = STEP_MODE_HALF) {
		_mode = STEP_MODE_HALF;
		_stepTable = STEP_TABLE_HALF;
	} else {
		_mode = STEP_MODE_FULL;
		_stepTable = STEP_TABLE_FULL;
	}
	_phaseState = _stepTable[_stepIndex];
	step();
}

void PWM8x8StepperMotorDriver::setPins(uint8_t A1_hi, uint8_t A1_lo, uint8_t A0_hi, uint8_t A0_lo, uint8_t B1_hi, uint8_t B1_lo, uint8_t B0_hi, uint8_t B0_lo) {
	_pins[0][0] = A0_lo;
	_pins[0][1] = A0_hi;
	_pins[1][0] = A1_lo;
	_pins[1][1] = A1_hi;
	_pins[2][0] = B0_lo;
	_pins[2][1] = B0_hi;
	_pins[3][0] = B1_lo;
	_pins[3][1] = B1_hi;
}

void PWM8x8StepperMotorDriver::setPins(uint8_t[STEP_NUM_PHASES][2] pins) {
	for (uint8_t i = 0; i < STEP_NUM_PHASES; i++) {
		for (uint8_t j = 0; j < 2; j++) {
			_pins[i][j] = pins[i][j];
		}
	}
}

void PWM8x8StepperMotorDriver::getPins(uint8_t *A1_hi, uint8_t *A1_lo, uint8_t *A0_hi, uint8_t *A0_lo, uint8_t *B1_hi, uint8_t *B1_lo, uint8_t *B0_hi, uint8_t *B0_lo) {
	*A0_lo = _pins[0][0];
	*A0_hi = _pins[0][1];
	*A1_lo = _pins[0][0];
	*A1_hi = _pins[0][1];
	*B0_lo = _pins[0][0];
	*B0_hi = _pins[0][1];
	*B1_lo = _pins[0][0];
	*B1_hi = _pins[0][1];
}

uint16_t PWM8x8StepperMotorDriver::getStepIndex(void) {
	return _stepIndex;
}

void PWM8x8StepperMotorDriver::setDir(uint8_t dir) {
	if (_dir == DIR_FWD) _dir = DIR_FWD;
	if (_dir == DIR_REV) _dir = DIR_REV;
}

uint8_t PWM8x8StepperMotorDriver::getDir(void) {
	return _dir;
}

void PWM8x8StepperMotorDriver::setMode(uint8_t mode) {
	if (mode == STEP_MODE_HALF) {
		_mode = mode;
		_stepTable = STEP_TABLE_HALF;
	} else if (mode = STEP_MODE_FULL) {
		_mode = mode;
		_stepTable = STEP_TABLE_FULL;
	}
}

uint8_t	PWM8x8StepperMotorDriver::getMode(void) {
	return _mode;
}

void PWM8x8StepperMotorDriver::step(void) {
	step(_dir);
}

void PWM8x8StepperMotorDriver::step(uint8_t dir) {
	if (dir == DIR_FWD) {
		_stepIndex++;
		if(_stepIndex >= STEP_TABLE_SIZE) _stepIndex = 0;
	} else {
		_stepIndex--;
		if(_stepIndex < 0) _stepIndex = 0;
	}
	for (uint8_t i = 0; i < STEP_NUM_PHASES; i++) {
		_setPhase(i, _stepTable[_stepIndex][i]);
	}
}

void PWM8x8StepperMotorDriver::setCurrent(uint16_t current) {
	if (current > PCA9685_MAX_VAL) current = PCA9685_MAX_VAL;
	_current = current;
}

uint16_t PWM8x8StepperMotorDriver::getCurrent(void) {
		return _current;
}

void PWM8x8StepperMotorDriver::_setPhase(uint8_t phase, bool state) {
	if (phase >= STEP_NUM_PHASES) return;
	if (state) {
		_board->setPWM(_pins[phase][0], 0);
		_board->setPWM(_pins[phase][1], _current);
	} else {
		_board->setPWM(_pins[phase][1], 0);
		_board->setPWM(_pins[phase][0], _current);
	}
}