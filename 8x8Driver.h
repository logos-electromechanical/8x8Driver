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

#ifndef _ADAFRUIT_PWMServoDriver_H
#define _ADAFRUIT_PWMServoDriver_H

#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif


#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD


class PWM8x8Driver {
	public:
		PWM8x8Driver(uint8_t addr = 0x40);
		void begin(void);
		void reset(void);
		void setPWMFreq(float freq);
		void setPWM(uint8_t num, uint16_t on, uint16_t off);
		void setPin(uint8_t num, uint16_t val, bool invert=false);

	private:
		uint8_t _i2caddr;
		uint8_t read8(uint8_t addr);
		void write8(uint8_t addr, uint8_t d);
};

class PWM8x8DCMotorDriver {
	public:
		PWM8x8Driver(PWM8x8Driver * board);
		void 		setPins(uint8_t high1, uint8_t low1, uint8_t high2, uint8_t low2);
		void 		getPins(uint8_t *high1, uint8_t *low1, uint8_t *high2, uint8_t *low2);
		void 		begin(void);
		void 		setDirection(uint8_t direction);
		uint8_t		getDirection(void);
		void 		setSpeed(uint16_t speed);
		uint16_t	getSpeed(void);
		void 		brake(void);
	private:
		PWM8x8Driver 	*_board;
		uint8_t 		_high1;
		uint8_t 		_low1;
		uint8_t 		_high2;
		uint8_t 		_low2;
		uint8_t			_dir;
		uint16_t		_speed;
};

class PWM8x8StepperMotorDriver {
	public:
		PWM8x8StepperMotorDriver(PWM8x8Driver * board);
		void 		setPins(uint8_t A1_hi, uint8_t A1_lo, uint8_t A0_hi, uint8_t A0_lo, uint8_t B1_hi, uint8_t B1_lo, uint8_t B0_hi, uint8_t B0_lo);
		void 		getPins(uint8_t A1_hi, uint8_t A1_lo, uint8_t A0_hi, uint8_t A0_lo, uint8_t B1_hi, uint8_t B1_lo, uint8_t B0_hi, uint8_t B0_lo);
		uint16_t	getStepIndex(void);
		void		setDir(uint8_t dir);
		uint8_t 	getDir(void);
		void		setMode(uint8_t mode);
		uint8_t		getMode(void);
		void 		step(void);
		void 		step(uint8_t dir);
	private:
		PWM8x8Driver 	*_board;
		uint8_t 		_A1_hi;
		uint8_t 		_A1_lo;
		uint8_t 		_A0_hi;
		uint8_t 		_A0_lo;
		uint8_t 		_B1_hi;
		uint8_t 		_B1_lo;
		uint8_t 		_B0_hi;
		uint8_t 		_B0_lo;
		uint8_t			_dir;
		uint8_t 		_mode;
		uint16_t 		_stepIndex;

}

#endif
