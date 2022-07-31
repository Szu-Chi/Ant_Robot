#ifndef _PCA9685_H
#define _PCA9685_H


#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             50

#define PCA9685_ADDR 		0x80
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

#include "stdint.h"
#include "math.h"
#include "stm32l0xx_hal.h"


void PWMServo_Init(void);
void reset(void);
void setPWMFreq(float freq);
void setPWM(uint8_t num, uint16_t on, uint16_t off);
void setAngle(uint8_t num, uint8_t angle);

void I2C_WRITE(uint8_t reg,uint8_t data);
void I2C_WRITE_buf(uint8_t reg,uint8_t *data,uint8_t size);
uint8_t I2C_READ(uint8_t reg);

long map(long x, long in_min, long in_max, long out_min, long out_max);
void delay(uint16_t ms);

#endif
