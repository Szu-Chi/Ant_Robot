#include "PCA9685.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void I2C_WRITE(uint8_t reg,uint8_t data){
	uint8_t DATA[]={reg,data};
	HAL_I2C_Master_Transmit(&hi2c1,PCA9685_ADDR,DATA,2,10);
}
	
void I2C_WRITE_buf(uint8_t reg,uint8_t *data,uint8_t size){
	HAL_I2C_Mem_Write(&hi2c1,PCA9685_ADDR,reg,1,data,size,100);
}
uint8_t I2C_READ(uint8_t reg){
	uint8_t DATA[1];
	HAL_I2C_Master_Transmit(&hi2c1,PCA9685_ADDR,&reg,1,100);
	HAL_I2C_Master_Receive(&hi2c1,PCA9685_ADDR|0x01,DATA,1,100);
	return DATA[0];
}

void PWMServo_Init(void) {
	reset();
  setPWMFreq(FREQUENCY);
}

void reset(void) {
  I2C_WRITE(PCA9685_MODE1, 0x80);
	delay(10);
}

void setPWMFreq(float freq) {
  freq *= 0.9;  								// Correct for overshoot in the frequency setting 
  float prescaleval = 25000000; //25 MHz typical internal oscillator 
  prescaleval /= 4096;          //12 bits PWM
  prescaleval /= freq;
  prescaleval -= 1;
	uint8_t prescale = floor(prescaleval + 0.5);
  uint8_t oldmode = I2C_READ(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10; 	// sleep
  I2C_WRITE(PCA9685_MODE1, newmode); 					// go to sleep
  I2C_WRITE(PCA9685_PRESCALE, prescale); 			// set the prescaler
  I2C_WRITE(PCA9685_MODE1, oldmode);
  delay(5);
  I2C_WRITE(PCA9685_MODE1, oldmode | 0xa0);  	//  This sets the MODE1 register to turn on auto increment.
}

void setPWM(uint8_t num, uint16_t on, uint16_t off) {

	uint8_t data_buf[]={on,(on>>8),off,(off>>8)};
	I2C_WRITE_buf(LED0_ON_L+4*(num),data_buf,4);
}

void setAngle(uint8_t num, uint8_t angle){
	int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = (int)((float)(pulse_wide) / 1000000 * 50 * 4096);
	setPWM(num,0,analog_value);
}
	
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void delay(uint16_t ms){
	for(uint16_t time=0;time<ms;time++){
		for(int i=1200;i>0;i--){
			__nop();
		}
	}
}
	


