/*******************************************************************************
 * Copyright (C) 2015 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */

#include "mbed.h"
#include "BMP180.h"
#include "L9110S.h"    

/***** Definitions *****/
#define I2C_ADDR            (0xEE) // 1110111x

#define REG_ADDR_RESET      (0xE0)
#define REG_ADDR_ID         (0xD0)
#define REG_ADDR_CTRL       (0xF4)
#define REG_ADDR_DATA       (0xF6)
#define REG_ADDR_AC1        (0xAA)

#define CTRL_REG_TEMP       (0x2E)
#define CTRL_REG_PRESS_0    (0x34)
#define CTRL_REG_PRESS_1    (0x74)
#define CTRL_REG_PRESS_2    (0xB4)
#define CTRL_REG_PRESS_3    (0xF4)



L9110S::L9110S(PinName cw_r, PinName ccw_r):
    cw_min(14), cw_max(70), ccw_min(14), ccw_max(70), _cw(cw_r), _ccw(ccw_r), periode(1000), power_act(0)
   {
    _cw.period_us( periode);
    _ccw.period_us(periode);
    _cw.pulsewidth_us( 0);
    _ccw.pulsewidth_us(0);
   }
    
  /** sets Pulswidth cw(+) ccw(-)
   * 
   * @param power positiv in clockwise Dir
   * @param power negativ in counter clockwise Dir
   *
   *  in % of range from min_(cw/ccw) to max_(cw/ccw)
   *
   */
  void L9110S::drive(int power)
  {

    // Limit PWM -100 to 100
    if (power >  100) power =  100;
    if (power < -100) power = -100; 
    power_act = power;
    
    // Calc PWM in us 
    if        (power > 0) {power = ((power * ( cw_max -  cw_min)) + ( cw_min * 100)) * periode / 10000;}
    else { if (power < 0) {power = ((power * (ccw_max - ccw_min)) - (ccw_min * 100)) * periode / 10000;}
          else power = 0;}
    
     //cw or ccw Rotate
    if (power >= 0){_ccw.pulsewidth_us(0); _cw.pulsewidth_us(power);}
     else {_cw.pulsewidth_us(0); _ccw.pulsewidth_us(-power);}
  }    
        
  /** sets Pulswidth and Dir 
   * 
   *  @param dir positiv dir is clockwise direction
   *  @param dir negativ dir is counter clockwise direction
   *  @param dir zero is stop 
   *
   *  @param power in % of range from min_(cw/ccw) to max_(cw/ccw) allways positiv
   *
   */
   
   void L9110S::frequency(int hz)
   {
    periode = 1000000/hz; 
   }
   
   float L9110S::deg_diff(float Soll, float Ist)
   {
    float diff = Soll - Ist;
    if(diff >  180) diff = -360 + diff;
    if(diff < -180) diff =  360 + diff;
    return diff;   
   }

  void L9110S::drive_diff(float Soll, float Ist, int kp ,int i_lim, int trash)
   {
    
    int     p = 0;                                    //Leistungsvariable
    
    float   diff = deg_diff(Soll, Ist      );            //Winkeldiffernz
            diff = diff - (power_act / 20.0);            //Winkeldiffernz - D Anteil aus Motorleistung 10? bei 100%        
    
    if ( fabs(diff) < (trash / 10.0)) {p = 0; drive_i *= 0.9;}        //Leistung 0, wenn innerhalb Threshold
     else
      {
       p    = diff * kp;                              //Leistungsbedarf aus Winkeldifferenz mal Kp
       drive_i = drive_i + ( p / 20.0);               //Berechnung I Anteil 
       if (drive_i >  i_lim) drive_i =  i_lim;
       if (drive_i < -i_lim) drive_i = -i_lim;
      }

    if (p >  100) (p =  100);
    if (p < -100) (p = -100);
    
    power_act = (6 * power_act + p) / 7;
     
    drive(drive_i/10 + p);   
   }
//******************************************************************************
BMP180::BMP180(PinName sda, PinName scl)
{
	i2c_ = new I2C(sda, scl);
	i2c_owner = true;

	i2c_->frequency(400000);
}

//******************************************************************************
BMP180::BMP180(I2C *i2c) :
	i2c_(i2c)
{
	i2c_owner = false;
}

//******************************************************************************
BMP180::~BMP180()
{
	if(i2c_owner) {
		delete i2c_;
	}
}

//******************************************************************************
int BMP180::init(void)
{
	char addr;
  	char data[22];
  	int i;

  	if (checkId() != 0) {
		return -1;
  	}

  	addr = REG_ADDR_AC1;
  	if (i2c_->write(I2C_ADDR, &addr, 1) != 0) {
		return -1;
  	}

  	if (i2c_->read(I2C_ADDR, data, 22) != 0) {
		return -1;
  	}

  	for (i = 0; i < 11; i++) {
		calib.value[i] = (data[2*i] << 8) | data[(2*i)+1];
  	}

  	return 0;
}

//******************************************************************************
int BMP180::reset(void)
{
  	char data;

  	data = REG_ADDR_RESET;
  	if (i2c_->write(I2C_ADDR, &data, 1) != 0) {
		return -1;
  	}

  	data = 0xB6;
  	if (i2c_->write(I2C_ADDR, &data, 1) != 0) {
		return -1;
  	}

  	return 0;
}

//******************************************************************************
int BMP180::checkId(void)
{
    char addr;
    char data;

    addr = REG_ADDR_ID;
    if (i2c_->write(I2C_ADDR, &addr, 1) != 0) {
    	return -1;
    }

    if (i2c_->read(I2C_ADDR, &data, 1) != 0) {
        return -1;
    }

    if (data != 0x55) {
        return -1;
    }

	return 0;
}

//******************************************************************************
int BMP180::startPressure(BMP180::oversampling_t oss)
{
	char data[2];

    data[0] = REG_ADDR_CTRL;
    data[1] = CTRL_REG_PRESS_0 | ((oss & 0x3) << 6);
    oss_ = oss;

    if (i2c_->write(I2C_ADDR, data, 2) != 0) {
        return -1;
    }

    return 0;
}

//******************************************************************************
int BMP180::getPressure(int *pressure)
{
    char addr, byte[3];
    uint32_t up;
    int32_t b6, x1, x2, x3, b3, p;
    uint32_t b4, b7;

    addr = REG_ADDR_DATA;
    if (i2c_->write(I2C_ADDR, &addr, 1) != 0) {
        return -1;
    }

    if (i2c_->read(I2C_ADDR, byte, 3) != 0) {
        return -1;
    }

    up = ((byte[0] << 16) | (byte[1] << 8) | byte[2]) >> (8 - oss_);

    b6 = b5 - 4000;
    x1 = (b6 * b6) >> 12;
    x1 *= calib.b2;
    x1 >>= 11;
    x2 = calib.ac2 * b6;
    x2 >>= 11;
    x3 = x1 + x2;
    b3 = (((((int32_t)calib.ac1) * 4 + x3) << oss_) + 2);
    b3 >>= 2;

    x1 = (calib.ac3 * b6) >> 13;
    x2 = (calib.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = (x1 + x2 + 2) >> 2;
    b4 = (calib.ac4 * (uint32_t)(x3 + 32768)) >> 15;
    b7 = ((uint32_t)up - b3) * (50000 >> oss_);
    p = ((b7 < 0x80000000) ? ((b7 << 1) / b4) : ((b7 / b4) * 2));
    x1 = p >> 8;
    x1 *= x1;
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;

    *pressure = p;

    return 0;
}

//******************************************************************************
int BMP180::startTemperature(void)
{
    char data[2] = { REG_ADDR_CTRL, CTRL_REG_TEMP };

    if (i2c_->write(I2C_ADDR, data, 2) != 0) {
        return -1;
    }

    return 0;
}

//******************************************************************************
int BMP180::getTemperature(float *tempC)
{
    char addr, byte[2];
    uint16_t ut;
    int32_t x1, x2;

    addr = REG_ADDR_DATA;
    if (i2c_->write(I2C_ADDR, &addr, 1) != 0) {
        return -1;
    }

    if (i2c_->read(I2C_ADDR, byte, 2) != 0) {
        return -1;
    }

    ut = (byte[0] << 8) | byte[1];

    x1 = ((ut - calib.ac6) * calib.ac5) >> 15;
    x2 = (calib.mc << 11) / (x1 + calib.md);
    b5 = x1 + x2;

    *tempC = (float)(b5 + 8) / 160;

    return 0;
}

//******************************************************************************
int BMP180::getTemperature(int16_t *tempCx10)
{
    char addr, byte[2];
    uint16_t ut;
    int32_t x1, x2;

    addr = REG_ADDR_DATA;
    if (i2c_->write(I2C_ADDR, &addr, 1) != 0) {
        return -1;
    }

    if (i2c_->read(I2C_ADDR, byte, 2) != 0) {
        return -1;
    }

    ut = (byte[0] << 8) | byte[1];

    x1 = ((ut - calib.ac6) * calib.ac5) >> 15;
    x2 = (calib.mc << 11) / (x1 + calib.md);
    b5 = x1 + x2;

    *tempCx10 = (b5 + 8) >> 4;

    return 0;
}

I2C i2c(I2C_SDA, I2C_SCL);
BMP180 bmp180(&i2c);
Serial pc(USBTX, USBRX);
AnalogIn ain(A0);
DigitalOut waterval(D8);
DigitalOut led1(PB_0);
L9110S motor_x(D5, D6); 

float brain_pressure;
float init_air;
float air;
float deadband = 1.5;                                              
int threshPressure = 1080;                                        
float setPressure;
float speed1 = 0;                                                   
int pumpState = 0;                                                  



int air_pre (void) {
	bmp180.startTemperature();
	wait_ms(5);     // Wait for conversion to complete
	float temp;
	if(bmp180.getTemperature(&temp) != 0) {
		pc.printf("Error getting temperature\n\r");
    }

    bmp180.startPressure(BMP180::ULTRA_LOW_POWER);
    wait_ms(10);    // Wait for conversion to complete
    int pressure = 0;
    if(bmp180.getPressure(&pressure) != 0) {
       	pc.printf("Error getting pressure\n\r");
    }
	return pressure;
} 

float water_pressure (void){
	AnalogIn ain(A0);
	float voltage = ain.read();
	float sensorValue = ((voltage - 0.04)/0.09); //Voltage = Vout/Vs , expressed in percentage // Vs = Vin reference voltage
		// print out the value you read:  
	return sensorValue;
}
 
void water_valve_flow (void){
	waterval = 1;
	wait(0.5);
	waterval = 0;
	wait(0.5);
}

int PumpControl(float meas, float set, float band, int th, int state){
  if(meas>th){
    state = 0;                                                                               
    pc.printf("Over the negative pressure threshold:%d hPa\n\r",th);                                  
    pc.printf(" .... Pump is stopped \n\r");                                               
    motor_x.drive(1);   
    return state; 
  }
  if(meas<=(set - band))	state = 1;                                                                                                                                            
  if(meas>set - band && meas<set + band){
		state = 2;                                                                                 
		water_valve_flow();
	}
  if(meas>=(set + band))	state = 3;                                                                                                                                                                                             
  return state; 
}

void initialize(void){
	while(1){
		if(bmp180.init() != 0) {
       	pc.printf("Error communicating with BMP180\n\r");
			  wait(1);
	  }
		else {
        pc.printf("Initialized BMP180\n\r");
        wait(1);
			  break;
		}
	}
}

float getbrain_pressure(void){
	float bra_pre = 0;
	for (int i = 0;i < 10;i++){
		bra_pre = bra_pre + water_pressure()*10;
	}
	bra_pre = (bra_pre/10)-8;
	return bra_pre;
}

void initialize2(void){
	waterval = 0;
}

float airpre(void){
	int a = 0;
	for (int i = 0;i < 10;i++){
		a = a + air_pre();
	}
	return  a/1000;
}

void action0(){
	initialize();
	initialize2();
}

void action1(){
	brain_pressure = getbrain_pressure();
	init_air = airpre();
	pc.printf("brain pressure: %f hPa\n\r Air pressure:%f hPa\n\r",brain_pressure, init_air);
	wait(1);	
}

void action2(){
	speed1 = ((brain_pressure-10)/5)*20; 
	setPressure = init_air+brain_pressure-3;                                                        
	pc.printf("Set pressure:%f\n\r", setPressure); 
	pumpState = PumpControl(air,setPressure,deadband,threshPressure,pumpState);  
	if(pumpState!=0){
		motor_x.drive(speed1);                                                              
		pc.printf("Pump is activated \n\r");                                                   
		}   
	if(pumpState!=0){
		pumpState = PumpControl(air,setPressure,deadband,threshPressure,pumpState);   
		if(pumpState==3){
			speed1 = speed1-1;
			motor_x.drive (speed1);
			pc.printf("Slow down the pump...  \n\r");                                              
		}
		if(pumpState==1 & speed1!=0){
			speed1 = speed1+1;
			motor_x.drive (speed1);
			pc.printf("Speed up the pump...  \n\r");                                              
		}                                                                               
	}
	brain_pressure = getbrain_pressure();
	air = airpre();
	pc.printf("brain pressure: %f hPa\n\r Air pressure:%f hPa\n\r",brain_pressure + init_air, air);
}	

int x21=0;
int x22=0;
int x0=1;
int x1=0;
int x2=0;
int x3=0;


void grafcet0(){
	if(x0==1){
		action0();
		x0=0;
		x1=1;
	}
	if(x1==1){
		action1();
		if(brain_pressure > 15){
			x1=0;
			x2=1;
		}
	}
	if(x2==1 && brain_pressure > 15 ){
		action2();
		if (brain_pressure < 10){
			x3=1;
			x2=0;
		}
	}
	if(x3==1 && brain_pressure < 10){
		x0=1;
		x3=0;
	}
}

int main(void) {
	
	while(1){
		grafcet0();
	}
}

	