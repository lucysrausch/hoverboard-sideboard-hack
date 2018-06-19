/*
The MIT License (MIT)

Copyright (c) 2015 silverx

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/


#include <inttypes.h>
#include "sixaxis.h"
#include "drv_time.h"
#include "gd32f1x0.h"
//#include "drv_softi2c.h"
#include "util.h"
#include "defines.h"

#include "drv_serial.h"

#include "drv_i2c.h"

#include <math.h>

#ifdef DEBUG
int gyroid;
#endif
// Gyro LPF filter frequency
// gyro filter 0 = 260hz
// gyro filter 1 = 184hz
// gyro filter 2 = 94hz
// gyro filter 3 = 42hz
// 4 , 5, 6
#define GYRO_LOW_PASS_FILTER 3


void sixaxis_init( void)
{
// gyro soft reset

 i2c_writereg(0x7F, 0);
 i2c_writereg(0x06, 0x80);
 i2c_writereg(0x05, 0x30);
 i2c_writereg(0x06, 0x05);
 i2c_writereg(0x07, 0x00);
 i2c_writereg(0x7F, 0x20);
 i2c_writereg(0x00, 0x00);
 i2c_writereg(0x01, 0x3D);
 i2c_writereg(0x02, 0x00);
 i2c_writereg(0x10, 0x00);
 i2c_writereg(0x11, 0x00);
 i2c_writereg(0x14, 0x39);
 i2c_writereg(0x15, 0x00);

 delay(40000);

// clear sleep bit on old type gyro (mpu-6050)
//i2c_writereg( 107 , 0);

// gyro scale 2000 deg (FS =3)
//i2c_writereg( 27 , 24);

// Gyro DLPF low pass filter
//i2c_writereg( 26 , GYRO_LOW_PASS_FILTER);

}


int sixaxis_check( void)
{
	// read "who am I" register
	int id = i2c_readreg( 0 );

	return ( id == 0xA0 );
}



float accel[3];
float gyro[3];

float gyrocal[3];


float gyroXangle, gyroYangle; // Angle calculate using the gyro only
unsigned long timer = 0;

typedef union _data {
  float f;
  char  s[4];
} floatToByte;

floatToByte compAngleX;
floatToByte compAngleY;

void gyro_read( void)
{
	int data[14];

	i2c_writereg(0x7F, 0x00);
	i2c_readdata(0x2D , data, 14 );

	float gyronew[3];


	gyronew[1] = (int16_t) ((data[0]<<8) + data[1]);
	gyronew[0] = (int16_t) ((data[2]<<8) + data[3]);
	gyronew[2] = (int16_t) ((data[4]<<8) + data[5]);

	for ( int i = 0 ; i < 3; i++)
	{
		//gyronew[i] = gyronew[i] *  0.033035156f * 0.017453292f ;

		accel[i] = gyronew[i];

	}

	gyronew[1] = (int16_t) ((data[6]<<8) + data[7]);
	gyronew[0] = (int16_t) ((data[8]<<8) + data[9]);
	gyronew[2] = (int16_t) ((data[10]<<8) + data[11]);


	for ( int i = 0 ; i < 3; i++)
	{
		gyronew[i] = gyronew[i] *  0.061035156f * 0.017453292f ;
		gyro[i] = gyronew[i];
	}

	#define RAD_TO_DEG 57.29578f

	float roll  = atan(accel[1] / sqrt(accel[0]  * accel[0] + accel[2] * accel[2])) * RAD_TO_DEG;
	float pitch = atan2(-accel[0], accel[2]) * RAD_TO_DEG;

	float gyroXrate = gyro[0] / 131.0; // Convert to deg/s
  float gyroYrate = gyro[1] / 131.0; // Convert to deg/s

	float dt = ((float)gettime() - timer) / 1000.0f;
	timer = gettime();

	gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;

  compAngleX.f = 0.9 * (compAngleX.f + gyroXrate * dt) + 0.1 * roll; // Calculate the angle using a Complimentary filter
	compAngleY.f = 0.9 * (compAngleY.f + gyroYrate * dt) + 0.1 * pitch;

	// Reset the gyro angle when it has drifted too much
	if (gyroXangle < -180 || gyroXangle > 180)
		gyroXangle = compAngleX.f;
	if (gyroYangle < -180 || gyroYangle > 180)
		gyroYangle = compAngleY.f;

	uint8_t status = 0;
	status =  GPIO_ReadInputBit(SENSOR1_GPIO_Port, SENSOR1_Pin) | GPIO_ReadInputBit(SENSOR2_GPIO_Port, SENSOR2_Pin) << 1;


	USART_DataSend(USART2 , (uint8_t) 0x0F);
	waitForTxReady();


	USART_DataSend(USART2 , (uint8_t) compAngleX.s[0]);
	waitForTxReady();
	USART_DataSend(USART2 , (uint8_t) compAngleX.s[1]);
	waitForTxReady();
	USART_DataSend(USART2 , (uint8_t) compAngleX.s[2]);
	waitForTxReady();
	USART_DataSend(USART2 , (uint8_t) compAngleX.s[3]);
	waitForTxReady();

	USART_DataSend(USART2 , (uint8_t) compAngleY.s[0]);
	waitForTxReady();
	USART_DataSend(USART2 , (uint8_t) compAngleY.s[1]);
	waitForTxReady();
	USART_DataSend(USART2 , (uint8_t) compAngleY.s[2]);
	waitForTxReady();
	USART_DataSend(USART2 , (uint8_t) compAngleY.s[3]);
	waitForTxReady();

	USART_DataSend(USART2 , (uint8_t) status);
	waitForTxReady();

	USART_DataSend(USART2 , (uint8_t) 0x00);
	waitForTxReady();
}


#define CAL_TIME 2e6

void gyro_cal(void)
{
int data[6];

unsigned long time = gettime();
unsigned long timestart = time;
unsigned long timemax = time;
unsigned long lastlooptime = time;

float gyro[3];
float limit[3];

 for ( int i = 0 ; i < 3 ; i++)
			{
			limit[i] = gyrocal[i];
			}

// 2 and 15 seconds
while ( time - timestart < CAL_TIME  &&  time - timemax < 15e6 )
	{

		unsigned long looptime;
		looptime = time - lastlooptime;
		lastlooptime = time;
		if ( looptime == 0 ) looptime = 1;

	i2c_readdata( 67 , data, 6 );

		gyro[0] = (int16_t) ((data[2]<<8) + data[3]);
		gyro[1] = (int16_t) ((data[0]<<8) + data[1]);
		gyro[2] = (int16_t) ((data[4]<<8) + data[5]);



		 for ( int i = 0 ; i < 3 ; i++)
			{

					if ( gyro[i] > limit[i] )  limit[i] += 0.1f; // 100 gyro bias / second change
					if ( gyro[i] < limit[i] )  limit[i] -= 0.1f;

					limitf( &limit[i] , 800);

					if ( fabs(gyro[i]) > 100+ fabs(limit[i]) )
					{
						timestart = gettime();
					}
					else
					{
					lpf( &gyrocal[i] , gyro[i], lpfcalc( (float) looptime , 0.5 * 1e6) );

					}

			}

while ( (gettime() - time) < 1000 ) delay(10);
time = gettime();

	}



if ( time - timestart < CAL_TIME )
{
	for ( int i = 0 ; i < 3; i++)
	{
	gyrocal[i] = 0;

	}

}


#ifdef SERIAL
printf("gyro calibration  %f %f %f \n "   , gyrocal[0] , gyrocal[1] , gyrocal[2]);
#endif

}
