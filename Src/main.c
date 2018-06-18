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


// Eachine H8mini acro firmware
// files of this project should be assumed MIT licence unless otherwise noted


#include "gd32f1x0.h"

#include "util.h"
//#include "sixaxis.h"
#include "drv_time.h"
#include "drv_softi2c.h"
#include "drv_gpio.h"
#include "drv_serial.h"
#include "drv_i2c.h"
#include "defines.h"

#include <inttypes.h>

// hal
void clk_init(void);

extern float gyro[3];
extern float accel[3];

int main(void)
{

	clk_init();

	delay(10000);

  gpio_init();

	serial_init();
	printf( "Otter!\n\r" );

	i2c_init();

  time_init();
	sixaxis_init();

	for (int i = 0; i < 6; i++) {
		GPIO_WriteBit(LED3_GPIO_Port, LED3_Pin, Bit_RESET);
		GPIO_WriteBit(LED1_GPIO_Port, LED1_Pin, Bit_SET);
		delay(30000);
		GPIO_WriteBit(LED2_GPIO_Port, LED2_Pin, Bit_SET);
		GPIO_WriteBit(LED1_GPIO_Port, LED1_Pin, Bit_RESET);
		delay(30000);
		GPIO_WriteBit(LED3_GPIO_Port, LED3_Pin, Bit_SET);
		GPIO_WriteBit(LED2_GPIO_Port, LED2_Pin, Bit_RESET);
		delay(30000);
	}

	GPIO_WriteBit(LED3_GPIO_Port, LED3_Pin, Bit_RESET);

	if ( !sixaxis_check() )
	{
		printf( "ERROR\n\r" );
	}

	delay(50000);

	//rx_init();

	//gyro_cal();


	extern unsigned int liberror;
	if ( liberror )
	{
		  printf( "ERROR: I2C \n\r" );
	}

	delay(50000);


//
//
// 		MAIN LOOP
//
//
	GPIO_WriteBit(LED1_GPIO_Port, LED1_Pin, Bit_SET);
	while(1)
	{
		// gettime() needs to be called at least once per second
		/*unsigned long time = gettime();
		//gyro_read();*/

		//delay(800);

		while(!GPIO_ReadInputBit(GPIOA, GPIO_PIN_3)) {}



		if (GPIO_ReadInputBit(SENSOR1_GPIO_Port, SENSOR1_Pin) | GPIO_ReadInputBit(SENSOR2_GPIO_Port, SENSOR2_Pin)) {
			GPIO_WriteBit(LED4_GPIO_Port, LED4_Pin, Bit_SET);
			GPIO_WriteBit(LED5_GPIO_Port, LED5_Pin, Bit_SET);
		} else {
			GPIO_WriteBit(LED4_GPIO_Port, LED4_Pin, Bit_RESET);
			GPIO_WriteBit(LED5_GPIO_Port, LED5_Pin, Bit_RESET);
		}


		gyro_read();
		delay(500);

		//printf( "X: %d Y: %d Z: %d\n\r", (int)(accel[0]*1000), (int)(accel[1]*1000), (int)(accel[2]*1000));

	}// end loop


}

void HardFault_Handler(void)
{

}
void MemManage_Handler(void)
{

}
void BusFault_Handler(void)
{

}
void UsageFault_Handler(void)
{

}
