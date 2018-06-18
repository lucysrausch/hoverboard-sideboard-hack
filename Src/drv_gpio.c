#include "gd32f1x0.h"
#include "drv_gpio.h"

#include "defines.h"
void gpio_init(void)
{
// clocks on to all ports
    RCC_AHBPeriphClock_Enable(RCC_AHBPERIPH_GPIOA|RCC_AHBPERIPH_GPIOB|RCC_AHBPERIPH_GPIOF,ENABLE);

    GPIO_InitPara GPIO_InitStructure;

// common settings to set ports
	  GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStructure.GPIO_OType = GPIO_OTYPE_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PUPD_NOPULL;

    GPIO_InitStructure.GPIO_Pin = LED1_Pin;
    GPIO_Init(LED1_GPIO_Port,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED2_Pin;
    GPIO_Init(LED2_GPIO_Port,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED3_Pin;
    GPIO_Init(LED3_GPIO_Port,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED4_Pin;
    GPIO_Init(LED4_GPIO_Port,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED5_Pin;
    GPIO_Init(LED5_GPIO_Port,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED_50MHZ;
    GPIO_InitStructure.GPIO_OType = GPIO_PUPD_NOPULL;

    GPIO_InitStructure.GPIO_Pin = SENSOR1_Pin;
    GPIO_Init(SENSOR1_GPIO_Port,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SENSOR2_Pin;
    GPIO_Init(SENSOR2_GPIO_Port,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_OType = GPIO_PUPD_PULLDOWN;

    GPIO_InitStructure.GPIO_Pin = GPIO_PIN_3;
    GPIO_Init(GPIOA,&GPIO_InitStructure);





}
