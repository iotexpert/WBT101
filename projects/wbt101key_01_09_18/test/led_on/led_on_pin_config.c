/*
 * $ Copyright Cypress Semiconductor $
 */

/*
 * This file is specific to this led_on app only, the default pin configuration
 * for this platform can be found in the platforms/CYW920719Q40EVB_01/wiced_platform_pin_config.c
 * Please note that its mandatory to name this file in the format "led_on_pin_config.c"
 */

#include "wiced_platform.h"

wiced_platform_gpio_t platform_gpio_pins[]=
	{
		[PLATFORM_GPIO_0]	= {GPIO_P00, WICED_SWD_IO},
		[PLATFORM_GPIO_1]	= {GPIO_P01, WICED_GPIO},
		[PLATFORM_GPIO_2]	= {GPIO_P02, WICED_PCM_SYNC_I2S_WS},
		[PLATFORM_GPIO_3]	= {GPIO_P03, WICED_UART_2_RXD},
		[PLATFORM_GPIO_4]	= {GPIO_P04, WICED_UART_2_RTS},
		[PLATFORM_GPIO_5]	= {GPIO_P05, WICED_UART_2_CTS},
		[PLATFORM_GPIO_6]	= {GPIO_P06, WICED_PCM_CLK_I2S_CLK},
		[PLATFORM_GPIO_7]	= {GPIO_P07, WICED_I2C_1_SDA},
		[PLATFORM_GPIO_8]	= {GPIO_P08, WICED_I2C_1_SCL},
		[PLATFORM_GPIO_9]	= {GPIO_P09, WICED_GPIO},
		[PLATFORM_GPIO_10]	= {GPIO_P10, WICED_GPIO},
		[PLATFORM_GPIO_11]	= {GPIO_P11, WICED_PCM_OUT_I2S_DO},
		[PLATFORM_GPIO_12]	= {GPIO_P12, WICED_PCM_IN_I2S_DI},
		[PLATFORM_GPIO_13]	= {GPIO_P13, WICED_UART_2_TXD},
		[PLATFORM_GPIO_14]	= {GPIO_P14, WICED_I2C_2_SDA},
		[PLATFORM_GPIO_15]	= {GPIO_P15, WICED_I2C_2_SCL},
		[PLATFORM_GPIO_16]	= {GPIO_P16, WICED_GPIO},
		[PLATFORM_GPIO_17]	= {GPIO_P17, WICED_GPIO},
		[PLATFORM_GPIO_18]	= {GPIO_P18, WICED_GPIO},
		[PLATFORM_GPIO_19]	= {GPIO_P19, WICED_GPIO},
		[PLATFORM_GPIO_20]	= {GPIO_P33, WICED_PUART_TXD},
	};

#define WICED_PLATFORM_GPIO_MAX	 7


wiced_platform_gpio_config_t platform_gpio[WICED_PLATFORM_GPIO_MAX]=
	{
		[WICED_PLATFORM_GPIO_1] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_1].gpio_pin,
				.config			= (GPIO_TRIGGER_NEG | GPIO_INTERRUPT_ENABLE),
				.default_state	= GPIO_PIN_OUTPUT_LOW,
			},
		[WICED_PLATFORM_GPIO_2] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_9].gpio_pin,
				.config			= (),
				.default_state	= ,
			},
		[WICED_PLATFORM_GPIO_3] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_10].gpio_pin,
				.config			= (),
				.default_state	= ,
			},
		[WICED_PLATFORM_GPIO_4] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_16].gpio_pin,
				.config			= (GPIO_PULL_UP),
				.default_state	= GPIO_PIN_OUTPUT_HIGH,
			},
		[WICED_PLATFORM_GPIO_5] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_17].gpio_pin,
				.config			= (),
				.default_state	= ,
			},
		[WICED_PLATFORM_GPIO_6] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_18].gpio_pin,
				.config			= (),
				.default_state	= ,
			},
		[WICED_PLATFORM_GPIO_7] =
			{
				.gpio			= &platform_gpio_pins[PLATFORM_GPIO_19].gpio_pin,
				.config			= (),
				.default_state	= ,
			},
	};
