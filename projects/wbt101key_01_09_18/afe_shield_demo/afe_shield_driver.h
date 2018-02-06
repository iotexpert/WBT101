/*
 * Copyright 2017, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor
 *  Corporation. All rights reserved. This software, including source code, documentation and  related
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection
 * (United States and foreign), United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit
 * products. Any reproduction, modification, translation, compilation,  or representation of this
 * Software except as specified above is prohibited without the express written permission of
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to
 * the Software without notice. Cypress does not assume any liability arising out of the application
 * or use of the Software or any product or circuit  described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or failure of the
 * Cypress product may reasonably be expected to result  in significant property damage, injury
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees
 * to indemnify Cypress against all liability.
 */

/** @file
 *
 * Provides the driver interface for AFE shield.
 */

#include "wiced.h"

/******************************************************
 *                      Constants
 ******************************************************/
#define AFE_SLAVE_ADDR 0x42

#define DAC_REG_ADDR 0x00
#define LED_VALUE_REG_ADDR 0x04
#define LED_CONTROL_REG_ADDR 0x05
#define BUTTON_STATE_REG_ADDR 0x06
#define TEMPERATURE_REG_ADDR 0x07
#define HUMIDITY_REG_ADDR 0x0B
#define AMBIENT_LIGHT_REG_ADDR 0x0F
#define POT_REG_ADDR 0x13

void afe_shield_init(void);

/*
 * Gets the DAC value on the AFE shield.
 * \param status Pointer to the float variable to store the DAC value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_dac_value(uint8_t *value);

/*
 * Sets the DAC value on the AFE shield.
 * \param status Pointer to the float variable that gives the DAC value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_set_dac_value(uint8_t *value);

/*
 * Gets the LED Value on the AFE shield.
 * \param status Pointer to the 8 bit variable that will hold the LED value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_led_value(uint8_t* value);

/*
 * Sets the LED value on the AFE shield.
 * \param status 8 bit variable that holds the value to set.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_set_led_value(uint8_t status);

/*
 * Gets the LED Control register value on the AFE shield.
 * \param status Pointer to the 8 bit variable that will hold the LED Controlvalue.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_led_control_value(uint8_t* value);

/*
 * Sets the LED Control value on the AFE shield.
 * \param status 8 bit variable that holds the LED control value to set.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_set_led_control_value(uint8_t status);

/*
 * Gets the Button State register value on the AFE shield.
 * \param status Pointer to the 8 bit variable that will hold the Button State value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_button_state_value(uint8_t* value);

/*
 * Gets the temperature value from the AFE shield.
 * \param status Pointer to the float variable that will hold the temperature value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_temperature_value(uint8_t* value);

/*
 * Gets the humidity value from the AFE shield.
 * \param status Pointer to the float variable that will hold the humidity value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_humidity_value(uint8_t *value);

/*
 * Gets the ambient value from the AFE shield.
 * \param status Pointer to the float variable that will hold the ambient value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_ambient_light_value(uint8_t *value);

/*
 * Gets the potentiometer value from the AFE shield.
 * \param status Pointer to the float variable that will hold the potentiometer value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_pot_value(uint8_t *value);
