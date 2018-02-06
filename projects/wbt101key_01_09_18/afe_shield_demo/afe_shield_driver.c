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
 * Drivers for the AFE shield.
 */

#include "wiced_bt_trace.h"
#include "wiced_hal_i2c.h"
#include "wiced_hal_gpio.h"
#include "afe_shield_driver.h"
#include "wiced_platform.h"

extern wiced_platform_gpio_t platform_gpio_pins[];

/******************************************************
 *           Local Function Declarations
 ******************************************************/
static uint8_t afe_shield_read_8_bit_reg(uint8_t reg, int8_t* value);
static uint8_t afe_shield_write_8_bit_reg(uint8_t reg, int8_t value);
static uint8_t afe_shield_read_32_bit_reg(uint8_t reg, uint8_t* value);
static uint8_t afe_shield_write_32_bit_reg(uint8_t reg, uint8_t* value);

/******************************************************
 *           Local function definitions.
 ******************************************************/

static uint8_t afe_shield_read_8_bit_reg(uint8_t reg, int8_t* value)
{
//    return i2cm_comboRead(value, sizeof(int8_t), &reg, sizeof(uint8_t), AFE_SLAVE_ADDR);
    wiced_hal_i2c_write(&reg, sizeof(reg), AFE_SLAVE_ADDR);
    wiced_hal_i2c_read(value, sizeof(int8_t), AFE_SLAVE_ADDR);
}

static uint8_t afe_shield_write_8_bit_reg(uint8_t reg, int8_t value)
{
    uint8_t reg_data_bytes[2];

    reg_data_bytes[0] = reg;
    reg_data_bytes[1] = value;

    return wiced_hal_i2c_write(reg_data_bytes, sizeof(reg_data_bytes), AFE_SLAVE_ADDR);
}

static uint8_t afe_shield_read_32_bit_reg(uint8_t reg, uint8_t* value)
{
    uint8_t status;
    uint8_t reg_bytes_to_read[4];

//    if ((status = i2cm_comboRead(reg_bytes_to_read, sizeof(reg_bytes_to_read), &reg, sizeof(uint8_t), AFE_SLAVE_ADDR) == I2CM_SUCCESS))
//    {
//        *value = (int32_t)(reg_bytes_to_read[3] | reg_bytes_to_read[2] << 8 | reg_bytes_to_read[1] << 16 | reg_bytes_to_read[0] << 24);
//        return 0;
//    }

    wiced_hal_i2c_write(&reg, sizeof(reg), AFE_SLAVE_ADDR);
    wiced_hal_i2c_read(value, sizeof(uint32_t), AFE_SLAVE_ADDR);

    return status;
}

static uint8_t afe_shield_write_32_bit_reg(uint8_t reg, uint8_t* value)
{
    uint8_t reg_bytes_to_write[5];
    reg_bytes_to_write[0] = reg;
    reg_bytes_to_write[4] = *(value);
    reg_bytes_to_write[3] = *(value + 1);
    reg_bytes_to_write[2] = *(value + 2);
    reg_bytes_to_write[1] = *(value + 3);
//    &reg_bytes_to_write[1] = value;

    return wiced_hal_i2c_write(reg_bytes_to_write, sizeof(reg_bytes_to_write), AFE_SLAVE_ADDR);
}

/******************************************************
 *           Exported function definitions.
 ******************************************************/
/*
 * Initializes the AFE Shield.
 * \return 0 for success; 1 for failure;
 */
void afe_shield_init(void)
{

    // Initialize the I2C HW block.
    wiced_hal_i2c_init();
    // Set the speed to 100 KHz
    wiced_hal_i2c_set_speed(I2CM_SPEED_100KHZ);

}

/*
 * Gets the DAC value on the AFE shield.
 * \param status Pointer to the float variable to store the DAC value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_dac_value(uint8_t *value)
{
    return afe_shield_read_32_bit_reg(DAC_REG_ADDR, value) == I2CM_SUCCESS ? 0 : 1;
}

/*
 * Sets the DAC value on the AFE shield.
 * \param status Pointer to the float variable that gives the DAC value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_set_dac_value(uint8_t *value)
{
    return afe_shield_write_32_bit_reg(DAC_REG_ADDR, value) == I2CM_SUCCESS ? 0 : 1;
}

/*
 * Gets the LED Value on the AFE shield.
 * \param status Pointer to the 8 bit variable that will hold the LED value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_led_value(uint8_t* value)
{
    return afe_shield_read_8_bit_reg(LED_VALUE_REG_ADDR, value) == I2CM_SUCCESS ? 0 : 1;
}

/*
 * Sets the LED value on the AFE shield.
 * \param status 8 bit variable that holds the value to set.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_set_led_value(uint8_t status)
{
    return afe_shield_write_8_bit_reg(LED_VALUE_REG_ADDR, status) == I2CM_SUCCESS ? 0 : 1;
}

/*
 * Gets the LED Control register value on the AFE shield.
 * \param status Pointer to the 8 bit variable that will hold the LED Controlvalue.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_led_control_value(uint8_t* value)
{
    return afe_shield_read_8_bit_reg(LED_CONTROL_REG_ADDR, value) == I2CM_SUCCESS ? 0 : 1;
}

/*
 * Sets the LED Control value on the AFE shield.
 * \param status 8 bit variable that holds the LED control value to set.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_set_led_control_value(uint8_t status)
{
    return afe_shield_write_8_bit_reg(LED_CONTROL_REG_ADDR, status) == I2CM_SUCCESS ? 0 : 1;
}

/*
 * Gets the Button State register value on the AFE shield.
 * \param status Pointer to the 8 bit variable that will hold the Button State value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_button_state_value(uint8_t* value)
{
    return afe_shield_read_8_bit_reg(BUTTON_STATE_REG_ADDR, value) == I2CM_SUCCESS ? 0 : 1;
}

/*
 * Gets the temperature value from the AFE shield.
 * \param status Pointer to the float variable that will hold the temperature value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_temperature_value(uint8_t *value)
{
    return afe_shield_read_32_bit_reg(TEMPERATURE_REG_ADDR, value) == I2CM_SUCCESS ? 0 : 1;
}

/*
 * Gets the humidity value from the AFE shield.
 * \param status Pointer to the float variable that will hold the humidity value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_humidity_value(uint8_t *value)
{
    return afe_shield_read_32_bit_reg(HUMIDITY_REG_ADDR, value) == I2CM_SUCCESS ? 0 : 1;
}

/*
 * Gets the ambient value from the AFE shield.
 * \param status Pointer to the float variable that will hold the ambient value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_ambient_light_value(uint8_t *value)
{
    return afe_shield_read_32_bit_reg(AMBIENT_LIGHT_REG_ADDR, value) == I2CM_SUCCESS ? 0 : 1;
}

/*
 * Gets the potentiometer value from the AFE shield.
 * \param status Pointer to the float variable that will hold the potentiometer value.
 *               Valid only when the function returns success.
 * \return 0 for success; 1 for failure.
 */
uint8_t afe_shield_get_pot_value(uint8_t *value)
{
    return afe_shield_read_32_bit_reg(POT_REG_ADDR, value) == I2CM_SUCCESS ? 0 : 1;
}
