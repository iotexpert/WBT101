/*
 * Copyright Cypress Semiconductor
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Cypress Semiconductor;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Cypress Semiconductor.
 */

 /** @file
 *
 * This file contains the APIs related to the lsm9ds1 interface
 *
 */

#include "wiced_bt_trace.h"
#include "wiced_hal_i2c.h"
#include "wiced_hal_adc.h"
#include "lsm9ds1_acc_gyro_driver.h"
#include "lsm9ds1_mag_driver.h"
#include "ex07_low_power_hw.h"
#include "wiced_rtos.h"
#include "ex07_low_power_gatt_db.h"
#include "rtc.h"
#include "spar_utils.h"

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                            Functions prototype
 ******************************************************************************/

void init_LSM9DS1_ACC_GYRO(void);

void init_LSM9DS1_MAG(void);

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

PLACE_DATA_IN_RETENTION_RAM tRTC_REAL_TIME_CLOCK snapshot_time;
PLACE_DATA_IN_RETENTION_RAM tRTC_REAL_TIME_CLOCK battery_snapshot_time;
PLACE_DATA_IN_RETENTION_RAM uint8_t battery_snapshot;

 /******************************************************************************
 *                                Function Definitions
 ******************************************************************************/

/**
 * Function         init_LSM9DS1_ACC_GYRO
 *
 * @brief           This function initializes accelerometer and gyroscope
 *
 * @return          : None
 */
void init_LSM9DS1_ACC_GYRO(void)
{
    status_t response;
    uint8_t value;
  /* Gyro ODR and full scale */
  response = LSM9DS1_ACC_GYRO_W_GyroDataRate(LSM9DS1_ACC_GYRO_ODR_G_15Hz);
  if(MEMS_ERROR == response) while(1); /*manage here communication error */

  response = LSM9DS1_ACC_GYRO_W_GyroFullScale(LSM9DS1_ACC_GYRO_FS_G_245dps);
  if(MEMS_ERROR == response) while(1); /*manage here communication error */

  response = LSM9DS1_ACC_GYRO_W_GyroLowPower(LSM9DS1_ACC_GYRO_LP_G_ENABLE);
  if(MEMS_ERROR == response) while(1); /*manage here communication error */

  /* Acc ODR and full scale */
  response = LSM9DS1_ACC_GYRO_W_AccelerometerDataRate(LSM9DS1_ACC_GYRO_ODR_XL_10Hz);
  if(MEMS_ERROR == response) while(1); /*manage here communication error */

  response = LSM9DS1_ACC_GYRO_W_AccelerometerFullScale(LSM9DS1_ACC_GYRO_FS_XL_8g);
  if(MEMS_ERROR == response) while(1); /*manage here communication error */

  /* BDU Enable */
  response = LSM9DS1_ACC_GYRO_W_BlockDataUpdate(LSM9DS1_ACC_GYRO_BDU_ENABLE);
  if(MEMS_ERROR == response) while(1); /*manage here communication error */

  /* Enable Acc interrupts to send periodic notifications in SDS */
  value = (LSM9DS1_ACC_GYRO_XHIE_XL_ENABLE | LSM9DS1_ACC_GYRO_YHIE_XL_ENABLE | LSM9DS1_ACC_GYRO_ZHIE_XL_ENABLE | LSM9DS1_ACC_GYRO_AOI_XL_OR);

  LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, value);
  LSM9DS1_ACC_GYRO_W_XL_InterruptThresholdAxisX(X_THRESHOLD);
  LSM9DS1_ACC_GYRO_W_XL_InterruptThresholdAxisY(Y_THRESHOLD);
  LSM9DS1_ACC_GYRO_W_XL_InterruptThresholdAxisZ(Z_THRESHOLD);

}

/**
 * Function         init_LSM9DS1_MAG
 *
 * @brief           This function initializes magnetometer
 *
 * @return          : None
 */
void init_LSM9DS1_MAG(void)
{
    status_t response;
  response = LSM9DS1_MAG_W_FullScale(LSM9DS1_MAG_FS_16Ga);
  if(MEMS_ERROR == response) while(1); /*manage here communication error */

  response = LSM9DS1_MAG_W_OutputDataRate(LSM9DS1_MAG_DO_0_625Hz);
  if(MEMS_ERROR == response) while(1); /*manage here communication error */

  response = LSM9DS1_MAG_W_BlockDataUpdate(LSM9DS1_MAG_BDU_ENABLE);
  if(MEMS_ERROR == response) while(1); /*manage here communication error */

  response = LSM9DS1_MAG_W_OperatingModeXY(LSM9DS1_MAG_OM_HIGH);
  if(MEMS_ERROR == response) while(1); /*manage here communication error */

  response = LSM9DS1_MAG_W_OperatingModeZ(LSM9DS1_MAG_OMZ_HIGH);
  if(MEMS_ERROR == response) while(1); /*manage here communication error */

  response = LSM9DS1_MAG_W_SystemOperatingMode(LSM9DS1_MAG_MD_CONTINUOUS);;
  if(MEMS_ERROR == response) while(1); /*manage here communication error */
}

/**
 * Function         initialize_all_sensors
 *
 * @brief           This function initializes all sensors
 *
 * @return          : None
 */
void initialize_all_sensors (void)
{
    init_LSM9DS1_ACC_GYRO();
    init_LSM9DS1_MAG();
}

/**
 * Function         get_accel_val
 *
 * @brief           This function gets accelerometer value
 *
 * @param[in] accel_struct       : Pointer to store accelerometer value
 *
 * @return                       : None
 */
void get_accel_val (Type3Axisint16* accel_struct)
{
    LSM9DS1_ACC_GYRO_XLDA_t value_XL;
    Type3Axis16bit_U Acceleration;
    status_t response;

    /*Read ACC output only if new ACC value is available */
    response =  LSM9DS1_ACC_GYRO_R_AccelerometerDataReadyFlag(&value_XL);
    if(MEMS_ERROR == response) while(1); /*manage here communication error */

    if (LSM9DS1_ACC_GYRO_XLDA_UP==value_XL)
    {
      LSM9DS1_ACC_GYRO_Get_Acceleration(Acceleration.u8bit);

      accel_struct->value_x=Acceleration.i16bit[0]/16;
      accel_struct->value_y=Acceleration.i16bit[1]/16;
      accel_struct->value_z=Acceleration.i16bit[2]/16;
    }
    else
    {
        WICED_BT_TRACE("Data Not Available\n\r");
    }
}

/**
 * Function         get_gyro_val
 *
 * @brief           This function gets gyroscope value
 *
 * @param[in] gyro_struct        : Pointer to store gyroscope value
 *
 * @return                       : None
 */
void get_gyro_val (Type3Axisint16* gyro_struct)
{
    LSM9DS1_ACC_GYRO_GDA_t value_G;
    Type3Axis16bit_U AngularRate;
    status_t response;

    response =  LSM9DS1_ACC_GYRO_R_GyroDataReadyFlag(&value_G);
    if(MEMS_ERROR == response) while(1); /*manage here communication error */

    if (LSM9DS1_ACC_GYRO_GDA_UP==value_G)
    {
      LSM9DS1_ACC_GYRO_Get_AngularRate(AngularRate.u8bit);

      gyro_struct->value_x=AngularRate.i16bit[0];
      gyro_struct->value_y=AngularRate.i16bit[1];
      gyro_struct->value_z=AngularRate.i16bit[2];
    }
}

/**
 * Function         get_mag_val
 *
 * @brief           This function gets magnetometer value
 *
 * @param[in] gyro_struct        : Pointer to store magnetometer value
 *
 * @return                       : None
 */
void get_mag_val (Type3Axisint16* mag_struct)
{
    LSM9DS1_MAG_ZYXDA_t value_M;
    Type3Axis16bit_U magnaticField;
    status_t response;

/*Read MAG output only if new value is available */
response =  LSM9DS1_MAG_R_NewXYZData(&value_M);
if(MEMS_ERROR == response) while(1); /*manage here communication error */

if (LSM9DS1_MAG_ZYXDA_AVAILABLE==value_M)
    {
      LSM9DS1_MAG_Get_Magnatic(magnaticField.u8bit);
      mag_struct->value_x=magnaticField.i16bit[0];
      mag_struct->value_y=magnaticField.i16bit[1];
      mag_struct->value_z=magnaticField.i16bit[2];
    }
}

/**
 * Function         get_temp_val
 *
 * @brief           This function gets temperature value
 *
 * @param[in] gyro_struct        : Pointer to store temperature value
 *
 * @return                       : None
 */
void get_temp_val (int16_t* temp_struct)
{
    LSM9DS1_ACC_GYRO_TDA_t value_G;
    Temperature16bit_U Temperature;
    status_t response;

    response =  LSM9DS1_ACC_GYRO_R_TemperatureDataReadyFlag(&value_G);
    if(MEMS_ERROR == response) while(1); /*manage here communication error */

    if (LSM9DS1_ACC_GYRO_TDA_UP==value_G)
    {
        LSM9DS1_ACC_GYRO_Get_Temperature(Temperature.u8bit);

      /* Transorm LSB into Celcius */
        *temp_struct=(int16_t) ROUND((((float) Temperature.i16bit)/SENSITIVITY_C) + TEMP_AT_0);
    }
}

/**
 * Function         read_all_sensors_update_gatt
 *
 * @brief           This function reads all sensors and updates the value in Gatt DB
 *
 * @return                       : None
 */
void read_all_sensors_update_gatt()
{

    tRTC_REAL_TIME_CLOCK samp_time;

    rtc_getRTCRawClock(&samp_time);

    WICED_BT_TRACE("Sample time: %d, Snapshot time: %d\n\r", samp_time.reg16map.rtc16[1], snapshot_time.rtc64);

    if(MS_500_32KHZ < (samp_time.rtc64 - snapshot_time.rtc64))
    {
        WICED_BT_TRACE("Updating data\n\r");
        power_up_ex07_low_power();

        get_accel_val(&(ex07_low_power_char_notify_value.sensor_data.sensor_values[0]));

        get_gyro_val(&(ex07_low_power_char_notify_value.sensor_data.sensor_values[1]));

        get_mag_val(&(ex07_low_power_char_notify_value.sensor_data.sensor_values[2]));

        power_down_ex07_low_power();

        rtc_getRTCRawClock(&snapshot_time);

        WICED_BT_TRACE("Snapshot time: %d\n\r", snapshot_time.reg16map.rtc16[0]);

        snapshot_time.rtc64 = samp_time.rtc64;
    }
}

/**
 * Function         power_down_ex07_low_power
 *
 * @brief           This function power downs the LSM9DS1 sensor
 *
 * @return                       : None
 */
void power_down_ex07_low_power(void)
{
    status_t response;

    response = LSM9DS1_ACC_GYRO_W_GyroDataRate(LSM9DS1_ACC_GYRO_ODR_G_POWER_DOWN);
    if(MEMS_ERROR == response) while(1); /*manage here communication error */

    response = LSM9DS1_MAG_W_SystemOperatingMode(LSM9DS1_MAG_MD_POWER_DOWN);
    if(MEMS_ERROR == response) while(1); /*manage here communication error */
}

/**
 * Function         power_up_ex07_low_power
 *
 * @brief           This function powers up the LSM9DS1 sensor
 *
 * @return                       : None
 */
void power_up_ex07_low_power(void)
{
    LSM9DS1_ACC_GYRO_GDA_t value_G;
    LSM9DS1_ACC_GYRO_XLDA_t value_XL;
    LSM9DS1_MAG_ZYXDA_t value_M;

    status_t response;

    response = LSM9DS1_ACC_GYRO_W_GyroDataRate(LSM9DS1_ACC_GYRO_ODR_G_15Hz);
    if(MEMS_ERROR == response) while(1); /*manage here communication error */

    response = LSM9DS1_MAG_W_SystemOperatingMode(LSM9DS1_MAG_MD_CONTINUOUS);
    if(MEMS_ERROR == response) while(1); /*manage here communication error */

    wiced_rtos_delay_milliseconds(DELAY_FOR_SENSOR_WAKEUP , ALLOW_THREAD_TO_SLEEP);
}

/**
 * Function         ex07_low_power_acc_interrupt_enable
 *
 * @brief           This function enables the accelerometer interrupt
 *
 * @return                       : None
 */
void ex07_low_power_acc_interrupt_enable(void)
{
    LSM9DS1_ACC_GYRO_W_Accelerometer_OnINT(LSM9DS1_ACC_GYRO_INT_IG_XL_ENABLE);
}

/**
 * Function         ex07_low_power_acc_interrupt_disable
 *
 * @brief           This function disables the accelerometer interrupt
 *
 * @return                       : None
 */
void ex07_low_power_acc_interrupt_disable(void)
{
    LSM9DS1_ACC_GYRO_W_Accelerometer_OnINT(LSM9DS1_ACC_GYRO_INT_IG_XL_DISABLE);
}

/**
 * Function         enable_battmon
 *
 * @brief           Enables battery monitoring
 *
 * @return                       : None
 */
void enable_battmon(void)
{
    configure_adc();
    update_battery_level_and_report();
}

/**
 * Function         update_battery_level_and_report
 *
 * @brief           This function updates the battery level
 *
 * @return                       : None
 */
void update_battery_level_and_report(void)
{
    uint8_t current_battery_level;
    tRTC_REAL_TIME_CLOCK samp_time;

    rtc_getRTCRawClock(&samp_time);

    if((S_60_32KHZ < (samp_time.rtc64 - battery_snapshot_time.rtc64)) || 0 == battery_snapshot_time.rtc64)
    {
        current_battery_level = batmon_battery_level();
        WICED_BT_TRACE("Battery level: %d\n\r", current_battery_level);

        if(SHUTDOWN_LEVEL < current_battery_level && CRITICAL_LEVEL > current_battery_level)
        {
            wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);
        }

        if(current_battery_level < SHUTDOWN_LEVEL)
        {
            shutdown_cb();
        }

        if(current_battery_level != battery_level)
        {
            send_battery_notification();
        }
        battery_level = current_battery_level;

        battery_snapshot_time.rtc64 = samp_time.rtc64;
    }
}

/**
 * Function         configure_adc
 *
 * @brief           configures the ADC for battery monitoring
 *
 * @return                       : None
 */
void configure_adc(void)
{
    wiced_hal_adc_init();
    wiced_hal_adc_set_input_range(ADC_RANGE_0_3P6V);

}

/**
 * Function         batmon_measure_average
 *
 * @brief           Measures and averages the battery voltage level
 *
 * @return                       : Returns the average voltage
 */
uint32_t batmon_measure_average(void)
{
    uint32_t new_measurement;
    uint8_t measurement_no = 0;

    for(measurement_no = SAMPLES_AVG; measurement_no > 0; measurement_no--)
    {
        new_measurement += wiced_hal_adc_read_voltage(BATTERY_INPUT);
    }

    new_measurement = new_measurement/SAMPLES_AVG;

    return new_measurement;
}

/**
 * Function         batmon_battery_level
 *
 * @brief           Converts the voltage level to percentage
 *
 * @return                       : Returns the battery percentage
 */
uint8_t batmon_battery_level(void)
{
    uint32_t tmp;
    uint8_t level;
    uint32_t average_value;

    average_value = batmon_measure_average();
    /* Take the current average and convert it into report units. Check ranges */
    if (average_value >= FULL_VOLTAGE)
    {
        level = MAX_LEVEL;
    }
    else if (average_value <= EMPTY_VOLTAGE)
    {
        level = 0;
    }
    else
    {
        /* The actual value is in between the full-empty range. Calculate using DWORD math and then assign */
        /* to new value */
        tmp = (average_value - EMPTY_VOLTAGE);
        tmp *= MAX_LEVEL;
        tmp /= (FULL_VOLTAGE - EMPTY_VOLTAGE);
        level = (uint8_t)tmp;
    }

    return level;
}


