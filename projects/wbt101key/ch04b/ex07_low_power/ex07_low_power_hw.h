#ifndef HW_H
#define HW_H

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define SENSITIVITY_16Ga     (float) 0.58/1000              /* Ga/LSB */

#define SENSITIVITY_8G    (float) 0.244/1000 /* G/LSB */

#define SENSITIVITY_245DPS    (float) 8.75/1000 /* dps/LSB */

#define SENSITIVITY_C           16 /* LSB/degC */

#define TEMP_AT_0               25

#define DELAY_FOR_SENSOR_WAKEUP 10 /* 10 ms */

#define X_THRESHOLD 50
#define Y_THRESHOLD 50
#define Z_THRESHOLD 50

#define MS_500_32KHZ 15000      /* 500 ms interval with 32 KHz freq */
#define S_60_32KHZ   32000 * 60 /* 60 s interval with 32 KHz freq */

#define BATTERY_INPUT          ADC_INPUT_VDDIO /* Measuring VDDIO for drop in voltage */
#define SAMPLES_AVG            8 /* Samples to average for battery monitoring */
#define FULL_VOLTAGE           3000 /* Full voltage 3V */
#define EMPTY_VOLTAGE          2000 /* Empty voltage 2V */
#define CRITICAL_LEVEL         30   /* Critical level. Turn off PUART */
#define SHUTDOWN_VOLTAGE       2100 /* Shutdown voltage 2.1V */
#define SHUTDOWN_LEVEL         8 /* Shutdown at 10% battery */
#define MAX_LEVEL              100 /* Value at full battery 100 */


#define ROUND(num) ((num < 0)? (num - (float) 0.5) : (num + (float) 0.5))

/******************************************************************************
 *                                Typedefs
 ******************************************************************************/
typedef struct {
    float value_x;
    float value_y;
    float value_z;
} Type3Axisfloat;

typedef struct {
    int16_t value_x;
    int16_t value_y;
    int16_t value_z;
} Type3Axisint16;

typedef struct{
    int16_t value_temp;
} TypeTempint16;

/******************************************************************************
*                             Function prototypes
******************************************************************************/
void get_accel_val (Type3Axisint16* accel_struct);

void get_gyro_val (Type3Axisint16* gyro_struct);

void get_mag_val (Type3Axisint16* gyro_struct);

void initialize_all_sensors (void);

void get_temp_val (int16_t* temp_struct);

void read_all_sensors_update_gatt();

void power_down_ex07_low_power(void);

void power_up_ex07_low_power(void);

void ex07_low_power_acc_interrupt_enable(void);

void update_battery_level_and_report(void);

void configure_adc(void);

uint32_t batmon_measure_average(void);

uint8_t batmon_battery_level(void);

void test_ob(uint32_t value);

void enable_battmon(void);

void ex07_low_power_acc_interrupt_disable(void);

#endif
