
/******************** (C) COPYRIGHT 2014 STMicroelectronics ********************
* File Name          : LSM9DS1_ACC_GYRO_driver.c
* Author             : MSH Application Team
* Version            : v1.00
* Date               : 15/05/2014
* Description        : LSM9DS1 ACC_GYRO driver source file
*                      
* HISTORY:
*-------------------------------------------------------------------------------
* Date:   15/05/2014
* Modification: Initial Revision                
* Author:	Platform Independent Driver Generator v0.03
* Reviewed by: Armando Visconti
*-------------------------------------------------------------------------------
*
*
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "wiced_bt_types.h"
#include "LSM9DS1_ACC_GYRO_driver.h"
#include "wiced_hal_i2c.h"
#include "wiced_bt_trace.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/*Select the interface that you use */
#define LSM9DS1_I2C_INTERFACE  
/*#define LSM9DS1_SPI_INTERFACE */

#define INVERSION_X_AXIS 			0x01
#define INVERSION_Y_AXIS 			0x02
#define INVERSION_Z_AXIS 			0x04
#define LSM9DS1_INITIALIZE		    0x80

#define I2C_SLAVE_OPERATION_READ                    (0)

/* Write operation to the lower level driver is 1. */
#define I2C_SLAVE_OPERATION_WRITE                   (1)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
UINT8 k_coeff[3];
UINT8 k_fs;

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name		: LSM9DS1_ACC_GYRO_ReadReg
* Description		: Generic Reading function. It must be fullfilled with either
*					: I2C or SPI reading functions					
* Input				: Register Address
* Output			: Data REad
* Return			: None
*******************************************************************************/
u8_t LSM9DS1_ACC_GYRO_ReadReg(u8_t Reg, u8_t* Data) 
{

      UINT8 result;

            result = i2cm_comboRead(Data, sizeof(u8_t), &Reg, sizeof(uint8_t), (LSM9DS1_ACC_GYRO_I2C_ADDRESS>>1));
            return !result;
}

/*******************************************************************************
* Function Name		: LSM9DS1_ACC_GYRO_WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input				: Register Address, Data to be written
* Output			: None
* Return			: None
*******************************************************************************/
u8_t LSM9DS1_ACC_GYRO_WriteReg(u8_t Reg, u8_t Data) 
{
    uint8_t reg_data_bytes[2];
    uint8_t result;

    reg_data_bytes[0] = Reg;
    reg_data_bytes[1] = Data;

    result = wiced_hal_i2c_write(reg_data_bytes, sizeof(reg_data_bytes), (LSM9DS1_ACC_GYRO_I2C_ADDRESS>>1));
    return !result;
}

/*******************************************************************************
* Function Name		: SwapHighLowByte
* Description		: Swap High/low byte in multiple byte values 
*                     It works with minimum 2 byte for every dimension.
*                     Example x,y,z with 2 byte for every dimension
*
* Input				: bufferToSwap -> buffer to swap 
*                     numberOfByte -> the buffer length in byte
*                     dimension -> number of dimension 
*
* Output			: bufferToSwap -> buffer swapped 
* Return			: None
*******************************************************************************/
void LSM9DS1_ACC_GYRO_SwapHighLowByte(u8_t *bufferToSwap, u8_t numberOfByte, u8_t dimension)
{

  u8_t numberOfByteForDimension, i, j;
  u8_t tempValue[10];
  
  numberOfByteForDimension=numberOfByte/dimension;
    
  for (i=0; i<dimension;i++ )
  {
     for (j=0; j<numberOfByteForDimension;j++ )
          tempValue[j]=bufferToSwap[j+i*numberOfByteForDimension];
     for (j=0; j<numberOfByteForDimension;j++ )
          *(bufferToSwap+i*(numberOfByteForDimension)+j)=*(tempValue+(numberOfByteForDimension-1)-j);
  } 
}

/* Exported functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroInactivityThreshold
* Description    : Write ACT_THS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroInactivityThreshold(u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM9DS1_ACC_GYRO_ACT_THS_POSITION; /*mask	 */
  newValue &= LSM9DS1_ACC_GYRO_ACT_THS_MASK; /*coerce */
    
  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ACT_THS, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_ACT_THS_MASK; 
  value |= newValue;
    
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_ACT_THS, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroInactivityThreshold
* Description    : Read ACT_THS
* Input          : Pointer to u8_t
* Output         : Status of ACT_THS 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroInactivityThreshold(u8_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ACT_THS, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_ACT_THS_MASK; /*coerce	 */
    *value = *value >> LSM9DS1_ACC_GYRO_ACT_THS_POSITION; /*mask	 */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroInactivityMode
* Description    : Write SLEEP_ON_INACT_EN
* Input          : LSM9DS1_ACC_GYRO_SLEEP_ON_INACT_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroInactivityMode(LSM9DS1_ACC_GYRO_SLEEP_ON_INACT_EN_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ACT_THS, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_SLEEP_ON_INACT_EN_MASK; 
  value |= newValue;
    
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_ACT_THS, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroInactivityMode
* Description    : Read SLEEP_ON_INACT_EN
* Input          : Pointer to LSM9DS1_ACC_GYRO_SLEEP_ON_INACT_EN_t
* Output         : Status of SLEEP_ON_INACT_EN see LSM9DS1_ACC_GYRO_SLEEP_ON_INACT_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroInactivityMode(LSM9DS1_ACC_GYRO_SLEEP_ON_INACT_EN_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ACT_THS, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_SLEEP_ON_INACT_EN_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroInactivityDuration
* Description    : Write ACT_DUR
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroInactivityDuration(u8_t newValue)
{
  u8_t value;
  
    newValue = newValue << LSM9DS1_ACC_GYRO_ACT_DUR_POSITION; /*mask	 */
    newValue &= LSM9DS1_ACC_GYRO_ACT_DUR_MASK; /*coerce */
      
    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ACT_DUR, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_ACT_DUR_MASK;
    value |= newValue;
      
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_ACT_DUR, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroInactivityDuration
* Description    : Read ACT_DUR
* Input          : Pointer to u8_t
* Output         : Status of ACT_DUR 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroInactivityDuration(u8_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ACT_DUR, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_ACT_DUR_MASK; /*coerce	 */
    *value = *value >> LSM9DS1_ACC_GYRO_ACT_DUR_POSITION; /*mask	 */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_InterruptAccelerometer_Low_AxisX
* Description    : Write XLIE_XL
* Input          : LSM9DS1_ACC_GYRO_XLIE_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_InterruptAccelerometer_Low_AxisX(LSM9DS1_ACC_GYRO_XLIE_XL_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_XLIE_XL_MASK; 
    value |= newValue;
      
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptAccelerometer_Low_AxisX
* Description    : Read XLIE_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_XLIE_XL_t
* Output         : Status of XLIE_XL see LSM9DS1_ACC_GYRO_XLIE_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptAccelerometer_Low_AxisX(LSM9DS1_ACC_GYRO_XLIE_XL_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_XLIE_XL_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_InterruptAccelerometer_High_AxisX
* Description    : Write XHIE_XL
* Input          : LSM9DS1_ACC_GYRO_XHIE_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_InterruptAccelerometer_High_AxisX(LSM9DS1_ACC_GYRO_XHIE_XL_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_XHIE_XL_MASK; 
    value |= newValue;
      
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptAccelerometer_High_AxisX
* Description    : Read XHIE_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_XHIE_XL_t
* Output         : Status of XHIE_XL see LSM9DS1_ACC_GYRO_XHIE_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptAccelerometer_High_AxisX(LSM9DS1_ACC_GYRO_XHIE_XL_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_XHIE_XL_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_InterruptAccelerometer_Low_AxisY
* Description    : Write YLIE_XL
* Input          : LSM9DS1_ACC_GYRO_YLIE_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_InterruptAccelerometer_Low_AxisY(LSM9DS1_ACC_GYRO_YLIE_XL_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_YLIE_XL_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptAccelerometer_Low_AxisY
* Description    : Read YLIE_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_YLIE_XL_t
* Output         : Status of YLIE_XL see LSM9DS1_ACC_GYRO_YLIE_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptAccelerometer_Low_AxisY(LSM9DS1_ACC_GYRO_YLIE_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_YLIE_XL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_InterruptAccelerometer_High_AxisY
* Description    : Write YHIE_XL
* Input          : LSM9DS1_ACC_GYRO_YHIE_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_InterruptAccelerometer_High_AxisY(LSM9DS1_ACC_GYRO_YHIE_XL_t newValue)
{
  u8_t value;
  
  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_YHIE_XL_MASK; 
  value |= newValue;
      
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptAccelerometer_High_AxisY
* Description    : Read YHIE_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_YHIE_XL_t
* Output         : Status of YHIE_XL see LSM9DS1_ACC_GYRO_YHIE_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptAccelerometer_High_AxisY(LSM9DS1_ACC_GYRO_YHIE_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_YHIE_XL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_InterruptAccelerometer_Low_AxisZ
* Description    : Write ZLIE_XL
* Input          : LSM9DS1_ACC_GYRO_ZLIE_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_InterruptAccelerometer_Low_AxisZ(LSM9DS1_ACC_GYRO_ZLIE_XL_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_ZLIE_XL_MASK; 
  value |= newValue;
    
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptAccelerometer_Low_AxisZ
* Description    : Read ZLIE_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_ZLIE_XL_t
* Output         : Status of ZLIE_XL see LSM9DS1_ACC_GYRO_ZLIE_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptAccelerometer_Low_AxisZ(LSM9DS1_ACC_GYRO_ZLIE_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_ZLIE_XL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_InterruptAccelerometer_High_AxisZ
* Description    : Write ZHIE_XL
* Input          : LSM9DS1_ACC_GYRO_ZHIE_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_InterruptAccelerometer_High_AxisZ(LSM9DS1_ACC_GYRO_ZHIE_XL_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_ZHIE_XL_MASK; 
  value |= newValue;
    
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptAccelerometer_High_AxisZ
* Description    : Read ZHIE_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_ZHIE_XL_t
* Output         : Status of ZHIE_XL see LSM9DS1_ACC_GYRO_ZHIE_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptAccelerometer_High_AxisZ(LSM9DS1_ACC_GYRO_ZHIE_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_ZHIE_XL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_Interrupt6D
* Description    : Write 6D
* Input          : LSM9DS1_ACC_GYRO_6D_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_Interrupt6D(LSM9DS1_ACC_GYRO_6D_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_6D_MASK; 
  value |= newValue;
    
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_Interrupt6D
* Description    : Read 6D
* Input          : Pointer to LSM9DS1_ACC_GYRO_6D_t
* Output         : Status of 6D see LSM9DS1_ACC_GYRO_6D_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_Interrupt6D(LSM9DS1_ACC_GYRO_6D_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_6D_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_AccelerometerInterruptCombination
* Description    : Write AOI_XL
* Input          : LSM9DS1_ACC_GYRO_AOI_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AccelerometerInterruptCombination(LSM9DS1_ACC_GYRO_AOI_XL_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_AOI_XL_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerInterruptCombination
* Description    : Read AOI_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_AOI_XL_t
* Output         : Status of AOI_XL see LSM9DS1_ACC_GYRO_AOI_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerInterruptCombination(LSM9DS1_ACC_GYRO_AOI_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_AOI_XL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_XL_InterruptThresholdAxisX
* Description    : Write THS_XL_X
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_XL_InterruptThresholdAxisX(u8_t newValue)
{
  u8_t value;

    newValue = newValue << LSM9DS1_ACC_GYRO_THS_XL_X_POSITION; /*mask	 */
    newValue &= LSM9DS1_ACC_GYRO_THS_XL_X_MASK; /*coerce */
    
    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_X_XL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_THS_XL_X_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_X_XL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_XL_InterruptThresholdAxisX
* Description    : Read THS_XL_X
* Input          : Pointer to u8_t
* Output         : Status of THS_XL_X 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_XL_InterruptThresholdAxisX(u8_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_X_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_THS_XL_X_MASK; /*coerce	 */
    *value = *value >> LSM9DS1_ACC_GYRO_THS_XL_X_POSITION; /*mask	 */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_XL_InterruptThresholdAxisY
* Description    : Write THS_XL_Y
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_XL_InterruptThresholdAxisY(u8_t newValue)
{
  u8_t value;

    newValue = newValue << LSM9DS1_ACC_GYRO_THS_XL_Y_POSITION; /*mask	 */
    newValue &= LSM9DS1_ACC_GYRO_THS_XL_Y_MASK; /*coerce */
    
    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_Y_XL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_THS_XL_Y_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_Y_XL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_XL_InterruptThresholdAxisY
* Description    : Read THS_XL_Y
* Input          : Pointer to u8_t
* Output         : Status of THS_XL_Y 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_XL_InterruptThresholdAxisY(u8_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_Y_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_THS_XL_Y_MASK; /*coerce	 */
    *value = *value >> LSM9DS1_ACC_GYRO_THS_XL_Y_POSITION; /*mask	 */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_XL_InterruptThresholdAxisZ
* Description    : Write THS_XL_Z
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_XL_InterruptThresholdAxisZ(u8_t newValue)
{
  u8_t value;

    newValue = newValue << LSM9DS1_ACC_GYRO_THS_XL_Z_POSITION; /*mask	 */
    newValue &= LSM9DS1_ACC_GYRO_THS_XL_Z_MASK; /*coerce */
    
    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_Z_XL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_THS_XL_Z_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_Z_XL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_XL_InterruptThresholdAxisZ
* Description    : Read THS_XL_Z
* Input          : Pointer to u8_t
* Output         : Status of THS_XL_Z 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_XL_InterruptThresholdAxisZ(u8_t *value)
{
     if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_Z_XL, (u8_t *)value) )
        return MEMS_ERROR;

      *value &= LSM9DS1_ACC_GYRO_THS_XL_Z_MASK; /*coerce	 */
      *value = *value >> LSM9DS1_ACC_GYRO_THS_XL_Z_POSITION; /*mask	 */

      return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_XL_InterruptDuration
* Description    : Write DUR_XL
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_XL_InterruptDuration(u8_t newValue)
{
  u8_t value;

    newValue = newValue << LSM9DS1_ACC_GYRO_DUR_XL_POSITION; /*mask	 */
    newValue &= LSM9DS1_ACC_GYRO_DUR_XL_MASK; /*coerce */
    
    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_DUR_XL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_DUR_XL_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_DUR_XL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_XL_InterruptDuration
* Description    : Read DUR_XL
* Input          : Pointer to u8_t
* Output         : Status of DUR_XL 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_XL_InterruptDuration(u8_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_DUR_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_DUR_XL_MASK; /*coerce	 */
    *value = *value >> LSM9DS1_ACC_GYRO_DUR_XL_POSITION; /*mask	 */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_XL_WaitFunction
* Description    : Write WAIT_XL
* Input          : LSM9DS1_ACC_GYRO_WAIT_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_XL_WaitFunction(LSM9DS1_ACC_GYRO_WAIT_XL_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_DUR_XL, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_WAIT_XL_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_DUR_XL, value) )
    return MEMS_ERROR;
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_XL_WaitFunction
* Description    : Read WAIT_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_WAIT_XL_t
* Output         : Status of WAIT_XL see LSM9DS1_ACC_GYRO_WAIT_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_XL_WaitFunction(LSM9DS1_ACC_GYRO_WAIT_XL_t *value)
{
     if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_DUR_XL, (u8_t *)value) )
        return MEMS_ERROR;

      *value &= LSM9DS1_ACC_GYRO_WAIT_XL_MASK; /*mask */

      return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroHighPassFilterReference
* Description    : Write REF_G
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroHighPassFilterReference(u8_t newValue)
{
  u8_t value;

    newValue = newValue << LSM9DS1_ACC_GYRO_REF_G_POSITION; /*mask	 */
    newValue &= LSM9DS1_ACC_GYRO_REF_G_MASK; /*coerce */
    
    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_REFERENCE_G, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_REF_G_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_REFERENCE_G, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;

}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroHighPassFilterReference
* Description    : Read REF_G
* Input          : Pointer to u8_t
* Output         : Status of REF_G 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroHighPassFilterReference(u8_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_REFERENCE_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_REF_G_MASK; /*coerce	 */
    *value = *value >> LSM9DS1_ACC_GYRO_REF_G_POSITION; /*mask	 */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_XL_DataReadyOnINT
* Description    : Write INT_DRDY_XL
* Input          : LSM9DS1_ACC_GYRO_INT_DRDY_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_XL_DataReadyOnINT(LSM9DS1_ACC_GYRO_INT_DRDY_XL_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_INT_DRDY_XL_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_CTRL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_XL_DataReadyOnINT
* Description    : Read INT_DRDY_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_INT_DRDY_XL_t
* Output         : Status of INT_DRDY_XL see LSM9DS1_ACC_GYRO_INT_DRDY_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_XL_DataReadyOnINT(LSM9DS1_ACC_GYRO_INT_DRDY_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_INT_DRDY_XL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GYRO_DataReadyOnINT
* Description    : Write INT_DRDY_G
* Input          : LSM9DS1_ACC_GYRO_INT_DRDY_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GYRO_DataReadyOnINT(LSM9DS1_ACC_GYRO_INT_DRDY_G_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_INT_DRDY_G_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_CTRL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GYRO_DataReadyOnINT
* Description    : Read INT_DRDY_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_INT_DRDY_G_t
* Output         : Status of INT_DRDY_G see LSM9DS1_ACC_GYRO_INT_DRDY_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GYRO_DataReadyOnINT(LSM9DS1_ACC_GYRO_INT_DRDY_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_INT_DRDY_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_BOOT_DataReadyOnINT
* Description    : Write INT__BOOT
* Input          : LSM9DS1_ACC_GYRO_INT__BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_BOOT_OnINT(LSM9DS1_ACC_GYRO_INT__BOOT_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_INT__BOOT_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_CTRL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_BOOT_DataReadyOnINT
* Description    : Read INT__BOOT
* Input          : Pointer to LSM9DS1_ACC_GYRO_INT__BOOT_t
* Output         : Status of INT__BOOT see LSM9DS1_ACC_GYRO_INT__BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_BOOT_OnINT(LSM9DS1_ACC_GYRO_INT__BOOT_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_INT__BOOT_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_FIFO_Threshold_OnINT
* Description    : Write INT_FTH
* Input          : LSM9DS1_ACC_GYRO_INT_FTH_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_FIFO_Threshold_OnINT(LSM9DS1_ACC_GYRO_INT_FTH_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_INT_FTH_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_CTRL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_FIFO_Threshold_OnINT
* Description    : Read INT_FTH
* Input          : Pointer to LSM9DS1_ACC_GYRO_INT_FTH_t
* Output         : Status of INT_FTH see LSM9DS1_ACC_GYRO_INT_FTH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_FIFO_Threshold_OnINT(LSM9DS1_ACC_GYRO_INT_FTH_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_INT_FTH_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_Overrun_OnINT
* Description    : Write INT_OVR
* Input          : LSM9DS1_ACC_GYRO_INT_OVR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_Overrun_OnINT(LSM9DS1_ACC_GYRO_INT_OVR_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_INT_OVR_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_CTRL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_Overrun_OnINT
* Description    : Read INT_OVR
* Input          : Pointer to LSM9DS1_ACC_GYRO_INT_OVR_t
* Output         : Status of INT_OVR see LSM9DS1_ACC_GYRO_INT_OVR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_Overrun_OnINT(LSM9DS1_ACC_GYRO_INT_OVR_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_INT_OVR_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_FIFO_Full_OnINT
* Description    : Write INT_FSS5
* Input          : LSM9DS1_ACC_GYRO_INT_FSS5_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_FIFO_Full_OnINT(LSM9DS1_ACC_GYRO_INT_FSS5_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_INT_FSS5_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_CTRL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_FIFO_Full_OnINT
* Description    : Read INT_FSS5
* Input          : Pointer to LSM9DS1_ACC_GYRO_INT_FSS5_t
* Output         : Status of INT_FSS5 see LSM9DS1_ACC_GYRO_INT_FSS5_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_FIFO_Full_OnINT(LSM9DS1_ACC_GYRO_INT_FSS5_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_INT_FSS5_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_Accelerometer_OnINT
* Description    : Write INT_IG_XL
* Input          : LSM9DS1_ACC_GYRO_INT_IG_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_Accelerometer_OnINT(LSM9DS1_ACC_GYRO_INT_IG_XL_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_INT_IG_XL_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_CTRL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_Accelerometer_OnINT
* Description    : Read INT_IG_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_INT_IG_XL_t
* Output         : Status of INT_IG_XL see LSM9DS1_ACC_GYRO_INT_IG_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_Accelerometer_OnINT(LSM9DS1_ACC_GYRO_INT_IG_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_INT_IG_XL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_Gyroscope_OnINT
* Description    : Write INT_IG_G
* Input          : LSM9DS1_ACC_GYRO_INT_IG_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_Gyroscope_OnINT(LSM9DS1_ACC_GYRO_INT_IG_G_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_INT_IG_G_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_CTRL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_Gyroscope_OnINT
* Description    : Read INT_IG_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_INT_IG_G_t
* Output         : Status of INT_IG_G see LSM9DS1_ACC_GYRO_INT_IG_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_Gyroscope_OnINT(LSM9DS1_ACC_GYRO_INT_IG_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_CTRL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_INT_IG_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_WHO_AM_I_
* Description    : Read WHO_AM_I_BIT
* Input          : Pointer to u8_t
* Output         : Status of WHO_AM_I_BIT 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_WHO_AM_I_(u8_t *value)
{
    
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_WHO_AM_I_REG, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_WHO_AM_I_BIT_MASK; /*coerce	 */
    *value = *value >> LSM9DS1_ACC_GYRO_WHO_AM_I_BIT_POSITION; /*mask	 */

    return MEMS_SUCCESS;

}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroBandwidthSelection
* Description    : Write BW_G
* Input          : LSM9DS1_ACC_GYRO_BW_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroBandwidthSelection(LSM9DS1_ACC_GYRO_BW_G_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG1_G, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_BW_G_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG1_G, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroLowPower
* Description    : Write LP_G
* Input          : LSM9DS1_ACC_GYRO_LP_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroLowPower(LSM9DS1_ACC_GYRO_LP_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG3_G, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_LP_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG3_G, value) )
    return MEMS_ERROR;
   
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroLowPower
* Description    : Read LP_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_LP_G_t
* Output         : Status of LP_G 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroLowPower(LSM9DS1_ACC_GYRO_LP_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG3_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_LP_G_MASK; /*mask */

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroBandwidthSelection
* Description    : Read BW_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_BW_G_t
* Output         : Status of BW_G see LSM9DS1_ACC_GYRO_BW_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroBandwidthSelection(LSM9DS1_ACC_GYRO_BW_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG1_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_BW_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroFullScale
* Description    : Write FS_G
* Input          : LSM9DS1_ACC_GYRO_FS_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroFullScale(LSM9DS1_ACC_GYRO_FS_G_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG1_G, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_FS_G_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG1_G, value) )
      return MEMS_ERROR;
    
    if (newValue==LSM9DS1_ACC_GYRO_FS_G_2000dps)
      k_fs=1;
    else if (newValue==LSM9DS1_ACC_GYRO_FS_G_1000dps)
      k_fs=2;
    else if  (newValue==LSM9DS1_ACC_GYRO_FS_G_500dps)
      k_fs=4;
    else  
      k_fs=8;
    
    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroFullScale
* Description    : Read FS_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_FS_G_t
* Output         : Status of FS_G see LSM9DS1_ACC_GYRO_FS_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroFullScale(LSM9DS1_ACC_GYRO_FS_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG1_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_FS_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroDataRate
* Description    : Write ODR_G
* Input          : LSM9DS1_ACC_GYRO_ODR_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroDataRate(LSM9DS1_ACC_GYRO_ODR_G_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG1_G, &value) )
    {
        return MEMS_ERROR;
    }
    value &= ~LSM9DS1_ACC_GYRO_ODR_G_MASK;
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG1_G, value) )
    {
      return MEMS_ERROR;
    }

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroDataRate
* Description    : Read ODR_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_ODR_G_t
* Output         : Status of ODR_G see LSM9DS1_ACC_GYRO_ODR_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroDataRate(LSM9DS1_ACC_GYRO_ODR_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG1_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_ODR_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GYRO_OutMode
* Description    : Write OUT_SEL
* Input          : LSM9DS1_ACC_GYRO_OUT_SEL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GYRO_OutMode(LSM9DS1_ACC_GYRO_OUT_SEL_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG2_G, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_OUT_SEL_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG2_G, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GYRO_OutMode
* Description    : Read OUT_SEL
* Input          : Pointer to LSM9DS1_ACC_GYRO_OUT_SEL_t
* Output         : Status of OUT_SEL see LSM9DS1_ACC_GYRO_OUT_SEL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GYRO_OutMode(LSM9DS1_ACC_GYRO_OUT_SEL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG2_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_OUT_SEL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GYRO_OutIntMode
* Description    : Write INT_SEL_G
* Input          : LSM9DS1_ACC_GYRO_INT_SEL_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GYRO_OutIntMode(LSM9DS1_ACC_GYRO_INT_SEL_G_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG2_G, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_INT_SEL_G_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG2_G, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GYRO_OutIntMode
* Description    : Read INT_SEL_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_INT_SEL_G_t
* Output         : Status of INT_SEL_G see LSM9DS1_ACC_GYRO_INT_SEL_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GYRO_OutIntMode(LSM9DS1_ACC_GYRO_INT_SEL_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG2_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_INT_SEL_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroHighPassFilterCutOffFrequency
* Description    : Write HPCF_G
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroHighPassFilterCutOffFrequency(u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM9DS1_ACC_GYRO_HPCF_G_POSITION; /*mask	 */
  newValue &= LSM9DS1_ACC_GYRO_HPCF_G_MASK; /*coerce */
  
  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG3_G, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_HPCF_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG3_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroHighPassFilterCutOffFrequency
* Description    : Read HPCF_G
* Input          : Pointer to u8_t
* Output         : Status of HPCF_G 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroHighPassFilterCutOffFrequency(u8_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG3_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_HPCF_G_MASK; /*coerce	 */
    *value = *value >> LSM9DS1_ACC_GYRO_HPCF_G_POSITION; /*mask	 */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroHighPassFilter
* Description    : Write HP_EN_G
* Input          : LSM9DS1_ACC_GYRO_HP_EN_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroHighPassFilter(LSM9DS1_ACC_GYRO_HP_EN_G_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG3_G, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_HP_EN_G_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG3_G, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroHighPassFilter
* Description    : Read HP_EN_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_HP_EN_G_t
* Output         : Status of HP_EN_G see LSM9DS1_ACC_GYRO_HP_EN_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroHighPassFilter(LSM9DS1_ACC_GYRO_HP_EN_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG3_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_HP_EN_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GYRO_DirectionalUserOrientationX
* Description    : Write ORIENT_0
* Input          : LSM9DS1_ACC_GYRO_ORIENT_0_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GYRO_DirectionalUserOrientationX(LSM9DS1_ACC_GYRO_ORIENT_0_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_ORIENT_0_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GYRO_DirectionalUserOrientationX
* Description    : Read ORIENT_0
* Input          : Pointer to LSM9DS1_ACC_GYRO_ORIENT_0_t
* Output         : Status of ORIENT_0 see LSM9DS1_ACC_GYRO_ORIENT_0_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GYRO_DirectionalUserOrientationX(LSM9DS1_ACC_GYRO_ORIENT_0_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_ORIENT_0_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GYRO_DirectionalUserOrientationY
* Description    : Write ORIENT_1
* Input          : LSM9DS1_ACC_GYRO_ORIENT_1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GYRO_DirectionalUserOrientationY(LSM9DS1_ACC_GYRO_ORIENT_1_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_ORIENT_1_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GYRO_DirectionalUserOrientationY
* Description    : Read ORIENT_1
* Input          : Pointer to LSM9DS1_ACC_GYRO_ORIENT_1_t
* Output         : Status of ORIENT_1 see LSM9DS1_ACC_GYRO_ORIENT_1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GYRO_DirectionalUserOrientationY(LSM9DS1_ACC_GYRO_ORIENT_1_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_ORIENT_1_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GYRO_DirectionalUserOrientationZ
* Description    : Write ORIENT_2
* Input          : LSM9DS1_ACC_GYRO_ORIENT_2_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GYRO_DirectionalUserOrientationZ(LSM9DS1_ACC_GYRO_ORIENT_2_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_ORIENT_2_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GYRO_DirectionalUserOrientationZ
* Description    : Read ORIENT_2
* Input          : Pointer to LSM9DS1_ACC_GYRO_ORIENT_2_t
* Output         : Status of ORIENT_2 see LSM9DS1_ACC_GYRO_ORIENT_2_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GYRO_DirectionalUserOrientationZ(LSM9DS1_ACC_GYRO_ORIENT_2_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_ORIENT_2_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GYRO_SignZ
* Description    : Write SIGNZ_G
* Input          : LSM9DS1_ACC_GYRO_SIGNZ_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GYRO_SignZ(LSM9DS1_ACC_GYRO_SIGNZ_G_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_SIGNZ_G_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GYRO_SignZ
* Description    : Read SIGNZ_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_SIGNZ_G_t
* Output         : Status of SIGNZ_G see LSM9DS1_ACC_GYRO_SIGNZ_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GYRO_SignZ(LSM9DS1_ACC_GYRO_SIGNZ_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_SIGNZ_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GYRO_SignY
* Description    : Write SIGNY_G
* Input          : LSM9DS1_ACC_GYRO_SIGNY_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GYRO_SignY(LSM9DS1_ACC_GYRO_SIGNY_G_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_SIGNY_G_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GYRO_SignY
* Description    : Read SIGNY_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_SIGNY_G_t
* Output         : Status of SIGNY_G see LSM9DS1_ACC_GYRO_SIGNY_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GYRO_SignY(LSM9DS1_ACC_GYRO_SIGNY_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_SIGNY_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GYRO_SignX
* Description    : Write SIGNX_G
* Input          : LSM9DS1_ACC_GYRO_SIGNX_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GYRO_SignX(LSM9DS1_ACC_GYRO_SIGNX_G_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_SIGNX_G_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, value) )
      return MEMS_ERROR;
       
    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GYRO_SignX
* Description    : Read SIGNX_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_SIGNX_G_t
* Output         : Status of SIGNX_G see LSM9DS1_ACC_GYRO_SIGNX_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GYRO_SignX(LSM9DS1_ACC_GYRO_SIGNX_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_ORIENT_CFG_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_SIGNX_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptGyroFlag_Low_AxisX
* Description    : Read XL_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_XL_G_t
* Output         : Status of XL_G see LSM9DS1_ACC_GYRO_XL_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptGyroFlag_Low_AxisX(LSM9DS1_ACC_GYRO_XL_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_SRC_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_XL_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptGyroFlag_High_AxisX
* Description    : Read XH_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_XH_G_t
* Output         : Status of XH_G see LSM9DS1_ACC_GYRO_XH_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptGyroFlag_High_AxisX(LSM9DS1_ACC_GYRO_XH_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_SRC_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_XH_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptGyroFlag_Low_AxisY
* Description    : Read YL_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_YL_G_t
* Output         : Status of YL_G see LSM9DS1_ACC_GYRO_YL_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptGyroFlag_Low_AxisY(LSM9DS1_ACC_GYRO_YL_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_SRC_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_YL_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptGyroFlag_High_AxisY
* Description    : Read YH_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_YH_G_t
* Output         : Status of YH_G see LSM9DS1_ACC_GYRO_YH_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptGyroFlag_High_AxisY(LSM9DS1_ACC_GYRO_YH_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_SRC_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_YH_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptGyroFlag_Low_AxisZ
* Description    : Read ZL_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_ZL_G_t
* Output         : Status of ZL_G see LSM9DS1_ACC_GYRO_ZL_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptGyroFlag_Low_AxisZ(LSM9DS1_ACC_GYRO_ZL_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_SRC_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_ZL_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptGyroFlag_High_AxisZ
* Description    : Read ZH_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_ZH_G_t
* Output         : Status of ZH_G see LSM9DS1_ACC_GYRO_ZH_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptGyroFlag_High_AxisZ(LSM9DS1_ACC_GYRO_ZH_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_SRC_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_ZH_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptGyroFlag
* Description    : Read IA_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_IA_G_t
* Output         : Status of IA_G see LSM9DS1_ACC_GYRO_IA_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptGyroFlag(LSM9DS1_ACC_GYRO_IA_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_SRC_G, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_IA_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerDataReadyFlag
* Description    : Read XLDA
* Input          : Pointer to LSM9DS1_ACC_GYRO_XLDA_t
* Output         : Status of XLDA see LSM9DS1_ACC_GYRO_XLDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerDataReadyFlag(LSM9DS1_ACC_GYRO_XLDA_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_STATUS_REG, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_XLDA_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroDataReadyFlag
* Description    : Read GDA
* Input          : Pointer to LSM9DS1_ACC_GYRO_GDA_t
* Output         : Status of GDA see LSM9DS1_ACC_GYRO_GDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroDataReadyFlag(LSM9DS1_ACC_GYRO_GDA_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_STATUS_REG, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_GDA_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_TemperatureDataReadyFlag
* Description    : Read TDA
* Input          : Pointer to LSM9DS1_ACC_GYRO_TDA_t
* Output         : Status of TDA see LSM9DS1_ACC_GYRO_TDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_TemperatureDataReadyFlag(LSM9DS1_ACC_GYRO_TDA_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_STATUS_REG, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_TDA_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_BootRunningFlag
* Description    : Read BOOT_STATUS
* Input          : Pointer to LSM9DS1_ACC_GYRO_BOOT_STATUS_t
* Output         : Status of BOOT_STATUS see LSM9DS1_ACC_GYRO_BOOT_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_BootRunningFlag(LSM9DS1_ACC_GYRO_BOOT_STATUS_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_STATUS_REG, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_BOOT_STATUS_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InactivityInterruptFlag
* Description    : Read INACT
* Input          : Pointer to LSM9DS1_ACC_GYRO_INACT_t
* Output         : Status of INACT see LSM9DS1_ACC_GYRO_INACT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InactivityInterruptFlag(LSM9DS1_ACC_GYRO_INACT_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_STATUS_REG, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_INACT_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroInterruptFlag
* Description    : Read IG_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_IG_G_t
* Output         : Status of IG_G see LSM9DS1_ACC_GYRO_IG_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroInterruptFlag(LSM9DS1_ACC_GYRO_IG_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_STATUS_REG, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_IG_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptAccelerometerFlag
* Description    : Read IG_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_IG_XL_t
* Output         : Status of IG_XL see LSM9DS1_ACC_GYRO_IG_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptAccelerometerFlag(LSM9DS1_ACC_GYRO_IG_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_STATUS_REG, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_IG_XL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_Interrupt4D
* Description    : Write 4D_XL
* Input          : LSM9DS1_ACC_GYRO_4D_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_Interrupt4D(LSM9DS1_ACC_GYRO_4D_XL_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG4, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_4D_XL_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG4, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_Interrupt4D
* Description    : Read 4D_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_4D_XL_t
* Output         : Status of 4D_XL see LSM9DS1_ACC_GYRO_4D_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_Interrupt4D(LSM9DS1_ACC_GYRO_4D_XL_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG4, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_4D_XL_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_InterruptSignalMode
* Description    : Write LIR_XL
* Input          : LSM9DS1_ACC_GYRO_LIR_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_InterruptSignalMode(LSM9DS1_ACC_GYRO_LIR_XL_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG4, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_LIR_XL_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG4, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
 
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptSignalMode
* Description    : Read LIR_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_LIR_XL_t
* Output         : Status of LIR_XL see LSM9DS1_ACC_GYRO_LIR_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptSignalMode(LSM9DS1_ACC_GYRO_LIR_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG4, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_LIR_XL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroAxisX
* Description    : Write XEN_G
* Input          : LSM9DS1_ACC_GYRO_XEN_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroAxisX(LSM9DS1_ACC_GYRO_XEN_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG4, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_XEN_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG4, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroAxisX
* Description    : Read XEN_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_XEN_G_t
* Output         : Status of XEN_G see LSM9DS1_ACC_GYRO_XEN_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroAxisX(LSM9DS1_ACC_GYRO_XEN_G_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG4, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_XEN_G_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroAxisY
* Description    : Write YEN_G
* Input          : LSM9DS1_ACC_GYRO_YEN_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroAxisY(LSM9DS1_ACC_GYRO_YEN_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG4, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_YEN_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG4, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroAxisY
* Description    : Read YEN_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_YEN_G_t
* Output         : Status of YEN_G see LSM9DS1_ACC_GYRO_YEN_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroAxisY(LSM9DS1_ACC_GYRO_YEN_G_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG4, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_YEN_G_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroAxisZ
* Description    : Write ZEN_G
* Input          : LSM9DS1_ACC_GYRO_ZEN_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroAxisZ(LSM9DS1_ACC_GYRO_ZEN_G_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG4, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_ZEN_G_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG4, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroAxisZ
* Description    : Read ZEN_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_ZEN_G_t
* Output         : Status of ZEN_G see LSM9DS1_ACC_GYRO_ZEN_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroAxisZ(LSM9DS1_ACC_GYRO_ZEN_G_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG4, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_ZEN_G_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_AccelerometerAxisX
* Description    : Write XEN_XL
* Input          : LSM9DS1_ACC_GYRO_XEN_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AccelerometerAxisX(LSM9DS1_ACC_GYRO_XEN_XL_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG5_XL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_XEN_XL_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG5_XL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerAxisX
* Description    : Read XEN_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_XEN_XL_t
* Output         : Status of XEN_XL see LSM9DS1_ACC_GYRO_XEN_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerAxisX(LSM9DS1_ACC_GYRO_XEN_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG5_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_XEN_XL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_AccelerometerAxisY
* Description    : Write YEN_XL
* Input          : LSM9DS1_ACC_GYRO_YEN_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AccelerometerAxisY(LSM9DS1_ACC_GYRO_YEN_XL_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG5_XL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_YEN_XL_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG5_XL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerAxisY
* Description    : Read YEN_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_YEN_XL_t
* Output         : Status of YEN_XL see LSM9DS1_ACC_GYRO_YEN_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerAxisY(LSM9DS1_ACC_GYRO_YEN_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG5_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_YEN_XL_MASK; /*mask */

    return MEMS_SUCCESS;
  
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_AccelerometerAxisZ
* Description    : Write ZEN_XL
* Input          : LSM9DS1_ACC_GYRO_ZEN_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AccelerometerAxisZ(LSM9DS1_ACC_GYRO_ZEN_XL_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG5_XL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_ZEN_XL_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG5_XL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerAxisZ
* Description    : Read ZEN_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_ZEN_XL_t
* Output         : Status of ZEN_XL see LSM9DS1_ACC_GYRO_ZEN_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerAxisZ(LSM9DS1_ACC_GYRO_ZEN_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG5_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_ZEN_XL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_AccelerometerDataDecimation
* Description    : Write DEC_XL
* Input          : LSM9DS1_ACC_GYRO_DEC_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AccelerometerDataDecimation(LSM9DS1_ACC_GYRO_DEC_XL_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG5_XL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_DEC_XL_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG5_XL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerDataDecimation
* Description    : Read DEC_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_DEC_XL_t
* Output         : Status of DEC_XL see LSM9DS1_ACC_GYRO_DEC_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerDataDecimation(LSM9DS1_ACC_GYRO_DEC_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG5_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_DEC_XL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_AccelerometerFilterBandwidth
* Description    : Write BW_XL
* Input          : LSM9DS1_ACC_GYRO_BW_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AccelerometerFilterBandwidth(LSM9DS1_ACC_GYRO_BW_XL_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG6_XL, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_BW_XL_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG6_XL, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerFilterBandwidth
* Description    : Read BW_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_BW_XL_t
* Output         : Status of BW_XL see LSM9DS1_ACC_GYRO_BW_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerFilterBandwidth(LSM9DS1_ACC_GYRO_BW_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG6_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_BW_XL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_AccelerometerBandWitdthSelection
* Description    : Write BW_SCAL_ODR
* Input          : LSM9DS1_ACC_GYRO_BW_SCAL_ODR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AccelerometerBandWitdthSelection(LSM9DS1_ACC_GYRO_BW_SCAL_ODR_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG6_XL, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_BW_SCAL_ODR_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG6_XL, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerBandWitdthSelection
* Description    : Read BW_SCAL_ODR
* Input          : Pointer to LSM9DS1_ACC_GYRO_BW_SCAL_ODR_t
* Output         : Status of BW_SCAL_ODR see LSM9DS1_ACC_GYRO_BW_SCAL_ODR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerBandWitdthSelection(LSM9DS1_ACC_GYRO_BW_SCAL_ODR_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG6_XL, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_BW_SCAL_ODR_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_AccelerometerFullScale
* Description    : Write FS_XL
* Input          : LSM9DS1_ACC_GYRO_FS_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AccelerometerFullScale(LSM9DS1_ACC_GYRO_FS_XL_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG6_XL, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_FS_XL_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG6_XL, value) )
    return MEMS_ERROR;
	
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerFullScale
* Description    : Read FS_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_FS_XL_t
* Output         : Status of FS_XL see LSM9DS1_ACC_GYRO_FS_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerFullScale(LSM9DS1_ACC_GYRO_FS_XL_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG6_XL, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_FS_XL_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_AccelerometerDataRate
* Description    : Write ODR_XL
* Input          : LSM9DS1_ACC_GYRO_ODR_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AccelerometerDataRate(LSM9DS1_ACC_GYRO_ODR_XL_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG6_XL, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_ODR_XL_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG6_XL, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerDataRate
* Description    : Read ODR_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_ODR_XL_t
* Output         : Status of ODR_XL see LSM9DS1_ACC_GYRO_ODR_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerDataRate(LSM9DS1_ACC_GYRO_ODR_XL_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG6_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_ODR_XL_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_AccelerometerHighPass_on_Interrupt
* Description    : Write HPIS
* Input          : LSM9DS1_ACC_GYRO_HPIS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AccelerometerHighPass_on_Interrupt(LSM9DS1_ACC_GYRO_HPIS_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG7_XL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_HPIS_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG7_XL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerHighPass_on_Interrupt
* Description    : Read HPIS
* Input          : Pointer to LSM9DS1_ACC_GYRO_HPIS_t
* Output         : Status of HPIS see LSM9DS1_ACC_GYRO_HPIS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerHighPass_on_Interrupt(LSM9DS1_ACC_GYRO_HPIS_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG7_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_HPIS_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_AccelerometerFilteredDataSelection
* Description    : Write FDS
* Input          : LSM9DS1_ACC_GYRO_FDS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AccelerometerFilteredDataSelection(LSM9DS1_ACC_GYRO_FDS_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG7_XL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_FDS_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG7_XL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerFilteredDataSelection
* Description    : Read FDS
* Input          : Pointer to LSM9DS1_ACC_GYRO_FDS_t
* Output         : Status of FDS see LSM9DS1_ACC_GYRO_FDS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerFilteredDataSelection(LSM9DS1_ACC_GYRO_FDS_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG7_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_FDS_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_AccelerometerCutOff_filter
* Description    : Write DCF
* Input          : LSM9DS1_ACC_GYRO_DCF_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AccelerometerCutOff_filter(LSM9DS1_ACC_GYRO_DCF_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG7_XL, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_DCF_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG7_XL, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerCutOff_filter
* Description    : Read DCF
* Input          : Pointer to LSM9DS1_ACC_GYRO_DCF_t
* Output         : Status of DCF see LSM9DS1_ACC_GYRO_DCF_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerCutOff_filter(LSM9DS1_ACC_GYRO_DCF_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG7_XL, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_DCF_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_AccelerometerHighResolutionMode
* Description    : Write HR
* Input          : LSM9DS1_ACC_GYRO_HR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AccelerometerHighResolutionMode(LSM9DS1_ACC_GYRO_HR_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG7_XL, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_HR_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG7_XL, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerHighResolutionMode
* Description    : Read HR
* Input          : Pointer to LSM9DS1_ACC_GYRO_HR_t
* Output         : Status of HR see LSM9DS1_ACC_GYRO_HR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerHighResolutionMode(LSM9DS1_ACC_GYRO_HR_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG7_XL, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_HR_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_ResetSW
* Description    : Write SW_RESET
* Input          : LSM9DS1_ACC_GYRO_SW_RESET_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_ResetSW(LSM9DS1_ACC_GYRO_SW_RESET_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_SW_RESET_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG8, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_ResetSW
* Description    : Read SW_RESET
* Input          : Pointer to LSM9DS1_ACC_GYRO_SW_RESET_t
* Output         : Status of SW_RESET see LSM9DS1_ACC_GYRO_SW_RESET_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_ResetSW(LSM9DS1_ACC_GYRO_SW_RESET_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_SW_RESET_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_BigLittleEndianDataSelection
* Description    : Write BLE
* Input          : LSM9DS1_ACC_GYRO_BLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_BigLittleEndianDataSelection(LSM9DS1_ACC_GYRO_BLE_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_BLE_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG8, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_BigLittleEndianDataSelection
* Description    : Read BLE
* Input          : Pointer to LSM9DS1_ACC_GYRO_BLE_t
* Output         : Status of BLE see LSM9DS1_ACC_GYRO_BLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_BigLittleEndianDataSelection(LSM9DS1_ACC_GYRO_BLE_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_BLE_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_AutoIndexOnMultiAccess
* Description    : Write IF_ADD_INC
* Input          : LSM9DS1_ACC_GYRO_IF_ADD_INC_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AutoIndexOnMultiAccess(LSM9DS1_ACC_GYRO_IF_ADD_INC_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_IF_ADD_INC_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG8, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AutoIndexOnMultiAccess
* Description    : Read IF_ADD_INC
* Input          : Pointer to LSM9DS1_ACC_GYRO_IF_ADD_INC_t
* Output         : Status of IF_ADD_INC see LSM9DS1_ACC_GYRO_IF_ADD_INC_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AutoIndexOnMultiAccess(LSM9DS1_ACC_GYRO_IF_ADD_INC_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_IF_ADD_INC_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_SPI_SerialInterfaceMode
* Description    : Write SIM
* Input          : LSM9DS1_ACC_GYRO_SIM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_SPI_SerialInterfaceMode(LSM9DS1_ACC_GYRO_SIM_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_SIM_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG8, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_SPI_SerialInterfaceMode
* Description    : Read SIM
* Input          : Pointer to LSM9DS1_ACC_GYRO_SIM_t
* Output         : Status of SIM see LSM9DS1_ACC_GYRO_SIM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_SPI_SerialInterfaceMode(LSM9DS1_ACC_GYRO_SIM_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_SIM_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_INT_Pin_Mode
* Description    : Write PP_OD
* Input          : LSM9DS1_ACC_GYRO_PP_OD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_INT_Pin_Mode(LSM9DS1_ACC_GYRO_PP_OD_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_PP_OD_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG8, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_INT_Pin_Mode
* Description    : Read PP_OD
* Input          : Pointer to LSM9DS1_ACC_GYRO_PP_OD_t
* Output         : Status of PP_OD see LSM9DS1_ACC_GYRO_PP_OD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_INT_Pin_Mode(LSM9DS1_ACC_GYRO_PP_OD_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_PP_OD_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_InterruptActive
* Description    : Write H_LACTIVE
* Input          : LSM9DS1_ACC_GYRO_H_LACTIVE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_InterruptActive(LSM9DS1_ACC_GYRO_H_LACTIVE_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_H_LACTIVE_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG8, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptActive
* Description    : Read H_LACTIVE
* Input          : Pointer to LSM9DS1_ACC_GYRO_H_LACTIVE_t
* Output         : Status of H_LACTIVE see LSM9DS1_ACC_GYRO_H_LACTIVE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptActive(LSM9DS1_ACC_GYRO_H_LACTIVE_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_H_LACTIVE_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_BlockDataUpdate
* Description    : Write BDU
* Input          : LSM9DS1_ACC_GYRO_BDU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_BlockDataUpdate(LSM9DS1_ACC_GYRO_BDU_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_BDU_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG8, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_BlockDataUpdate
* Description    : Read BDU
* Input          : Pointer to LSM9DS1_ACC_GYRO_BDU_t
* Output         : Status of BDU see LSM9DS1_ACC_GYRO_BDU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_BlockDataUpdate(LSM9DS1_ACC_GYRO_BDU_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_BDU_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_Reboot
* Description    : Write BOOT
* Input          : LSM9DS1_ACC_GYRO_BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_Reboot(LSM9DS1_ACC_GYRO_BOOT_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_BOOT_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG8, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_Reboot
* Description    : Read BOOT
* Input          : Pointer to LSM9DS1_ACC_GYRO_BOOT_t
* Output         : Status of BOOT see LSM9DS1_ACC_GYRO_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_Reboot(LSM9DS1_ACC_GYRO_BOOT_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG8, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_BOOT_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_FIFO_Threshold_Level
* Description    : Write STOP_ON_FTH
* Input          : LSM9DS1_ACC_GYRO_STOP_ON_FTH_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_FIFO_Threshold_Level(LSM9DS1_ACC_GYRO_STOP_ON_FTH_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG9, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_STOP_ON_FTH_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG9, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_FIFO_Threshold_Level
* Description    : Read STOP_ON_FTH
* Input          : Pointer to LSM9DS1_ACC_GYRO_STOP_ON_FTH_t
* Output         : Status of STOP_ON_FTH see LSM9DS1_ACC_GYRO_STOP_ON_FTH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_FIFO_Threshold_Level(LSM9DS1_ACC_GYRO_STOP_ON_FTH_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG9, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_STOP_ON_FTH_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_FIFO
* Description    : Write FIFO_EN
* Input          : LSM9DS1_ACC_GYRO_FIFO_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_FIFO(LSM9DS1_ACC_GYRO_FIFO_EN_t newValue)
{
  u8_t value;

    if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG9, &value) )
      return MEMS_ERROR;

    value &= ~LSM9DS1_ACC_GYRO_FIFO_EN_MASK; 
    value |= newValue;
    
    if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG9, value) )
      return MEMS_ERROR;

    return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_FIFO
* Description    : Read FIFO_EN
* Input          : Pointer to LSM9DS1_ACC_GYRO_FIFO_EN_t
* Output         : Status of FIFO_EN see LSM9DS1_ACC_GYRO_FIFO_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_FIFO(LSM9DS1_ACC_GYRO_FIFO_EN_t *value)
{
   if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG9, (u8_t *)value) )
      return MEMS_ERROR;

    *value &= LSM9DS1_ACC_GYRO_FIFO_EN_MASK; /*mask */

    return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_DigitalInterface
* Description    : Write I2C_DISABLE
* Input          : LSM9DS1_ACC_GYRO_I2C_DISABLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_DigitalInterface(LSM9DS1_ACC_GYRO_I2C_DISABLE_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG9, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_I2C_DISABLE_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG9, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_DigitalInterface
* Description    : Read I2C_DISABLE
* Input          : Pointer to LSM9DS1_ACC_GYRO_I2C_DISABLE_t
* Output         : Status of I2C_DISABLE see LSM9DS1_ACC_GYRO_I2C_DISABLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_DigitalInterface(LSM9DS1_ACC_GYRO_I2C_DISABLE_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG9, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_I2C_DISABLE_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_DataReadyTimer
* Description    : Write DRDY_MASK_BIT
* Input          : LSM9DS1_ACC_GYRO_DRDY_MASK_BIT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_DataReadyTimer(LSM9DS1_ACC_GYRO_DRDY_MASK_BIT_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG9, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_DRDY_MASK_BIT_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG9, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_DataReadyTimer
* Description    : Read DRDY_MASK_BIT
* Input          : Pointer to LSM9DS1_ACC_GYRO_DRDY_MASK_BIT_t
* Output         : Status of DRDY_MASK_BIT see LSM9DS1_ACC_GYRO_DRDY_MASK_BIT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_DataReadyTimer(LSM9DS1_ACC_GYRO_DRDY_MASK_BIT_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG9, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_DRDY_MASK_BIT_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_Temperature_In_FIFO
* Description    : Write FIFO_TEMP_EN
* Input          : LSM9DS1_ACC_GYRO_FIFO_TEMP_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_Temperature_In_FIFO(LSM9DS1_ACC_GYRO_FIFO_TEMP_EN_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG9, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_FIFO_TEMP_EN_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG9, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_Temperature_In_FIFO
* Description    : Read FIFO_TEMP_EN
* Input          : Pointer to LSM9DS1_ACC_GYRO_FIFO_TEMP_EN_t
* Output         : Status of FIFO_TEMP_EN see LSM9DS1_ACC_GYRO_FIFO_TEMP_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_Temperature_In_FIFO(LSM9DS1_ACC_GYRO_FIFO_TEMP_EN_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG9, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_FIFO_TEMP_EN_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroSleepMode
* Description    : Write SLEEP_G
* Input          : LSM9DS1_ACC_GYRO_SLEEP_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroSleepMode(LSM9DS1_ACC_GYRO_SLEEP_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG9, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_SLEEP_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG9, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroSleepMode
* Description    : Read SLEEP_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_SLEEP_G_t
* Output         : Status of SLEEP_G see LSM9DS1_ACC_GYRO_SLEEP_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroSleepMode(LSM9DS1_ACC_GYRO_SLEEP_G_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG9, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_SLEEP_G_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_AccelerometerSelfTest
* Description    : Write ST_XL
* Input          : LSM9DS1_ACC_GYRO_ST_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_AccelerometerSelfTest(LSM9DS1_ACC_GYRO_ST_XL_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG10, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_ST_XL_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG10, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerSelfTest
* Description    : Read ST_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_ST_XL_t
* Output         : Status of ST_XL see LSM9DS1_ACC_GYRO_ST_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerSelfTest(LSM9DS1_ACC_GYRO_ST_XL_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG10, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_ST_XL_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroSelfTest
* Description    : Write ST_G
* Input          : LSM9DS1_ACC_GYRO_ST_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroSelfTest(LSM9DS1_ACC_GYRO_ST_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG10, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_ST_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_CTRL_REG10, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroSelfTest
* Description    : Read ST_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_ST_G_t
* Output         : Status of ST_G see LSM9DS1_ACC_GYRO_ST_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroSelfTest(LSM9DS1_ACC_GYRO_ST_G_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_CTRL_REG10, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_ST_G_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerInterruptFlag_Low_X
* Description    : Read XL_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_XL_XL_t
* Output         : Status of XL_XL see LSM9DS1_ACC_GYRO_XL_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerInterruptFlag_Low_X(LSM9DS1_ACC_GYRO_XL_XL_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_SRC_XL, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_XL_XL_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerInterruptFlag_High_X
* Description    : Read XH_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_XH_XL_t
* Output         : Status of XH_XL see LSM9DS1_ACC_GYRO_XH_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerInterruptFlag_High_X(LSM9DS1_ACC_GYRO_XH_XL_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_SRC_XL, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_XH_XL_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerInterruptFlag_Low_Y
* Description    : Read YL_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_YL_XL_t
* Output         : Status of YL_XL see LSM9DS1_ACC_GYRO_YL_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerInterruptFlag_Low_Y(LSM9DS1_ACC_GYRO_YL_XL_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_SRC_XL, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_YL_XL_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerInterruptFlag_High_Y
* Description    : Read YH_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_YH_XL_t
* Output         : Status of YH_XL see LSM9DS1_ACC_GYRO_YH_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerInterruptFlag_High_Y(LSM9DS1_ACC_GYRO_YH_XL_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_SRC_XL, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_YH_XL_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerInterruptFlag_Low_Z
* Description    : Read ZL_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_ZL_XL_t
* Output         : Status of ZL_XL see LSM9DS1_ACC_GYRO_ZL_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerInterruptFlag_Low_Z(LSM9DS1_ACC_GYRO_ZL_XL_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_SRC_XL, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_ZL_XL_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerInterruptFlag_High_Z
* Description    : Read ZH_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_ZH_XL_t
* Output         : Status of ZH_XL see LSM9DS1_ACC_GYRO_ZH_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerInterruptFlag_High_Z(LSM9DS1_ACC_GYRO_ZH_XL_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_SRC_XL, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_ZH_XL_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_AccelerometerInterruptFlag
* Description    : Read IA_XL
* Input          : Pointer to LSM9DS1_ACC_GYRO_IA_XL_t
* Output         : Status of IA_XL see LSM9DS1_ACC_GYRO_IA_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_AccelerometerInterruptFlag(LSM9DS1_ACC_GYRO_IA_XL_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_SRC_XL, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_IA_XL_MASK; /*mask */

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_FIFO_Threshold
* Description    : Write FTH
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_FIFO_Threshold(u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM9DS1_ACC_GYRO_FTH_POSITION; /*mask	 */
  newValue &= LSM9DS1_ACC_GYRO_FTH_MASK; /*coerce */
  
  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_FIFO_CTRL, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_FTH_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_FIFO_CTRL, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_FIFO_Threshold
* Description    : Read FTH
* Input          : Pointer to u8_t
* Output         : Status of FTH 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_FIFO_Threshold(u8_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_FIFO_CTRL, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_FTH_MASK; /*coerce	 */
  *value = *value >> LSM9DS1_ACC_GYRO_FTH_POSITION; /*mask	 */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_FIFO_Mode
* Description    : Write FMODE
* Input          : LSM9DS1_ACC_GYRO_FMODE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_FIFO_Mode(LSM9DS1_ACC_GYRO_FMODE_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_FIFO_CTRL, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_FMODE_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_FIFO_CTRL, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_FIFO_Mode
* Description    : Read FMODE
* Input          : Pointer to LSM9DS1_ACC_GYRO_FMODE_t
* Output         : Status of FMODE see LSM9DS1_ACC_GYRO_FMODE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_FIFO_Mode(LSM9DS1_ACC_GYRO_FMODE_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_FIFO_CTRL, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_FMODE_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_FIFO_Samples
* Description    : Read FSS
* Input          : Pointer to u8_t
* Output         : Status of FSS 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_FIFO_Samples(u8_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_FIFO_SRC, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_FSS_MASK; /*coerce	 */
  *value = *value >> LSM9DS1_ACC_GYRO_FSS_POSITION; /*mask	 */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_FIFO_OverrunFlag
* Description    : Read OVRN
* Input          : Pointer to LSM9DS1_ACC_GYRO_OVRN_t
* Output         : Status of OVRN see LSM9DS1_ACC_GYRO_OVRN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_FIFO_OverrunFlag(LSM9DS1_ACC_GYRO_OVRN_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_FIFO_SRC, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_OVRN_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_FIFO_ThresholdFlag
* Description    : Read FTH
* Input          : Pointer to LSM9DS1_ACC_GYRO_FTH_t
* Output         : Status of FTH see LSM9DS1_ACC_GYRO_FTH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_FIFO_ThresholdFlag(LSM9DS1_ACC_GYRO_FTH_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_FIFO_SRC, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_FTH_FLAG_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroInterrupt_Low_AxisX
* Description    : Write XLIE_G
* Input          : LSM9DS1_ACC_GYRO_XLIE_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroInterrupt_Low_AxisX(LSM9DS1_ACC_GYRO_XLIE_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_XLIE_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroInterrupt_Low_AxisX
* Description    : Read XLIE_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_XLIE_G_t
* Output         : Status of XLIE_G see LSM9DS1_ACC_GYRO_XLIE_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroInterrupt_Low_AxisX(LSM9DS1_ACC_GYRO_XLIE_G_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_XLIE_G_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroInterrupt_High_AxisX
* Description    : Write XHIE_G
* Input          : LSM9DS1_ACC_GYRO_XHIE_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroInterrupt_High_AxisX(LSM9DS1_ACC_GYRO_XHIE_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_XHIE_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroInterrupt_High_AxisX
* Description    : Read XHIE_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_XHIE_G_t
* Output         : Status of XHIE_G see LSM9DS1_ACC_GYRO_XHIE_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroInterrupt_High_AxisX(LSM9DS1_ACC_GYRO_XHIE_G_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_XHIE_G_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroInterrupt_Low_AxisY
* Description    : Write YLIE_G
* Input          : LSM9DS1_ACC_GYRO_YLIE_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroInterrupt_Low_AxisY(LSM9DS1_ACC_GYRO_YLIE_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_YLIE_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroInterrupt_Low_AxisY
* Description    : Read YLIE_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_YLIE_G_t
* Output         : Status of YLIE_G see LSM9DS1_ACC_GYRO_YLIE_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroInterrupt_Low_AxisY(LSM9DS1_ACC_GYRO_YLIE_G_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_YLIE_G_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroInterrupt_High_AxisY
* Description    : Write YHIE_G
* Input          : LSM9DS1_ACC_GYRO_YHIE_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroInterrupt_High_AxisY(LSM9DS1_ACC_GYRO_YHIE_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_YHIE_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroInterrupt_High_AxisY
* Description    : Read YHIE_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_YHIE_G_t
* Output         : Status of YHIE_G see LSM9DS1_ACC_GYRO_YHIE_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroInterrupt_High_AxisY(LSM9DS1_ACC_GYRO_YHIE_G_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_YHIE_G_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroInterrupt_Low_AxisZ
* Description    : Write ZLIE_G
* Input          : LSM9DS1_ACC_GYRO_ZLIE_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroInterrupt_Low_AxisZ(LSM9DS1_ACC_GYRO_ZLIE_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_ZLIE_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroInterrupt_Low_AxisZ
* Description    : Read ZLIE_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_ZLIE_G_t
* Output         : Status of ZLIE_G see LSM9DS1_ACC_GYRO_ZLIE_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroInterrupt_Low_AxisZ(LSM9DS1_ACC_GYRO_ZLIE_G_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_ZLIE_G_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroInterrupt_High_AxisZ
* Description    : Write ZHIE_G
* Input          : LSM9DS1_ACC_GYRO_ZHIE_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroInterrupt_High_AxisZ(LSM9DS1_ACC_GYRO_ZHIE_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_ZHIE_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroInterrupt_High_AxisZ
* Description    : Read ZHIE_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_ZHIE_G_t
* Output         : Status of ZHIE_G see LSM9DS1_ACC_GYRO_ZHIE_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroInterrupt_High_AxisZ(LSM9DS1_ACC_GYRO_ZHIE_G_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_ZHIE_G_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroInterruptSignalType
* Description    : Write LIR_G
* Input          : LSM9DS1_ACC_GYRO_LIR_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroInterruptSignalType(LSM9DS1_ACC_GYRO_LIR_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_LIR_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroInterruptSignalType
* Description    : Read LIR_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_LIR_G_t
* Output         : Status of LIR_G see LSM9DS1_ACC_GYRO_LIR_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroInterruptSignalType(LSM9DS1_ACC_GYRO_LIR_G_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_LIR_G_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroInterruptCombinationEvent
* Description    : Write AOI_G
* Input          : LSM9DS1_ACC_GYRO_AOI_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroInterruptCombinationEvent(LSM9DS1_ACC_GYRO_AOI_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_AOI_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroInterruptCombinationEvent
* Description    : Read AOI_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_AOI_G_t
* Output         : Status of AOI_G see LSM9DS1_ACC_GYRO_AOI_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroInterruptCombinationEvent(LSM9DS1_ACC_GYRO_AOI_G_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_CFG_G, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_AOI_G_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_GyroCounterModeSelection
* Description    : Write DCRM_G
* Input          : LSM9DS1_ACC_GYRO_DCRM_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_GyroCounterModeSelection(LSM9DS1_ACC_GYRO_DCRM_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_XH_G, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_DCRM_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_XH_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_GyroCounterModeSelection
* Description    : Read DCRM_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_DCRM_G_t
* Output         : Status of DCRM_G see LSM9DS1_ACC_GYRO_DCRM_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_GyroCounterModeSelection(LSM9DS1_ACC_GYRO_DCRM_G_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_XH_G, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_DCRM_G_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_InterruptDurationValue
* Description    : Write DUR_G
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_InterruptDurationValue(u8_t newValue)
{
  u8_t value;

  newValue = newValue << LSM9DS1_ACC_GYRO_DUR_G_POSITION; /*mask	 */
  newValue &= LSM9DS1_ACC_GYRO_DUR_G_MASK; /*coerce */
  
  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_DUR_G, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_DUR_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_DUR_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_InterruptDurationValue
* Description    : Read DUR_G
* Input          : Pointer to u8_t
* Output         : Status of DUR_G 
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_InterruptDurationValue(u8_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_DUR_G, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_DUR_G_MASK; /*coerce	 */
  *value = *value >> LSM9DS1_ACC_GYRO_DUR_G_POSITION; /*mask	 */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_W_WaitFunction
* Description    : Write WAIT_G
* Input          : LSM9DS1_ACC_GYRO_WAIT_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t  LSM9DS1_ACC_GYRO_W_WaitFunction(LSM9DS1_ACC_GYRO_WAIT_G_t newValue)
{
  u8_t value;

  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_DUR_G, &value) )
    return MEMS_ERROR;

  value &= ~LSM9DS1_ACC_GYRO_WAIT_G_MASK; 
  value |= newValue;
  
  if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_DUR_G, value) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LSM9DS1_ACC_GYRO_R_WaitFunction
* Description    : Read WAIT_G
* Input          : Pointer to LSM9DS1_ACC_GYRO_WAIT_G_t
* Output         : Status of WAIT_G see LSM9DS1_ACC_GYRO_WAIT_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/

status_t LSM9DS1_ACC_GYRO_R_WaitFunction(LSM9DS1_ACC_GYRO_WAIT_G_t *value)
{
 if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_DUR_G, (u8_t *)value) )
    return MEMS_ERROR;

  *value &= LSM9DS1_ACC_GYRO_WAIT_G_MASK; /*mask */

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : status_t LSM9DS1_ACC_GYRO_Get_Temperature(u8_t *buff)
* Description    : Read Temperature output register
* Input          : pointer to [u8_t]
* Output         : Temperature buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM9DS1_ACC_GYRO_Get_Temperature(u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=2/1;

  k=0;
  for (i=0; i<1;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_OUT_TEMP_L+k, &buff[k]))
		  return MEMS_ERROR;
		k++;	
	}
  }

  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : status_t LSM9DS1_ACC_GYRO_Get_AngularRate(u8_t *buff)
* Description    : Read AngularRate output register
* Input          : pointer to [u8_t]
* Output         : AngularRate buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM9DS1_ACC_GYRO_Get_AngularRate(u8_t *buff)
{
  u16_t i, j, k;
  u16_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_OUT_X_L_G+k, &buff[k]))
		  return MEMS_ERROR;
		k++;	
	}
  }

  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : status_t LSM9DS1_ACC_GYRO_Get_Acceleration(u8_t *buff)
* Description    : Read Acceleration output register
* Input          : pointer to [u8_t]
* Output         : Acceleration buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM9DS1_ACC_GYRO_Get_Acceleration(u8_t *buff)
{
  u16_t i, j, k;
  u16_t numberOfByteForDimension;
  
   numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_OUT_X_L_XL+k, &buff[k]))
		  return MEMS_ERROR;
		k++;	
	}
  }

  return MEMS_SUCCESS; 
}


/*******************************************************************************
* Function Name  : status_t LSM9DS1_ACC_GYRO_Set_AngularRateThreshold(u8_t *buff) 
* Description    : Set AngularRateThreshold data row
* Input          : pointer to [u8_t]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM9DS1_ACC_GYRO_Set_AngularRateThreshold(u8_t *buff) 
{
  u8_t  i, value;
  u8_t numberOfByteForDimension;
  
  numberOfByteForDimension=6/3;
  LSM9DS1_ACC_GYRO_SwapHighLowByte(buff, 6, numberOfByteForDimension);
 
  if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_XH_G, &value) )
  return MEMS_ERROR;

  /*Coerce 15 bit two's complement value*/
  value &= LSM9DS1_ACC_GYRO_DCRM_G_MASK; 
  buff[0] &= ~LSM9DS1_ACC_GYRO_DCRM_G_MASK;
  buff[0] |= value;
  buff[2] &= 0x7F;
  buff[4] &= 0x7F;
  
  for (i=0; i<6;i++ ) 
  {
	if( !LSM9DS1_ACC_GYRO_WriteReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_XH_G+i,  buff[i]) )
		return MEMS_ERROR;
  }
  return MEMS_SUCCESS;  
}

/*******************************************************************************
* Function Name  : status_t LSM9DS1_ACC_GYRO_Get_AngularRateThreshold(u8_t *buff)
* Description    : Read AngularRateThreshold output register
* Input          : pointer to [u8_t]
* Output         : AngularRateThreshold buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LSM9DS1_ACC_GYRO_Get_AngularRateThreshold(u8_t *buff) 
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ ) 
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{	
		if( !LSM9DS1_ACC_GYRO_ReadReg(LSM9DS1_ACC_GYRO_INT_GEN_THS_XH_G+k, &buff[k]))
		  return MEMS_ERROR;
		k++;	
	}
  }
  
  /*Sign Extension on 16 bit*/
  if (buff[0]&0x40)
	buff[0]|=0x80;
  if (buff[2]&0x40)
	buff[2]|=0x80;	
  if (buff[4]&0x40)
	buff[4]|=0x80;	
	
  LSM9DS1_ACC_GYRO_SwapHighLowByte(buff, 6, numberOfByteForDimension);

  return MEMS_SUCCESS; 
}


