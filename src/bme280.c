/*
 * bme280.c
 *
 *  Created on: Feb 24, 2022
 *      Author: Mukta
 */
/*******************************************************************************
 Headers
*******************************************************************************/
#include <src/BME280.h>
// Include logging for this file
#define INCLUDE_LOG_DEBUG       (1)
#include "src/log.h"
#include "ble.h"

/*******************************************************************************
 Macros
*******************************************************************************/
#define ERROR                   (-1)
#define DEBUG_1                 (0)
/*******************************************************************************
 Global
*******************************************************************************/
/*!
 * @brief Calibration data
 */
struct bme280_calib_data
{
  /*< Calibration coefficient for the temperature sensor */
  uint16_t dig_t1;
  int16_t dig_t2;
  int16_t dig_t3;

  /*< Calibration coefficient for the humidity sensor */
  uint8_t dig_h1;
  int16_t dig_h2;
  uint8_t dig_h3;
  int16_t dig_h4;
  int16_t dig_h5;
  int8_t dig_h6;

  /*< Variable to store the intermediate temperature coefficient */
  int32_t t_fine;
};

uint8_t bme280_wr_buff[8] = { 0 };
uint8_t bme280_rd_buff[30] = { 0 };
struct bme280_calib_data calib_data;
uint32_t uncomp_temp = 0, uncomp_hum = 0;
double temperature=0, humidity=0;
/*******************************************************************************
 Function Prototypes
*******************************************************************************/
bool BME280_VerifyIdentity(uint8_t* rd_buff);
void BME280_write(uint8_t reg, uint8_t byte_val);
static double compensate_temperature(const uint32_t temp_data);
static double compensate_humidity(const uint32_t hum_data);
static void parse_temp_press_calib_data(const uint8_t *reg_data);
static void parse_humidity_calib_data(const uint8_t *reg_data);
/*******************************************************************************
 Function Definition
*******************************************************************************/
asset_monitoring_state_t bme280_read_machine(sl_bt_msg_t *evt)
{
  ble_ext_signal_event_t event = evt->data.evt_system_external_signal.extsignals;
  /* return state logic */
  asset_monitoring_state_t return_state = BME280_READ;
  /* current machine logic */
  BME280_state_t currentState;
  static BME280_state_t nextState = BME280_READ_TRH_DATA;
  int ret_status = 0;
  if (SL_BT_MSG_ID(evt->header) != sl_bt_evt_system_external_signal_id) {
      return return_state;
  }
  currentState = nextState;
  switch(currentState)
  {
    case BME280_READ_TRH_DATA:
    {
      //default state
      return_state = BME280_READ;
#if DEBUG_1
      LOG_INFO("BME280_READ_TRH_DATA\r");
#endif
      if(event == ev_LETIMER0_UF || event == ev_LETIMER0_COMP1)
      {
        memset(bme280_rd_buff,0,sizeof(bme280_rd_buff));
        bme280_wr_buff[0] = BME280_DATA_ADDR;
        I2C0_WriteRead(BME280_ADDRESS, &bme280_wr_buff[0], 1, &bme280_rd_buff[0], BME280_T_RH_DATA_LEN);
        nextState = BME280_DISP_TRH_DATA;
        gpioDebugLEDSetOn();
      }
      break;
    }

    case BME280_DISP_TRH_DATA:
    {
      //default state
      return_state = BME280_READ;
#if DEBUG_1
      LOG_INFO("BME280_DISP_TRH_DATA\r");
#endif
      if(event == ev_I2C0_TRANSFER_DONE)
      {
        uncomp_temp = 0; uncomp_hum = 0;
        /* Store the parsed register values for temperature data */
        uncomp_temp = ((uint32_t)bme280_rd_buff[0] << 12) | ((uint32_t)bme280_rd_buff[1] << 4) | ((uint32_t)bme280_rd_buff[2] >> 4);

        /* Store the parsed register values for humidity data */
        uncomp_hum = (uint32_t)(BME280_CONCAT_BYTES(bme280_rd_buff[3], bme280_rd_buff[4]));
        //LOG_INFO("BME280 unT = %d  unRH = %d\r", uncomp_temp, uncomp_hum);

        temperature = compensate_temperature(uncomp_temp);
        humidity = compensate_humidity(uncomp_hum);
        LOG_INFO("BME280 T = %f  RH = %f\r", temperature, humidity);
        //LOG_INFO("BME280 T1 = %d T2 = %d T3 = %d\r", calib_data.dig_t1, calib_data.dig_t2, calib_data.dig_t3);
        //LOG_INFO("BME280 H1 = %d H2 = %d H3 = %d H4 = %d H5 = %d H6 = %d\r",
        //         calib_data.dig_h1, calib_data.dig_h2, calib_data.dig_h3, calib_data.dig_h4, calib_data.dig_h5, calib_data.dig_h6);

        gpioDebugLEDSetOff();

        ret_status = timerWaitUs_irq(STD_DELAY);
        if(ret_status == ERROR)
        {
          LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
          //update return state for next iteration
          nextState = BME280_READ_TRH_DATA;
          //next state to switch in Asset SM
          return_state = BNO055_READ;
        }
      }
      break;
    }
    default:
      break;
  }
  return return_state;
}

asset_monitoring_state_t init_bme280_machine(sl_bt_msg_t *evt)
{
  ble_ext_signal_event_t event = evt->data.evt_system_external_signal.extsignals;
  /* return state logic */
  asset_monitoring_state_t return_state = BME280_INIT_CONFIG;
  /* current machine logic */
  BME280_state_t currentState;
  static BME280_state_t nextState = BME280_ADD_VERIFN;
  bool address_verification = false;
  int ret_status = 0;
  if (SL_BT_MSG_ID(evt->header) != sl_bt_evt_system_external_signal_id) {
      return return_state;
  }
  currentState = nextState;

  switch(currentState)
  {
    case BME280_ADD_VERIFN:
    {
      if(event == ev_LETIMER0_UF)
      {
        //default state
        return_state = BME280_INIT_CONFIG;
#if DEBUG_1
        LOG_INFO("BME280_ADD_VERIFN\r");
#endif
        address_verification = false;
        address_verification = BME280_VerifyIdentity(&bme280_rd_buff[0]);
        if(address_verification == true)
        {
          nextState = BME280_REG_SOFTRESET;
        }
        else
        {
          //Add a counter logic
        }
      }
      break;
    }
    case BME280_REG_SOFTRESET:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
#if DEBUG_1
      LOG_INFO("BME280_REG_SOFTRESET\r");
#endif
      BME280_write(BME280_REGISTER_SOFTRESET,0xB6);
      nextState = BME280_REG_SOFTRESET_DELAY_1;
      break;
    }
    case BME280_REG_SOFTRESET_DELAY_1:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
      if(event == ev_I2C0_TRANSFER_DONE)
      {
        ret_status = timerWaitUs_irq(STD_DELAY);
        if(ret_status == ERROR)
        {
          LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
          nextState = BME280_READ_STATUS;
        }
      }
      break;
    }

    case BME280_READ_STATUS:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
      if(event == ev_LETIMER0_COMP1)
      {
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
        LOG_INFO("BME280_READ_STATUS\r");
#endif
        memset(bme280_rd_buff,0,sizeof(bme280_rd_buff));
        bme280_wr_buff[0] = BME280_REGISTER_STATUS;
        I2C0_WriteRead(BME280_ADDRESS, &bme280_wr_buff[0], 1, &bme280_rd_buff[0], 1);
        nextState = BME280_READ_STATUS_RESPONSE;
      }
      break;
    }
    case BME280_READ_STATUS_RESPONSE:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if DEBUG_1
          LOG_INFO("BME280_READ_STATUS_RESPONSE\r");
#endif
        if((bme280_rd_buff[0] & (1<<0)) != 0)
        {
          ret_status = timerWaitUs_irq(STD_DELAY);
          if(ret_status == ERROR)
          {
            LOG_ERROR("The value is more than the routine can provide\r");
          }
          else
          {
            nextState = BME280_READ_STATUS;
          }
        }
        else
        {
          nextState = BME280_READ_CALIB_1;
        }
      }
      break;
    }

    case BME280_READ_CALIB_1:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
#if DEBUG_1
      LOG_INFO("BME280_READ_CALIB_1\r");
#endif
      memset(bme280_rd_buff,0,sizeof(bme280_rd_buff));
      bme280_wr_buff[0] = BME280_TEMP_PRESS_CALIB_DATA_ADDR;
      I2C0_WriteRead(BME280_ADDRESS, &bme280_wr_buff[0], 1, &bme280_rd_buff[0], BME280_TEMP_PRESS_CALIB_DATA_LEN);
      nextState = BME280_READ_CALIB_1_RESPONSE;
      break;
    }

    case BME280_READ_CALIB_1_RESPONSE:
    {
#if DEBUG_1
      LOG_INFO("BME280_READ_CALIB_1_RESPONSE\r");
#endif
      if(event == ev_I2C0_TRANSFER_DONE)
      {
        parse_temp_press_calib_data(&bme280_rd_buff[0]);
        nextState = BME280_READ_CALIB_2;
      }
      break;
    }

    case BME280_READ_CALIB_2:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
#if DEBUG_1
      LOG_INFO("BME280_READ_CALIB_2\r");
#endif
      memset(bme280_rd_buff,0,sizeof(bme280_rd_buff));
      bme280_wr_buff[0] = BME280_HUMIDITY_CALIB1_DATA_ADDR;
      I2C0_WriteRead(BME280_ADDRESS, &bme280_wr_buff[0], 1, &bme280_rd_buff[0], 1);
      nextState = BME280_READ_CALIB_2_RESPONSE;
      break;
    }

    case BME280_READ_CALIB_2_RESPONSE:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
#if DEBUG_1
      LOG_INFO("BME280_READ_CALIB_2_RESPONSE\r");
#endif
      if(event == ev_I2C0_TRANSFER_DONE)
      {
        calib_data.dig_h1 = bme280_rd_buff[0];
        nextState = BME280_READ_CALIB_3;
      }
      break;
    }

    case BME280_READ_CALIB_3:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
#if DEBUG_1
      LOG_INFO("BME280_READ_CALIB_3\r");
#endif
      memset(bme280_rd_buff,0,sizeof(bme280_rd_buff));
      bme280_wr_buff[0] = BME280_HUMIDITY_CALIB_DATA_ADDR;
      I2C0_WriteRead(BME280_ADDRESS, &bme280_wr_buff[0], 1, &bme280_rd_buff[0], BME280_HUMIDITY_CALIB_DATA_LEN);
      nextState = BME280_READ_CALIB_3_RESPONSE;
      break;
    }

    case BME280_READ_CALIB_3_RESPONSE:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
#if DEBUG_1
      LOG_INFO("BME280_READ_CALIB_3_RESPONSE\r");
#endif
      if(event == ev_I2C0_TRANSFER_DONE)
      {
        parse_humidity_calib_data(&bme280_rd_buff[0]);
        nextState = BME280_SLEEP_MODE_SET;
      }
      break;
    }

    case BME280_SLEEP_MODE_SET:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
#if DEBUG_1
      LOG_INFO("BME280_SLEEP_MODE_SET\r");
#endif
      memset(bme280_rd_buff,0,sizeof(bme280_rd_buff));
      bme280_wr_buff[0] = BME280_CTRL_MEAS_ADDR;
      bme280_wr_buff[1] = BME280_SLEEP_MODE_SEL;
      I2C0_Write(BME280_ADDRESS, &bme280_wr_buff[0], 2);
      nextState = BME280_CONFIG_SET;
      break;
    }

    case BME280_CONFIG_SET:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
#if DEBUG_1
      LOG_INFO("BME280_CONFIG_SET\r");
#endif
      if(event == ev_I2C0_TRANSFER_DONE)
      {
        memset(bme280_rd_buff,0,sizeof(bme280_rd_buff));
        bme280_wr_buff[0] = BME280_CONFIG_ADDR;
        bme280_wr_buff[1] = BME280_CON_1SEC_FILT_OFF;
        I2C0_Write(BME280_ADDRESS, &bme280_wr_buff[0], 2);
        nextState = BME280_HUM_CTRL_SET;
      }
      break;
    }

    case BME280_HUM_CTRL_SET:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
#if DEBUG_1
      LOG_INFO("BME280_HUM_CTRL_SET\r");
#endif
      if(event == ev_I2C0_TRANSFER_DONE)
      {
        memset(bme280_rd_buff,0,sizeof(bme280_rd_buff));
        bme280_wr_buff[0] = BME280_CTRL_HUM_ADDR;
        bme280_wr_buff[1] = BME280_HUM_SAMP_ON;
        I2C0_Write(BME280_ADDRESS, &bme280_wr_buff[0], 2);
        nextState = BME280_MEAS_CTRL_SET;
      }
      break;
    }

    case BME280_MEAS_CTRL_SET:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
#if DEBUG_1
      LOG_INFO("BME280_MEAS_CTRL_SET\r");
#endif
      if(event == ev_I2C0_TRANSFER_DONE)
      {
        memset(bme280_rd_buff,0,sizeof(bme280_rd_buff));
        bme280_wr_buff[0] = BME280_CTRL_MEAS_ADDR;
        bme280_wr_buff[1] = BME280_TEMP_ON_PRS_OFF;
        I2C0_Write(BME280_ADDRESS, &bme280_wr_buff[0], 2);
        nextState = BME280_AFTER_SET_DELAY_2;
      }
      break;
    }

    case BME280_AFTER_SET_DELAY_2:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
#if DEBUG_1
      LOG_INFO("BME280_AFTER_SET_DELAY_2\r");
#endif
      if(event == ev_I2C0_TRANSFER_DONE)
      {
        timerWaitUs_irq(STD_DELAY);
        nextState = BME280_AFTER_INIT_DONE;
      }
      break;
    }

    case BME280_AFTER_INIT_DONE:
    {
      //default state
      return_state = BME280_INIT_CONFIG;
#if DEBUG_1
      LOG_INFO("BME280_AFTER_INIT_DONE\r");
#endif
      if(event == ev_LETIMER0_COMP1)
      {
        //RESET state of the machine
        nextState = BME280_ADD_VERIFN;
        //next state to switch in Asset monitoring SM
        return_state = BNO055_INIT_CONFIG;
        LOG_INFO("BME280 - TRANSITIONED\r");
      }
      break;
    }
    default:
      break;
  }
  return return_state;
}


void BME280_write(uint8_t reg, uint8_t byte_val)
{
  bme280_wr_buff[0] = reg;
  bme280_wr_buff[1] = byte_val;
  I2C0_Write(BME280_ADDRESS,&bme280_wr_buff[0],2);
}

bool BME280_VerifyIdentity(uint8_t* rd_buff)
{
  bme280_wr_buff[0] = BME280_CHIP_ID_ADDR;
  I2C0_WriteRead(BME280_ADDRESS, &bme280_wr_buff[0], 1, &rd_buff[0], 1);
  if(rd_buff[0] == BME280_ID)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
static double compensate_temperature(const uint32_t temp_data)
{
  int32_t var1;
  int32_t var2;
  double temperature_min = -40;
  double temperature_max = 85;

  //    var1 = ((double)temp_data) / 16384.0 - ((double)calib_data.dig_t1) / 1024.0;
  //    var1 = var1 * ((double)calib_data.dig_t2);
  //    var2 = (((double)temp_data) / 131072.0 - ((double)calib_data.dig_t1) / 8192.0);
  //    var2 = (var2 * var2) * ((double)calib_data.dig_t3);
  //    calib_data.t_fine = (int32_t)(var1 + var2);
  //    double temp = (var1 + var2) / 5120.0;

  var1 = (int32_t)((temp_data / 8) - ((int32_t)calib_data.dig_t1 * 2));
  var1 = (var1 * ((int32_t)calib_data.dig_t2)) / 2048;
  var2 = (int32_t)((temp_data / 16) - ((int32_t)calib_data.dig_t1));
  var2 = (((var2 * var2) / 4096) * ((int32_t)calib_data.dig_t3)) / 16384;

  calib_data.t_fine = var1 + var2;// + t_fine_adjust;

  int32_t T = (calib_data.t_fine * 5 + 128) / 256;

  double temp = (double)T / 100;

  if (temp < temperature_min)
  {
      temp = temperature_min;
  }
  else if (temp > temperature_max)
  {
      temp = temperature_max;
  }

  return temp;
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in double data type.
 */
static double compensate_humidity(const uint32_t hum_data)
{
  double hum;
  double humidity_min = 0.0;
  double humidity_max = 100.0;
  int32_t var1, var2, var3, var4, var5, var6;

  //    var1 = ((double)calib_data.t_fine) - 76800.0;
  //    var2 = (((double)calib_data.dig_h4) * 64.0 + (((double)calib_data.dig_h5) / 16384.0) * var1);
  //    var3 = hum_data - var2;
  //    var4 = ((double)calib_data.dig_h2) / 65536.0;
  //    var5 = (1.0 + (((double)calib_data.dig_h3) / 67108864.0) * var1);
  //    var6 = 1.0 + (((double)calib_data.dig_h6) / 67108864.0) * var1 * var5;
  //    var6 = var3 * var4 * (var5 * var6);
  //    hum = var6 * (1.0 - ((double)calib_data.dig_h1) * var6 / 524288.0);

  var1 = calib_data.t_fine - ((int32_t)76800);
  var2 = (int32_t)(hum_data * 16384);
  var3 = (int32_t)(((int32_t)calib_data.dig_h4) * 1048576);
  var4 = ((int32_t)calib_data.dig_h5) * var1;
  var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
  var2 = (var1 * ((int32_t)calib_data.dig_h6)) / 1024;
  var3 = (var1 * ((int32_t)calib_data.dig_h3)) / 2048;
  var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
  var2 = ((var4 * ((int32_t)calib_data.dig_h2)) + 8192) / 16384;
  var3 = var5 * var2;
  var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
  var5 = var3 - ((var4 * ((int32_t)calib_data.dig_h1)) / 16);
  var5 = (var5 < 0 ? 0 : var5);
  var5 = (var5 > 419430400 ? 419430400 : var5);
  uint32_t H = (uint32_t)(var5 / 4096);

  hum = (float)H / 1024.0;

  //    if (hum > humidity_max)
  //    {
  //        hum = humidity_max;
  //    }
  //    else if (hum < humidity_min)
  //    {
  //        hum = humidity_min;
  //    }

  return hum;
}

/*!
 *  @brief This internal API is used to parse the temperature and
 *  pressure calibration data and store it in device structure.
 */
static void parse_temp_press_calib_data(const uint8_t *reg_data)
{
  calib_data.dig_t1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
  calib_data.dig_t2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
  calib_data.dig_t3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
}

/*!
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device structure.
 */
static void parse_humidity_calib_data(const uint8_t *reg_data)
{
  int16_t dig_h4_lsb;
  int16_t dig_h4_msb;
  int16_t dig_h5_lsb;
  int16_t dig_h5_msb;

  calib_data.dig_h2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
  calib_data.dig_h3 = reg_data[2];
  dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
  dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
  calib_data.dig_h4 = dig_h4_msb | dig_h4_lsb;
  dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
  dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
  calib_data.dig_h5 = dig_h5_msb | dig_h5_lsb;
  calib_data.dig_h6 = (int8_t)reg_data[6];
}
