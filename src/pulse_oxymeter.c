/*
 * pulse_oxymeter.c
 *
 *  Created on: Nov 23, 2021
 *      Author: rishab
 *  References: https://github.com/sparkfun/SparkFun_Bio_Sensor_Hub_Library
 */

#include "pulse_oxymeter.h"
// Include logging for this file
#define INCLUDE_LOG_DEBUG       (0)
#include "src/log.h"
#include "ble.h"

#define DEBUG_READBACK                  (0)

//PROJECT::MAX30101 - Appl mode config
#define APPLICATION_CONFIG_DELAY        ((10)*(MSEC_TO_USEC))
#define APPLICATION_MODE                ((1000)*(MSEC_TO_USEC))
#define CMD_DELAY                       ((6)*(MSEC_TO_USEC))
#define ENABLE_CMD_DELAY                ((45)*(1000))

#define CONFIG_TO_RUN_STATE             ((1000)*(MSEC_TO_USEC))
#define SWITCH_I2C_DEVICE               ((50)*(MSEC_TO_USEC))
#define MAX30101_SLAVE_ADDRESS          (0x55)

#define ENABLE                          (0x01)

//init HB sensor
#define READ_DEVICE_MODE                (0x02)

//configuration HB sensor
#define OUTPUT_MODE                     (0x10)
//setOutputMode
#define SET_FORMAT                      (0x00)
#define ALGO_DATA                       (0x02)
//setFifoThreshold
#define WRITE_SET_THRESHOLD             (0x01)
//agcAlgoControl
#define ENABLE_ALGORITHM                (0x52)
#define ENABLE_AGC_ALGO                 (0x00)
//max30101Control
#define ENABLE_SENSOR                   (0x44)
#define ENABLE_MAX30101                 (0x03)
//maximFastAlgoControl
#define ENABLE_ALGORITHM                (0x52)
#define ENABLE_WHRM_ALGO                (0x02)
#define MODE_ONE                        (0x01)
//readAlgoSamples
#define READ_ALGORITHM_CONFIG           (0x51)
#define READ_AGC_NUM_SAMPLES            (0x00)
#define READ_AGC_NUM_SAMPLES_ID         (0x03)

//Running state
//numSamplesOutFifo
#define READ_DATA_OUTPUT                (0x12)
#define NUM_SAMPLES                     (0x00)

#define READ_DATA                       (0x01)
#define MAXFAST_ARRAY_SIZE              (6)

struct bioData
{
  // LSB = 0.1bpm
  uint16_t heartRate;
  // 0: Success, 1: Not Ready, 2: Object Detectected, 3: Finger Detected
  uint8_t  status;
};

//configuration machine functions
void setOutputMode(uint8_t output_type);
void setFifoThreshold(uint8_t in_threshold);
void agcAlgoControl(uint8_t enable);
void max30101Control(uint8_t senSwitch);
void maximFastAlgoControl(uint8_t mode);
void readAlgoSamples();

//running machine functions
void readSensorHubStatus();
void numSamplesOutFifo();
void readFillArray(uint8_t firstbyte, uint8_t secondbyte);
/*******************************************************************************
 Global
*******************************************************************************/
//uint8_t event_requested;
uint8_t write_i2c0_buffer[8] = {0};
uint8_t read_i2c0_buffer[10] = {0};

activity_monitoring_state_t heartbeat_machine_running(sl_bt_msg_t *evt)
{
  ble_ext_signal_event_t event = evt->data.evt_system_external_signal.extsignals;
  /* return state logic */
  activity_monitoring_state_t return_state = HEARTBEAT_READ;
  /* current machine logic */
  heartbeat_running_states currentState;
  static heartbeat_running_states nextState = HB_RUN_STATE_0;
  currentState = nextState;
  //support logic
  int ret_status = 0;
  uint8_t bpmArr[MAXFAST_ARRAY_SIZE] = {0};
  struct bioData sensorData = {0};

  ble_data_struct_t* bleDataPtr = BLE_GetDataStruct();

  switch(currentState)
  {
    case HB_RUN_STATE_0:
    {
      //Default states
      return_state = HEARTBEAT_READ;
      nextState = HB_RUN_STATE_0;
      LOG_INFO("HB_RUN_STATE_0\r");
      if((event == ev_LETIMER0_COMP1 || event == ev_LETIMER0_UF)
            && (bleDataPtr->s_Bonded == true))
      {
          if(event == ev_LETIMER0_COMP1)
          {
              LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
          }
          readSensorHubStatus();
      }
      else if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
          //Enter EM2 mode
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
          ret_status = timerWaitUs_irq(CMD_DELAY);
          if(ret_status == -1)
          {
            LOG_ERROR("The value is more than the routine can provide \r");
          }
          else
          {
            nextState = HB_RUN_STATE_1;
          }
      }
      else
      {
          if(event == ev_LETIMER0_COMP1)
          {
              LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
          }
          ret_status = timerWaitUs_irq(SWITCH_I2C_DEVICE);
          if(ret_status == -1)
          {
              LOG_ERROR("The value is more than the routine can provide \r");
          }
          else
          {
              return_state = ACCEL_READ;
              nextState = HB_RUN_STATE_0;
          }
      }
      break;
    }

    case HB_RUN_STATE_1:
    {
      //Default states
      return_state = HEARTBEAT_READ;
      nextState = HB_RUN_STATE_1;
      LOG_INFO("HB_RUN_STATE_1\r");

      if(event == ev_LETIMER0_COMP1)
      {
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if POWER_MANAGEMENT
          //Enter EM1 mode
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
          I2C0_Read(MAX30101_SLAVE_ADDRESS,read_i2c0_buffer,2);
      }

      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
          //Enter EM2 mode
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
#if DEBUG_READBACK
          LOG_INFO("HB_RUN_STATE_1:: Data is data1 = %d, data2 = %d\r",read_i2c0_buffer[0], read_i2c0_buffer[1]);
#endif
          ret_status = timerWaitUs_irq(PSEUDO_TRIGGER);
          if(ret_status == -1)
          {
              LOG_ERROR("The value is more than the routine can provide \r");
          }
          else
          {
              nextState = HB_RUN_STATE_2;
          }
      }

      break;
    }

    case HB_RUN_STATE_2:
    {
      //Default states
      return_state = HEARTBEAT_READ;
      nextState = HB_RUN_STATE_2;
      LOG_INFO("HB_RUN_STATE_2\r");

      if(event == ev_LETIMER0_COMP1)
      {
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
          numSamplesOutFifo();
      }

      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
          //Enter EM2 mode
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
          ret_status = timerWaitUs_irq(CMD_DELAY);
          if(ret_status == -1)
          {
            LOG_ERROR("The value is more than the routine can provide \r");
          }
          else
          {
            nextState = HB_RUN_STATE_3;
          }
      }
      break;
    }

    case HB_RUN_STATE_3:
    {
      //Default states
      return_state = HEARTBEAT_READ;
      nextState = HB_RUN_STATE_3;
      LOG_INFO("HB_RUN_STATE_3\r");

      if(event == ev_LETIMER0_COMP1)
      {
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if POWER_MANAGEMENT
          //Enter EM1 mode
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
          I2C0_Read(MAX30101_SLAVE_ADDRESS,read_i2c0_buffer,2);
      }

      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
          //Enter EM2 mode
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
#if DEBUG_READBACK
          LOG_INFO("HB_RUN_STATE_3:: Data is data1 = %d, data2 = %d\r",read_i2c0_buffer[0], read_i2c0_buffer[1]);
#endif
          ret_status = timerWaitUs_irq(PSEUDO_TRIGGER);
          if(ret_status == -1)
          {
              LOG_ERROR("The value is more than the routine can provide \r");
          }
          else
          {
              nextState = HB_RUN_STATE_4;
          }
      }

      break;
    }

    case HB_RUN_STATE_4:
    {
      //Default states
      return_state = HEARTBEAT_READ;
      nextState = HB_RUN_STATE_4;
      LOG_INFO("HB_RUN_STATE_4\r");

      if(event == ev_LETIMER0_COMP1)
      {
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
          readFillArray(READ_DATA_OUTPUT,READ_DATA);
      }

      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
          //Enter EM2 mode
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
          ret_status = timerWaitUs_irq(CMD_DELAY);
          if(ret_status == -1)
          {
            LOG_ERROR("The value is more than the routine can provide \r");
          }
          else
          {
            nextState = HB_RUN_STATE_5;
          }
      }
      break;
    }

    case HB_RUN_STATE_5:
    {
      //Default states
      return_state = HEARTBEAT_READ;
      nextState = HB_RUN_STATE_5;
      LOG_INFO("HB_RUN_STATE_5\r");

      if(event == ev_LETIMER0_COMP1)
      {
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if POWER_MANAGEMENT
          //Enter EM1 mode
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
          I2C0_Read(MAX30101_SLAVE_ADDRESS,read_i2c0_buffer,(MAXFAST_ARRAY_SIZE + 1));
      }

      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
          //Enter EM2 mode
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
#if DEBUG_READBACK
          LOG_INFO("HB_RUN_STATE_5:: Data is data0 = %d, data1 = %d data2 = %d, data3 = %d data4 = %d, data5 = %d data6 = %d\r",read_i2c0_buffer[0], read_i2c0_buffer[1],
                   read_i2c0_buffer[2], read_i2c0_buffer[3],
                   read_i2c0_buffer[4], read_i2c0_buffer[5],read_i2c0_buffer[6]);
#endif
          //processing logic for data
          for(uint8_t i = 1; i<(MAXFAST_ARRAY_SIZE+1); i++)
          {
              bpmArr[i-1] = read_i2c0_buffer[i];
          }

          // Heart Rate formatting
          sensorData.heartRate = (uint16_t)(bpmArr[0] << 8);
          sensorData.heartRate |= (bpmArr[1]);
          sensorData.heartRate /= 10;

          //"Machine State" - has a finger been detected?
          sensorData.status = bpmArr[5];

          LOG_INFO("HB = %d status = %d\r",sensorData.heartRate,sensorData.status);

          displayPrintf(DISPLAY_ROW_HEARTBEAT, "HeartBeat = %d",sensorData.heartRate);

          if(bleDataPtr->s_HealthIndicating == true
              && bleDataPtr->s_ClientConnected == true
                && bleDataPtr->s_Bonded == true
                  && sensorData.status == 3)
          {
              BleServer_SendHearbeatDataToClient(sensorData.heartRate);
          }

          ret_status = timerWaitUs_irq(SWITCH_I2C_DEVICE);
          if(ret_status == -1)
          {
              LOG_ERROR("The value is more than the routine can provide \r");
          }
          else
          {
              return_state = ACCEL_READ;
              nextState = HB_RUN_STATE_0;
          }
      }
      break;
    }
    default:
      break;
  }
  return return_state;
}

void readSensorHubStatus()
{
  write_i2c0_buffer[0] = 0x00;
  write_i2c0_buffer[1] = 0x00;
#if POWER_MANAGEMENT
  //Enter EM1 mode
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  I2C0_Write(MAX30101_SLAVE_ADDRESS,write_i2c0_buffer,2);
}

void numSamplesOutFifo()
{
  write_i2c0_buffer[0] = READ_DATA_OUTPUT;
  write_i2c0_buffer[1] = NUM_SAMPLES;
#if POWER_MANAGEMENT
  //Enter EM1 mode
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  I2C0_Write(MAX30101_SLAVE_ADDRESS,write_i2c0_buffer,2);
}

void readFillArray(uint8_t firstbyte, uint8_t secondbyte)
{
  write_i2c0_buffer[0] = firstbyte;
  write_i2c0_buffer[1] = secondbyte;
#if POWER_MANAGEMENT
  //Enter EM1 mode
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  I2C0_Write(MAX30101_SLAVE_ADDRESS,write_i2c0_buffer,2);
}

activity_monitoring_state_t config_heartbeat_machine(sl_bt_msg_t *evt)
{
    ble_ext_signal_event_t event = evt->data.evt_system_external_signal.extsignals;
    /* return state logic */
    activity_monitoring_state_t return_state = HEARTBEAT_CONFIGURE;
    /* current machine logic */
    heartbeat_config_states currentState;
    static heartbeat_config_states nextState = HB_CONFIG_STATE_0;
    currentState = nextState;
    //support logic
    int ret_status = 0;

    switch(currentState)
    {
      case HB_CONFIG_STATE_0:
      {
        //Default states
        return_state = HEARTBEAT_CONFIGURE;
        nextState = HB_CONFIG_STATE_0;
        LOG_INFO("HB_CONFIG_STATE_0\r");

        if(event == ev_LETIMER0_COMP1)
        {
            LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
            setOutputMode(ALGO_DATA);
        }

        if(event == ev_I2C0_TRANSFER_DONE)
        {
#if POWER_MANAGEMENT
            //Enter EM2 mode
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
            ret_status = timerWaitUs_irq(CMD_DELAY);
            if(ret_status == -1)
            {
              LOG_ERROR("The value is more than the routine can provide \r");
            }
            else
            {
              nextState = HB_CONFIG_STATE_1;
            }
        }
        break;
      }
      case HB_CONFIG_STATE_1:
      {
        //Default states
        return_state = HEARTBEAT_CONFIGURE;
        nextState = HB_CONFIG_STATE_1;
        LOG_INFO("HB_CONFIG_STATE_1\r");

        if(event == ev_LETIMER0_COMP1)
        {
            LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if POWER_MANAGEMENT
            //Enter EM1 mode
            sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
            I2C0_Read(MAX30101_SLAVE_ADDRESS,read_i2c0_buffer,1);
        }

        if(event == ev_I2C0_TRANSFER_DONE)
        {
#if POWER_MANAGEMENT
            //Enter EM2 mode
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
#if DEBUG_READBACK
            LOG_INFO("HB_CONFIG_STATE_1:: Data is data1 = %d, data2 = %d\r",read_i2c0_buffer[0], read_i2c0_buffer[1]);
#endif
            ret_status = timerWaitUs_irq(PSEUDO_TRIGGER);
            if(ret_status == -1)
            {
                LOG_ERROR("The value is more than the routine can provide \r");
            }
            else
            {
                nextState = HB_CONFIG_STATE_2;
            }
        }

        break;
      }

      case HB_CONFIG_STATE_2:
      {
        //Default states
        return_state = HEARTBEAT_CONFIGURE;
        nextState = HB_CONFIG_STATE_2;
        LOG_INFO("HB_CONFIG_STATE_2\r");

        if(event == ev_LETIMER0_COMP1)
        {
            //ENABLE
            LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
            setFifoThreshold(0x01);
        }

        if(event == ev_I2C0_TRANSFER_DONE)
        {
#if POWER_MANAGEMENT
            //Enter EM2 mode
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
            ret_status = timerWaitUs_irq(CMD_DELAY);
            if(ret_status == -1)
            {
              LOG_ERROR("The value is more than the routine can provide \r");
            }
            else
            {
              nextState = HB_CONFIG_STATE_3;
            }
        }
        break;
      }

      case HB_CONFIG_STATE_3:
      {
        //Default states
        return_state = HEARTBEAT_CONFIGURE;
        nextState = HB_CONFIG_STATE_3;
        LOG_INFO("HB_CONFIG_STATE_3\r");

        if(event == ev_LETIMER0_COMP1)
        {
            LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if POWER_MANAGEMENT
            //Enter EM1 mode
            sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
            I2C0_Read(MAX30101_SLAVE_ADDRESS,read_i2c0_buffer,1);
        }

        if(event == ev_I2C0_TRANSFER_DONE)
        {
#if DEBUG_READBACK
            LOG_INFO("HB_CONFIG_STATE_3:: Data is data1 = %d, data2 = %d\r",read_i2c0_buffer[0], read_i2c0_buffer[1]);
#endif
#if POWER_MANAGEMENT
            //Enter EM2 mode
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
            ret_status = timerWaitUs_irq(PSEUDO_TRIGGER);
            if(ret_status == -1)
            {
                LOG_ERROR("The value is more than the routine can provide \r");
            }
            else
            {
                nextState = HB_CONFIG_STATE_4;
            }
        }
        break;
      }

      case HB_CONFIG_STATE_4:
      {
        //Default states
        return_state = HEARTBEAT_CONFIGURE;
        nextState = HB_CONFIG_STATE_4;
        LOG_INFO("HB_CONFIG_STATE_4\r");

        if(event == ev_LETIMER0_COMP1)
        {
            LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
            agcAlgoControl(ENABLE);
        }

        if(event == ev_I2C0_TRANSFER_DONE)
        {
#if POWER_MANAGEMENT
            //Enter EM2 mode
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
            ret_status = timerWaitUs_irq(ENABLE_CMD_DELAY);
            if(ret_status == -1)
            {
              LOG_ERROR("The value is more than the routine can provide \r");
            }
            else
            {
              nextState = HB_CONFIG_STATE_5;
            }
        }
        break;
      }

      case HB_CONFIG_STATE_5:
      {
        //Default states
        return_state = HEARTBEAT_CONFIGURE;
        nextState = HB_CONFIG_STATE_5;
        LOG_INFO("HB_CONFIG_STATE_5\r");

        if(event == ev_LETIMER0_COMP1)
        {
            LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if POWER_MANAGEMENT
            //Enter EM1 mode
            sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
            I2C0_Read(MAX30101_SLAVE_ADDRESS,read_i2c0_buffer,1);
        }

        if(event == ev_I2C0_TRANSFER_DONE)
        {
#if POWER_MANAGEMENT
            //Enter EM2 mode
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
#if DEBUG_READBACK
            LOG_INFO("HB_CONFIG_STATE_5:: Data is data1 = %d, data2 = %d\r",read_i2c0_buffer[0], read_i2c0_buffer[1]);
#endif
            ret_status = timerWaitUs_irq(PSEUDO_TRIGGER);
            if(ret_status == -1)
            {
                LOG_ERROR("The value is more than the routine can provide \r");
            }
            else
            {
                nextState = HB_CONFIG_STATE_6;
            }
        }
        break;
      }

      case HB_CONFIG_STATE_6:
      {
        //Default states
        return_state = HEARTBEAT_CONFIGURE;
        nextState = HB_CONFIG_STATE_6;
        LOG_INFO("HB_CONFIG_STATE_6\r");

        if(event == ev_LETIMER0_COMP1)
        {
            LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
            max30101Control(ENABLE);
        }

        if(event == ev_I2C0_TRANSFER_DONE)
        {
#if POWER_MANAGEMENT
            //Enter EM2 mode
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
            ret_status = timerWaitUs_irq(ENABLE_CMD_DELAY);
            if(ret_status == -1)
            {
              LOG_ERROR("The value is more than the routine can provide \r");
            }
            else
            {
              nextState = HB_CONFIG_STATE_7;
            }
        }
        break;
      }

      case HB_CONFIG_STATE_7:
      {
        //Default states
        return_state = HEARTBEAT_CONFIGURE;
        nextState = HB_CONFIG_STATE_7;
        LOG_INFO("HB_CONFIG_STATE_7\r");

        if(event == ev_LETIMER0_COMP1)
        {
            LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if POWER_MANAGEMENT
            //Enter EM1 mode
            sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
            I2C0_Read(MAX30101_SLAVE_ADDRESS,read_i2c0_buffer,1);
        }

        if(event == ev_I2C0_TRANSFER_DONE)
        {
#if POWER_MANAGEMENT
            //Enter EM2 mode
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
#if DEBUG_READBACK
            LOG_INFO("HB_CONFIG_STATE_7:: Data is data1 = %d, data2 = %d\r",read_i2c0_buffer[0], read_i2c0_buffer[1]);
#endif
            ret_status = timerWaitUs_irq(PSEUDO_TRIGGER);
            if(ret_status == -1)
            {
                LOG_ERROR("The value is more than the routine can provide \r");
            }
            else
            {
                nextState = HB_CONFIG_STATE_8;
            }
        }
        break;
      }

      case HB_CONFIG_STATE_8:
      {
        //Default states
        return_state = HEARTBEAT_CONFIGURE;
        nextState = HB_CONFIG_STATE_8;
        LOG_INFO("HB_CONFIG_STATE_8\r");

        if(event == ev_LETIMER0_COMP1)
        {
            LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
            maximFastAlgoControl(MODE_ONE);
        }

        if(event == ev_I2C0_TRANSFER_DONE)
        {
#if POWER_MANAGEMENT
            //Enter EM2 mode
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
            ret_status = timerWaitUs_irq(ENABLE_CMD_DELAY);
            if(ret_status == -1)
            {
              LOG_ERROR("The value is more than the routine can provide \r");
            }
            else
            {
              nextState = HB_CONFIG_STATE_9;
            }
        }
        break;
      }

      case HB_CONFIG_STATE_9:
      {
        //Default states
        return_state = HEARTBEAT_CONFIGURE;
        nextState = HB_CONFIG_STATE_9;
        LOG_INFO("HB_CONFIG_STATE_9\r");

        if(event == ev_LETIMER0_COMP1)
        {
            LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if POWER_MANAGEMENT
            //Enter EM1 mode
            sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
            I2C0_Read(MAX30101_SLAVE_ADDRESS,read_i2c0_buffer,1);
        }

        if(event == ev_I2C0_TRANSFER_DONE)
        {
#if POWER_MANAGEMENT
            //Enter EM2 mode
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
#if DEBUG_READBACK
            LOG_INFO("HB_CONFIG_STATE_9:: Data is data1 = %d, data2 = %d\r",read_i2c0_buffer[0], read_i2c0_buffer[1]);
#endif
            ret_status = timerWaitUs_irq(PSEUDO_TRIGGER);
            if(ret_status == -1)
            {
                LOG_ERROR("The value is more than the routine can provide \r");
            }
            else
            {
                nextState = HB_CONFIG_STATE_10;
            }
        }
        break;
      }

      case HB_CONFIG_STATE_10:
      {
        //Default states
        return_state = HEARTBEAT_CONFIGURE;
        nextState = HB_CONFIG_STATE_10;
        LOG_INFO("HB_CONFIG_STATE_10\r");

        if(event == ev_LETIMER0_COMP1)
        {
            LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
            readAlgoSamples();
        }

        if(event == ev_I2C0_TRANSFER_DONE)
        {
#if POWER_MANAGEMENT
            //Enter EM2 mode
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
            ret_status = timerWaitUs_irq(CMD_DELAY);
            if(ret_status == -1)
            {
              LOG_ERROR("The value is more than the routine can provide \r");
            }
            else
            {
              nextState = HB_CONFIG_STATE_11;
            }
        }
        break;
      }

      case HB_CONFIG_STATE_11:
      {
        //Default states
        return_state = HEARTBEAT_CONFIGURE;
        nextState = HB_CONFIG_STATE_11;
        LOG_INFO("HB_CONFIG_STATE_11\r");

        if(event == ev_LETIMER0_COMP1)
        {
            LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if POWER_MANAGEMENT
            //Enter EM1 mode
            sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
            I2C0_Read(MAX30101_SLAVE_ADDRESS,read_i2c0_buffer,2);
        }

        if(event == ev_I2C0_TRANSFER_DONE)
        {
#if POWER_MANAGEMENT
            //Enter EM2 mode
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
#if DEBUG_READBACK
            LOG_INFO("HB_CONFIG_STATE_11:: Data is data1 = %d, data2 = %d\r",read_i2c0_buffer[0], read_i2c0_buffer[1]);
#endif
            ret_status = timerWaitUs_irq(CONFIG_TO_RUN_STATE);
            if(ret_status == -1)
            {
                LOG_ERROR("The value is more than the routine can provide \r");
            }
            else
            {
                return_state = HEARTBEAT_READ;
                nextState = HB_CONFIG_STATE_0;
            }
        }
        break;
      }

      default:
        break;
    }

    return return_state;
}


void setOutputMode(uint8_t output_type)
{
  write_i2c0_buffer[0] = OUTPUT_MODE;
  write_i2c0_buffer[1] = SET_FORMAT;
  write_i2c0_buffer[2] = output_type;
#if POWER_MANAGEMENT
  //Enter EM1 mode
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  I2C0_Write(MAX30101_SLAVE_ADDRESS,write_i2c0_buffer,3);
}

void setFifoThreshold(uint8_t in_threshold)
{
  write_i2c0_buffer[0] = OUTPUT_MODE;
  write_i2c0_buffer[1] = WRITE_SET_THRESHOLD;
  write_i2c0_buffer[2] = in_threshold;
#if POWER_MANAGEMENT
  //Enter EM1 mode
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  I2C0_Write(MAX30101_SLAVE_ADDRESS,write_i2c0_buffer,3);
}

void agcAlgoControl(uint8_t enable)
{
  write_i2c0_buffer[0] = ENABLE_ALGORITHM;
  write_i2c0_buffer[1] = ENABLE_AGC_ALGO;
  write_i2c0_buffer[2] = enable;
#if POWER_MANAGEMENT
  //Enter EM1 mode
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  I2C0_Write(MAX30101_SLAVE_ADDRESS,write_i2c0_buffer,3);
}

void max30101Control(uint8_t senSwitch)
{
  write_i2c0_buffer[0] = ENABLE_SENSOR;
  write_i2c0_buffer[1] = ENABLE_MAX30101;
  write_i2c0_buffer[2] = senSwitch;
#if POWER_MANAGEMENT
  //Enter EM1 mode
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  I2C0_Write(MAX30101_SLAVE_ADDRESS,write_i2c0_buffer,3);
}

void maximFastAlgoControl(uint8_t mode)
{
  write_i2c0_buffer[0] = ENABLE_ALGORITHM;
  write_i2c0_buffer[1] = ENABLE_WHRM_ALGO;
  write_i2c0_buffer[2] = mode;
#if POWER_MANAGEMENT
  //Enter EM1 mode
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  I2C0_Write(MAX30101_SLAVE_ADDRESS,write_i2c0_buffer,3);
}

void readAlgoSamples()
{
  write_i2c0_buffer[0] = READ_ALGORITHM_CONFIG;
  write_i2c0_buffer[1] = READ_AGC_NUM_SAMPLES;
  write_i2c0_buffer[2] = READ_AGC_NUM_SAMPLES_ID ;
#if POWER_MANAGEMENT
  //Enter EM1 mode
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  I2C0_Write(MAX30101_SLAVE_ADDRESS,write_i2c0_buffer,3);
}

activity_monitoring_state_t init_heartbeat_machine(sl_bt_msg_t *evt)
{
  ble_ext_signal_event_t event = evt->data.evt_system_external_signal.extsignals;
  /* return state logic */
  activity_monitoring_state_t return_state = HEARTBEAT_INIT;
  /* current machine logic */
  heartbeat_init_states currentState;
  static heartbeat_init_states nextState = HB_INIT_STATE_0;
  currentState = nextState;
  //support logic
  int ret_status = 0;

  switch(currentState)
  {
    case HB_INIT_STATE_0:
    {
      //Default states
      return_state = HEARTBEAT_INIT;
      nextState = HB_INIT_STATE_0;
      LOG_INFO("HB_INIT_STATE_0\r");

      LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
      gpioMAX30101_InitConfigurations();
      /* state - 0 logic  start*/
      gpioPowerOff_reset_MAX30101();
      gpioPowerOn_mfio_MAX30101();
      //Wait for 10 msec
      ret_status = timerWaitUs_irq(APPLICATION_CONFIG_DELAY);
      if(ret_status == -1)
      {
        LOG_ERROR("The value is more than the routine can provide \r");
      }
      else
      {
        nextState = HB_INIT_STATE_1;
      }

      break;

    }

    case HB_INIT_STATE_1:
    {
      //Default states
      return_state = HEARTBEAT_INIT;
      nextState = HB_INIT_STATE_1;
      LOG_INFO("HB_INIT_STATE_1\r");

       /* state - 1 logic  start*/
       if(event == ev_LETIMER0_COMP1)
       {
         LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
         gpioPowerOn_reset_MAX30101();

         //Wait for 1000 msec
         ret_status = timerWaitUs_irq(APPLICATION_MODE);
         if(ret_status == -1)
         {
           LOG_ERROR("The value is more than the routine can provide \r");
         }
         nextState = HB_INIT_STATE_2;
       }

      break;
    }

    case HB_INIT_STATE_2:
    {
      //Default states
      return_state = HEARTBEAT_INIT;
      nextState = HB_INIT_STATE_2;
      LOG_INFO("HB_INIT_STATE_2\r");

      /* state - 2 logic  start*/
      if(event == ev_LETIMER0_COMP1)
      {
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
        //confirm
        GPIO_PinModeSet(MAX30101_port, MAX30101_mfio_pin, gpioModeInputPull, true);
        GPIO_ExtIntConfig(MAX30101_port,MAX30101_mfio_pin,MAX30101_mfio_pin,false,true,true);

        write_i2c0_buffer[0] = READ_DEVICE_MODE;
        write_i2c0_buffer[1] = 0x00;
#if POWER_MANAGEMENT
        //Enter EM1 mode
        sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        I2C0_Write(MAX30101_SLAVE_ADDRESS,write_i2c0_buffer,2);
      }

      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        //Wait for 6 msec
        ret_status = timerWaitUs_irq(CMD_DELAY);
        if(ret_status == -1)
        {
            LOG_ERROR("The value is more than the routine can provide \r");
        }
        else
        {
            nextState = HB_INIT_STATE_3;
        }
      }
      break;
    }

    case HB_INIT_STATE_3:
    {
      //Default states
      return_state = HEARTBEAT_INIT;
      nextState = HB_INIT_STATE_3;
      LOG_INFO("HB_INIT_STATE_3\r");

      /* state - 3 logic  start*/
      if(event == ev_LETIMER0_COMP1)
      {
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if POWER_MANAGEMENT
        //Enter EM1 mode
        sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        I2C0_Read(MAX30101_SLAVE_ADDRESS,read_i2c0_buffer,2);
      }

      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
          //Enter EM2 mode
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
#if DEBUG_READBACK
          LOG_INFO("HB_INIT_STATE_3:: Data is data1 = %d, data2 = %d\r",read_i2c0_buffer[0], read_i2c0_buffer[1]);
#endif
          //Reset the sequence for next time
          //execute pseudo timer to trigger machine
          ret_status = timerWaitUs_irq(PSEUDO_TRIGGER);
          if(ret_status == -1)
          {
              LOG_ERROR("The value is more than the routine can provide \r");
          }
          else
          {
              return_state = HEARTBEAT_CONFIGURE;
              nextState = HB_INIT_STATE_0;
          }
      }

      break;
    }

    default:
      break;
  }

  return return_state;
}
