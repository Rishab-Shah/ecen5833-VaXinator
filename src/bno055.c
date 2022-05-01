/*
 * bno055.c
 *
 *  Created on: Feb 17, 2022
 *      Author: rishab
 */

/*******************************************************************************
 Headers
*******************************************************************************/
#include <src/bno055.h>
// Include logging for this file
#define INCLUDE_LOG_DEBUG       (1)
#include "src/log.h"
#include "ble.h"

/*******************************************************************************
 Macros
*******************************************************************************/
#define ERROR                (-1)
/*******************************************************************************
 Global
*******************************************************************************/
uint8_t bno055_wr_buff[8] = { 0 };
uint8_t bno055_rd_buff[8] = { 0 };
/*******************************************************************************
 Function Prototypes
*******************************************************************************/
bool BNO055_VerifyIdentity(uint8_t* rd_buff);
void setMode(uint8_t set_mode);
void BNO055_write(uint8_t reg, uint8_t byte_val);
/*******************************************************************************
 Function Definition
*******************************************************************************/
#if NO_BL
asset_monitoring_state_t bno055_read_machine(sl_bt_msg_t *evt)
#else
BNO055_state_t init_bno055_machine(ble_ext_signal_event_t evt)
#endif
{
  ble_data_struct_t* ble_data = BLE_GetDataStruct();
#if NO_BL
  ble_ext_signal_event_t event = evt->data.evt_system_external_signal.extsignals;
#else
  ble_ext_signal_event_t event = evt;
#endif
  /* return state logic */
  asset_monitoring_state_t return_state = BNO055_READ;
  /* current machine logic */
  BNO055_state_t currentState;
  static BNO055_state_t nextState = READ_XYZ_DATA;
  int ret_status = 0;
#if NO_BL
  if (SL_BT_MSG_ID(evt->header) != sl_bt_evt_system_external_signal_id) {
      return return_state;
  }
#endif
  currentState = nextState;

  switch(currentState)
  {
    case READ_XYZ_DATA:
    {
      //default state
      return_state = BNO055_READ;
      if(event == ev_LETIMER0_COMP1)
      {
        gpioDebugLEDSetOn();
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
        LOG_INFO("READ_XYZ_DATA\r");
#endif
        memset(bno055_rd_buff,0,sizeof(bno055_rd_buff));
        bno055_wr_buff[0] = BNO055_EULER_H_LSB_ADDR;
#if PWR_MGMT_COMP
        //Enter EM1 mode
        sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif

#if PWR_MGMT_RUN_MODE
        //Enter EM1 mode
        sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        I2C0_WriteRead(BNO055_ADDRESS_A, &bno055_wr_buff[0], 1, &bno055_rd_buff[0], 6);
        nextState = READ_XYZ_DATA_DELAY;
      }
      break;
    }
    case READ_XYZ_DATA_DELAY:
    {
      //default state
      return_state = BNO055_READ;
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if PWR_MGMT_COMP
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        gpioDebugLEDSetOff();
        //processing
        int16_t current_X = 0, current_Y = 0, current_Z = 0;
        current_X = ((int16_t)(bno055_rd_buff[0]) | (int16_t)(bno055_rd_buff[1] << 8));
        current_Y = ((int16_t)(bno055_rd_buff[2]) | (int16_t)(bno055_rd_buff[3] << 8));
        current_Z = ((int16_t)(bno055_rd_buff[4]) | (int16_t)(bno055_rd_buff[5] << 8));

        LOG_INFO("X= %d::Y=%d::Z=%d\r",current_X,current_Y,current_Z);
#if NO_BL
        if(ble_data->s_AccelIndication && ble_data->s_ClientConnected)
        {
            char xyz_data[100]; memset(xyz_data,0,sizeof(xyz_data));
            sprintf(xyz_data,"X:%d,Y:%d,Z:%d",current_X,current_Y,current_Z);
            BleServer_SendAccelDataToClient(&xyz_data[0]);
        }
#endif
        static uint8_t running_status = 0; static uint8_t ref_cnt = 0;
        ref_cnt++;
        if(ref_cnt == 5)
        {
            if(((current_X > ble_data->prev_AccelX - ble_data->ignore_accl_threshold) && ( current_X < ble_data->prev_AccelX + ble_data->ignore_accl_threshold))
                && ((current_Y > ble_data->prev_AccelY - ble_data->ignore_accl_threshold) && (current_Y < ble_data->prev_AccelY + ble_data->ignore_accl_threshold))
                && ((current_Z > ble_data->prev_AccelZ - ble_data->ignore_accl_threshold) && (current_Z < ble_data->prev_AccelZ + ble_data->ignore_accl_threshold)))
            {
              LOG_INFO("0\r");
              if(running_status == 1)
              {
                strncpy(ble_data->xyz_array,"0",1);
                running_status = 0;
              }
            }
            else if(((current_X > ble_data->prev_AccelX - ble_data->low_accl_threshold) && ( current_X < ble_data->prev_AccelX + ble_data->low_accl_threshold))
                || ((current_Y > ble_data->prev_AccelY - ble_data->low_accl_threshold) && (current_Y < ble_data->prev_AccelY + ble_data->low_accl_threshold))
                || ((current_Z > ble_data->prev_AccelZ - ble_data->low_accl_threshold) && (current_Z < ble_data->prev_AccelZ + ble_data->low_accl_threshold)))
            {
              LOG_INFO("1\r");
              running_status = 1;
              strncpy(ble_data->xyz_array,"1",1);
            }
        }

        ble_data->prev_AccelX = current_X;
        ble_data->prev_AccelY = current_Y;
        ble_data->prev_AccelZ = current_Z;

#if 0
        else if(((current_X > ble_data->prev_AccelX - ble_data->high_accl_threshold) && ( current_X < ble_data->prev_AccelX + ble_data->high_accl_threshold))
            || ((current_Y > ble_data->prev_AccelY - ble_data->high_accl_threshold) && (current_Y < ble_data->prev_AccelY + ble_data->high_accl_threshold))
            || ((current_Z > ble_data->prev_AccelZ - ble_data->high_accl_threshold) && (current_Z < ble_data->prev_AccelZ + ble_data->high_accl_threshold)))
        {
          LOG_INFO("2\r");
          running_status = 1;
          strncpy(ble_data->xyz_array,"2",1);
        }
#endif

#if PWR_MGMT_RUN_MODE
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        ret_status = timerWaitUs_irq(BNO055_DATA_POLL);
        if(ret_status == -1)
        {
          LOG_ERROR("The value is more than the routine can provide \r");
        }
        else
        {
          //reset the state of the current machine
          nextState = READ_XYZ_DATA;
          //next state to switch in Asset SM
          return_state = DEBUG_READ;
        }
      }
      break;
    }
    default:
      break;
  }
  return return_state;
}

#if NO_BL
asset_monitoring_state_t init_bno055_machine(sl_bt_msg_t *evt)
#else
BNO055_state_t init_bno055_machine(ble_ext_signal_event_t evt)
#endif
{
  //ble_data_struct_t* ble_data = BLE_GetDataStruct();
#if NO_BL
  ble_ext_signal_event_t event = evt->data.evt_system_external_signal.extsignals;
#else
  ble_ext_signal_event_t event = evt;
#endif
  /* return state logic */
  asset_monitoring_state_t return_state = BNO055_INIT_CONFIG;
  /* current machine logic */
  BNO055_state_t currentState;
  static BNO055_state_t nextState = BNO055_SETMODE;
  bool address_verification = false;
  int ret_status = 0;
#if NO_BL
  if (SL_BT_MSG_ID(evt->header) != sl_bt_evt_system_external_signal_id) {
      return return_state;
  }
#endif
  currentState = nextState;

  switch(currentState)
  {
    case BNO055_ADD_VERIFN:
    {
      if(event == ev_LETIMER0_UF)
      {
#if DEBUG_1
        LOG_INFO("BNO055_ADD_VERIFN\r");
#endif

#if 0
        address_verification = false;
        address_verification = BNO055_VerifyIdentity(&bno055_rd_buff[0]);
        if(address_verification == true)
        {
          nextState = BNO055_SETMODE;
        }
        else
        {
          //Add a counter logic
        }
#endif
      }
      break;
    }
    case BNO055_SETMODE:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
#if DEBUG_1
      LOG_INFO("BNO055_SETMODE\r");
#endif
      if(event == ev_LETIMER0_UF)
      {
        setMode(OPERATION_MODE_CONFIG); //Accelerometer only
#if 0
        nextState = BNO055_SETMODE_DELAY_1;
#else
        ret_status = timerWaitUs_irq(PSEUDO_TRIGGER);
        if(ret_status == ERROR)
        {
          LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
          nextState = BNO055_SETMODE_DELAY_1;
        }
#endif
      }
      break;
    }
    case BNO055_SETMODE_DELAY_1:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if PWR_MGMT_COMP
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        ret_status = timerWaitUs_irq(SETMODE_DELAY);
        if(ret_status == ERROR)
        {
          LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
          nextState = BNO055_RESET;
        }
      }
      break;
    }
    case BNO055_RESET:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_LETIMER0_COMP1)
      {
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
        LOG_INFO("BNO055_RESET\r");
#endif
        BNO055_write(BNO055_SYS_TRIGGER_ADDR,0x20);
        nextState = BNO055_RESET_DELAY_2;
      }
      break;
    }
    case BNO055_RESET_DELAY_2:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if PWR_MGMT_COMP
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        ret_status = timerWaitUs_irq(POST_RESET_STARTUP_DELAY);
        if(ret_status == ERROR)
        {
          LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
          nextState = BNO055_READ_POST_RESET;
        }
       }
       break;
    }
    case BNO055_READ_POST_RESET:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_LETIMER0_COMP1)
      {
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
        LOG_INFO("BNO055_READ_POST_RESET\r");
#endif
        address_verification = false;
        address_verification = BNO055_VerifyIdentity(&bno055_rd_buff[0]);
        if(address_verification == true)
        {
#if PWR_MGMT_COMP
          //Enter EM2 mode
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
          ret_status = timerWaitUs_irq(BNO055_STD_DELAY);
          if(ret_status == ERROR)
          {
            LOG_ERROR("The value is more than the routine can provide\r");
          }
          else
          {
            nextState = BNO055_READ_POST_RESET_DELAY_3;
          }
        }
        else
        {
#if PWR_MGMT_COMP
          //Enter EM2 mode
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
          ret_status = timerWaitUs_irq(BNO055_STD_DELAY);
          if(ret_status == ERROR)
          {
            LOG_ERROR("The value is more than the routine can provide\r");
          }
          else
          {
            nextState = BNO055_READ_POST_RESET;
          }
        }
      }
      break;
    }
    case BNO055_READ_POST_RESET_DELAY_3:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_LETIMER0_COMP1)
      {
        ret_status = timerWaitUs_irq(POWER_MODE_DELAY);
        if(ret_status == ERROR)
        {
          LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
          nextState = BNO055_POWER_MODE_SET;
        }
      }
      break;
    }
    case BNO055_POWER_MODE_SET:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_LETIMER0_COMP1)
      {
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
        LOG_INFO("BNO055_POWER_MODE_SET\r");
#endif
        BNO055_write(BNO055_PWR_MODE_ADDR,POWER_MODE_LOW_POWER);
        nextState = BNO055_POWER_MODE_SET_DELAY_4;
      }
      break;
    }
    case BNO055_POWER_MODE_SET_DELAY_4:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if PWR_MGMT_COMP
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        ret_status = timerWaitUs_irq(BNO055_STD_DELAY);
        if(ret_status == -1)
        {
          LOG_ERROR("The value is more than the routine can provide \r");
        }
        else
        {
          nextState = BNO055_PAGE_ADDR;
        }
      }
      break;
    }
    case BNO055_PAGE_ADDR:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_LETIMER0_COMP1)
      {
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
        LOG_INFO("BNO055_PAGE_ADDR\r");
#endif
        BNO055_write(BNO055_PAGE_ID_ADDR,BNO055_PAGE_0);
        nextState = BNO055_PAGE_ADDR_DELAY_5;
      }
      break;
    }
    case BNO055_PAGE_ADDR_DELAY_5:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if PWR_MGMT_COMP
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        ret_status = timerWaitUs_irq(BNO055_STD_DELAY);
        if(ret_status == -1)
        {
          LOG_ERROR("The value is more than the routine can provide \r");
        }
        else
        {
          nextState = BNO055_SYS_TRIGGER;
        }
      }
      break;
    }
    case BNO055_SYS_TRIGGER:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_LETIMER0_COMP1)
      {
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
        LOG_INFO("BNO055_SYS_TRIGGER\r");
#endif
        BNO055_write(BNO055_SYS_TRIGGER_ADDR,0x00);
        nextState = BNO055_SYS_TRIGGER_DELAY_6;
      }
      break;
    }
    case BNO055_SYS_TRIGGER_DELAY_6:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if PWR_MGMT_COMP
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        ret_status = timerWaitUs_irq(BNO055_STD_DELAY);
        if(ret_status == -1)
        {
          LOG_ERROR("The value is more than the routine can provide \r");
        }
        else
        {
          nextState = BNO055_SET_REQ_MODE;
        }
      }
      break;
     }
    case BNO055_SET_REQ_MODE:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_LETIMER0_COMP1)
      {
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
        LOG_INFO("BNO055_SET_REQ_MODE\r");
#endif
        setMode(OPERATION_MODE_NDOF);
        nextState = BNO055_SET_REQ_MODE_DELAY_7;
      }
      break;
    }
    case BNO055_SET_REQ_MODE_DELAY_7:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if PWR_MGMT_COMP
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif

#if PWR_MGMT_RUN_MODE
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        ret_status = timerWaitUs_irq(POST_OPERATION_MODE_DELAY);
        if(ret_status == -1)
        {
          LOG_ERROR("The value is more than the routine can provide \r");
        }
        else
        {
#if 1
          //RESET the status of the machine
          nextState = BNO055_SETMODE;
          //next state to switch in Asset SM
          return_state = BME280_READ;
#endif
          //nextState = BNO055_SET_PAGE_ADDR_1;
        }
      }
      break;
     }
#if 0
    case BNO055_SET_PAGE_ADDR_1:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_LETIMER0_COMP1)
      {
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
        LOG_INFO("BNO055_SET_PAGE_ADDR_1\r");
#endif
        BNO055_write(BNO055_PAGE_ID_ADDR,BNO055_PAGE_1);
        nextState = BNO055_PAGE_ADDR_1_DELAY_8;
      }
      break;
    }
    case BNO055_PAGE_ADDR_1_DELAY_8:
    {
      //default state
      return_state = BNO055_INIT_CONFIG;
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if PWR_MGMT_COMP
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        ret_status = timerWaitUs_irq(BNO055_STD_DELAY);
        if(ret_status == -1)
        {
          LOG_ERROR("The value is more than the routine can provide \r");
        }
        else
        {
          nextState = BNO055_INT_MSK;
        }
      }
      break;
     }
      case BNO055_INT_MSK:
      {
        //default state
        return_state = BNO055_INIT_CONFIG;
        if(event == ev_LETIMER0_COMP1)
        {
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
  #if DEBUG_1
          LOG_INFO("BNO055_INT_MSK\r");
  #endif
          BNO055_write(INT_MSK,0x10);
          nextState = BNO055_INT_MSK_DELAY_9;
        }
        break;
      }
      case BNO055_INT_MSK_DELAY_9:
      {
        //default state
        return_state = BNO055_INIT_CONFIG;
        if(event == ev_I2C0_TRANSFER_DONE)
        {
  #if PWR_MGMT_COMP
          //Enter EM2 mode
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
  #endif
          ret_status = timerWaitUs_irq(BNO055_STD_DELAY);
          if(ret_status == -1)
          {
            LOG_ERROR("The value is more than the routine can provide \r");
          }
          else
          {
            nextState = BNO055_INT_PIN_SET;
          }
        }
        break;
      }
      case BNO055_INT_PIN_SET:
      {
        //default state
        return_state = BNO055_INIT_CONFIG;
        if(event == ev_LETIMER0_COMP1)
        {
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
  #if DEBUG_1
          LOG_INFO("BNO055_INT_PIN_SET\r");
  #endif
          BNO055_write(INT_EN,0x40);
          nextState = BNO055_INT_PIN_SET_DELAY_10;
        }
        break;
      }
  case BNO055_INT_PIN_SET_DELAY_10:
  {
    //default state
    return_state = BNO055_INIT_CONFIG;
    if(event == ev_I2C0_TRANSFER_DONE)
    {
#if PWR_MGMT_COMP
      //Enter EM2 mode
      sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
      ret_status = timerWaitUs_irq(BNO055_STD_DELAY);
      if(ret_status == -1)
      {
        LOG_ERROR("The value is more than the routine can provide \r");
      }
      else
      {
        //nextState = BNO055_INT_PIN_SET;
#if 0
          //RESET the status of the machine
          nextState = BNO055_SETMODE;
          //next state to switch in Asset SM
          return_state = BME280_READ;
#endif
      }
    }
    break;
  }

#endif
    default:
      break;
  }
  return return_state;
}

void BNO055_write(uint8_t reg, uint8_t byte_val)
{
  bno055_wr_buff[0] = reg;
  bno055_wr_buff[1] = byte_val;
#if PWR_MGMT_COMP
  //Enter EM1 mode
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  I2C0_Write(BNO055_ADDRESS_A,&bno055_wr_buff[0],2);
}

void setMode(uint8_t set_mode)
{
  bno055_wr_buff[0] = BNO055_OPR_MODE_ADDR;
  bno055_wr_buff[1] = set_mode;
#if PWR_MGMT_COMP
  //Enter EM1 mode
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  I2C0_Write(BNO055_ADDRESS_A,&bno055_wr_buff[0],2);
}

bool BNO055_VerifyIdentity(uint8_t* rd_buff)
{
  bno055_wr_buff[0] = BNO055_CHIP_ID_ADDR;
#if PWR_MGMT_COMP
  //Enter EM1 mode
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  I2C0_WriteRead(BNO055_ADDRESS_A, &bno055_wr_buff[0], 1, &rd_buff[0], 1);
  if(rd_buff[0] == BNO055_ID)
  {
    return true;
  }
  else
  {
    return false;
  }
}
