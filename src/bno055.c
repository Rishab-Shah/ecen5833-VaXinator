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
BNO055_state_t init_bno055_machine(sl_bt_msg_t *evt)
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
  BNO055_state_t return_state = BNO055_DEFAULT;
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
#if DEBUG_1
      LOG_INFO("BNO055_SETMODE\r");
#endif
      if(event == ev_LETIMER0_UF)
      {
        setMode(OPERATION_MODE_CONFIG);
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
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
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
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
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
#if POWER_MANAGEMENT
          //Enter EM2 mode
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
          ret_status = timerWaitUs_irq(STD_DELAY);
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
#if POWER_MANAGEMENT
          //Enter EM2 mode
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
          ret_status = timerWaitUs_irq(STD_DELAY);
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
      if(event == ev_LETIMER0_COMP1)
      {
        ret_status = timerWaitUs_irq(NORMAL_MODE_DELAY);
        if(ret_status == ERROR)
        {
          LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
          nextState = BNO055_NORMAL_MODE_SET;
        }
      }
      break;
    }
    case BNO055_NORMAL_MODE_SET:
    {
      if(event == ev_LETIMER0_COMP1)
      {
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
        LOG_INFO("BNO055_NORMAL_MODE_SET\r");
#endif
        BNO055_write(BNO055_PWR_MODE_ADDR,POWER_MODE_NORMAL);
        nextState = BNO055_NORMAL_MODE_SET_DELAY_4;
      }
      break;
    }
    case BNO055_NORMAL_MODE_SET_DELAY_4:
    {
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        ret_status = timerWaitUs_irq(STD_DELAY);
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
      if(event == ev_LETIMER0_COMP1)
      {
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
        LOG_INFO("BNO055_PAGE_ADDR\r");
#endif
        BNO055_write(BNO055_PAGE_ID_ADDR,0x00);
        nextState = BNO055_PAGE_ADDR_DELAY_5;
      }
      break;
    }
    case BNO055_PAGE_ADDR_DELAY_5:
    {
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        ret_status = timerWaitUs_irq(STD_DELAY);
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
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        ret_status = timerWaitUs_irq(STD_DELAY);
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
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        ret_status = timerWaitUs_irq((1000)*(1000));
        if(ret_status == -1)
        {
          LOG_ERROR("The value is more than the routine can provide \r");
        }
        else
        {
          nextState = READ_XYZ_DATA;
        }
      }
      break;
     }
    case READ_XYZ_DATA:
    {
      if(event == ev_LETIMER0_COMP1)
      {
        gpioDebugLEDSetOn();
        LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
        LOG_INFO("READ_XYZ_DATA\r");
#endif
        memset(bno055_rd_buff,0,sizeof(bno055_rd_buff));
        bno055_wr_buff[0] = BNO055_EULER_H_LSB_ADDR;
#if POWER_MANAGEMENT
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
      if(event == ev_I2C0_TRANSFER_DONE)
      {
#if POWER_MANAGEMENT
        //Enter EM2 mode
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
#endif
        gpioDebugLEDSetOff();
        //processing
        int16_t x = 0,y = 0, z= 0;
        x = ((int16_t)(bno055_rd_buff[0]) | (int16_t)(bno055_rd_buff[1] << 8));
        y = ((int16_t)(bno055_rd_buff[2]) | (int16_t)(bno055_rd_buff[3] << 8));
        z = ((int16_t)(bno055_rd_buff[4]) | (int16_t)(bno055_rd_buff[5] << 8));

        LOG_INFO("X= %d::Y=%d::Z=%d\r",x,y,z);
#if NO_BL
        if(ble_data->s_AccelIndication && ble_data->s_ClientConnected)
        {
          LOG_INFO("%x %x %x %x %x %x\r", bno055_rd_buff[0], bno055_rd_buff[1], bno055_rd_buff[2], bno055_rd_buff[3], bno055_rd_buff[4], bno055_rd_buff[5]);
          BleServer_SendAccelDataToClient(&bno055_rd_buff[0]);
        }
#endif
        ret_status = timerWaitUs_irq((1000)*(1000));
        if(ret_status == -1)
        {
          LOG_ERROR("The value is more than the routine can provide \r");
        }
        else
        {
          nextState = READ_XYZ_DATA;
        }
      }
      break;
    }

    default:
      break;
  }
  return return_state;
}

void BNO055_write(uint8_t reg, uint8_t byte_val)
{
  bno055_wr_buff[0] = reg;
  bno055_wr_buff[1] = byte_val;
#if POWER_MANAGEMENT
  //Enter EM1 mode
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  I2C0_Write(BNO055_ADDRESS_A,&bno055_wr_buff[0],2);
}

void setMode(uint8_t set_mode)
{
  bno055_wr_buff[0] = BNO055_OPR_MODE_ADDR;
  bno055_wr_buff[1] = set_mode;
#if POWER_MANAGEMENT
  //Enter EM1 mode
  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#endif
  I2C0_Write(BNO055_ADDRESS_A,&bno055_wr_buff[0],2);
}

bool BNO055_VerifyIdentity(uint8_t* rd_buff)
{
  bno055_wr_buff[0] = BNO055_CHIP_ID_ADDR;
#if POWER_MANAGEMENT
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
