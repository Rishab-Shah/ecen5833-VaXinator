/*
 * bme280.c
 *
 *  Created on: Feb 24, 2022
 *      Author: rishab
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

/*******************************************************************************
 Function Prototypes
*******************************************************************************/

/*******************************************************************************
 Function Definition
*******************************************************************************/
bool BME280_VerifyIdentity(uint8_t* rd_buff);

uint8_t bme280_wr_buff[8] = { 0 };
uint8_t bme280_rd_buff[8] = { 0 };

BME280_state_t init_bme280_machine(sl_bt_msg_t *evt)
{
  ble_ext_signal_event_t event = evt->data.evt_system_external_signal.extsignals;
  /* return state logic */
  BME280_state_t return_state = BME280_DEFAULT;
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
#if DEBUG_1
      LOG_INFO("BME280_REG_SOFTRESET\r");
#endif
      BME280_write(BME280_REGISTER_SOFTRESET,0xB6);
      nextState = BME280_REG_SOFTRESET_DELAY_1;
      break;
    }
    case BME280_REG_SOFTRESET_DELAY_1:
    {
      if(event == ev_I2C0_TRANSFER_DONE)
      {
        ret_status = timerWaitUs_irq(STD_DELAY);
        if(ret_status == ERROR)
        {
          LOG_ERROR("The value is more than the routine can provide\r");
        }
        else
        {
          nextState = BME280_READ_CALIB;
        }
      }
      break;
    }
    case BME280_READ_CALIB:
    {
      if(event == ev_LETIMER0_COMP1)
      {
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
          LOG_INFO("BME280_READ_CALIB\r");
#endif
         // BME280_read(BNO055_SYS_TRIGGER_ADDR,0x20);
          nextState = BME280_ADD_VERIFN;
      }
      break;
    }
#if 0
    case BNO055_RESET_DELAY_2:
    {
      if(event == ev_I2C0_TRANSFER_DONE)
      {
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
       ret_status = timerWaitUs_irq(NORMAL_MODE_DELAY);
       if(ret_status == ERROR)
       {
           LOG_ERROR("The value is more than the routine can provide\r");
       }
       else
       {
           nextState = BNO055_NORMAL_MODE_SET;
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
          LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
#if DEBUG_1
          LOG_INFO("READ_XYZ_DATA\r");
#endif
          memset(bno055_rd_buff,0,sizeof(bno055_rd_buff));
          bno055_wr_buff[0] = BNO055_EULER_H_LSB_ADDR;
          I2C0_WriteRead(BNO055_ADDRESS_A, &bno055_wr_buff[0], 1, &bno055_rd_buff[0], 6);
          nextState = READ_XYZ_DATA_DELAY;
      }

      break;
    }
    case READ_XYZ_DATA_DELAY:
    {
      if(event == ev_I2C0_TRANSFER_DONE)
      {
          //processing
          int16_t x = 0,y = 0, z= 0;
          x = ((int16_t)(bno055_rd_buff[0]) | (int16_t)(bno055_rd_buff[1] << 8));
          y = ((int16_t)(bno055_rd_buff[2]) | (int16_t)(bno055_rd_buff[3] << 8));
          z = ((int16_t)(bno055_rd_buff[4]) | (int16_t)(bno055_rd_buff[5] << 8));

          LOG_INFO("X= %d::Y=%d::Z=%d\r",x,y,z);

          ret_status = timerWaitUs_irq((500)*(1000));
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
#endif

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
