/*
 * bno055.c
 *
 *  Created on: Feb 17, 2022
 *      Author: risha
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
#define ERROR           (-1)
/*******************************************************************************
 Global
*******************************************************************************/

/*******************************************************************************
 Function Prototypes
*******************************************************************************/

/*******************************************************************************
 Function Definition
*******************************************************************************/
bool BNO055_VerifyIdentity(uint8_t* rd_buff);
void setMode(uint8_t set_mode);
void BNO055_write(uint8_t reg, uint8_t byte_val);

uint8_t bno055_wr_buff[8] = { 0 };
uint8_t bno055_rd_buff[8] = { 0 };

BNO055_state_t init_bno055_machine(sl_bt_msg_t *evt)
{
  ble_ext_signal_event_t event = evt->data.evt_system_external_signal.extsignals;
  /* return state logic */
  BNO055_state_t return_state = BNO055_DEFAULT;
  /* current machine logic */
  BNO055_state_t currentState;
  static BNO055_state_t nextState = BNO055_ADD_VERIFN;
  bool address_verification = false;
  int ret_status = 0;
  if (SL_BT_MSG_ID(evt->header) != sl_bt_evt_system_external_signal_id) {
      return return_state;
  }
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
      }
      break;
    }
    case BNO055_SETMODE:
    {
#if DEBUG_1
      LOG_INFO("BNO055_SETMODE\r");
#endif
      setMode(OPERATION_MODE_CONFIG);
      nextState = BNO055_SETMODE_DELAY_1;
      break;
    }
    case BNO055_SETMODE_DELAY_1:
    {
      if(event == ev_I2C0_TRANSFER_DONE)
      {
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
          displayPrintf(DISPLAY_ROW_X, "X = %d mg", x);
          displayPrintf(DISPLAY_ROW_Y, "Y = %d mg", y);
          displayPrintf(DISPLAY_ROW_Z, "Z = %d mg", z);

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

    default:
      break;
  }
  return return_state;
}

void BNO055_write(uint8_t reg, uint8_t byte_val)
{
  bno055_wr_buff[0] = reg;
  bno055_wr_buff[1] = byte_val;
  I2C0_Write(BNO055_ADDRESS_A,&bno055_wr_buff[0],2);
}

void setMode(uint8_t set_mode)
{
  bno055_wr_buff[0] = BNO055_OPR_MODE_ADDR;
  bno055_wr_buff[1] = set_mode;
  I2C0_Write(BNO055_ADDRESS_A,&bno055_wr_buff[0],2);
}

bool BNO055_VerifyIdentity(uint8_t* rd_buff)
{
    bno055_wr_buff[0] = BNO055_CHIP_ID_ADDR;
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


#if 0
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
#endif
