/*
 * scheduler.c - Scheduler functions
 *
 *  Created on: Sep 12, 2021
 *      Author: vishn
 */

#include "scheduler.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

static event_t event_q[EVENT_QUEUE_SIZE];
static uint8_t rd_ptr = 0;
static uint8_t wr_ptr = 0;
static uint8_t q_len = 0;



/*
 * Enqueues event onto event queue
 *
 * @param event - Event to be enqueued
 *
 * @return None
 */
static void EventQ_EnqueueEvent(event_t event);


/*
 * Dequeues event from event queue
 *
 * @param None
 *
 * @return Next event
 */
static event_t EventQ_DequeueEvent(void);


static void EventQ_EnqueueEvent(event_t event) {
    if (q_len == EVENT_QUEUE_SIZE) {
        return;
    }
    event_q[(wr_ptr++) & EVENT_QUEUE_SIZE_MASK] = event;
    q_len++;
}


static event_t EventQ_DequeueEvent(void) {
    event_t current_event;
    if (q_len == 0) {
        current_event = ev_NONE;
    }
    else {
        current_event = event_q[(rd_ptr++) & EVENT_QUEUE_SIZE_MASK];
        q_len--;
    }
    return current_event;
}


void Scheduler_SetEvent_LETIMER0_UF(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    EventQ_EnqueueEvent(ev_LETIMER0_UF);
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_LETIMER0_COMP1(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    EventQ_EnqueueEvent(ev_LETIMER0_COMP1);
    CORE_EXIT_CRITICAL();
}


void Scheduler_SetEvent_I2C0_TRANSFER_DONE(void) {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    EventQ_EnqueueEvent(ev_I2C0_TRANSFER_DONE);
    CORE_EXIT_CRITICAL();
}


event_t Scheduler_GetNextEvent(void) {
    event_t current_event;
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    current_event = EventQ_DequeueEvent();
    CORE_EXIT_CRITICAL();
    return current_event;
}


void PowerUp(void) {
    I2C0_Init();
    I2C0_Enable(true);
    timerWaitUs_irq(TEMP_SENSOR_POWER_ON_WAIT_US);
}


void SendReadTempCommand(void) {
    I2C_TransferReturn_TypeDef temp_sensor_transfer_ret;

    LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);
    sl_power_manager_add_em_requirement(EM1);

    temp_sensor_transfer_ret = I2C0_SendCommand(CMD_READ_TEMP);
    /*if (temp_sensor_transfer_ret != i2cTransferDone) {
        LOG_ERROR("%d\r\n", temp_sensor_transfer_ret);
    }*/
}


void WaitForTempSensorReading(void) {
    sl_power_manager_remove_em_requirement(EM1);
    I2C0_DisableIntForTransfer();

    timerWaitUs_irq(TEMP_SENSOR_READ_TEMP_WAIT_US);
}


void RequestTempSensorReading(void) {
    I2C_TransferReturn_TypeDef temp_sensor_transfer_ret;

    LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);
    sl_power_manager_add_em_requirement(EM1);

    temp_sensor_transfer_ret = I2C0_RequestRead();
    /*if (temp_sensor_transfer_ret != i2cTransferDone) {
        LOG_ERROR("%d\r\n", temp_sensor_transfer_ret);
    }*/
}


void ReadTempSensorReading(void) {
    uint8_t read_buff[2];
    int16_t temperature_code;
    int16_t temperature_C;

    sl_power_manager_remove_em_requirement(EM1);
    I2C0_DisableIntForTransfer();

    I2C0_ReadBytes(read_buff, 2);
    I2C0_Enable(false);

    temperature_code = (read_buff[0] << 8) | read_buff[1];
    temperature_C = (175.25 * temperature_code) / 65536.0 - 46.85; //calculation taken from data sheet

    LOG_INFO("%d\r\n", temperature_C);
    I2C0_Teardown();
}
