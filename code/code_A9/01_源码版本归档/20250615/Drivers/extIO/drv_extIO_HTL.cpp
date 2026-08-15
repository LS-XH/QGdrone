#include "drv_extIO_HTL.hpp"

#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "Parameters.hpp"
#include "StorageSystem.hpp"
#include "drv_PWMOut.hpp"
#include "drv_can.hpp"
//#include "drv_ExtIO.hpp"

static void extIO_HTL_Server(void *pvParameters)
{
    while (getInitializationCompleted() == false)
        os_delay(0.1);

    uint8_t port_id = 0;
    const Port *port = get_CommuPortByPortId(MAXPorts - 1, &port_id);
    while (port == 0)
    {
        os_delay(0.1);
        port = get_CommuPortByPortId(MAXPorts - 1, &port_id);
    }

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // 400hz
        vTaskDelayUntil(&xLastWakeTime, 2);

        extern volatile float PWM_CHANS[MAX_PWM_COUNT];
        if (mavlink_lock_chan(port_id, 0.01))
        {
            mavlink_message_t msg_sd;

            uint16_t chans[MAX_PWM_COUNT] = {0};
            for (uint8_t i = 0; i < MAX_PWM_COUNT; ++i)
            {
                chans[i] = PWM_CHANS[i] * 10 + 1000;
            }
            mavlink_msg_acflyhtl_pwmchans_fb_pack_chan(
                get_CommulinkSysId(),  // system id
                get_CommulinkCompId(), // component id
                port_id,               // chan
                &msg_sd,
                chans);
            mavlink_msg_to_send_buffer(port->write,
                port->lock,
                port->unlock,
                &msg_sd, 0, -1);
            mavlink_unlock_chan(port_id);
        }
    }
}

void init_drv_extIO_HTL()
{
    xTaskCreate(extIO_HTL_Server, "EXTIO_HTL", 1024, 0, SysPriority_ExtSensor, NULL);
}