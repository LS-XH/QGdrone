#include "drv_CAN_Hobbywing_ESC.hpp"

#include "AuxFuncs.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "MavlinkCMDProcess.hpp"
#include "MeasurementSystem.hpp"
#include "Modes.hpp"
#include "Parameters.hpp"
#include "SensorsBackend.hpp"
#include "StorageSystem.hpp"
#include "dgtPwm.hpp"
#include "drv_CAN.hpp"
#include "drv_PWMOut.hpp"
#include "task.h"
#include <string.h>

struct DriverInfo
{
    CanMailBox *mail_box;
    uint32_t sensor_key;
};

#define fc_can_id (10)

typedef enum
{ // 优先级
    Esc_Priority_Highest = 0x0,
    Esc_Priority_High = 0x8,
    Esc_Priority_Medium = 0x1,
    Esc_Priority_Low = 0x18,
    Esc_Priority_Lowest = 0x1f
} _EscPriority;

typedef struct
{                              // 广播帧
    uint8_t Soure_node_id : 7; // 自身节点id 取值为1~127 其中126~127为保留id
    uint8_t notBroadcast : 1;  // 数据帧类型 0为message帧 1为service帧
    uint16_t Message_type_id;  // 消息id 0~65535
    uint8_t Priority : 5;      // 优先级 HIGHEST=0x0 HIGH=0x8 MEDIUM=0x1 LOW=0x18 LOWEST=0x1f
    uint8_t none : 3;          // 空 0
} __PACKED _EscMessageFrame;

typedef struct
{                                     // 非广播帧
    uint8_t Soure_node_id : 7;        // 自身节点id 取值为1~127 其中0、126~127为保留id
    uint8_t notBroadcast : 1;         // 数据帧类型 0为message帧 1为service帧
    uint8_t Destination_node_id : 7;  // 对方节点id 取值为1~127 其中0、126~127为保留id
    uint8_t Request_not_response : 1; // 数据帧类型 0为应答 1为请求
    uint8_t Service_type_id;          // 消息id 0~255
    uint8_t Priority : 5;             // 优先级 HIGHEST=0x0 HIGH=0x8 MEDIUM=0x1 LOW=0x18 LOWEST=0x1f
    uint8_t none : 3;                 // 空 0
} __PACKED _EscServiceFrame;

typedef struct
{                                  // 尾部数据
    uint8_t Transfer_id : 5;       // 对于相同id的数据，每发一包数据加一，0~31循环累加；同一包数据中的多帧该值不变
    uint8_t Toggle_bit : 1;        // 单帧传输=0；多帧传输，首帧=0，此后每发一帧翻转一次
    uint8_t End_of_transfer : 1;   // 单帧传输=1；多帧传输，最后一帧=1，否则=0
    uint8_t Start_of_transfer : 1; // 单帧传输=1；多帧传输，首帧=1，否则=0
} __PACKED _EscTailByte;

static uint16_t crcAddByte(uint16_t crc_val, uint8_t byte)
{
    crc_val ^= (uint16_t)((uint16_t)(byte) << 8);
    for (int j = 0; j < 8; j++)
    {
        if (crc_val & 0x8000U)
        {
            crc_val = (uint16_t)((uint16_t)(crc_val << 1) ^ 0x1021U);
        }
        else
        {
            crc_val = (uint16_t)(crc_val << 1);
        }
    }
    return crc_val;
}

static uint16_t crcAddSignature(uint16_t crc_val, uint64_t data_type_signature)
{
    for (int shift_val = 0; shift_val < 64; shift_val += 8)
    {
        crc_val = crcAddByte(crc_val, (uint8_t)(data_type_signature >> shift_val));
    }
    return crc_val;
}
static uint16_t crcAdd(uint16_t crc_val, const uint8_t *bytes, size_t len)
{
    while (len--)
    {
        crc_val = crcAddByte(crc_val, *bytes++);
    }
    return crc_val;
}

static void CAN_Hobbywing_ESC_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *)pvParameters;
    delete (DriverInfo *)pvParameters;

    // 获取校准电调id
    DGT_PWM_CONFIG escConfig = {0};
    ReadParamGroup("DGTPWM", (uint64_t *)&escConfig, 0);

// PWM设置
#define PWM_Db_Down 5

    uint16_t pwm_send_counter = 0;
    uint16_t param_send_counter = 0;
    uint16_t param_counter = 0;

    TIME lastGcsSdTIME = TIME::now();
    float maxTemp = -300;
    uint32_t currentGBits = 0;
    uint32_t lastGBits = 0;
    float totalCurrent = 0;
    TIME lastErrSdTIME = TIME::now();

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        // 800hz
        vTaskDelayUntil(&xLastWakeTime, 1);

        if (++param_counter >= 200)
        {
            param_counter = 0;
            ReadParamGroup("DGTPWM", (uint64_t *)&escConfig, 0);
        }

        bool calibrating = false;
        bool attCtrlEna;
        is_Attitude_Control_Enabled(&attCtrlEna);
        if (!attCtrlEna && escConfig.calib[0] != 0)
        { // 校准
            calibrating = true;
            if (++param_send_counter >= 5)
            {
                param_send_counter = 0;

                switch (escConfig.calib[0])
                {
                    case 1:
                    { // 设置id
                        CanPacket mail;
                        mail.IdType = 1;
                        mail.FrameType = 0;
                        mail.FDFormat = 0;

                        _EscServiceFrame frame;
                        frame.none = 0;
                        frame.Priority = Esc_Priority_Highest;
                        frame.Service_type_id = 210;
                        frame.notBroadcast = 1;
                        frame.Request_not_response = 1;
                        frame.Destination_node_id = 0;
                        frame.Soure_node_id = fc_can_id;
                        mail.Identifier = *(uint32_t *)&frame;

                        _EscTailByte tail;
                        tail.Start_of_transfer = 1;
                        tail.End_of_transfer = 1;
                        tail.Toggle_bit = 0;
                        tail.Transfer_id = 0;

                        mail.DataLength = 3;
                        mail.data[0] = escConfig.calib_id[0];     // 电调id
                        mail.data[1] = escConfig.calib_param1[0]; // 油门id
                        mail.data[2] = *(uint8_t *)&tail;

                        driver_info.mail_box->SendMail(mail);
                        break;
                    }
                    case 2:
                    { // 设置id(特定id)
                        CanPacket mail;
                        mail.IdType = 1;
                        mail.FrameType = 0;
                        mail.FDFormat = 0;

                        _EscServiceFrame frame;
                        frame.none = 0;
                        frame.Priority = Esc_Priority_Highest;
                        frame.Service_type_id = 220;
                        frame.notBroadcast = 1;
                        frame.Request_not_response = 1;
                        frame.Destination_node_id = escConfig.calib_id[0];
                        frame.Soure_node_id = fc_can_id;
                        mail.Identifier = *(uint32_t *)&frame;

                        _EscTailByte tail;
                        tail.Start_of_transfer = 1;
                        tail.End_of_transfer = 1;
                        tail.Toggle_bit = 0;
                        tail.Transfer_id = 0;

                        mail.DataLength = 3;
                        mail.data[0] = escConfig.calib_param1[0];
                        mail.data[1] = escConfig.calib_param2[0];
                        mail.data[2] = *(uint8_t *)&tail;

                        driver_info.mail_box->SendMail(mail);
                        break;
                    }

                    case 3:
                    { // 设置默认油门源为CAN
                        if (escConfig.calib_id[0] == 0)
                        { // 全部设置
                            CanPacket mail;
                            mail.IdType = 1;
                            mail.FrameType = 0;
                            mail.FDFormat = 0;
                            for (int i = 0x1; i <= 0x7d; i++)
                            { // 遍历所有ID
                                _EscServiceFrame frame;
                                frame.none = 0;
                                frame.Priority = Esc_Priority_Highest;
                                frame.Service_type_id = 215;
                                frame.notBroadcast = 1;
                                frame.Request_not_response = 1;
                                frame.Destination_node_id = i;
                                frame.Soure_node_id = fc_can_id;
                                mail.Identifier = *(uint32_t *)&frame;

                                _EscTailByte tail;
                                tail.Start_of_transfer = 1;
                                tail.End_of_transfer = 1;
                                tail.Toggle_bit = 0;
                                tail.Transfer_id = 0;

                                mail.DataLength = 2;
                                mail.data[0] = 0; // 0-CAN 1-PWM
                                mail.data[1] = *(uint8_t *)&tail;

                                driver_info.mail_box->SendMail(mail);
                            }
                        }
                        else
                        {
                            CanPacket mail;
                            mail.IdType = 1;
                            mail.FrameType = 0;
                            mail.FDFormat = 0;

                            _EscServiceFrame frame;
                            frame.none = 0;
                            frame.Priority = Esc_Priority_Highest;
                            frame.Service_type_id = 215;
                            frame.notBroadcast = 1;
                            frame.Request_not_response = 1;
                            frame.Destination_node_id = escConfig.calib_id[0];
                            frame.Soure_node_id = fc_can_id;
                            mail.Identifier = *(uint32_t *)&frame;

                            _EscTailByte tail;
                            tail.Start_of_transfer = 1;
                            tail.End_of_transfer = 1;
                            tail.Toggle_bit = 0;
                            tail.Transfer_id = 0;

                            mail.DataLength = 2;
                            mail.data[0] = 0; // 0-CAN 1-PWM
                            mail.data[1] = *(uint8_t *)&tail;

                            driver_info.mail_box->SendMail(mail);
                        }
                        break;
                    }

                    case 4:
                    { // 设置默认油门源为PWM
                        if (escConfig.calib_id[0] == 0)
                        { // 全部设置
                            CanPacket mail;
                            mail.IdType = 1;
                            mail.FrameType = 0;
                            mail.FDFormat = 0;
                            for (int i = 0x1; i <= 0x7d; i++)
                            { // 遍历所有ID
                                _EscServiceFrame frame;
                                frame.none = 0;
                                frame.Priority = Esc_Priority_Highest;
                                frame.Service_type_id = 215;
                                frame.notBroadcast = 1;
                                frame.Request_not_response = 1;
                                frame.Destination_node_id = i;
                                frame.Soure_node_id = fc_can_id;
                                mail.Identifier = *(uint32_t *)&frame;

                                _EscTailByte tail;
                                tail.Start_of_transfer = 1;
                                tail.End_of_transfer = 1;
                                tail.Toggle_bit = 0;
                                tail.Transfer_id = 0;

                                mail.DataLength = 2;
                                mail.data[0] = 1; // 0-CAN 1-PWM
                                mail.data[1] = *(uint8_t *)&tail;

                                driver_info.mail_box->SendMail(mail);
                            }
                        }
                        else
                        {
                            CanPacket mail;
                            mail.IdType = 1;
                            mail.FrameType = 0;
                            mail.FDFormat = 0;

                            _EscServiceFrame frame;
                            frame.none = 0;
                            frame.Priority = Esc_Priority_Highest;
                            frame.Service_type_id = 215;
                            frame.notBroadcast = 1;
                            frame.Request_not_response = 1;
                            frame.Destination_node_id = escConfig.calib_id[0];
                            frame.Soure_node_id = fc_can_id;
                            mail.Identifier = *(uint32_t *)&frame;

                            _EscTailByte tail;
                            tail.Start_of_transfer = 1;
                            tail.End_of_transfer = 1;
                            tail.Toggle_bit = 0;
                            tail.Transfer_id = 0;

                            mail.DataLength = 2;
                            mail.data[0] = 1; // 0-CAN 1-PWM
                            mail.data[1] = *(uint8_t *)&tail;

                            driver_info.mail_box->SendMail(mail);
                        }
                        break;
                    }

                    case 5:
                    { // 设置波特率
                        if (escConfig.calib_id[0] == 0)
                        { // 全部设置
                            CanPacket mail;
                            mail.IdType = 1;
                            mail.FrameType = 0;
                            mail.FDFormat = 0;
                            for (int i = 0x1; i <= 0x7d; i++)
                            { // 遍历所有ID
                                _EscServiceFrame frame;
                                frame.none = 0;
                                frame.Priority = Esc_Priority_Highest;
                                frame.Service_type_id = 211;
                                frame.notBroadcast = 1;
                                frame.Request_not_response = 1;
                                frame.Destination_node_id = i;
                                frame.Soure_node_id = fc_can_id;
                                mail.Identifier = *(uint32_t *)&frame;

                                _EscTailByte tail;
                                tail.Start_of_transfer = 1;
                                tail.End_of_transfer = 1;
                                tail.Toggle_bit = 0;
                                tail.Transfer_id = 0;

                                mail.DataLength = 2;
                                mail.data[0] = escConfig.calib_param1[0];
                                mail.data[1] = *(uint8_t *)&tail;

                                driver_info.mail_box->SendMail(mail);
                            }
                        }
                        else
                        {
                            CanPacket mail;
                            mail.IdType = 1;
                            mail.FrameType = 0;
                            mail.FDFormat = 0;

                            _EscServiceFrame frame;
                            frame.none = 0;
                            frame.Priority = Esc_Priority_Highest;
                            frame.Service_type_id = 211;
                            frame.notBroadcast = 1;
                            frame.Request_not_response = 1;
                            frame.Destination_node_id = escConfig.calib_id[0];
                            frame.Soure_node_id = fc_can_id;
                            mail.Identifier = *(uint32_t *)&frame;

                            _EscTailByte tail;
                            tail.Start_of_transfer = 1;
                            tail.End_of_transfer = 1;
                            tail.Toggle_bit = 0;
                            tail.Transfer_id = 0;

                            mail.DataLength = 2;
                            mail.data[0] = escConfig.calib_param1[0];
                            mail.data[1] = *(uint8_t *)&tail;

                            driver_info.mail_box->SendMail(mail);
                        }
                        break;
                    }

                    case 6:
                    { // 设置速率
                        CanPacket mail;
                        mail.IdType = 1;
                        mail.FrameType = 0;
                        mail.FDFormat = 0;

                        _EscServiceFrame frame;
                        frame.none = 0;
                        frame.Priority = Esc_Priority_Highest;
                        frame.Service_type_id = 214;
                        frame.notBroadcast = 1;
                        frame.Request_not_response = 1;
                        frame.Destination_node_id = escConfig.calib_id[0];
                        frame.Soure_node_id = fc_can_id;
                        mail.Identifier = *(uint32_t *)&frame;

                        _EscTailByte tail;
                        tail.Start_of_transfer = 1;
                        tail.End_of_transfer = 1;
                        tail.Toggle_bit = 0;
                        tail.Transfer_id = 0;

                        mail.DataLength = 5;
                        mail.data[0] = 1;
                        mail.data[1] = escConfig.calib_param1[0] & 0xff;
                        mail.data[2] = escConfig.calib_param1[0] >> 8;
                        mail.data[3] = escConfig.calib_param2[0];
                        mail.data[4] = *(uint8_t *)&tail;

                        driver_info.mail_box->SendMail(mail);
                        break;
                    }

                    case 21:
                    { // 关闭回传
                        CanPacket mail;
                        mail.IdType = 1;
                        mail.FrameType = 0;
                        mail.FDFormat = 0;

                        _EscMessageFrame frame;
                        frame.none = 0;
                        frame.Priority = Esc_Priority_Highest;
                        frame.Message_type_id = 20010;
                        frame.notBroadcast = 0;
                        frame.Soure_node_id = fc_can_id;
                        mail.Identifier = *(uint32_t *)&frame;

                        _EscTailByte tail;
                        tail.Start_of_transfer = 1;
                        tail.End_of_transfer = 1;
                        tail.Toggle_bit = 0;
                        tail.Transfer_id = 0;

                        mail.DataLength = 6;
                        mail.data[0] = 0;
                        uint32_t *command = (uint32_t *)&mail.data[1];
                        *command = 0x55555555;
                        mail.data[5] = *(uint8_t *)&tail;

                        driver_info.mail_box->SendMail(mail);
                        break;
                    }
                    case 22:
                    { // 开启回传
                        CanPacket mail;
                        mail.IdType = 1;
                        mail.FrameType = 0;
                        mail.FDFormat = 0;

                        _EscMessageFrame frame;
                        frame.none = 0;
                        frame.Priority = Esc_Priority_Highest;
                        frame.Message_type_id = 20010;
                        frame.notBroadcast = 0;
                        frame.Soure_node_id = fc_can_id;
                        mail.Identifier = *(uint32_t *)&frame;

                        _EscTailByte tail;
                        tail.Start_of_transfer = 1;
                        tail.End_of_transfer = 1;
                        tail.Toggle_bit = 0;
                        tail.Transfer_id = 0;

                        mail.DataLength = 6;
                        mail.data[0] = 0;
                        uint32_t *command = (uint32_t *)&mail.data[1];
                        *command = 0xaaaaaaaa;
                        mail.data[5] = *(uint8_t *)&tail;

                        driver_info.mail_box->SendMail(mail);
                        break;
                    }
                }
            }
        }

        if (++pwm_send_counter >= 4)
        {
            pwm_send_counter = 0;

//            CanPacket mail;
//            mail.IdType = 1;
//            mail.FrameType = 0;
//            mail.FDFormat = 0;

//            _EscMessageFrame frame;
//            frame.none = 0;
//            frame.Priority = Esc_Priority_Highest;
//            frame.Message_type_id = 20100;
//            frame.notBroadcast = 0;
//            frame.Soure_node_id = fc_can_id;
//            mail.Identifier = *(uint32_t *)&frame;

//            extern float PWM_CHANS[MAX_PWM_COUNT];
//            uint8_t src[16] = {0};
//            uint8_t throttle_data[14] = {0};
//            int index = 0;
//            for (int i = 0; i < 8; ++i)
//            {
//                int16_t throttle;
//                if (PWM_CHANS[i] >= -20 && PWM_CHANS[i] <= 120)
//                {
//                    if (PWM_CHANS[i] > PWM_Db_Down)
//                        throttle = (PWM_CHANS[i] - PWM_Db_Down) * (8191.0 / (100 - PWM_Db_Down));
//                    else
//                        throttle = 0;
//                    throttle = constrain(throttle, (int16_t)0, (int16_t)8191);
//                }
//                else
//                    throttle = 0;
//                if (calibrating)
//                    throttle = 0;

//                src[index] = (throttle & 0xff);
//                index++;
//                src[index] = ((throttle >> 8) & 0xff);
//                index++;
//            }

//            //			uint8_t bit_count = 0;
//            //			index = 0;
//            //			for( uint8_t i=0; i<16; ++i )
//            //			{
//            //				uint8_t r_data;
//            //				uint8_t new_bits;
//            //				if( i & 1 )
//            //				{
//            //					new_bits = 6;
//            //					r_data = src[i] & 0x3f;
//            //				}
//            //				else
//            //				{
//            //					new_bits = 8;
//            //					r_data = src[i];
//            //				}
//            //
//            //				uint8_t rm_bits = 8 - bit_count;
//            //				uint8_t wrt_bits = rm_bits < new_bits ? rm_bits : new_bits;
//            //				throttle_data[index] = (throttle_data[index] << wrt_bits) | (r_data >> (new_bits-wrt_bits));
//            //
//            //				bit_count += new_bits;
//            //				if( bit_count >= 8 )
//            //				{
//            //					++index;
//            //					bit_count -= 8;
//            //					//throttle_data[index] = r_data & ((1<<bit_count)-1);
//            //					throttle_data[index] = r_data;
//            //				}
//            //			}

//            throttle_data[0] = src[0];
//            throttle_data[1] = (src[1] & 0x3f) << 2 | (src[2] >> 6);
//            throttle_data[2] = (src[2] & 0x3f) << 2 | (src[3] & 0x3f) >> 4;
//            throttle_data[3] = (src[3] & 0xf) << 4 | (src[4] & 0xf0) >> 4;
//            throttle_data[4] = (src[4] & 0xf) << 4 | (src[5] & 0x3f) >> 2;
//            throttle_data[5] = (src[5] & 0x3) << 6 | (src[6] & 0xfc) >> 2;
//            throttle_data[6] = (src[6] & 0x3) << 6 | (src[7] & 0x3f);

//            throttle_data[7] = src[8];
//            throttle_data[8] = (src[9] & 0x3f) << 2 | (src[10] >> 6);
//            throttle_data[9] = (src[10] & 0x3f) << 2 | (src[11] & 0x3f) >> 4;
//            throttle_data[10] = (src[11] & 0xf) << 4 | (src[12] & 0xf0) >> 4;
//            throttle_data[11] = (src[12] & 0xf) << 4 | (src[13] & 0x3f) >> 2;
//            throttle_data[12] = (src[13] & 0x3) << 6 | (src[14] & 0xfc) >> 2;
//            throttle_data[13] = (src[14] & 0x3) << 6 | (src[15] & 0x3f);

//            // crc
//            uint16_t signatureCrc = crcAddSignature(0xffff, 0xbdf086c79f6640ad);
//            uint16_t crc = crcAdd(signatureCrc, (uint8_t *)&throttle_data[0], 14);

//            _EscTailByte tail;
//            // 多帧第一包
//            mail.DataLength = 8;
//            mail.data[0] = crc & 0xff;
//            mail.data[1] = (crc & 0xff00) >> 8;
//            memcpy((uint8_t *)&mail.data[2], (const uint8_t *)&throttle_data[0], 5);

//            tail.Start_of_transfer = 1;
//            tail.End_of_transfer = 0;
//            tail.Toggle_bit = 0;
//            tail.Transfer_id = 0;
//            mail.data[7] = *(uint8_t *)&tail;
//            driver_info.mail_box->SendMail(mail);
//            // 多帧第二包
//            mail.DataLength = 8;
//            memcpy((uint8_t *)&mail.data[0], (const uint8_t *)&throttle_data[5], 7);

//            tail.Start_of_transfer = 0;
//            tail.End_of_transfer = 0;
//            tail.Toggle_bit = 1;
//            tail.Transfer_id = 0;
//            mail.data[7] = *(uint8_t *)&tail;
//            driver_info.mail_box->SendMail(mail);
//            // 多帧第三包
//            mail.DataLength = 3;
//            memcpy((uint8_t *)&mail.data[0], (const uint8_t *)&throttle_data[12], 2);

//            tail.Start_of_transfer = 0;
//            tail.End_of_transfer = 1;
//            tail.Toggle_bit = 0;
//            tail.Transfer_id = 0;
//            mail.data[2] = *(uint8_t *)&tail;
//            driver_info.mail_box->SendMail(mail);
        }

        CanPacket mail;
        if (driver_info.mail_box->receiveMail(&mail, 0))
        {
            _EscMessageFrame *frame = (_EscMessageFrame *)&mail.Identifier;

            switch (frame->Message_type_id)
            {
                case 20050:
                { // MSG1
                    //					if(esc_calib_id[0] == frame->Soure_node_id)
                    //					{
                    //						sendLedSignal(LEDSignal_Success1);
                    //					}

                    struct EscMSG1
                    {                    // MSG1
                        uint16_t RPM;    // 转速
                        uint16_t PWM;    // 0~8191
                        uint16_t Status; // 电调运行状态
                    } __PACKED;
                    EscMSG1 *msg = (EscMSG1 *)&mail.data[0];

                    if (attCtrlEna)
                    {
                        double logMsg[3];
                        logMsg[0] = msg->RPM;
                        logMsg[1] = (msg->PWM * ((100 - PWM_Db_Down) / 8191.0) + PWM_Db_Down) * 10 + 1000;
                        logMsg[2] = msg->Status;

                        char logName[16];
                        sprintf(logName, "ESC%dSta1", frame->Soure_node_id);
                        SDLog_Msg_DebugVect(logName, logMsg, 3);
                    }
                    break;
                }
                case 20051:
                { // MSG2
                    struct EscMSG2
                    {                         // MSG2
                        uint16_t voltage;     // 输入电压 单位V 实际电压 = voltage/10
                        uint16_t current;     // 母线电流 单位A 实际电流 = current/10
                        uint16_t temperature; // 温度 单位℃
                    } __PACKED;
                    EscMSG2 *msg = (EscMSG2 *)&mail.data[0];

                    if ((currentGBits & (1 << frame->Soure_node_id)) == 0)
                    {
                        currentGBits |= (1 << frame->Soure_node_id);
                        totalCurrent += msg->current * 0.1f;
                    }

                    if (attCtrlEna)
                    {
                        double logMsg[3];
                        logMsg[0] = msg->voltage * 0.1;
                        logMsg[1] = msg->current * 0.1;
                        logMsg[2] = msg->temperature;

                        char logName[16];
                        sprintf(logName, "ESC%dSta2", frame->Soure_node_id);
                        SDLog_Msg_DebugVect(logName, logMsg, 3);
                    }
                    break;
                }
                case 20052:
                { // MSG3
                    struct EscMSG3
                    {                    // MSG3
                        uint8_t MOS_T;   // mos温度 单位℃
                        uint8_t CAP_T;   // 电容温度 单位℃
                        uint8_t MOTOR_T; // 电机温度 单位℃
                    } __PACKED;
                    EscMSG3 *msg = (EscMSG3 *)&mail.data[0];

                    if (maxTemp < msg->MOS_T)
                        maxTemp = msg->MOS_T;
                    if (attCtrlEna)
                    {
                        double logMsg[3];
                        logMsg[0] = msg->MOS_T;
                        logMsg[1] = msg->CAP_T;
                        logMsg[2] = msg->MOTOR_T;

                        char logName[16];
                        sprintf(logName, "ESC%dSta3", frame->Soure_node_id);
                        SDLog_Msg_DebugVect(logName, logMsg, 3);
                    }
                    break;
                }
            }
        }

        if (lastGcsSdTIME.get_pass_time() > 1.5)
        {
            lastGcsSdTIME = TIME::now();

						uint8_t escCnt = 0;
						for (uint8_t i = 0; i < 32; ++i)
						{
								if (currentGBits & (1 << i))
										++escCnt;
						}
					
            uint8_t payload[43];
            memset(payload, 0, 43);
            uint16_t message_type = 65000; // need >=65000
            uint8_t availbale = true;
            char nameChar[13] = {"电调数据"};
            char uint1[6] = {"度"};
            char uint2[6] = {"安"};
            char uint3[6] = {"个"};
            float value1 = 0, value2 = 0, value3 = 0;
            value1 = maxTemp;
            value2 = totalCurrent;
            value3 = escCnt;
            memcpy(&payload[0], &nameChar[0], 13); // name
            memcpy(&payload[13], &availbale, 1);   // availbale
            memcpy(&payload[14], &value1, 4);      // value1
            memcpy(&payload[18], &uint1, 5);       // uint1
            memcpy(&payload[23], &value2, 4);      // value2
            memcpy(&payload[27], &uint2, 5);       // uint2
            memcpy(&payload[32], &value3, 4);      // value3
            memcpy(&payload[36], &uint3, 5);       // uint3

            for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
            {
                const Port *port = get_CommuPort(i);
                if (port->write != 0)
                {
                    if (mavlink_lock_chan(i, 0.01))
                    {
                        mavlink_message_t msg_sd;
                        mavlink_msg_component_extension43_pack_chan(
                            get_CommulinkSysId(),  // system id
                            get_CommulinkCompId(), // component id
                            i,                     // chan
                            &msg_sd,
                            0,                    // target_network
                            get_CommulinkSysId(), // target system
                            MAV_COMP_ID_USER6,    // target component
                            message_type,         // message_type
                            payload);
                        mavlink_msg_to_send_buffer(port->write,
                            port->lock,
                            port->unlock,
                            &msg_sd, 0.01, 0.01);
                        mavlink_unlock_chan(i);
                    }
                }
            }

            if ((lastGBits & currentGBits) != lastGBits)
            { // 有电机失联
                if (lastErrSdTIME.get_pass_time() > 7.5)
                {
                    lastErrSdTIME = TIME::now();

                    for (uint8_t i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i)
                    {
                        const Port *port = get_CommuPort(i);
                        if (port->write != 0)
                        {
                            if (mavlink_lock_chan(i, 0.01))
                            {
                                char text[50];
                                sprintf(text, "CAN ESC Disconnected! Please check!");

                                mavlink_message_t msg_sd;
                                mavlink_msg_statustext_pack_chan(
                                    get_CommulinkSysId(),  // system id
                                    get_CommulinkCompId(), // component id
                                    i,                     // chan
                                    &msg_sd,
                                    MAV_SEVERITY_ALERT,
                                    text,
                                    0, 0);
                                mavlink_msg_to_send_buffer(port->write,
                                    port->lock,
                                    port->unlock,
                                    &msg_sd, 0.01, 0.01);
                                mavlink_unlock_chan(i);
                            }
                        }
                    }
                }
            }
            lastGBits |= currentGBits;

            // 清空累积状态
            maxTemp = -300;
            currentGBits = 0;
            totalCurrent = 0;
        }
    }
}

static bool CAN_Hobbywing_ESC_DriverInit()
{
    return true;
}
static bool CAN_Hobbywing_ESC_DriverRun()
{
    CanIdWithMask Ids[3];

    _EscMessageFrame frame;
    _EscMessageFrame maskFrame;
    maskFrame.notBroadcast = 1;
    maskFrame.Message_type_id = 0xffff;

    // ESC发送实时信息包
    frame.Priority = Esc_Priority_Lowest;
    frame.Message_type_id = 20050;
    frame.notBroadcast = 0;
    Ids[0].IdType = 1;
    Ids[0].mask = *(uint32_t *)&maskFrame;
    Ids[0].Identifier = *(uint32_t *)&frame;

    frame.Priority = Esc_Priority_Lowest;
    frame.Message_type_id = 20051;
    frame.notBroadcast = 0;
    Ids[1].IdType = 1;
    Ids[1].mask = *(uint32_t *)&maskFrame;
    Ids[1].Identifier = *(uint32_t *)&frame;

    frame.Priority = Esc_Priority_Lowest;
    frame.Message_type_id = 20052;
    frame.notBroadcast = 0;
    Ids[2].IdType = 1;
    Ids[2].mask = *(uint32_t *)&maskFrame;
    Ids[2].Identifier = *(uint32_t *)&frame;

    CanMailBox *mail_box = new CanMailBox(48, Ids, 3);
    DriverInfo *driver_info = new DriverInfo;
    driver_info->mail_box = mail_box;
    xTaskCreate(CAN_Hobbywing_ESC_Server, "CAN_Hobbywing_ESC", 1600, (void *)driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_CAN_Hobbywing_ESC()
{
    CanFunc_Register(10, CAN_Hobbywing_ESC_DriverInit, CAN_Hobbywing_ESC_DriverRun);
}