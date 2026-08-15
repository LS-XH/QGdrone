#include "drv_GPS1_UM982.hpp"
#include "gpsConfig.hpp"
#include "Basic.hpp"
#include "Commulink.hpp"
#include "ControlSystem.hpp"
#include "FreeRTOS.h"
#include "Parameters.hpp"
#include "SensorsBackend.hpp"
#include "StorageSystem.hpp"
#include "drv_CRC.hpp"
#include "task.h"
#include <assert.h>
#include <stdlib.h>
#include <string.h>

struct DriverInfo
{
    uint32_t param;
    Port port;
};

//static const uint32_t aulCrcTable[256] =
//    {
//        0x00000000UL, 0x77073096UL, 0xee0e612cUL, 0x990951baUL, 0x076dc419UL,
//        0x706af48fUL,
//        0xe963a535UL, 0x9e6495a3UL, 0x0edb8832UL, 0x79dcb8a4UL, 0xe0d5e91eUL,
//        0x97d2d988UL,
//        0x09b64c2bUL, 0x7eb17cbdUL, 0xe7b82d07UL, 0x90bf1d91UL, 0x1db71064UL,
//        0x6ab020f2UL,
//        0xf3b97148UL, 0x84be41deUL, 0x1adad47dUL, 0x6ddde4ebUL, 0xf4d4b551UL,
//        0x83d385c7UL,
//        0x136c9856UL, 0x646ba8c0UL, 0xfd62f97aUL, 0x8a65c9ecUL, 0x14015c4fUL,
//        0x63066cd9UL,
//        0xfa0f3d63UL, 0x8d080df5UL, 0x3b6e20c8UL, 0x4c69105eUL, 0xd56041e4UL,
//        0xa2677172UL,
//        0x3c03e4d1UL, 0x4b04d447UL, 0xd20d85fdUL, 0xa50ab56bUL, 0x35b5a8faUL,
//        0x42b2986cUL,
//        0xdbbbc9d6UL, 0xacbcf940UL, 0x32d86ce3UL, 0x45df5c75UL, 0xdcd60dcfUL,
//        0xabd13d59UL,
//        0x26d930acUL, 0x51de003aUL, 0xc8d75180UL, 0xbfd06116UL, 0x21b4f4b5UL,
//        0x56b3c423UL,
//        0xcfba9599UL, 0xb8bda50fUL, 0x2802b89eUL, 0x5f058808UL, 0xc60cd9b2UL,
//        0xb10be924UL,
//        0x2f6f7c87UL, 0x58684c11UL, 0xc1611dabUL, 0xb6662d3dUL, 0x76dc4190UL,
//        0x01db7106UL,
//        0x98d220bcUL, 0xefd5102aUL, 0x71b18589UL, 0x06b6b51fUL, 0x9fbfe4a5UL,
//        0xe8b8d433UL,
//        0x7807c9a2UL, 0x0f00f934UL, 0x9609a88eUL, 0xe10e9818UL, 0x7f6a0dbbUL,
//        0x086d3d2dUL,
//        0x91646c97UL, 0xe6635c01UL, 0x6b6b51f4UL, 0x1c6c6162UL, 0x856530d8UL,
//        0xf262004eUL,
//        0x6c0695edUL, 0x1b01a57bUL, 0x8208f4c1UL, 0xf50fc457UL, 0x65b0d9c6UL,
//        0x12b7e950UL,
//        0x8bbeb8eaUL, 0xfcb9887cUL, 0x62dd1ddfUL, 0x15da2d49UL, 0x8cd37cf3UL,
//        0xfbd44c65UL,
//        0x4db26158UL, 0x3ab551ceUL, 0xa3bc0074UL, 0xd4bb30e2UL, 0x4adfa541UL,
//        0x3dd895d7UL,
//        0xa4d1c46dUL, 0xd3d6f4fbUL, 0x4369e96aUL, 0x346ed9fcUL, 0xad678846UL,
//        0xda60b8d0UL,
//        0x44042d73UL, 0x33031de5UL, 0xaa0a4c5fUL, 0xdd0d7cc9UL, 0x5005713cUL,
//        0x270241aaUL,
//        0xbe0b1010UL, 0xc90c2086UL, 0x5768b525UL, 0x206f85b3UL, 0xb966d409UL,
//        0xce61e49fUL,
//        0x5edef90eUL, 0x29d9c998UL, 0xb0d09822UL, 0xc7d7a8b4UL, 0x59b33d17UL,
//        0x2eb40d81UL,
//        0xb7bd5c3bUL, 0xc0ba6cadUL, 0xedb88320UL, 0x9abfb3b6UL, 0x03b6e20cUL,
//        0x74b1d29aUL,
//        0xead54739UL, 0x9dd277afUL, 0x04db2615UL, 0x73dc1683UL, 0xe3630b12UL,
//        0x94643b84UL,
//        0x0d6d6a3eUL, 0x7a6a5aa8UL, 0xe40ecf0bUL, 0x9309ff9dUL, 0x0a00ae27UL,
//        0x7d079eb1UL,
//        0xf00f9344UL, 0x8708a3d2UL, 0x1e01f268UL, 0x6906c2feUL, 0xf762575dUL,
//        0x806567cbUL,
//        0x196c3671UL, 0x6e6b06e7UL, 0xfed41b76UL, 0x89d32be0UL, 0x10da7a5aUL,
//        0x67dd4accUL,
//        0xf9b9df6fUL, 0x8ebeeff9UL, 0x17b7be43UL, 0x60b08ed5UL, 0xd6d6a3e8UL,
//        0xa1d1937eUL,
//        0x38d8c2c4UL, 0x4fdff252UL, 0xd1bb67f1UL, 0xa6bc5767UL, 0x3fb506ddUL,
//        0x48b2364bUL,
//        0xd80d2bdaUL, 0xaf0a1b4cUL, 0x36034af6UL, 0x41047a60UL, 0xdf60efc3UL,
//        0xa867df55UL,
//        0x316e8eefUL, 0x4669be79UL, 0xcb61b38cUL, 0xbc66831aUL, 0x256fd2a0UL,
//        0x5268e236UL,
//        0xcc0c7795UL, 0xbb0b4703UL, 0x220216b9UL, 0x5505262fUL, 0xc5ba3bbeUL,
//        0xb2bd0b28UL,
//        0x2bb45a92UL, 0x5cb36a04UL, 0xc2d7ffa7UL, 0xb5d0cf31UL, 0x2cd99e8bUL,
//        0x5bdeae1dUL,
//        0x9b64c2b0UL, 0xec63f226UL, 0x756aa39cUL, 0x026d930aUL, 0x9c0906a9UL,
//        0xeb0e363fUL,
//        0x72076785UL, 0x05005713UL, 0x95bf4a82UL, 0xe2b87a14UL, 0x7bb12baeUL,
//        0x0cb61b38UL,
//        0x92d28e9bUL, 0xe5d5be0dUL, 0x7cdcefb7UL, 0x0bdbdf21UL, 0x86d3d2d4UL,
//        0xf1d4e242UL,
//        0x68ddb3f8UL, 0x1fda836eUL, 0x81be16cdUL, 0xf6b9265bUL, 0x6fb077e1UL,
//        0x18b74777UL,
//        0x88085ae6UL, 0xff0f6a70UL, 0x66063bcaUL, 0x11010b5cUL, 0x8f659effUL,
//        0xf862ae69UL,
//        0x616bffd3UL, 0x166ccf45UL, 0xa00ae278UL, 0xd70dd2eeUL, 0x4e048354UL,
//        0x3903b3c2UL,
//        0xa7672661UL, 0xd06016f7UL, 0x4969474dUL, 0x3e6e77dbUL, 0xaed16a4aUL,
//        0xd9d65adcUL,
//        0x40df0b66UL, 0x37d83bf0UL, 0xa9bcae53UL, 0xdebb9ec5UL, 0x47b2cf7fUL,
//        0x30b5ffe9UL,
//        0xbdbdf21cUL, 0xcabac28aUL, 0x53b39330UL, 0x24b4a3a6UL, 0xbad03605UL,
//        0xcdd70693UL,
//        0x54de5729UL, 0x23d967bfUL, 0xb3667a2eUL, 0xc4614ab8UL, 0x5d681b02UL,
//        0x2a6f2b94UL,
//        0xb40bbe37UL, 0xc30c8ea1UL, 0x5a05df1bUL, 0x2d02ef8dUL};
//// Calculate and return the CRC for usA binary buffer
//static uint32_t CalculateCRC32(uint8_t *szBuf, uint32_t iSize)
//{
//    int iIndex;
//    uint32_t ulCRC = 0;
//    for (iIndex = 0; iIndex < iSize; iIndex++)
//    {
//        ulCRC = aulCrcTable[(ulCRC ^ szBuf[iIndex]) & 0xff] ^ (ulCRC >> 8);
//    }
//    return ulCRC;
//}

enum GPS_Scan_Operation
{
    // 在指定波特率下发送初始化信息
    GPS_Scan_Baud9600 = 9600,
    GPS_Scan_Baud38400 = 38400,
    GPS_Scan_Baud230400 = 230400,
    GPS_Scan_Baud460800 = 460800,
    GPS_Scan_Baud115200 = 115200,
    GPS_Scan_Baud57600 = 57600,

    // 检查是否设置成功
    GPS_Check_Baud,
    // 在当前波特率下再次发送配置
    GPS_ResendConfig,
    // GPS已检测存在
    GPS_Present,
};

enum FrameName
{
    FrameNone = 0,
    FrameKSXT,
    FrameGGA,
    FrameGSA,
    FrameGST,
    FrameGSV,
    FrameHDT,
    FrameHDT2,
    FrameGNRMC,
    FrameGPRMC,
    FrameGPVTG,
    FrameGPGLL,
    FrameGPZDA,
    FrameEVENTMARK
};

struct GPS_State_Machine
{
    uint16_t frame_id;
    uint8_t frameType;
    uint8_t header_length;
    FrameName frame_name = FrameNone;
    uint32_t frame_datas_ind = 0;
    uint16_t frame_datas_length;
    uint8_t read_state = 0;
    uint16_t crc = 0;
};

static inline void ResetRxStateMachine(GPS_State_Machine *state_machine)
{
    state_machine->read_state = state_machine->frame_datas_ind = 0;
}

static inline uint16_t GPS_ParseByte(GPS_State_Machine *state_machine, uint8_t *frame_datas, uint8_t r_data)
{
    frame_datas[state_machine->frame_datas_ind++] = r_data;
    switch (state_machine->read_state)
    {
        case 0: // 找包头
        {
            if (state_machine->frame_datas_ind == 1)
            {
                if (r_data != 0xAA)
                    state_machine->frame_datas_ind = 0;
            }
            else if (state_machine->frame_datas_ind == 2)
            {
                if (r_data != 0x44)
                    state_machine->frame_datas_ind = 0;
            }
            else
            {
                if (r_data == 0xB5 || r_data == 0x12) // header found
                {
                    if (r_data == 0xB5)
                        state_machine->header_length = 24;
                    else if (r_data == 0x12)
                        state_machine->header_length = 28;
                    state_machine->frameType = r_data;
                    state_machine->read_state = 1;
                    //					state_machine->frame_datas_ind = 0;
                    state_machine->crc = 0; // reset checksum
                }
                else
                    state_machine->frame_datas_ind = 0;
            }
            break;
        }
        case 1: // 读Class ID和包长度
        {
            if (state_machine->frameType == 0xB5 && state_machine->frame_datas_ind == 8)
            {
                state_machine->frame_id = *(unsigned short *)&frame_datas[4];
                state_machine->frame_datas_length = (*(unsigned short *)&frame_datas[6]) + 24 + 4;
                if (state_machine->frame_datas_length > 28 && state_machine->frame_datas_length < 2000)
                    state_machine->read_state = 2;
                else
                    ResetRxStateMachine(state_machine);
            }
            else if (state_machine->frameType == 0x12 && state_machine->frame_datas_ind == 10)
            {
                state_machine->frame_id = *(unsigned short *)&frame_datas[4];
                state_machine->frame_datas_length = (*(unsigned short *)&frame_datas[8]) + 28 + 4;
                if (state_machine->frame_datas_length > 32 && state_machine->frame_datas_length < 2000)
                    state_machine->read_state = 2;
                else
                    ResetRxStateMachine(state_machine);
            }
            break;
        }
        case 2: // 读包内容
        {
            if (state_machine->frameType == 0xB5)
            {
                if (state_machine->frame_datas_ind == state_machine->frame_datas_length - 12)
                {
                    // payload read completed
                    state_machine->read_state = 3;
                }
            }
            else if (state_machine->frameType == 0x12)
            {
                if (state_machine->frame_datas_ind == state_machine->frame_datas_length - 14)
                {
                    // payload read completed
                    state_machine->read_state = 3;
                }
            }
            break;
        }
        case 3: // 校验
        {
            if (state_machine->frame_datas_ind == state_machine->frame_datas_length)
            {
                //uint32_t res = CalculateCRC32((uint8_t *)&frame_datas[0], state_machine->frame_datas_length - 4) ;
								uint32_t res =0;
								CRC_Calc( cg_CRC_CONFIG_CRC32_MAVLINK, (uint8_t*)&frame_datas[0], state_machine->frame_datas_length-4, &res);
                ResetRxStateMachine(state_machine);
                if (res == *(uint32_t *)&frame_datas[state_machine->frame_datas_length - 4])
                    return state_machine->frame_datas_length;
                else
                    return 0;
            }
        }
    }
    return 0;
}

static int8_t send_init_msg(DriverInfo &driver_info, uint8_t gps_ind)
{
    int len = 0;
    char UM982Clr[100];

    len = sprintf(UM982Clr, "UNLOG\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    driver_info.port.reset_rx(0.1);

    int8_t COM = -1;
    len = sprintf(UM982Clr, "LOG BESTPOSA ONTIME 1\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.15);
    const char CmdResult[] = "#BESTPOSA,COM";
    const uint8_t CmdResultLen = strlen(CmdResult);
    len = 0;
    TIME waitCorrectResponseTime = TIME::now();
    while (waitCorrectResponseTime.get_pass_time() < 3)
    {
        if (driver_info.port.read((uint8_t *)&UM982Clr[0], 1, 0.5, 1) == 0)
            return -1;
        if (len < CmdResultLen)
        {
            if (UM982Clr[0] == CmdResult[len])
                ++len;
            else
                len = 0;
        }
        else
        {
            if (UM982Clr[0] >= '1' && UM982Clr[0] <= '3')
                COM = UM982Clr[0];
            break;
        }
    }
    if (COM == -1)
        return -1;

    len = sprintf(UM982Clr, "UNLOG\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 发送GNSS配置
    bool isGPSCfgAvailble = false;
    GpsConfig gps_cfg;
    char gpsName[] = "GPS1";
    gpsName[3] = '0' + gps_ind;
    if (ReadParamGroup(SName(gpsName) + SName("Cfg"), (uint64_t *)&gps_cfg, 0) == PR_OK)
    {
        gps_cfg.GNSS_Mode[0] &= 0b1111111;
        if (gps_cfg.GNSS_Mode[0] != 0)
        { // 发送GNSS配置
            isGPSCfgAvailble = true;
        }
    }

    // 停止所有信息输出
    len = sprintf(UM982Clr, "UNLOG\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 流动站工作模式
    len = sprintf(UM982Clr, "MODE ROVER\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 设置 RTK 模块指令，数据最大龄期，秒为单位
    len = sprintf(UM982Clr, "CONFIG RTK TIMEOUT 600\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 设置接收的 DGPS差分数据的最大龄期，秒为单位，接收到的滞后于指定龄期的 DGPS 差分数据被忽略，也用于禁止 DGPS 定位计算
    len = sprintf(UM982Clr, "CONFIG DGPS TIMEOUT 600\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 设置双天线接收机的主天线（ANT1）与从天线（ANT2）之间距离保持固定
    len = sprintf(UM982Clr, "CONFIG HEADING FIXLENGTH\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 设置 EVENT 功能，下降沿有效、两个有效脉冲之间的最短时间要求, 单位ms。若小于 TGuard，则第二个 Event 被忽视。默认值: 4，最小： 2；最大：3,599,999
    len = sprintf(UM982Clr, "CONFIG EVENT ENABLE NEGATIVE 2\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 设置关闭单频定位状态的判定条件，基线长度超过 5km 时，精度满足需求依然可以使用固定解
    len = sprintf(UM982Clr, "CONFIG SFRTK Enable\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 将 BDS 系统卫星的 B1C&B2a 信号编入 RTCM 协议
    len = sprintf(UM982Clr, "CONFIG RTCMB1CB2a Enable\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 使能接收机跟踪 GPS 卫星系
    if (isGPSCfgAvailble)
    {
        if ((gps_cfg.GNSS_Mode[0] & (1 << 0)) == (1 << 0))
        {
            len = sprintf(UM982Clr, "UNMASK GPS\r\n");
        }
        else
        {
            len = sprintf(UM982Clr, "MASK GPS\r\n");
        }
    }
    else
        len = sprintf(UM982Clr, "UNMASK GPS\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 使能接收机跟踪 BDS 卫星系统
    if (isGPSCfgAvailble)
    {
        if ((gps_cfg.GNSS_Mode[0] & (1 << 3)) == (1 << 3))
        {
            len = sprintf(UM982Clr, "UNMASK BDS\r\n");
        }
        else
        {
            len = sprintf(UM982Clr, "MASK BDS\r\n");
        }
    }
    else
        len = sprintf(UM982Clr, "UNMASK BDS\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 使能接收机跟踪 GLO 卫星系统
    if (isGPSCfgAvailble)
    {
        if ((gps_cfg.GNSS_Mode[0] & (1 << 6)) == (1 << 6))
        {
            len = sprintf(UM982Clr, "UNMASK GLO\r\n");
        }
        else
        {
            len = sprintf(UM982Clr, "MASK GLO\r\n");
        }
    }
    else
        len = sprintf(UM982Clr, "UNMASK GLO\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 使能接收机跟踪 GAL 卫星系统
    if (isGPSCfgAvailble)
    {
        if ((gps_cfg.GNSS_Mode[0] & (1 << 2)) == (1 << 2))
        {
            len = sprintf(UM982Clr, "UNMASK GAL\r\n");
        }
        else
        {
            len = sprintf(UM982Clr, "MASK GAL\r\n");
        }
    }
    else
        len = sprintf(UM982Clr, "UNMASK GAL\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 使能接收机跟踪 QZSS 卫星系统
    if (isGPSCfgAvailble)
    {
        if ((gps_cfg.GNSS_Mode[0] & (1 << 5)) == (1 << 5))
        {
            len = sprintf(UM982Clr, "UNMASK QZSS\r\n");
        }
        else
        {
            len = sprintf(UM982Clr, "MASK QZSS\r\n");
        }
    }
    else
        len = sprintf(UM982Clr, "UNMASK QZSS\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 输出 EVENT 发生时刻的精确绝对时间及相对时间
    len = sprintf(UM982Clr, "LOG EVENTMARKB ONCHANGED\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 设置接收机跟踪卫星的卫星截止角度
    len = sprintf(UM982Clr, "MASK 10\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 接收机默设置为动态模式
    len = sprintf(UM982Clr, "RTKDYNAMICS DYNAMIC\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 接收机运动的航向
    len = sprintf(UM982Clr, "UNIHEADINGB 0.1\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // UTC 时间
    len = sprintf(UM982Clr, "LOG TIMEB ONTIME 0.1\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 精度因子信息
    len = sprintf(UM982Clr, "STADOPB 1\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // UTC 时间
    len = sprintf(UM982Clr, "RECTIMEB 1\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 主天线计算出的最佳可用位置和速度Bestnav
    len = sprintf(UM982Clr, "BESTNAVB 0.05\r\n");
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    // 设置com波特率为460800,8位数据位,无校验,一位停止位
    len = sprintf(UM982Clr, "CONFIG COM%d 460800 8 n 1\r\n", COM - '0');
    driver_info.port.write((uint8_t *)&UM982Clr[0], len, portMAX_DELAY, portMAX_DELAY);
    driver_info.port.wait_sent(1);
    os_delay(0.1);

    return COM;
}

static void RTK_Server(void *pvParameters)
{
    DriverInfo driver_info = *(DriverInfo *)pvParameters;
    delete (DriverInfo *)pvParameters;

    // GPS识别状态
    GPS_Scan_Operation current_GPS_Operation = GPS_Scan_Baud115200;
    // 数据读取状态机
    __attribute__((aligned(4))) uint8_t frame_datas[2048];
    // 上次更新时间
    TIME last_update_time;

    // 等待初始化完成
    while (getInitializationCompleted() == false)
        os_delay(0.1);

    // 注册Rtk端口
    RtkPort rtk_port;
    rtk_port.ena = false;
    rtk_port.write = driver_info.port.write;
    rtk_port.lock = driver_info.port.lock;
    rtk_port.unlock = driver_info.port.unlock;
    int8_t rtk_port_ind = RtkPortRegister(rtk_port);

    bool rtc_updated = false;

    // 读取是否需要记录PPK
    bool record_ppk = false;
    uint8_t log_ppk[8];
    if (ReadParam("SDLog_PPK", 0, 0, (uint64_t *)log_ppk, 0) == PR_OK)
    {
        if (log_ppk[0] == 2)
            record_ppk = true;
    }

    uint8_t gps_ind = 1;
    if (driver_info.param >= 1 && driver_info.param <= MAX_GPS_COUNT)
        gps_ind = driver_info.param;

GPS_CheckBaud:
    while (1)
    {
        // 更改指定波特率
        driver_info.port.SetBaudRate(current_GPS_Operation, 3, 0.1);
        // 切换波特率
        switch (current_GPS_Operation)
        {
            case GPS_Scan_Baud9600:
                current_GPS_Operation = GPS_Scan_Baud57600;
                break;
            case GPS_Scan_Baud57600:
                current_GPS_Operation = GPS_Scan_Baud38400;
                break;
            case GPS_Scan_Baud38400:
                current_GPS_Operation = GPS_Scan_Baud230400;
                break;
            case GPS_Scan_Baud230400:
                current_GPS_Operation = GPS_Scan_Baud460800;
                break;
            case GPS_Scan_Baud460800:
                current_GPS_Operation = GPS_Scan_Baud115200;
                break;
            default:
            case GPS_Scan_Baud115200:
                current_GPS_Operation = GPS_Scan_Baud9600;
                break;
        }
        // 发送配置
        if (send_init_msg(driver_info, gps_ind) < 0)
            continue;

        // 更改波特率
        driver_info.port.SetBaudRate(460800, 1, 0.1);
        // 清空接收缓冲区准备接收数据
        driver_info.port.reset_rx(0.1);
        GPS_State_Machine gps_state;
        ResetRxStateMachine(&gps_state);
        TIME RxChkStartTime = TIME::now();

        while (RxChkStartTime.get_pass_time() < 3)
        {
            uint8_t r_data;
            if (driver_info.port.read(&r_data, 1, 0.5, 0.1))
            {
                bool res = GPS_ParseByte(&gps_state, frame_datas, r_data);
                if (res)
                {
                    if (gps_state.frame_id == 2118)
                    { // 已识别,跳转到gps接收程序
                        goto GPS_Present;
                    }
                }
            }
        }
    }

GPS_Present:
    // 重发配置
    // send_init_msg(driver_info);
    uint32_t sensor_key = 0;
    uint8_t dao_ind = 0;
    uint32_t dao_key = 0;
    uint8_t gps_sensor_index = default_gps1_sensor_index + gps_ind - 1;
    GpsConfig gps_cfg;
    char gpsName[] = "GPS1";
    gpsName[3] = '0' + gps_ind;
    if (ReadParamGroup(SName(gpsName) + SName("Cfg"), (uint64_t *)&gps_cfg, 0) == PR_OK)
    {
        // 注册传感器
        sensor_key = PositionSensorRegister(gps_sensor_index,
            SName(gpsName) + SName("_Um982"),
            Position_Sensor_Type_GlobalPositioning,
            Position_Sensor_DataType_sv_xy,
            Position_Sensor_frame_ENU,
            gps_cfg.delay[0], // 延时
            30,               // xy信任度
            30                // z信任度
        );

        // 注册侧向传感器
        dao_ind = gps_ind - 1;
        dao_key = DAOSensorRegister(dao_ind, "DRTK", vector3<double>(gps_cfg.daoVecX[0], gps_cfg.daoVecY[0], gps_cfg.daoVecZ[0]), false, gps_cfg.delay[0]);
    }
    else
    { // 注册传感器
        sensor_key = PositionSensorRegister(gps_sensor_index,
            SName(gpsName) + SName("_Um982"),
            Position_Sensor_Type_GlobalPositioning,
            Position_Sensor_DataType_sv_xy,
            Position_Sensor_frame_ENU,
            0.1, // 延时
            30,  // xy信任度
            30   // z信任度
        );
        gps_cfg.daoVecX[0] = gps_cfg.daoVecY[0] = gps_cfg.daoVecZ[0] = 0;
    }
    if (sensor_key == 0)
        vTaskDelete(NULL);

    // 开启Rtk注入
    RtkPort_setEna(rtk_port_ind, true);

    // gps状态
    bool gps_available = false;
    bool z_available = false;
    bool zHighPrec = false;
    TIME GPS_stable_start_time(false);
    double gps_alt;
    TIME gps_update_TIME;

    // 速度积分
    double last_velcocity_z = 0;

    // 清空接收缓冲区准备接收数据
    driver_info.port.reset_rx(0.1);
    GPS_State_Machine gps_state, gps_state_nmea;
    ResetRxStateMachine(&gps_state);
    frame_datas[0] = frame_datas[1] = 0;
    last_update_time = TIME::now();

    struct Dop_Pack
    {
        uint32_t itow;
        float gdop;   // 几何精度因子
        float pdop;   // 位置精度因子
        float tdop;   // 时间精度因子
        float vdop;   // 垂直精度因子
        float hdop;   // 水平精度因子
        float ndop;   // 北向精度因子
        float edop;   // 东向精度因子
        float Cutoff; // 截至高度角
        float rsv2;
        uint16_t svNum;
        uint16_t prn;
    } __attribute__((packed));
    Dop_Pack dop_pack = {0};

    // dao状态
    TIME daoAvailableStartTime(false);
    // 附加数据
    double addition_inf[8] = {0};

    while (1)
    {
        uint8_t r_data;
        if (driver_info.port.read(&r_data, 1, 2, 0.1))
        {
            if (GPS_ParseByte(&gps_state, frame_datas, r_data))
            {
                if (gps_state.frame_id == 2118)
                { // BESTNAV 最佳位置和速度
                    // 记录
                    if (record_ppk)
                        SDLog_Ubx((const char *)frame_datas, gps_state.frame_datas_length);

                    last_update_time = TIME::now();
                    struct Best_Nav_Pack
                    {
                        uint32_t PosStatus; // 位置解状态
                        uint32_t PosType;   // 位置类型
                        double lat;         // 纬度，deg
                        double lon;         // 经度，deg
                        double height;      // 海拔高，m
                        float undulation;   // 大地水准面差距- 大地水准面和WGS84 椭球面之间的距,米
                        uint32_t datum_id;  // 坐标系 ID 号,当前仅支持 WGS84
                        float LatAcc;       // 纬度标准差，m
                        float LonAcc;       // 经度标准差，m
                        float HeightAcc;    // 高度标准差，m
                        uint8_t stn_id[4];  // 基站 ID
                        float diff_age;     // 差分龄期，s
                        float sol_age;      // 解的龄期，s
                        uint8_t SVTrack;    // 跟踪的卫星数
                        uint8_t SVUse;      // 在解中使用的卫星数
                        uint8_t rsv2[3];
                        uint8_t ext_sol_stat; // 扩展解的状态
                        uint8_t mask1;        // Galileo 使用的信号掩码
                        uint8_t mask2;        // GPS, GLONASS 和 BDS 使用的信号掩码
                        uint32_t VelStatus;   // 速度解状态
                        uint32_t VelType;     // 速度类型
                        float latency;        // 据速度时标计算的延迟值，以秒为单位
                        float age;            // 差分龄期，s
                        double horSpeed;      // 对地水平速度，m/s
                        double horSpeedAngle; // 相对于真北的实际对地运动方向（相对地面轨迹），deg
                        double VerticalSpeed; // 垂直速度，m/s
                        float VerspdStd;      // 高程速度标准差，单位 m/s
                        float HorspdStd;      // 水平速度标准差，单位 m/s
                    } __attribute__((packed));
                    Best_Nav_Pack *pack = (Best_Nav_Pack *)&frame_datas[gps_state.header_length];

                    uint8_t gps_fix = 0;
                    if (pack->PosType == 16 || (pack->PosType == 17) || (pack->PosType == 18)) // 单点定位
                        gps_fix = 3;
                    else if (pack->PosType == 32 || pack->PosType == 33 || pack->PosType == 34) // RTK 浮点解
                        gps_fix = 5;
                    else if (pack->PosType == 48 || pack->PosType == 49 || pack->PosType == 50) // RTK 固定解
                        gps_fix = 6;

                    double hAcc = sqrt(pack->LatAcc * pack->LatAcc + pack->LonAcc * pack->LonAcc);
                    double vAcc = pack->HeightAcc;

                    if (gps_fix != 0 && pack->SVTrack >= 7)
                    {
                        if (gps_available == false)
                        {
                            if (hAcc < 6)
                            {
                                if (GPS_stable_start_time.is_valid() == false)
                                    GPS_stable_start_time = TIME::now();
                                else if (GPS_stable_start_time.get_pass_time() > 3.0f)
                                {
                                    gps_available = true;
                                    GPS_stable_start_time.set_invalid();
                                }
                            }
                            else
                                GPS_stable_start_time.set_invalid();
                        }
                        else
                        {
                            bool inFlight;
                            get_is_inFlight(&inFlight);
                            double qAcc = 8;
                            if (inFlight)
                                qAcc = 20;
                            if (hAcc > qAcc)
                                gps_available = z_available = false;
                        }
                    }
                    else
                    {
                        gps_available = z_available = false;
                        GPS_stable_start_time.set_invalid();
                    }

                    addition_inf[0] = pack->SVTrack;
                    addition_inf[1] = gps_fix;
                    addition_inf[4] = hAcc * 100;
                    addition_inf[5] = vAcc * 100;
                    addition_inf[6] = 0;

                    if (z_available)
                    {
                        if (vAcc > 16)
                            z_available = false;
                    }
                    else
                    {
                        if (vAcc < 8)
                        {
                            gps_alt = pack->height * 100;
                            z_available = true;
                        }
                    }
                    double t = gps_update_TIME.get_pass_time_st();
                    if (t > 1)
                        t = 1;

                    vector3<double> velocity;
                    velocity.y = pack->horSpeed * cos(degree2rad(pack->horSpeedAngle)) * 100; // North
                    velocity.x = pack->horSpeed * sin(degree2rad(pack->horSpeedAngle)) * 100; // East
                    velocity.z = pack->VerticalSpeed * 100;                                   // Up

                    gps_alt += 0.5 * (velocity.z + last_velcocity_z) * t;
                    last_velcocity_z = velocity.z;

                    // 高度切换高低精度模式
                    bool zSwitch = false;
                    if (zHighPrec == false)
                    {
                        if (gps_fix == 6 && vAcc < 0.5)
                        {
                            double r_height = pack->height * 100;
                            gps_alt = r_height;

                            zSwitch = true;
                            zHighPrec = true;
                        }
                    }
                    else
                    {
                        if (gps_fix != 6 || vAcc > 1)
                        {
                            zHighPrec = false;
                        }
                    }
                    // 高度源
                    if (zHighPrec)
                    { // 高精度高度位置结果
                        // 使用绝对高度
                        double r_height = pack->height * 100;
                        gps_alt += 0.5 * t * (r_height - gps_alt);
                    }
                    else
                    {
                    }

                    vector3<double> position_Global;
                    position_Global.x = pack->lat;
                    position_Global.y = pack->lon;
                    position_Global.z = gps_alt;

                    if (z_available && !zSwitch)
                        PositionSensorChangeDataType(gps_sensor_index, sensor_key, Position_Sensor_DataType_sv_xyz);
                    else
                        PositionSensorChangeDataType(gps_sensor_index, sensor_key, Position_Sensor_DataType_sv_xy);

                    // 信任度
                    double xy_trustD = hAcc * 10;
                    double z_trustD = vAcc * 10;
                    PositionSensorUpdatePositionGlobalVel(gps_sensor_index, sensor_key, position_Global, velocity,
                        gps_available, // available
                        -1,            // delay
                        xy_trustD,
                        z_trustD,
                        addition_inf,
                        xy_trustD, // xy LTtrust
                        z_trustD   // z LTtrust
                    );
                }
                else if (gps_state.frame_id == 972)
                { // UNIHEADING 航向信息
                    last_update_time = TIME::now();
                    struct Heading_Pack
                    {
                        uint32_t solStatus; // 位置解状态
                        uint32_t posType;   // 位置类型
                        float length;       // 基线长 (0 到 3000 m)
                        float heading;      // 航向 (0 到 360.0 deg)
                        float pitch;        // 俯仰(±90 deg)
                        float rsv2;
                        float hdgstddev;   // 航向标准偏差
                        float ptchstddev;  // 俯仰标准偏差
                        uint8_t stn_id[4]; // 基站 ID
                        uint8_t SVTrack;   // 跟踪的卫星数
                        uint8_t SVUse;     // 在解中使用的卫星数
                        uint8_t obs;       // 截止高度角以上的卫星数
                        uint8_t multi;     // 截止高度角以上有L2观测的卫星数
                        uint8_t rsv3;
                        uint8_t ext_sol_stat; // 扩展解的状态
                        uint8_t mask1;        // Galileo 使用的信号掩码
                        uint8_t mask2;        // GPS, GLONASS 和 BDS 使用的信号掩码
                    } __attribute__((packed));
                    Heading_Pack *pack = (Heading_Pack *)&frame_datas[gps_state.header_length];
                    double len_pos = sq(gps_cfg.daoVecX[0]) + sq(gps_cfg.daoVecY[0]) + sq(gps_cfg.daoVecZ[0]);
                    bool available = false;
                    if (len_pos > 7 * 7)
                    {
                        double heading = 90.0f - pack->heading;
                        double sinY, cosY;
                        double sinPit, cosPit;
                        fast_sin_cos(degree2rad(heading), &sinY, &cosY);
                        fast_sin_cos(degree2rad(pack->pitch), &sinPit, &cosPit);

                        vector3<double> relPos(
                            pack->length * cosPit * 100 * cosY,
                            pack->length * cosPit * 100 * sinY,
                            pack->length * sinPit * 100);
                        if (pack->solStatus == 0 && (pack->posType == 50 || pack->posType == 48))
                        {
                            daoAvailableStartTime = TIME::now();
                            available = true;
                        }
                        if (available || daoAvailableStartTime.is_valid() == false || daoAvailableStartTime.get_pass_time() > 2)
                            DAOSensorUpdate(dao_ind, dao_key, relPos, available);
                    }
                }
                else if (gps_state.frame_id == 309)
                { // EVENT触发
                    last_update_time = TIME::now();
                    struct Event_Pack
                    {
                        uint32_t itow;
                        uint8_t eventID; // 事件编码（Event 1 或Event 2）
                        uint8_t status;  // 事件状态（待定）
                        uint8_t reserved0;
                        uint8_t reserved1;
                        uint32_t week; // 周
                        uint32_t reserved2;
                        uint32_t offset_second;    // 按当前 GGA 输出频率，EVENT 时刻与最接近的GGA 输出的绝对时间之间的偏移值(second)
                        uint32_t offset_SubSecond; // 按当前 GGA 输出频率，EVENT 时刻与最接近的GGA 输出的绝对时间之间的偏移值(nanosecond)
                        uint16_t prn;
                    } __attribute__((packed));
                    Event_Pack *pack = (Event_Pack *)&frame_datas[gps_state.header_length];
                    if (record_ppk)
                        SDLog_Ubx((const char *)frame_datas, gps_state.frame_datas_length);
                }
                else if (gps_state.frame_id == 954)
                { // STADOP DOP信息
                    last_update_time = TIME::now();
                    dop_pack = *(Dop_Pack *)&frame_datas[gps_state.header_length];
                }
                else if (gps_state.frame_id == 102)
                { // RECTIME 信息
                    last_update_time = TIME::now();
                    struct Time_Pack
                    {
                        uint32_t clock_status; // 时钟模型状态
                        double offset;         // 相对于 GPS 时的接收机钟差
                        double Offset_std;     // GPS 时间到 UTC 时间的偏
                        double utc_Offset;
                        uint32_t year;         // UTC 年
                        uint8_t month;         // UTC 月 (0-12)
                        uint8_t day;           // UTC 天 (0-31)
                        uint8_t hour;          // UTC 小时 (0-23)
                        uint8_t min;           // UTC 分钟 (0-59)
                        uint32_t ms;           // UTC 毫秒 (0-60999)
                        uint32_t utc_status;   // UTC 状态：0 = INVALID，无效； 1 =VALID，有效； 2 = WARNING 11
                    } __attribute__((packed));
                    Time_Pack *pack = (Time_Pack *)&frame_datas[gps_state.header_length];

                    if (pack->utc_status == 1)
                    {
                        uint64_t gps_time = gps_packUtc(pack->year, pack->month, pack->day,
                            pack->hour, pack->min, pack->ms * 1e-3);
                        addition_inf[2] = *(double *)&gps_time;

                        if (!gps_rtcUpdated())
                        {
                            Position_Sensor_Data sensor;
                            if (GetPositionSensorData(gps_sensor_index, &sensor) && sensor.available)
                            {
                                gps_setRtcOnce(sensor.position_Global.x, sensor.position_Global.y,
                                    pack->year, pack->month, pack->day, pack->hour, pack->min, pack->ms * 1e-3);
                            }
                        }
                    }
                    else
                        addition_inf[2] = 0;
                    // if (record_ppk)
                    //     SDLog_Ubx((const char *)frame_datas, gps_state.frame_datas_length);
                }
                else if (gps_state.frame_id == 43)
                { // 原始观测数据信息
                    if (record_ppk)
                        SDLog_Ubx((const char *)frame_datas, gps_state.frame_datas_length);
                }
                else if (gps_state.frame_id == 1695)
                { // BDS 导航电文
                    if (record_ppk)
                        SDLog_Ubx((const char *)frame_datas, gps_state.frame_datas_length);
                }
                else if (gps_state.frame_id == 722)
                { // GLONASS 导航电文
                    if (record_ppk)
                        SDLog_Ubx((const char *)frame_datas, gps_state.frame_datas_length);
                }
                else if (gps_state.frame_id == 1330)
                { // QZSS 导航电文
                    if (record_ppk)
                        SDLog_Ubx((const char *)frame_datas, gps_state.frame_datas_length);
                }
                else if (gps_state.frame_id == 25)
                { // GPS 导航电文
                    if (record_ppk)
                        SDLog_Ubx((const char *)frame_datas, gps_state.frame_datas_length);
                }
            }

            if (last_update_time.get_pass_time() > 2)
            { // 接收不到数据
                PositionSensorUnRegister(gps_sensor_index, sensor_key);
                DAOSensorUnRegister(dao_ind, dao_key);
                // 关闭Rtk注入
                RtkPort_setEna(rtk_port_ind, false);
                goto GPS_CheckBaud;
            }
        }
        else
        { // 接收不到数据
            PositionSensorUnRegister(gps_sensor_index, sensor_key);
            DAOSensorUnRegister(dao_ind, dao_key);
            // 关闭Rtk注入
            RtkPort_setEna(rtk_port_ind, false);
            goto GPS_CheckBaud;
        }
    }
}

static bool RTK_DriverInit(Port port, uint32_t param)
{
    DriverInfo *driver_info = new DriverInfo;
    driver_info->param = param;
    driver_info->port = port;
    xTaskCreate(RTK_Server, "RTK982", 2500, driver_info, SysPriority_ExtSensor, NULL);
    return true;
}

void init_drv_GPS1_UM982()
{
    PortFunc_Register(17, RTK_DriverInit);
}
