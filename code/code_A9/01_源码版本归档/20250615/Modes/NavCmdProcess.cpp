#include "NavCmdProcess.hpp"
#include "Basic.hpp"
#include "ControlSystem.hpp"
#include "InFlightCmdProcess.hpp"
#include "MeasurementSystem.hpp"
#include "Parameters.hpp"
#include "Receiver.hpp"
#include "Sensors.hpp"
#include "followTarget.hpp"
#include "mavlink.h"
#include "precLand.hpp"

/*NavCmd16_WAYPOINT坐标转换
        MAV_CMD_NAV_WAYPOINT
        获取航点飞行指令的飞行距离
        参数:
                <description>Navigate to waypoint.</description>
                <param index="0" label="Hold" units="s" minValue="0">Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)</param>
                <param index="1" label="Accept Radius" units="m" minValue="0">Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)</param>
                <param index="2" label="Pass Radius" units="m">0 to pass through the WP, if &gt; 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.</param>
                <param index="3" label="Yaw" units="deg">Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).</param>
                <param index="4" label="Latitude">Latitude</param>
                <param index="5" label="Longitude">Longitude</param>
                <param index="6" label="Altitude" units="m">Altitude</param>

                pO: 当前点Local坐标
                OC: 从当前点到目标点向量
                pC: 目标点Local坐标
*/
bool NavCmd16_WAYPOINT_GetInfo(uint8_t frame, double params[], vector3<double> *pO, vector3<double> *OC, vector3<double> *pC)
{
    if (get_Position_MSStatus() != MS_Ready)
    { // 无定位出错
        return false;
    }

    vector3<double> pos;
    // get_Position_Ctrl(&pos);
    get_TargetPosInf(0, 0, &pos, 0);
    if (pO)
        *pO = pos;

    switch (frame)
    {
        case MAV_FRAME_GLOBAL_INT:
        case MAV_FRAME_GLOBAL:
        {
            if (params[4] < -90 || params[4] > 90 || params[5] < -180 || params[5] > 180)
            { // 经纬度不正确只进行高度调整

                double target_pos_z = params[6] * 100;
                if (!isvalid(target_pos_z))
                    return false;

                // 获取最优全球定位传感器信息
                PosSensorHealthInf1 global_inf;
                if (get_OptimalGlobal_Z(&global_inf) == false)
                    return false;
                target_pos_z -= global_inf.HOffset;
                if (pC)
                    *pC = vector3<double>(pos.x, pos.y, target_pos_z);
                if (OC)
                    *OC = vector3<double>(0, 0, target_pos_z - pos.z);
            }
            else // 经纬度正确进行三维飞行
            {
                double Lat = params[4];
                double Lon = params[5];
                double posz = params[6] * 100;

                if (!isvalid(Lat) || !isvalid(Lon) || !isvalid(posz))
                    return false;

                // 获取最优全球定位传感器信息
                PosSensorHealthInf3 global_inf;
                if (get_OptimalGlobal_XYZ(&global_inf) == false)
                    return false;
                // 获取指定经纬度平面坐标
                double x, y;
                map_projection_project(&global_inf.mp, Lat, Lon, &x, &y);
                x -= global_inf.HOffset.x;
                y -= global_inf.HOffset.y;
                posz -= global_inf.HOffset.z;
                vector3<double> target_pos(x, y, posz);
                if (pC)
                    *pC = target_pos;
                if (OC)
                    *OC = target_pos - pos;
            }
            break;
        }
        case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
        {
            if (params[4] < -90 || params[4] > 90 || params[5] < -180 || params[5] > 180)
            { // 经纬度不正确只进行高度调整
                if (isvalid(params[6]) == false)
                    return false;
                double homeZ;
                getHomeLocalZ(&homeZ);
                double target_pos_z = homeZ + params[6] * 100;
                if (pC)
                    *pC = vector3<double>(pos.x, pos.y, target_pos_z);
                if (OC)
                    *OC = vector3<double>(0, 0, target_pos_z - pos.z);
            }
            else // 经纬度正确进行三维飞行
            {
                double Lat = params[4];
                double Lon = params[5];
                double posz = params[6] * 100;
                if (!isvalid(Lat) || !isvalid(Lon) || !isvalid(posz))
                    return false;

                // 获取最优全球定位传感器信息
                PosSensorHealthInf2 global_inf;
                if (get_OptimalGlobal_XY(&global_inf) == false)
                    return false;
                // 获取指定经纬度平面坐标
                double x, y;
                map_projection_project(&global_inf.mp, Lat, Lon, &x, &y);
                x -= global_inf.HOffset.x;
                y -= global_inf.HOffset.y;
                // 获取起飞位置Z坐标
                double homeZ;
                getHomeLocalZ(&homeZ);
                vector3<double> target_pos(x, y, homeZ + posz);
                if (pC)
                    *pC = target_pos;
                if (OC)
                    *OC = target_pos - pos;
            }
            break;
        }

        case MAV_FRAME_GLOBAL_TERRAIN_ALT:
        case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
        {
            if (params[4] < -90 || params[4] > 90 || params[5] < -180 || params[5] > 180 || !isvalid(params[4]) || !isvalid(params[5]))
            { // 经纬度不正确只进行高度调整
                if (isvalid(params[6]) == false)
                    return false;

                PosSensorHealthInf1 pos_inf;
                get_OptimalRange_Z(&pos_inf);
                Position_Sensor_Data sensor;
                if (GetPositionSensorData(pos_inf.sensor_ind, &sensor) && isvalid(params[6]))
                { // 移动到指定高度
                    double target_pos_z = pos.z + params[6] * 100 - sensor.position.z;
                    if (pC)
                        *pC = vector3<double>(pos.x, pos.y, target_pos_z);
                    if (OC)
                        *OC = vector3<double>(0, 0, target_pos_z - pos.z);
                }
                else // 无高度数据
                {
                    if (pC)
                        *pC = vector3<double>(pos.x, pos.y, pos.z);
                    if (OC)
                        *OC = vector3<double>(0, 0, 0);
                }
            }
            else // 经纬度正确进行三维飞行
            {
                double Lat = params[4];
                double Lon = params[5];
                double posz = params[6] * 100;

                // 获取最优全球定位传感器信息
                PosSensorHealthInf2 global_inf;
                if (get_OptimalGlobal_XY(&global_inf) == false)
                    return false;
                // 获取指定经纬度平面坐标
                double x, y;
                map_projection_project(&global_inf.mp, Lat, Lon, &x, &y);
                x -= global_inf.HOffset.x;
                y -= global_inf.HOffset.y;
                // 获取Z坐标
                double target_pos_z;
                if (isvalid(posz))
                { // 高度正确移动高度
                    PosSensorHealthInf1 pos_inf;
                    get_OptimalRange_Z(&pos_inf);
                    Position_Sensor_Data sensor;
                    if (GetPositionSensorData(pos_inf.sensor_ind, &sensor) && isvalid(params[6]))
                    {
                        target_pos_z = pos.z + params[6] * 100 - sensor.position.z;
                    }
                    else // 无高度数据
                    {
                        target_pos_z = pos.z;
                    }
                }
                else
                { // 高度不正确不移动高度
                    target_pos_z = pos.z;
                }
                vector3<double> target_pos(x, y, target_pos_z);
                if (pC)
                    *pC = target_pos;
                if (OC)
                    *OC = target_pos - pos;
            }
            break;
        }

        case MAV_FRAME_LOCAL_NED:
        {
            if (!isvalid(params[4]) || !isvalid(params[5]) || !isvalid(params[6]))
                return false;
            vector3<double> target_pos(params[5] * 100, params[4] * 100, -params[6] * 100);
            if (pC)
                *pC = target_pos;
            if (OC)
                *OC = target_pos - pos;
            break;
        }

        case MAV_FRAME_LOCAL_ENU:
        {
            if (!isvalid(params[4]) || !isvalid(params[5]) || !isvalid(params[6]))
                return false;
            vector3<double> target_pos(params[4] * 100, params[5] * 100, params[6] * 100);
            if (pC)
                *pC = target_pos;
            if (OC)
                *OC = target_pos - pos;
            break;
        }

        case MAV_FRAME_LOCAL_OFFSET_NED:
        {
            if (!isvalid(params[4]) || !isvalid(params[5]) || !isvalid(params[6]))
                return false;
            vector3<double> relVec(params[5] * 100, params[4] * 100, -params[6] * 100);
            if (pC)
                *pC = pos + relVec;
            if (OC)
                *OC = relVec;
            break;
        }

        case MAV_FRAME_BODY_NED:
        case MAV_FRAME_BODY_FRD:
        case MAV_FRAME_BODY_OFFSET_NED:
        {
            Quaternion att;
            get_Attitude_quat(&att);
            double yaw = att.getYaw();
            double sin_Yaw, cos_Yaw;
            fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
            // body系修正航向后只向前走
            vector3<double> relVec;
            if (isvalid(params[3]) == false)
            {
                relVec.x = BodyHeading2ENU_x(safe_sqrt(params[5] * params[5] + params[4] * params[4]) * 100, 0, sin_Yaw, cos_Yaw);
                relVec.y = BodyHeading2ENU_y(safe_sqrt(params[5] * params[5] + params[4] * params[4]) * 100, 0, sin_Yaw, cos_Yaw);
                relVec.z = -params[6] * 100;
            }
            else
            {
                relVec.x = BodyHeading2ENU_x(params[5] * 100, params[4] * 100, sin_Yaw, cos_Yaw);
                relVec.y = BodyHeading2ENU_y(params[5] * 100, params[4] * 100, sin_Yaw, cos_Yaw);
                relVec.z = -params[6] * 100;
            }
            if (pC)
                *pC = pos + relVec;
            if (OC)
                *OC = relVec;
            break;
        }

        case MAV_FRAME_BODY_FLU:
        {
            Quaternion att;
            get_Attitude_quat(&att);
            double yaw = att.getYaw();
            double sin_Yaw, cos_Yaw;
            fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
            // body系修正航向后只向前走
            vector3<double> relVec;
            if (isvalid(params[3]) == false)
            {
                relVec.x = BodyHeading2ENU_x(safe_sqrt(params[4] * params[4] + params[5] * params[5]) * 100, 0, sin_Yaw, cos_Yaw);
                relVec.y = BodyHeading2ENU_y(safe_sqrt(params[4] * params[4] + params[5] * params[5]) * 100, 0, sin_Yaw, cos_Yaw);
                relVec.z = params[6] * 100;
            }
            else
            {
                relVec.x = BodyHeading2ENU_x(params[4] * 100, params[5] * 100, sin_Yaw, cos_Yaw);
                relVec.y = BodyHeading2ENU_y(params[4] * 100, params[5] * 100, sin_Yaw, cos_Yaw);
                relVec.z = params[6] * 100;
            }
            if (pC)
                *pC = pos + relVec;
            if (OC)
                *OC = relVec;
            break;
        }

        default:
            return false;
    }
    return true;
}

/*
        Nav飞行控制指令处理
        所有指令必须在水平位置控制器打开的前提下执行
        所有参数单位角度为度，距离速度为米
        参数：
                freq：运行频率
                params：7个参数数组
                inf：中间信息（执行前后需调用init_NavCmdInf清空）
        返回：
                <-3：错误
                -3：未完成（可执行InFlightCmd）
                -2：未完成（不可执行InFlightCmd）
                -1：完成
                >=0：完成且要求切换到指定Mission
*/

/*NavCmd16_WAYPOINT
        MAV_CMD_NAV_WAYPOINT
        航点飞行（调转机头并飞行到指定点）
        参数:
                <description>Navigate to waypoint.</description>
                <param index="0" label="Hold" units="s" minValue="0">Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)</param>
                <param index="1" label="Accept Radius" units="m" minValue="0">Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)</param>
                <param index="2" label="Pass Radius" units="m">0 to pass through the WP, if &gt; 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.</param>
                <param index="3" label="Yaw" units="deg">Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).</param>
                <param index="4" label="Latitude">Latitude</param>
                <param index="5" label="Longitude">Longitude</param>
                <param index="6" label="Altitude" units="m">Altitude</param>
*/
static int32_t NavCmd16_WAYPOINT(double freq, uint8_t frame, double params[], NavCmdInf *inf)
{
    if (get_Position_MSStatus() != MS_Ready)
    { // 无定位出错
        inf->counter1 = inf->counter2 = 0;
        return -100;
    }
    bool inFlight;
    get_is_inFlight(&inFlight);
    if (inFlight == false)
    { // 未起飞出错
        inf->counter1 = inf->counter2 = 0;
        return -100;
    }

    switch (inf->counter1)
    {
        case 0:
        { // 判断执行旋转偏航
            if (isvalid(params[3]) == false)
            { // 机头指向航点方向
                Position_Control_set_XYLock();
                Position_Control_set_ZLock();

                double LA, LB;
                switch (frame)
                {
                    case MAV_FRAME_GLOBAL:
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                    case MAV_FRAME_GLOBAL_INT:
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                    { // 全球定位
                        if (!isvalid(params[4]) || !isvalid(params[5]) ||
                            params[4] < -90 || params[4] > 90 ||
                            params[5] < -180 || params[5] > 180)
                        { // 经纬度为不正确不转偏航
                            inf->counter1 = 1;
                            inf->counter2 = freq * 3;
                            return -2;
                        }

                        // 获取最优全球定位传感器信息
                        PosSensorHealthInf2 global_inf;
                        if (get_OptimalGlobal_XY(&global_inf) == false)
                        {
                            inf->counter1 = inf->counter2 = 0;
                            return -100;
                        }
                        // 获取指定经纬度平面坐标
                        double x, y;
                        map_projection_project(&global_inf.mp, params[4], params[5], &x, &y);
                        x -= global_inf.HOffset.x;
                        y -= global_inf.HOffset.y;
                        // 求LA LB
                        vector3<double> t_pos;
                        get_TargetPosInf(0, 0, &t_pos, 0);
                        LA = y - t_pos.y;
                        LB = x - t_pos.x;
                        break;
                    }

                    case MAV_FRAME_LOCAL_NED:
                    {
                        vector3<double> position;
                        get_Position_Ctrl(&position);
                        LA = params[4] * 100 - position.y;
                        LB = params[5] * 100 - position.x;
                        break;
                    }

                    case MAV_FRAME_LOCAL_ENU:
                    {
                        vector3<double> position;
                        get_Position_Ctrl(&position);
                        LA = params[5] * 100 - position.y;
                        LB = params[4] * 100 - position.x;
                        break;
                    }

                    case MAV_FRAME_LOCAL_OFFSET_NED:
                    {
                        LA = params[4];
                        LB = params[5];
                        break;
                    }

                    case MAV_FRAME_BODY_NED:
                    case MAV_FRAME_BODY_FRD:
                    case MAV_FRAME_BODY_OFFSET_NED:
                    {
                        Quaternion att;
                        get_Attitude_quat(&att);
                        double yaw = att.getYaw();
                        double sin_Yaw, cos_Yaw;
                        fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                        double posx_enu = BodyHeading2ENU_x(params[5], params[4], sin_Yaw, cos_Yaw);
                        double posy_enu = BodyHeading2ENU_y(params[5], params[4], sin_Yaw, cos_Yaw);
                        LA = posy_enu;
                        LB = posx_enu;
                        break;
                    }

                    case MAV_FRAME_BODY_FLU:
                    {
                        Quaternion att;
                        get_Attitude_quat(&att);
                        double yaw = att.getYaw();
                        double sin_Yaw, cos_Yaw;
                        fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                        double posx_enu = BodyHeading2ENU_x(params[4], params[5], sin_Yaw, cos_Yaw);
                        double posy_enu = BodyHeading2ENU_y(params[4], params[5], sin_Yaw, cos_Yaw);
                        LA = posy_enu;
                        LB = posx_enu;
                        break;
                    }

                    default:
                        inf->counter1 = inf->counter2 = 0;
                        return -100;
                }

                if (sq(LA) + sq(LB) > sq(5))
                    Attitude_Control_set_Target_Yaw(atan2(LA, LB));
                inf->counter1 = 1;
                inf->counter2 = 0;
            }
            else if (params[3] > -360 && params[3] < 360)
            { // 指定偏航朝向
                Attitude_Control_set_Target_Yaw(degree2rad(90 - params[3]));
                inf->counter1 = 1;
                inf->counter2 = 0;
            }
            else
            { // 不旋转偏航
                inf->counter1 = 1;
                inf->counter2 = freq * 3;
            }
            break;
        }

        case 1:
        { // 等待偏航旋转开始航点飞行
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            double yawTrackErr;
            Attitude_Control_get_YawTrackErr(&yawTrackErr);
            if (yawTrackErr < 0.01)
            {
                bool res;
                switch (frame)
                {
                    case MAV_FRAME_GLOBAL_INT:
                    case MAV_FRAME_GLOBAL:
                    {
                        if (!isvalid(params[4]) || !isvalid(params[5]) ||
                            params[4] < -90 || params[4] > 90 ||
                            params[5] < -180 || params[5] > 180)
                        { // 经纬度不正确只进行高度调整
                            Position_Control_set_XYLock();
                            if (isvalid(params[6]) == false) // 高度不正确
                                res = false;
                            else
                                res = Position_Control_set_TargetPositionZGlobal(params[6] * 100, 0);
                        }
                        else // 经纬度正确进行三维飞行
                        {
                            if (isvalid(params[6]) == false) // 高度不正确不移动高度
                                res = Position_Control_set_TargetPositionXYZRelative_LatLon(params[4], params[5], 0, 0);
                            else
                                res = Position_Control_set_TargetPositionXYZ_LatLon(params[4], params[5], params[6] * 100, 0);
                        }
                        break;
                    }
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                    {
                        if (!isvalid(params[4]) || !isvalid(params[5]) ||
                            params[4] < -90 || params[4] > 90 ||
                            params[5] < -180 || params[5] > 180)
                        { // 经纬度不正确只进行高度调整
                            Position_Control_set_XYLock();
                            if (isvalid(params[6]) == false) // 高度不正确
                                res = false;
                            else
                                res = Position_Control_set_TargetPositionZRA(params[6] * 100, 0);
                        }
                        else // 经纬度正确进行三维飞行
                        {
                            if (isvalid(params[6]) == false) // 高度不正确不移动高度
                                res = Position_Control_set_TargetPositionXYZRelative_LatLon(params[4], params[5], 0, 0);
                            else
                                res = Position_Control_set_TargetPositionXYZRA_LatLon(params[4], params[5], params[6] * 100, 0);
                        }
                        break;
                    }
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                    {
                        if (!isvalid(params[4]) || !isvalid(params[5]) ||
                            params[4] < -90 || params[4] > 90 ||
                            params[5] < -180 || params[5] > 180)
                        { // 经纬度不正确只进行高度调整
                            Position_Control_set_XYLock();
                            if (isvalid(params[6]) == false) // 高度不正确
                                res = false;
                            else
                            {
                                PosSensorHealthInf1 pos_inf;
                                get_OptimalRange_Z(&pos_inf);
                                Position_Sensor_Data sensor;
                                if (GetPositionSensorData(pos_inf.sensor_ind, &sensor) && isvalid(params[6]))
                                { // 移动到指定高度
                                    res = true;
                                    Position_Control_set_ZLock();
                                    // res = Position_Control_set_TargetPositionZRelative( params[6]*100 - sensor.position.z, 0 );
                                    inf->temp[0] = -11;
                                }
                                else // 无高度数据报错
                                    res = false;
                            }
                        }
                        else // 经纬度正确进行三维飞行
                        {
                            if (isvalid(params[6]) == false)
                            { // 高度不正确
                                // 保持当前对地高度飞行
                                // 获取对地传感器高度
                                PosSensorHealthInf1 pos_inf;
                                get_OptimalRange_Z(&pos_inf);
                                Position_Sensor_Data sensor;
                                if (GetPositionSensorData(pos_inf.sensor_ind, &sensor) && isvalid(params[6]))
                                    inf->temp[0] = sensor.position.z;
                                else
                                { // 无定位锁定Z
                                    Position_Control_set_ZLock();
                                    inf->temp[0] = -11;
                                }
                            }
                            else
                            { // 按提供的高度飞行
                                inf->temp[0] = -1;
                            }
                            res = Position_Control_set_TargetPositionXY_LatLon(params[4], params[5], 0);
                        }
                        break;
                    }

                    case MAV_FRAME_LOCAL_NED:
                        if (isvalid(params[6]) == false) // 高度不正确不移动高度
                        {
                            Position_Control_set_ZLock();
                            res = Position_Control_set_TargetPositionXY(params[5] * 100, params[4] * 100, 0);
                        }
                        else
                            res = Position_Control_set_TargetPositionXYZ(params[5] * 100, params[4] * 100, -params[6] * 100, 0);
                        break;

                    case MAV_FRAME_LOCAL_ENU:
                        if (isvalid(params[6]) == false) // 高度不正确不移动高度
                        {
                            Position_Control_set_ZLock();
                            res = Position_Control_set_TargetPositionXY(params[4] * 100, params[5] * 100, 0);
                        }
                        else
                            res = Position_Control_set_TargetPositionXYZ(params[4] * 100, params[5] * 100, params[6] * 100, 0);
                        break;

                    case MAV_FRAME_LOCAL_OFFSET_NED:
                        if (isvalid(params[6]) == false) // 高度不正确不移动高度
                        {
                            Position_Control_set_ZLock();
                            res = Position_Control_set_TargetPositionXYRelative(params[5] * 100, params[4] * 100, 0);
                        }
                        else
                            res = Position_Control_set_TargetPositionXYZRelative(params[5] * 100, params[4] * 100, -params[6] * 100, 0);
                        break;

                    case MAV_FRAME_BODY_NED:
                    case MAV_FRAME_BODY_FRD:
                    case MAV_FRAME_BODY_OFFSET_NED:
                    {
                        // body系修正航向后只向前走
                        if (isvalid(params[3]) == false)
                        {
                            if (isvalid(params[6]) == false) // 高度不正确不移动高度
                            {
                                Position_Control_set_ZLock();
                                res = Position_Control_set_TargetPositionXYRelativeBodyheading(safe_sqrt(params[5] * params[5] + params[4] * params[4]) * 100, 0, 0);
                            }
                            else
                                res = Position_Control_set_TargetPositionXYZRelativeBodyheading(safe_sqrt(params[5] * params[5] + params[4] * params[4]) * 100, 0, -params[6] * 100, 0);
                        }
                        else
                        {
                            if (isvalid(params[6]) == false) // 高度不正确不移动高度
                            {
                                Position_Control_set_ZLock();
                                res = Position_Control_set_TargetPositionXYRelativeBodyheading(params[5] * 100, params[4] * 100, 0);
                            }
                            else
                                res = Position_Control_set_TargetPositionXYZRelativeBodyheading(params[5] * 100, params[4] * 100, -params[6] * 100, 0);
                        }
                        break;
                    }

                    case MAV_FRAME_BODY_FLU:
                    {
                        // body系修正航向后只向前走
                        if (isvalid(params[3]) == false)
                        {
                            if (isvalid(params[6]) == false) // 高度不正确不移动高度
                            {
                                Position_Control_set_ZLock();
                                res = Position_Control_set_TargetPositionXYRelativeBodyheading(safe_sqrt(params[4] * params[4] + params[5] * params[5]) * 100, 0, 0);
                            }
                            else
                                res = Position_Control_set_TargetPositionXYZRelativeBodyheading(safe_sqrt(params[4] * params[4] + params[5] * params[5]) * 100, 0, params[6] * 100, 0);
                        }
                        else
                        {
                            if (isvalid(params[6]) == false) // 高度不正确不移动高度
                            {
                                Position_Control_set_ZLock();
                                res = Position_Control_set_TargetPositionXYRelativeBodyheading(params[4] * 100, params[5] * 100, 0);
                            }
                            else
                                res = Position_Control_set_TargetPositionXYZRelativeBodyheading(params[4] * 100, params[5] * 100, params[6] * 100, 0);
                        }
                        break;
                    }

                    default:
                        inf->counter1 = inf->counter2 = 0;
                        return -100;
                }

                if (res)
                { // 成功
                    // Position_Control_set_WpRange(params[1]>0 ? params[1]*100 : params[1]);
                    inf->counter1 = 2;
                    inf->counter2 = 0;
                }
                else
                { // 失败
                    inf->counter1 = 0;
                    inf->counter2 = 0;
                    return -100;
                }
            }
            break;
        }

        case 2:
        { // 等待航点飞行完成
            Position_ControlMode alt_mode, pos_mode;
            get_Altitude_ControlMode(&alt_mode);
            get_Position_ControlMode(&pos_mode);
            if (pos_mode == Position_ControlMode_Position)
            {
                Position_Control_set_XYLock();
            }

            // 获取避障参数
            double AvDist = -1;
            const AvoidanceCfg *avCfg = getAvCfg();
            if (Is_AutoMode(pos_mode))
            { // 航线避障
                if (AvMode_Enabled(avCfg->AvoidanceMode[0]))
                { // 需要进行避障
                    double AvDistance = -1;
                    vector3<double> lineAB;
                    Position_Control_get_LineFlightABDistance(&lineAB, 0);
                    get_AvLineDistanceEnu(&AvDistance, -lineAB, avCfg->wheelbase[0]);
                    if (AvDistance >= 0)
                    {
                        AvDistance -= avCfg->AvoidanceDist[0] + 0.5f * avCfg->wheelbase[0];
                        if (AvDistance < 0)
                            AvDistance = 0;
                        AvDist = AvDistance;
                    }
                }
            }

            bool terrain_mode = false;
            if (inf->temp[0] > -10 && (frame == MAV_FRAME_GLOBAL_TERRAIN_ALT || frame == MAV_FRAME_GLOBAL_TERRAIN_ALT_INT))
            {
                terrain_mode = true;

                Receiver rc;
                getReceiver(&rc);
                // 读取模式配置
                float NeutralZone[2] = {0};
                ReadParam("MFunc_NeutralZ", 0, 0, (uint64_t *)NeutralZone, 0);

                bool thrNeutral = true;
                if (rc.available && !in_symmetry_range_mid(rc.data[0], 50, NeutralZone[0]))
                    thrNeutral = false;

                if (thrNeutral == false)
                {
                    double thr_stick = remove_deadband(rc.data[0] - 50.0, (double)NeutralZone[0]);
                    if (thr_stick > 0)
                        thr_stick *= getPosCtrlCfg()->maxVelUp[0] / 50;
                    else
                        thr_stick *= getPosCtrlCfg()->maxVelDown[0] / 50;
                    Position_Control_set_TargetVelocityZ(thr_stick);
                }
                else
                {
                    PosSensorHealthInf1 pos_inf;
                    get_OptimalRange_Z(&pos_inf);
                    Position_Sensor_Data sensor;
                    if (GetPositionSensorData(pos_inf.sensor_ind, &sensor) && isvalid(params[6]))
                    { // 根据测高传感器调整对地高度
                        vector3<double> pos;
                        get_Position_Ctrl(&pos);
                        double target_height;
                        if (inf->temp[0] > 2)
                            target_height = inf->temp[0];
                        else if (isvalid(params[6]))
                            target_height = params[6] * 100;
                        else
                            target_height = 500;

                        double err = target_height - sensor.position.z;
                        Position_Control_set_TargetVelocityZ(constrain(err, 200.0));

                        // 根据对地高度限速
                        double maxFrontPointL = -1;
                        double abs_err = fabs(err);

                        double errLimit = getPosCtrlCfg()->maxVelXY[0] - 3 * (abs_err - 50);
                        double groundLimit = 1.0 * (sensor.position.z - 20);
                        if (groundLimit < 20)
                            groundLimit = 20;

                        if (errLimit < groundLimit)
                            maxFrontPointL = errLimit;
                        else
                            maxFrontPointL = groundLimit;

                        if (maxFrontPointL >= 0 && (AvDist < 0 || maxFrontPointL < AvDist))
                            AvDist = maxFrontPointL;
                    }
                    else
                    { // 无高度传感器锁定高度
                        Position_Control_set_ZLock();
                    }
                }
            }
            else
            {
                if (alt_mode == Position_ControlMode_Locking || alt_mode == Position_ControlMode_Position)
                    Position_Control_set_ZLock();
            }

            // 设置避障距离
            Position_Control_set_RouteLineAvoidanceRelative(AvDist);

            if ((terrain_mode || alt_mode == Position_ControlMode_Position) && pos_mode == Position_ControlMode_Position)
            { // 飞行完成进入hold等待
                inf->counter1 = 3;
                inf->counter2 = 0;
            }
            return -3;
            break;
        }

        case 3:
        { // hold
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            if (++(inf->counter2) >= freq * params[0])
            {
                inf->counter1 = 0;
                inf->counter2 = 0;
                return -1;
            }
            return -3;
            break;
        }
    }
    return -2;
}
/*NavCmd192_DO_REPOSITION
        MAV_CMD_DO_REPOSITION
        航点飞行（调转机头并飞行到指定点）
        参数:
                <description>Reposition the vehicle to a specific WGS84 global position.</description>
                <param index="1" label="Speed" units="m/s" minValue="-1">Ground speed, less than 0 (-1) for default</param>
                <param index="2" label="Bitmask" enum="MAV_DO_REPOSITION_FLAGS">Bitmask of option flags.</param>
                <param index="3">Reserved</param>
                <param index="4" label="Yaw" units="deg">Yaw heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise)</param>
                <param index="5" label="Latitude">Latitude</param>
                <param index="6" label="Longitude">Longitude</param>
                <param index="7" label="Altitude" units="m">Altitude</param>
*/
static int32_t NavCmd192_DO_REPOSITION(double freq, uint8_t frame, double params[], NavCmdInf *inf)
{
    if (get_Position_MSStatus() != MS_Ready)
    { // 无定位出错
        inf->counter1 = inf->counter2 = 0;
        return -100;
    }
    bool inFlight;
    get_is_inFlight(&inFlight);
    if (inFlight == false)
    { // 未起飞出错
        inf->counter1 = inf->counter2 = 0;
        return -100;
    }

    switch (inf->counter1)
    {
        case 0:
        { // 先升高
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            Attitude_Control_set_YawLock();

            Position_ControlMode alt_mode, pos_mode;
            get_Altitude_ControlMode(&alt_mode);
            get_Position_ControlMode(&pos_mode);

            if (pos_mode == Position_ControlMode_Position && alt_mode == Position_ControlMode_Position)
            {
                uint32_t bitMask = params[1];

                vector3<double> OC;
                NavCmd16_WAYPOINT_GetInfo(frame, params, 0, &OC, 0);
                // 执行结果
                if (OC.z > 0 && (bitMask & MAV_DO_REPOSITION_FLAGS_ALTITUDE_MODE))
                {                     // 需要升高
                    inf->temp[1] = 0; // 不需要降高
                    bool res = false;
                    switch (frame)
                    {
                        case MAV_FRAME_GLOBAL_INT:
                        case MAV_FRAME_GLOBAL:
                        {
                            res = Position_Control_set_TargetPositionZGlobal(params[6] * 100, 0);
                            break;
                        }
                        case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                        {
                            res = Position_Control_set_TargetPositionZRA(params[6] * 100, 0);
                            break;
                        }
                        case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                        case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                        {
                            res = Position_Control_set_TargetPositionZRA(params[6] * 100, 0);
                            break;
                        }

                        case MAV_FRAME_LOCAL_NED:
                            res = Position_Control_set_TargetPositionZ(-(params[6] * 100), 0);
                            break;

                        case MAV_FRAME_LOCAL_ENU:
                            res = Position_Control_set_TargetPositionZ(params[6] * 100, 0);
                            break;

                        case MAV_FRAME_BODY_NED:
                        case MAV_FRAME_BODY_FRD:
                        case MAV_FRAME_BODY_OFFSET_NED:
                        case MAV_FRAME_LOCAL_OFFSET_NED:
                            res = Position_Control_set_TargetPositionZRelative(-(params[6] * 100), 0);
                            break;

                        case MAV_FRAME_BODY_FLU:
                        {
                            res = Position_Control_set_TargetPositionZRelative(params[6] * 100, 0);
                            break;
                        }

                        default:
                            inf->counter1 = inf->counter2 = 0;
                            return -100;
                    }

                    if (res)
                    { // 成功
                        inf->counter1 = 1;
                        inf->counter2 = 0;
                    }
                    else
                    { // 失败
                        inf->counter1 = 0;
                        inf->counter2 = 0;
                        return -100;
                    }
                }
                else
                {
                    if (OC.z < 0) // 需要降高
                        inf->temp[1] = 1;
                    else
                        inf->temp[1] = 0;
                    inf->counter1 = 1;
                    inf->counter2 = 0;
                }
            }
            break;
        }

        case 1:
        { // 判断执行旋转偏航
            Attitude_Control_set_YawLock();
            Position_ControlMode alt_mode;
            get_Altitude_ControlMode(&alt_mode);
            Position_Control_set_XYLock();

            if (alt_mode == Position_ControlMode_Position)
            { // 高度调整完成
                if (isvalid(params[3]) == false)
                { // 机头指向航点方向
                    Position_Control_set_XYLock();
                    Position_Control_set_ZLock();

                    // 获取当前航点Local坐标
                    vector3<double> OC;
                    if (NavCmd16_WAYPOINT_GetInfo(frame, params, 0, &OC, 0) && sq(OC.x) + sq(OC.y) > sq(5))
                        Attitude_Control_set_Target_Yaw(atan2(OC.y, OC.x));

                    inf->counter1 = 2;
                    inf->counter2 = 0;
                }
                else if (params[3] > -360 && params[3] < 360)
                { // 指定偏航朝向
                    Attitude_Control_set_Target_Yaw(degree2rad(90 - params[3]));
                    inf->counter1 = 2;
                    inf->counter2 = 0;
                }
                else
                { // 不旋转偏航
                    inf->counter1 = 2;
                    inf->counter2 = 0;
                }
            }
            break;
        }

        case 2:
        { // 等待偏航旋转完成
            Position_Control_set_ZLock();
            Position_Control_set_XYLock();
            double yawTrackErr;
            Attitude_Control_get_YawTrackErr(&yawTrackErr);
            if (yawTrackErr < 0.01)
            { // 偏航旋转完成
                Attitude_Control_set_YawLock();

                double vel = -1;
                if (params[0] >= 0)
                    vel = params[0] * 100;

                // 获取仿地高度
                switch (frame)
                {
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                    {
                        PosSensorHealthInf1 pos_inf;
                        get_OptimalRange_Z(&pos_inf);
                        Position_Sensor_Data sensor;
                        if (GetPositionSensorData(pos_inf.sensor_ind, &sensor) && isvalid(params[6]))
                            inf->temp[0] = sensor.position.z;
                        break;
                    }
                    default:
                        break;
                }

                // 获取当前航点Local坐标
                vector3<double> pC;
                bool res = false;
                if (NavCmd16_WAYPOINT_GetInfo(frame, params, 0, 0, &pC))
                {
                    uint32_t bitMask = params[1];
                    if (bitMask & MAV_DO_REPOSITION_FLAGS_ALTITUDE_MODE)
                        res = Position_Control_set_TargetPositionXY(pC.x, pC.y, vel);
                    else
                        res = Position_Control_set_TargetPositionXYZ(pC.x, pC.y, pC.z, vel);
                }
                if (res)
                { // 成功
                    inf->counter1 = 3;
                    inf->counter2 = 0;
                }
                else
                { // 失败
                    inf->counter1 = 0;
                    inf->counter2 = 0;
                    return -100;
                }
            }
            break;
        }

        case 3:
        { // 等待航点飞行完成
            Position_ControlMode alt_mode, pos_mode;
            get_Altitude_ControlMode(&alt_mode);
            get_Position_ControlMode(&pos_mode);
            if (alt_mode == Position_ControlMode_Position)
                Position_Control_set_ZLock();

            // 获取避障参数
            double AvDist = -1;
            const AvoidanceCfg *avCfg = getAvCfg();
            if (Is_AutoMode(pos_mode))
            { // 航线避障
                if (AvMode_Enabled(avCfg->AvoidanceMode[0]))
                { // 需要进行避障
                    double AvDistance = -1;
                    vector3<double> lineAB;
                    Position_Control_get_LineFlightABDistance(&lineAB, 0);
                    get_AvLineDistanceEnu(&AvDistance, -lineAB, avCfg->wheelbase[0]);
                    if (AvDistance >= 0)
                    {
                        AvDistance -= avCfg->AvoidanceDist[0] + 0.5f * avCfg->wheelbase[0];
                        if (AvDistance < 0)
                            AvDistance = 0;
                        AvDist = AvDistance;
                    }
                }
            }

            bool terrain_mode = false;
            if (inf->temp[0] > -10 && (frame == MAV_FRAME_GLOBAL_TERRAIN_ALT || frame == MAV_FRAME_GLOBAL_TERRAIN_ALT_INT))
            {
                terrain_mode = true;

                PosSensorHealthInf1 pos_inf;
                get_OptimalRange_Z(&pos_inf);
                Position_Sensor_Data sensor;
                if (GetPositionSensorData(pos_inf.sensor_ind, &sensor) && isvalid(params[6]))
                { // 根据测高传感器调整对地高度
                    vector3<double> pos;
                    get_Position_Ctrl(&pos);
                    double target_height;
                    if (inf->temp[0] > 2)
                        target_height = inf->temp[0];
                    else if (isvalid(params[6]))
                        target_height = params[6] * 100;
                    else
                        target_height = 500;

                    double err = target_height - sensor.position.z;
                    Position_Control_set_TargetVelocityZ(constrain(err, 200.0));

                    // 根据对地高度限速
                    double maxFrontPointL = -1;
                    double abs_err = fabs(err);

                    double errLimit = getPosCtrlCfg()->maxVelXY[0] - 3 * (abs_err - 50);
                    double groundLimit = 1.0 * (sensor.position.z - 20);
                    if (groundLimit < 20)
                        groundLimit = 20;

                    if (errLimit < groundLimit)
                        maxFrontPointL = errLimit;
                    else
                        maxFrontPointL = groundLimit;

                    if (maxFrontPointL >= 0 && (AvDist < 0 || maxFrontPointL < AvDist))
                        AvDist = maxFrontPointL;
                }
                else
                { // 无高度传感器锁定高度
                    Position_Control_set_ZLock();
                }
            }

            // 设置避障距离
            Position_Control_set_RouteLineAvoidanceRelative(AvDist);

            if (pos_mode == Position_ControlMode_Position)
            { // 飞行完成判断是否需要下降高度
                if (inf->temp[1] == 1)
                { // 需要降高
                    inf->counter1 = 4;
                    inf->counter2 = 0;
                }
                else
                { // 不需要降高
                    inf->counter1 = 0;
                    inf->counter2 = 0;
                    return -1;
                }
            }
            return -3;
            break;
        }

        case 4:
        { // 需要降高
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            Attitude_Control_set_YawLock();

            uint32_t bitMask = params[1];

            // 执行结果
            bool res = false;
            switch (frame)
            {
                case MAV_FRAME_GLOBAL_INT:
                case MAV_FRAME_GLOBAL:
                {
                    res = Position_Control_set_TargetPositionZGlobal(params[6] * 100, 0);
                    break;
                }
                case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                {
                    res = Position_Control_set_TargetPositionZRA(params[6] * 100, 0);
                    break;
                }
                case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                {
                    res = Position_Control_set_TargetPositionZRA(params[6] * 100, 0);
                    break;
                }

                case MAV_FRAME_LOCAL_NED:
                    res = Position_Control_set_TargetPositionZ(-(params[6] * 100), 0);
                    break;

                case MAV_FRAME_LOCAL_ENU:
                    res = Position_Control_set_TargetPositionZ(params[6] * 100, 0);
                    break;

                case MAV_FRAME_BODY_NED:
                case MAV_FRAME_BODY_FRD:
                case MAV_FRAME_BODY_OFFSET_NED:
                case MAV_FRAME_LOCAL_OFFSET_NED:
                    res = Position_Control_set_TargetPositionZRelative(-(params[6] * 100), 0);
                    break;

                case MAV_FRAME_BODY_FLU:
                {
                    res = Position_Control_set_TargetPositionZRelative(params[6] * 100, 0);
                    break;
                }

                default:
                    inf->counter1 = inf->counter2 = 0;
                    return -100;
            }

            if (res)
            { // 成功
                inf->counter1 = 5;
                inf->counter2 = 0;
            }
            else
            { // 失败
                inf->counter1 = 0;
                inf->counter2 = 0;
                return -100;
            }
            break;
        }

        case 5:
        { // 判断高度下降完成
            Attitude_Control_set_YawLock();
            Position_ControlMode alt_mode;
            get_Altitude_ControlMode(&alt_mode);
            Position_Control_set_XYLock();

            if (alt_mode == Position_ControlMode_Position)
            { // 高度调整完成
                inf->counter1 = 0;
                inf->counter2 = 0;
                return -1;
            }
            break;
        }
    }
    return -2;
}

/*NavCmd20_RETURN_TO_LAUNCH
        MAV_CMD_NAV_RETURN_TO_LAUNCH
        回到起飞点
        参数:
                <description>Return to launch location</description>
                <param index="1">Empty</param>
                <param index="2">Empty</param>
                <param index="3">Empty</param>
                <param index="4">Empty</param>
                <param index="5">Empty</param>
                <param index="6">Empty</param>
                <param index="7">Empty</param>
*/
static int32_t NavCmd20_RETURN_TO_LAUNCH(double freq, uint8_t frame, double params[], NavCmdInf *inf)
{
    if (get_Position_MSStatus() != MS_Ready)
    { // 无定位出错
        inf->counter1 = inf->counter2 = 0;
        return -100;
    }
    bool inFlight;
    get_is_inFlight(&inFlight);
    if (inFlight == false)
    { // 未起飞出错
        inf->counter1 = inf->counter2 = 0;
        return -10;
    }

    switch (inf->counter1)
    {
        case 0:
        { // 旋转偏航
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();

            Position_ControlMode pos_mode, alt_mode;
            get_Position_ControlMode(&pos_mode);
            get_Altitude_ControlMode(&alt_mode);

            if (pos_mode == Position_ControlMode_Position && alt_mode == Position_ControlMode_Position)
            {
                double LA, LB;

                vector2<double> homeP;
                float RtRotateRange[2];
                RtRotateRange[0] = 1000;

                // 先判断跟随传感器状态
                bool followSensorAvailable = false;
                followSensor followS;
                if (get_followSensor(&followS) && followS.available &&
                    isFollowDataType_NeedRTL(followS.dataType) &&
                    (!isFollowDataType_GlobalXY(followS.dataType) || followS.globalXYPosAvailable))
                    followSensorAvailable = true;

                if (followSensorAvailable)
                {
                    // 求LA LB
                    vector3<double> t_pos;
                    get_TargetPosInf(0, 0, &t_pos, 0);
                    LA = followS.pos.y - t_pos.y;
                    LB = followS.pos.x - t_pos.x;

                    // 获取掉头范围
                    ReadParam("Sf_GbRtHRange", 0, 0, (uint64_t *)RtRotateRange, 0);
                }
                else if (getHomeLatLon(&homeP))
                { // 返航至经纬度

                    // 获取最优全球定位传感器信息
                    PosSensorHealthInf2 global_inf;
                    if (get_OptimalGlobal_XY(&global_inf) == false)
                    {
                        if (getHomePoint(&homeP))
                            goto TurnYawLocalP;
                        else
                        { // 无返航点
                            inf->counter1 = inf->counter2 = 0;
                            return -100;
                        }
                    }
                    // 获取指定经纬度平面坐标
                    double x, y;
                    map_projection_project(&global_inf.mp, homeP.x, homeP.y, &x, &y);
                    x -= global_inf.HOffset.x;
                    y -= global_inf.HOffset.y;
                    // 求LA LB
                    vector3<double> t_pos;
                    get_TargetPosInf(0, 0, &t_pos, 0);
                    LA = y - t_pos.y;
                    LB = x - t_pos.x;

                    // 获取掉头范围
                    ReadParam("Sf_GbRtHRange", 0, 0, (uint64_t *)RtRotateRange, 0);
                }
                else if (getHomePoint(&homeP))
                { // 返航至Local坐标
                TurnYawLocalP:
                    vector3<double> position;
                    get_Position_Ctrl(&position);
                    LA = params[5] - position.y;
                    LB = params[4] - position.x;

                    // 获取掉头范围
                    ReadParam("Sf_LcRtHRange", 0, 0, (uint64_t *)RtRotateRange, 0);
                }
                else
                { // 无返航点
                    inf->counter1 = inf->counter2 = 0;
                    return -100;
                }

                // 设置偏航角
                if (sq(LA) + sq(LB) > sq(RtRotateRange[0]))
                    Attitude_Control_set_Target_Yaw(atan2(LA, LB));
                inf->counter1 = 1;
                inf->counter2 = 0;
            }
            break;
        }

        case 1:
        { // 等待偏航旋转开始航点飞行
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            double yawTrackErr;
            Attitude_Control_get_YawTrackErr(&yawTrackErr);
            if (yawTrackErr < 0.01)
            {
                bool res;

                // 获取返航速度
                float RTL_speed[2];
                RTL_speed[0] = -1;
                ReadParam("Sf_RtSpeed", 0, 0, (uint64_t *)RTL_speed, 0);

                vector2<double> homeP;
                // 先判断跟随传感器状态
                bool followSensorAvailable = false;
                followSensor followS;
                if (get_followSensor(&followS) && followS.available &&
                    isFollowDataType_NeedRTL(followS.dataType) &&
                    (!isFollowDataType_GlobalXY(followS.dataType) || followS.globalXYPosAvailable))
                    followSensorAvailable = true;
                if (followSensorAvailable)
                { // 返航至跟随点
                    res = Position_Control_set_TargetPositionXY(followS.pos.x, followS.pos.y, RTL_speed[0]);
                }
                else if (getHomeLatLon(&homeP))
                { // 返航至经纬度
                    res = Position_Control_set_TargetPositionXY_LatLon(homeP.x, homeP.y, RTL_speed[0]);
                    if (res == false)
                    {
                        if (getHomePoint(&homeP))
                            goto FlyLocalP;
                        else
                        { // 无返航点
                            inf->counter1 = inf->counter2 = 0;
                            return -100;
                        }
                    }
                }
                else if (getHomePoint(&homeP))
                { // 返航至local坐标
                FlyLocalP:
                    res = Position_Control_set_TargetPositionXY(homeP.x, homeP.y, RTL_speed[0]);
                }
                else
                { // 无返航点
                    inf->counter1 = inf->counter2 = 0;
                    return -100;
                }

                if (res)
                { // 成功
                    inf->counter1 = 2;
                    inf->counter2 = 0;
                }
                else
                { // 失败
                    inf->counter1 = 0;
                    inf->counter2 = 0;
                    return -100;
                }
            }
            break;
        }

        case 2:
        { // 等待航点飞行完成
            Position_Control_set_ZLock();
            Position_ControlMode pos_mode;
            get_Position_ControlMode(&pos_mode);
            if (pos_mode == Position_ControlMode_Position)
            { // 飞行完成
                inf->counter1 = 3;
                inf->counter2 = 0;
            }
            return -3;
            break;
        }

        case 3:
        { // 旋转至起飞航向
            double homeYaw;
            getHomeLocalZ(0, &homeYaw);
            Attitude_Control_set_Target_Yaw(homeYaw);
            inf->counter1 = 4;
            inf->counter2 = 0;
            break;
        }

        case 4:
        { // 等待偏航旋转
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            double yawTrackErr;
            Attitude_Control_get_YawTrackErr(&yawTrackErr);
            if (yawTrackErr < 0.01)
            {
                inf->counter1 = 5;
                inf->counter2 = 0;
            }
            break;
        }

        case 5:
        { // 降落
            precLandSensor precSensor;
            if (get_precLandSensor(&precSensor) && precSensor.available && precSensor.updateTime.get_pass_time() < 0.2)
            {
                Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit(precSensor.pos.x, precSensor.pos.y, 0.3);
            }
            else
                Position_Control_set_XYLock();

            // 获取对地高度
            double homeZ;
            getHomeLocalZ(&homeZ);
            vector3<double> pos;
            get_Position_Ctrl(&pos);
            double height = pos.z - homeZ;
            double homeSenosrZ;
            if (getHomeOptSensorZ(&homeSenosrZ))
            {
                if (homeSenosrZ < height)
                    height = homeSenosrZ;
            }

            // 获取降落速度
            float sp[2];
            sp[0] = 200;
            ReadParam("PC_maxAutoVelDn", 0, 0, (uint64_t *)sp, 0);
            if (sp[0] < 100)
                sp[0] = 100;

#define max_acc 50.0
            double land_vel = getPosCtrlCfg()->LandVel[0];
            if (height > 1000 + sp[0] * 2)
                Position_Control_set_TargetVelocityZ(-sp[0]);
            else if (height > 1000)
                Position_Control_set_TargetVelocityZ(-((height - 1000) * (sp[0] - land_vel) / sp[0] / 2 + land_vel));
            else
                Position_Control_set_TargetVelocityZ(-land_vel);

            if (inFlight == false)
            { // 降落完成
                inf->counter1 = 0;
                inf->counter2 = 0;
                // 已降落要求停止
                return -10;
            }

            return -3;
            break;
        }
    }
    return -2;
}

/*NavCmd21_LAND
        MAV_CMD_NAV_LAND
        原地降落
        参数:
                <description>Land at location.</description>
                <param index="1" label="Abort Alt" units="m">Minimum target altitude if landing is aborted (0 = undefined/use system default).</param>
                <param index="2" label="Land Mode" enum="PRECISION_LAND_MODE">Precision land mode.</param>
                <param index="3">Empty</param>
                <param index="4" label="Yaw Angle" units="deg">Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).</param>
                <param index="5" label="Latitude">Latitude.</param>
                <param index="6" label="Longitude">Longitude.</param>
                <param index="7" label="Altitude" units="m">Landing altitude (ground level in current frame).</param>
*/
static int32_t NavCmd21_LAND(double freq, uint8_t frame, double params[], NavCmdInf *inf)
{
    bool inFlight;
    get_is_inFlight(&inFlight);
    if (inFlight == false)
    { // 未起飞出错
        inf->counter1 = inf->counter2 = 0;
        return -10;
    }

    if (get_Position_MSStatus() != MS_Ready)
    { // 无定位出错
        inf->counter1 = inf->counter2 = 0;
        return -100;
    }

    switch (inf->counter1)
    {
        case 0:
        { // 判断执行旋转偏航
            if (isvalid(params[3]) == false)
            { // 机头指向航点方向
                Position_Control_set_XYLock();
                Position_Control_set_ZLock();

                double LA, LB;
                switch (frame)
                {
                    case MAV_FRAME_GLOBAL:
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                    case MAV_FRAME_GLOBAL_INT:
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                    { // 全球定位
                        if (params[4] < -90 || params[4] > 90 || params[5] < -180 || params[5] > 180)
                        { // 经纬度为不正确不转偏航
                            inf->counter1 = 1;
                            inf->counter2 = freq * 3;
                            return -2;
                        }

                        // 获取最优全球定位传感器信息
                        PosSensorHealthInf2 global_inf;
                        if (get_OptimalGlobal_XY(&global_inf) == false)
                        {
                            inf->counter1 = inf->counter2 = 0;
                            return -100;
                        }
                        // 获取指定经纬度平面坐标
                        double x, y;
                        map_projection_project(&global_inf.mp, params[4], params[5], &x, &y);
                        x -= global_inf.HOffset.x;
                        y -= global_inf.HOffset.y;
                        // 求LA LB
                        vector3<double> t_pos;
                        get_TargetPosInf(0, 0, &t_pos, 0);
                        LA = y - t_pos.y;
                        LB = x - t_pos.x;
                        break;
                    }

                    case MAV_FRAME_LOCAL_NED:
                    {
                        vector3<double> position;
                        get_Position_Ctrl(&position);
                        LA = params[4] * 100 - position.y;
                        LB = params[5] * 100 - position.x;
                        break;
                    }

                    case MAV_FRAME_LOCAL_ENU:
                    {
                        vector3<double> position;
                        get_Position_Ctrl(&position);
                        LA = params[5] * 100 - position.y;
                        LB = params[4] * 100 - position.x;
                        break;
                    }

                    case MAV_FRAME_LOCAL_OFFSET_NED:
                    {
                        LA = params[4];
                        LB = params[5];
                        break;
                    }

                    case MAV_FRAME_BODY_NED:
                    case MAV_FRAME_BODY_FRD:
                    case MAV_FRAME_BODY_OFFSET_NED:
                    {
                        Quaternion att;
                        get_Attitude_quat(&att);
                        double yaw = att.getYaw();
                        double sin_Yaw, cos_Yaw;
                        fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                        double posx_enu = BodyHeading2ENU_x(params[5], params[4], sin_Yaw, cos_Yaw);
                        double posy_enu = BodyHeading2ENU_y(params[5], params[4], sin_Yaw, cos_Yaw);
                        LA = posy_enu;
                        LB = posx_enu;
                        break;
                    }

                    case MAV_FRAME_BODY_FLU:
                    {
                        Quaternion att;
                        get_Attitude_quat(&att);
                        double yaw = att.getYaw();
                        double sin_Yaw, cos_Yaw;
                        fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                        double posx_enu = BodyHeading2ENU_x(params[4], params[5], sin_Yaw, cos_Yaw);
                        double posy_enu = BodyHeading2ENU_y(params[4], params[5], sin_Yaw, cos_Yaw);
                        LA = posy_enu;
                        LB = posx_enu;
                        break;
                    }

                    default:
                        inf->counter1 = inf->counter2 = 0;
                        return -100;
                }

                if (sq(LA) + sq(LB) > sq(200))
                    Attitude_Control_set_Target_Yaw(atan2(LA, LB));
                inf->counter1 = 1;
                inf->counter2 = 0;
            }
            else if (params[3] > -360 && params[3] < 360)
            { // 指定偏航朝向
                Attitude_Control_set_Target_Yaw(degree2rad(90 - params[3]));
                inf->counter1 = 1;
                inf->counter2 = 0;
            }
            else
            { // 不旋转偏航
                inf->counter1 = 1;
                inf->counter2 = freq * 3;
            }
            break;
        }

        case 1:
        { // 等待偏航旋转开始航点飞行
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            double yawTrackErr;
            Attitude_Control_get_YawTrackErr(&yawTrackErr);
            if (yawTrackErr < 0.01)
            {
                bool res;
                switch (frame)
                {
                    case MAV_FRAME_GLOBAL_INT:
                    case MAV_FRAME_GLOBAL:
                    {
                        if (params[4] < -90 || params[4] > 90 || params[5] < -180 || params[5] > 180)
                        { // 经纬度不正确直接降落
                            Position_Control_set_XYLock();
                            res = true;
                        }
                        else // 经纬度正确进行三维飞行
                            res = Position_Control_set_TargetPositionXY_LatLon(params[4], params[5], 0);
                        break;
                    }
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                    case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                    {
                        if (params[4] < -90 || params[4] > 90 || params[5] < -180 || params[5] > 180)
                        { // 经纬度不正确直接降落
                            Position_Control_set_XYLock();
                            res = true;
                        }
                        else // 经纬度正确进行三维飞行
                            res = Position_Control_set_TargetPositionXY_LatLon(params[4], params[5], 0);
                        break;
                    }
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                    case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                    {
                        if (params[4] < -90 || params[4] > 90 || params[5] < -180 || params[5] > 180)
                        { // 经纬度不正确直接降落
                            Position_Control_set_XYLock();
                            res = true;
                        }
                        else // 经纬度正确进行三维飞行
                            res = Position_Control_set_TargetPositionXY_LatLon(params[4], params[5], 0);
                        break;
                    }

                    case MAV_FRAME_LOCAL_NED:
                        res = Position_Control_set_TargetPositionXY(params[5] * 100, params[4] * 100, 0);
                        break;

                    case MAV_FRAME_LOCAL_ENU:
                        res = Position_Control_set_TargetPositionXY(params[4] * 100, params[5] * 100, 0);
                        break;

                    case MAV_FRAME_LOCAL_OFFSET_NED:
                        res = Position_Control_set_TargetPositionXYRelative(params[5] * 100, params[4] * 100, 0);
                        break;

                    case MAV_FRAME_BODY_NED:
                    case MAV_FRAME_BODY_FRD:
                    case MAV_FRAME_BODY_OFFSET_NED:
                    {
                        // body系修正航向后只向前走
                        if (isvalid(params[3]) == false)
                            res = Position_Control_set_TargetPositionXYRelativeBodyheading(safe_sqrt(params[5] * params[5] + params[4] * params[4]) * 100, 0, 0);
                        else
                            res = Position_Control_set_TargetPositionXYRelativeBodyheading(params[5] * 100, params[4] * 100, 0);
                        break;
                    }

                    case MAV_FRAME_BODY_FLU:
                    {
                        // body系修正航向后只向前走
                        if (isvalid(params[3]) == false)
                            res = Position_Control_set_TargetPositionXYRelativeBodyheading(safe_sqrt(params[4] * params[4] + params[5] * params[5]) * 100, 0, 0);
                        else
                            res = Position_Control_set_TargetPositionXYRelativeBodyheading(params[4] * 100, params[5] * 100, 0);
                        break;
                    }

                    default:
                        inf->counter1 = inf->counter2 = 0;
                        return -100;
                }

                if (res)
                { // 成功
                    inf->counter1 = 2;
                    inf->counter2 = 0;
                }
                else
                { // 失败
                    inf->counter1 = 0;
                    inf->counter2 = 0;
                    return -100;
                }
            }
            break;
        }

        case 2:
        { // 等待航点飞行完成
            Position_ControlMode alt_mode, pos_mode;
            get_Altitude_ControlMode(&alt_mode);
            get_Position_ControlMode(&pos_mode);
            if (alt_mode == Position_ControlMode_Position)
                Position_Control_set_ZLock();
            if (pos_mode == Position_ControlMode_Position)
                Position_Control_set_XYLock();
            if (alt_mode == Position_ControlMode_Position && pos_mode == Position_ControlMode_Position)
            { // 飞行完成进入hold等待
                inf->counter1 = 3;
                inf->counter2 = 0;
            }
            return -3;
            break;
        }

        case 3:
        { // 降落

            Position_Control_set_XYLock();
            // 获取对地高度
            double homeZ;
            getHomeLocalZ(&homeZ);
            vector3<double> pos;
            get_Position_Ctrl(&pos);
            double height = pos.z - homeZ;

            // 获取降落速度
            float sp[2];
            sp[0] = 200;
            ReadParam("PC_maxAutoVelDn", 0, 0, (uint64_t *)sp, 0);
            if (sp[0] < 100)
                sp[0] = 100;

            float ground_height = params[6] * 100;

#define max_acc 50.0
            double land_vel = getPosCtrlCfg()->LandVel[0];
            if (height > 1000 + ground_height + sp[0] * 2)
                Position_Control_set_TargetVelocityZ(-sp[0]);
            else if (height > 1000 + ground_height)
                Position_Control_set_TargetVelocityZ(-((height - 1000 - ground_height) * (sp[0] - land_vel) / sp[0] / 2 + land_vel));
            else
                Position_Control_set_TargetVelocityZ(-land_vel);

            if (inFlight == false)
            { // 降落完成
                inf->counter1 = 0;
                inf->counter2 = 0;
                // 已降落要求停止
                return -10;
            }

            return -3;
            break;
        }
    }
    return -2;
}

/*NavCmd23_LAND_LOCAL
        MAV_CMD_NAV_LAND_LOCAL
        原地降落
        参数:
                <description>Land at local position (local frame only)</description>
                <param index="1" label="Target" minValue="0" increment="1">Landing target number (if available)</param>
                <param index="2" label="Offset" units="m" minValue="0">Maximum accepted offset from desired landing position - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land</param>
                <param index="3" label="Descend Rate" units="m/s">Landing descend rate</param>
                <param index="4" label="Yaw" units="rad">Desired yaw angle</param>
                <param index="5" label="Y Position" units="m">Y-axis position</param>
                <param index="6" label="X Position" units="m">X-axis position</param>
                <param index="7" label="Z Position" units="m">Z-axis / ground level position</param>
*/
static int32_t NavCmd23_LAND_LOCAL(double freq, uint8_t frame, double params[], NavCmdInf *inf)
{
    bool inFlight;
    get_is_inFlight(&inFlight);
    if (inFlight == false)
    { // 未起飞出错
        inf->counter1 = inf->counter2 = 0;
        return -10;
    }

    if (get_Position_MSStatus() != MS_Ready)
    { // 无定位出错
        inf->counter1 = inf->counter2 = 0;
        return -100;
    }

    switch (inf->counter1)
    {
        case 0:
        { // 判断执行旋转偏航
            if (isvalid(params[3]) == false)
            { // 机头指向航点方向
                Position_Control_set_XYLock();
                Position_Control_set_ZLock();

                double LA, LB;
                switch (frame)
                {
                    case MAV_FRAME_LOCAL_NED:
                    {
                        vector3<double> position;
                        get_Position_Ctrl(&position);
                        LA = params[4] * 100 - position.y;
                        LB = params[5] * 100 - position.x;
                        break;
                    }

                    case MAV_FRAME_LOCAL_ENU:
                    {
                        vector3<double> position;
                        get_Position_Ctrl(&position);
                        LA = params[5] * 100 - position.y;
                        LB = params[4] * 100 - position.x;
                        break;
                    }

                    case MAV_FRAME_LOCAL_OFFSET_NED:
                    {
                        LA = params[4];
                        LB = params[5];
                        break;
                    }

                    case MAV_FRAME_BODY_NED:
                    case MAV_FRAME_BODY_FRD:
                    case MAV_FRAME_BODY_OFFSET_NED:
                    {
                        Quaternion att;
                        get_Attitude_quat(&att);
                        double yaw = att.getYaw();
                        double sin_Yaw, cos_Yaw;
                        fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                        double posx_enu = BodyHeading2ENU_x(params[5], params[4], sin_Yaw, cos_Yaw);
                        double posy_enu = BodyHeading2ENU_y(params[5], params[4], sin_Yaw, cos_Yaw);
                        LA = posy_enu;
                        LB = posx_enu;
                        break;
                    }

                    case MAV_FRAME_BODY_FLU:
                    {
                        Quaternion att;
                        get_Attitude_quat(&att);
                        double yaw = att.getYaw();
                        double sin_Yaw, cos_Yaw;
                        fast_sin_cos(yaw, &sin_Yaw, &cos_Yaw);
                        double posx_enu = BodyHeading2ENU_x(params[4], params[5], sin_Yaw, cos_Yaw);
                        double posy_enu = BodyHeading2ENU_y(params[4], params[5], sin_Yaw, cos_Yaw);
                        LA = posy_enu;
                        LB = posx_enu;
                        break;
                    }

                    default:
                        inf->counter1 = inf->counter2 = 0;
                        return -100;
                }

                if (sq(LA) + sq(LB) > sq(5))
                    Attitude_Control_set_Target_Yaw(atan2(LA, LB));
                inf->counter1 = 1;
                inf->counter2 = 0;
            }
            else if (params[3] > -360 && params[3] < 360)
            { // 指定偏航朝向
                Attitude_Control_set_Target_Yaw(degree2rad(90 - params[3]));
                inf->counter1 = 1;
                inf->counter2 = 0;
            }
            else
            { // 不旋转偏航
                inf->counter1 = 1;
                inf->counter2 = freq * 3;
            }
            break;
        }

        case 1:
        { // 等待偏航旋转开始航点飞行
            Position_Control_set_XYLock();
            Position_Control_set_ZLock();
            double yawTrackErr;
            Attitude_Control_get_YawTrackErr(&yawTrackErr);
            if (yawTrackErr < 0.01)
            {
                bool res;
                switch (frame)
                {
                    case MAV_FRAME_LOCAL_NED:
                        res = Position_Control_set_TargetPositionXY(params[5] * 100, params[4] * 100, 0);
                        break;

                    case MAV_FRAME_LOCAL_ENU:
                        res = Position_Control_set_TargetPositionXY(params[4] * 100, params[5] * 100, 0);
                        break;

                    case MAV_FRAME_LOCAL_OFFSET_NED:
                        res = Position_Control_set_TargetPositionXYRelative(params[5] * 100, params[4] * 100, 0);
                        break;

                    case MAV_FRAME_BODY_NED:
                    case MAV_FRAME_BODY_FRD:
                    case MAV_FRAME_BODY_OFFSET_NED:
                    {
                        // body系修正航向后只向前走
                        if (isvalid(params[3]) == false)
                            res = Position_Control_set_TargetPositionXYRelativeBodyheading(safe_sqrt(params[5] * params[5] + params[4] * params[4]) * 100, 0, 0);
                        else
                            res = Position_Control_set_TargetPositionXYRelativeBodyheading(params[5] * 100, params[4] * 100, 0);
                        break;
                    }

                    case MAV_FRAME_BODY_FLU:
                    {
                        // body系修正航向后只向前走
                        if (isvalid(params[3]) == false)
                            res = Position_Control_set_TargetPositionXYRelativeBodyheading(safe_sqrt(params[4] * params[4] + params[5] * params[5]) * 100, 0, 0);
                        else
                            res = Position_Control_set_TargetPositionXYRelativeBodyheading(params[4] * 100, params[5] * 100, 0);
                        break;
                    }

                    default:
                        inf->counter1 = inf->counter2 = 0;
                        return -100;
                }

                if (res)
                { // 成功
                    inf->counter1 = 2;
                    inf->counter2 = 0;
                }
                else
                { // 失败
                    inf->counter1 = 0;
                    inf->counter2 = 0;
                    return -100;
                }
            }
            break;
        }

        case 2:
        { // 等待航点飞行完成
            Position_ControlMode alt_mode, pos_mode;
            get_Altitude_ControlMode(&alt_mode);
            get_Position_ControlMode(&pos_mode);
            if (alt_mode == Position_ControlMode_Position)
                Position_Control_set_ZLock();
            if (pos_mode == Position_ControlMode_Position)
                Position_Control_set_XYLock();
            if (alt_mode == Position_ControlMode_Position && pos_mode == Position_ControlMode_Position)
            { // 飞行完成进入hold等待
                inf->counter1 = 3;
                inf->counter2 = 0;
            }
            return -3;
            break;
        }

        case 3:
        { // 降落

            Position_Control_set_XYLock();
            // 获取对地高度
            double homeZ;
            getHomeLocalZ(&homeZ);
            vector3<double> pos;
            get_Position_Ctrl(&pos);
            double height = pos.z - homeZ;

            // 获取降落速度
            float sp[2];
            sp[0] = 200;
            ReadParam("PC_maxAutoVelDn", 0, 0, (uint64_t *)sp, 0);
            if (sp[0] < 100)
                sp[0] = 100;

            float ground_height = params[6] * 100;

#define max_acc 50.0
            double land_vel = getPosCtrlCfg()->LandVel[0];
            if (height > 1000 + ground_height + sp[0] * 2)
                Position_Control_set_TargetVelocityZ(-sp[0]);
            else if (height > 1000 + ground_height)
                Position_Control_set_TargetVelocityZ(-((height - 1000 - ground_height) * (sp[0] - land_vel) / sp[0] / 2 + land_vel));
            else
                Position_Control_set_TargetVelocityZ(-land_vel);

            if (inFlight == false)
            { // 降落完成
                inf->counter1 = 0;
                inf->counter2 = 0;
                // 已降落要求停止
                return -10;
            }

            return -3;
            break;
        }
    }
    return -2;
}

/*NavCmd22_TAKEOFF
        MAV_CMD_NAV_TAKEOFF
        起飞并飞往指定地点
        参数:
                <description>Takeoff from ground / hand</description>
                <param index="0" label="Pitch">Minimum pitch (if airspeed sensor present), desired pitch without sensor</param>
                <param index="1">Empty</param>
                <param index="2">Empty</param>
                <param index="3" label="Yaw" units="deg">Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).</param>
                <param index="4" label="Latitude">Latitude</param>
                <param index="5" label="Longitude">Longitude</param>
                <param index="6" label="Altitude" units="m">Altitude</param>
*/
static int32_t NavCmd22_TAKEOFF(double freq, uint8_t frame, double params[], NavCmdInf *inf)
{
    if (get_Position_MSStatus() != MS_Ready)
    { // 无定位出错
        inf->counter1 = inf->counter2 = 0;
        return -100;
    }

    switch (inf->counter1)
    {
        case 0:
        { // 起飞到指定高度
            bool inFlight;
            get_is_inFlight(&inFlight);

            bool res;
            switch (frame)
            {
                case MAV_FRAME_GLOBAL_INT:
                case MAV_FRAME_GLOBAL:
                {
                    if (!inFlight)
                        res = Position_Control_Takeoff_HeightGlobal(params[6] * 100);
                    else
                        res = Position_Control_set_TargetPositionZGlobal(params[6] * 100);
                    break;
                }

                case MAV_FRAME_LOCAL_OFFSET_NED:
                case MAV_FRAME_BODY_NED:
                case MAV_FRAME_BODY_FRD:
                case MAV_FRAME_BODY_OFFSET_NED:
                case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
                case MAV_FRAME_GLOBAL_RELATIVE_ALT:
                case MAV_FRAME_GLOBAL_TERRAIN_ALT:
                case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
                case MAV_FRAME_BODY_FLU:
                {
                    if (!inFlight)
                        res = Position_Control_Takeoff_HeightRelative(params[6] * 100);
                    else
                        res = Position_Control_set_TargetPositionZRA(params[6] * 100);
                    break;
                }

                case MAV_FRAME_LOCAL_NED:
                case MAV_FRAME_LOCAL_ENU:
                {
                    if (!inFlight)
                        res = Position_Control_Takeoff_Height(params[6] * 100);
                    else
                        res = Position_Control_set_TargetPositionZ(params[6] * 100);
                    break;
                }

                default:
                    inf->counter1 = inf->counter2 = 0;
                    return -100;
            }

            if (res)
            { // 成功
                inf->counter1 = 1;
                inf->counter2 = 0;
            }
            else
            { // 失败
                inf->counter1 = 0;
                inf->counter2 = 0;
                return -100;
            }
            break;
        }

        case 1:
        { // 等待起飞完成旋转偏航
            Position_ControlMode mode;
            Position_Control_set_XYLock();
            get_Altitude_ControlMode(&mode);
            if (mode == Position_ControlMode_Position)
            { // 判断执行旋转偏航
                inf->counter1 = 0;
                inf->counter2 = 0;
                return -1;
            }
            return -3;
            break;
        }
    }
    return -2;
}

/*NavCmd93_DELAY
        MAV_CMD_NAV_DELAY
        延时
        参数:
                <description>Delay the next navigation command a number of seconds or until a specified time</description>
                <param index="1" label="Delay" units="s" minValue="-1" increment="1">Delay (-1 to enable time-of-day fields)</param>
                <param index="2" label="Hour" minValue="-1" maxValue="23" increment="1">hour (24h format, UTC, -1 to ignore)</param>
                <param index="3" label="Minute" minValue="-1" maxValue="59" increment="1">minute (24h format, UTC, -1 to ignore)</param>
                <param index="4" label="Second" minValue="-1" maxValue="59" increment="1">second (24h format, UTC)</param>
                <param index="5">Empty</param>
                <param index="6">Empty</param>
                <param index="7">Empty</param>
*/
static int32_t NavCmd93_DELAY(double freq, uint8_t frame, double params[], NavCmdInf *inf)
{
    if (get_Position_MSStatus() != MS_Ready)
    { // 无定位出错
        inf->counter1 = inf->counter2 = 0;
        return -100;
    }

    Position_Control_set_XYLock();
    Position_Control_set_ZLock();
    if (params[0] >= 0)
    { // 延时指定时间
        if (++(inf->counter2) >= freq * params[0])
        {
            inf->counter1 = 0;
            inf->counter2 = 0;
            return -1;
        }
    }
    else
    { // 延时至指定时钟时间

        if (params[1] > 23 || params[2] > 59 || params[3] > 59 || params[3] < 0)
        { // 时间信息错误
            inf->counter1 = 0;
            inf->counter2 = 0;
            return -100;
        }

        RTC_TimeStruct current_time = Get_RTC_Time();
        if (params[1] >= 0)
        {
            if (params[2] >= 0)
            { // 对比时分秒
                uint32_t current_seconds = current_time.Hours * 3600 + current_time.Minutes * 60 + current_time.Seconds;
                uint32_t target_secongds = params[1] * 3600 + params[2] * 60 + params[3];
                if (current_seconds >= target_secongds)
                {
                    inf->counter1 = 0;
                    inf->counter2 = 0;
                    return -1;
                }
            }
            else
            { // 错误
                inf->counter1 = 0;
                inf->counter2 = 0;
                return -100;
            }
        }
        else
        {
            if (params[2] >= 0)
            { // 对比分秒
                uint32_t current_seconds = current_time.Minutes * 60 + current_time.Seconds;
                uint32_t target_secongds = params[2] * 60 + params[3];
                if (current_seconds >= target_secongds)
                {
                    inf->counter1 = 0;
                    inf->counter2 = 0;
                    return -1;
                }
            }
            else
            { // 对比秒
                uint32_t current_seconds = current_time.Seconds;
                uint32_t target_secongds = params[3];
                if (current_seconds >= target_secongds)
                {
                    inf->counter1 = 0;
                    inf->counter2 = 0;
                    return -1;
                }
            }
        }
    }
    return -3;
}

/*NavCmd112_MAV_CMD_CONDITION_DELAY
        MAV_CMD_CONDITION_DELAY
        延时
        参数:
                <description>Delay mission state machine.</description>
                <param index="1" label="Delay" units="s" minValue="0">Delay</param>
                <param index="2">Empty</param>
                <param index="3">Empty</param>
                <param index="4">Empty</param>
                <param index="5">Empty</param>
                <param index="6">Empty</param>
                <param index="7">Empty</param>
*/
static int32_t NavCmd112_CONDITION_DELAY(double freq, uint8_t frame, double params[], NavCmdInf *inf)
{
    if (get_Position_MSStatus() != MS_Ready)
    { // 无定位出错
        inf->counter1 = inf->counter2 = 0;
        return -100;
    }

    Position_Control_set_XYLock();
    Position_Control_set_ZLock();
    if (params[0] >= 0)
    { // 延时指定时间
        if (++(inf->counter2) >= freq * params[0])
        {
            inf->counter1 = 0;
            inf->counter2 = 0;
            return -1;
        }
    }
    return -3;
}

/*NavCmd115_CONDITION_YAW
        MAV_CMD_CONDITION_YAW
        延时
        参数:
                <description>Reach a certain target angle.</description>
                <param index="1" label="Angle" units="deg" minValue="0" maxValue="360">target angle, 0 is north</param>
                <param index="2" label="Angular Speed" units="deg/s">angular speed</param>
                <param index="3" label="Direction" minValue="-1" maxValue="1" increment="2">direction: -1: counter clockwise, 1: clockwise</param>
                <param index="4" label="Relative" minValue="0" maxValue="1" increment="1">0: absolute angle, 1: relative offset</param>
                <param index="5">Empty</param>
                <param index="6">Empty</param>
                <param index="7">Empty</param>
*/
static int32_t NavCmd115_CONDITION_YAW(double freq, uint8_t frame, double params[], NavCmdInf *inf)
{
    if (get_Position_MSStatus() != MS_Ready)
    { // 无定位出错
        inf->counter1 = inf->counter2 = 0;
        return -100;
    }

    Position_Control_set_XYLock();
    Position_Control_set_ZLock();
    switch (inf->counter1)
    {
        case 0:
        { // 旋转偏航
            if (params[3] == 0)
            { // 绝对角度
                Attitude_Control_set_Target_Yaw(degree2rad(90 - params[0]));
            }
            else
            { // 相对角度
                double dir = 1;
                if (params[2] > 0)
                    dir = -1;
                Attitude_Control_set_Target_YawRelative(dir * params[3]);
            }

            inf->counter1 = 1;
            inf->counter2 = 0;

            break;
        }

        case 1:
        { // 等待偏航旋转开始航点飞行
            double yawTrackErr;
            Attitude_Control_get_YawTrackErr(&yawTrackErr);
            if (yawTrackErr < 0.01)
            {
                inf->counter1 = 0;
                inf->counter2 = 0;
                return -1;
            }
            break;
        }
    }
    return -3;
}

///*NavCmd115_CONDITION_YAW
//	MAV_CMD_CONDITION_YAW
//	延时
//	参数:
//		<description>Jump to the desired command in the mission list.  Repeat this action only the specified number of times</description>
//		<param index="1" label="Number" minValue="0" increment="1">Sequence number</param>
//		<param index="2" label="Repeat" minValue="0" increment="1">Repeat count</param>
//		<param index="3">Empty</param>
//		<param index="4">Empty</param>
//		<param index="5">Empty</param>
//		<param index="6">Empty</param>
//		<param index="7">Empty</param>
//*/
// static int32_t NavCmd177_MAV_CMD_DO_JUMP( double freq, uint8_t frame, double params[], NavCmdInf* inf )
//{
//	if( get_Position_MSStatus() != MS_Ready )
//	{	//无定位出错
//		inf->counter1 = inf->counter2 = 0;
//		return -100;
//	}
//
//	Position_Control_set_XYLock();
//	Position_Control_set_ZLock();
//	switch( inf->counter1 )
//	{
//		case 0:
//		{	//旋转偏航
//			if( params[3] == 0 )
//			{	//绝对角度
//				Attitude_Control_set_Target_Yaw( degree2rad( 90-params[0] ) );
//			}
//			else
//			{	//相对角度
//				double dir = 1;
//				if( params[2] > 0 )
//					dir = -1;
//				Attitude_Control_set_Target_YawRelative( dir * params[3] );
//			}
//
//			inf->counter1 = 1;
//			inf->counter2 = 0;
//
//			break;
//		}
//
//		case 1:
//		{	//等待偏航旋转开始航点飞行
//			double yawTrackErr;
//			Attitude_Control_get_YawTrackErr(&yawTrackErr);
//			if( yawTrackErr < 0.01 )
//			{
//				inf->counter1 = 0;
//				inf->counter2 = 0;
//				return -1;
//			}
//			break;
//		}
//	}
//	return -3;
//}

static int32_t (*const NavCmdProcess[])(double freq, uint8_t frame, double params[], NavCmdInf *inf) =
    {
        /*000-*/ 0,
        /*001-*/ 0,
        /*002-*/ 0,
        /*003-*/ 0,
        /*004-*/ 0,
        /*005-*/ 0,
        /*006-*/ 0,
        /*007-*/ 0,
        /*008-*/ 0,
        /*009-*/ 0,
        /*010-*/ 0,
        /*011-*/ 0,
        /*012-*/ 0,
        /*013-*/ 0,
        /*014-*/ 0,
        /*015-*/ 0,
        /*016-*/ NavCmd16_WAYPOINT,
        /*017-*/ 0,
        /*018-*/ 0,
        /*019-*/ 0,
        /*020-*/ NavCmd20_RETURN_TO_LAUNCH,
        /*021-*/ NavCmd21_LAND,
        /*022-*/ NavCmd22_TAKEOFF,
        /*023-*/ NavCmd23_LAND_LOCAL,
        /*024-*/ 0,
        /*025-*/ 0,
        /*026-*/ 0,
        /*027-*/ 0,
        /*028-*/ 0,
        /*029-*/ 0,
        /*030-*/ 0,
        /*031-*/ 0,
        /*032-*/ 0,
        /*033-*/ 0,
        /*034-*/ 0,
        /*035-*/ 0,
        /*036-*/ 0,
        /*037-*/ 0,
        /*038-*/ 0,
        /*039-*/ 0,
        /*040-*/ 0,
        /*041-*/ 0,
        /*042-*/ 0,
        /*043-*/ 0,
        /*044-*/ 0,
        /*045-*/ 0,
        /*046-*/ 0,
        /*047-*/ 0,
        /*048-*/ 0,
        /*049-*/ 0,
        /*050-*/ 0,
        /*051-*/ 0,
        /*052-*/ 0,
        /*053-*/ 0,
        /*054-*/ 0,
        /*055-*/ 0,
        /*056-*/ 0,
        /*057-*/ 0,
        /*058-*/ 0,
        /*059-*/ 0,
        /*060-*/ 0,
        /*061-*/ 0,
        /*062-*/ 0,
        /*063-*/ 0,
        /*064-*/ 0,
        /*065-*/ 0,
        /*066-*/ 0,
        /*067-*/ 0,
        /*068-*/ 0,
        /*069-*/ 0,
        /*070-*/ 0,
        /*071-*/ 0,
        /*072-*/ 0,
        /*073-*/ 0,
        /*074-*/ 0,
        /*075-*/ 0,
        /*076-*/ 0,
        /*077-*/ 0,
        /*078-*/ 0,
        /*079-*/ 0,
        /*080-*/ 0,
        /*081-*/ 0,
        /*082-*/ 0,
        /*083-*/ 0,
        /*084-*/ 0,
        /*085-*/ 0,
        /*086-*/ 0,
        /*087-*/ 0,
        /*088-*/ 0,
        /*089-*/ 0,
        /*090-*/ 0,
        /*091-*/ 0,
        /*092-*/ 0,
        /*093-*/ NavCmd93_DELAY,
        /*094-*/ 0,
        /*095-*/ 0,
        /*096-*/ 0,
        /*097-*/ 0,
        /*098-*/ 0,
        /*099-*/ 0,
        /*100-*/ 0,
        /*101-*/ 0,
        /*102-*/ 0,
        /*103-*/ 0,
        /*104-*/ 0,
        /*105-*/ 0,
        /*106-*/ 0,
        /*107-*/ 0,
        /*108-*/ 0,
        /*109-*/ 0,
        /*110-*/ 0,
        /*111-*/ 0,
        /*112-*/ NavCmd112_CONDITION_DELAY,
        /*113-*/ 0,
        /*114-*/ 0,
        /*115-*/ NavCmd115_CONDITION_YAW,
        /*116-*/ 0,
        /*117-*/ 0,
        /*118-*/ 0,
        /*119-*/ 0,
        /*120-*/ 0,
        /*121-*/ 0,
        /*122-*/ 0,
        /*123-*/ 0,
        /*124-*/ 0,
        /*125-*/ 0,
        /*126-*/ 0,
        /*127-*/ 0,
        /*128-*/ 0,
        /*129-*/ 0,
        /*130-*/ 0,
        /*131-*/ 0,
        /*132-*/ 0,
        /*133-*/ 0,
        /*134-*/ 0,
        /*135-*/ 0,
        /*136-*/ 0,
        /*137-*/ 0,
        /*138-*/ 0,
        /*139-*/ 0,
        /*140-*/ 0,
        /*141-*/ 0,
        /*142-*/ 0,
        /*143-*/ 0,
        /*144-*/ 0,
        /*145-*/ 0,
        /*146-*/ 0,
        /*147-*/ 0,
        /*148-*/ 0,
        /*149-*/ 0,
        /*150-*/ 0,
        /*151-*/ 0,
        /*152-*/ 0,
        /*153-*/ 0,
        /*154-*/ 0,
        /*155-*/ 0,
        /*156-*/ 0,
        /*157-*/ 0,
        /*158-*/ 0,
        /*159-*/ 0,
        /*160-*/ 0,
        /*161-*/ 0,
        /*162-*/ 0,
        /*163-*/ 0,
        /*164-*/ 0,
        /*165-*/ 0,
        /*166-*/ 0,
        /*167-*/ 0,
        /*168-*/ 0,
        /*169-*/ 0,
        /*170-*/ 0,
        /*171-*/ 0,
        /*172-*/ 0,
        /*173-*/ 0,
        /*174-*/ 0,
        /*175-*/ 0,
        /*176-*/ 0,
        /*177-*/ 0,
        /*178-*/ 0,
        /*179-*/ 0,
        /*180-*/ 0,
        /*181-*/ 0,
        /*182-*/ 0,
        /*183-*/ 0,
        /*184-*/ 0,
        /*185-*/ 0,
        /*186-*/ 0,
        /*187-*/ 0,
        /*188-*/ 0,
        /*189-*/ 0,
        /*190-*/ 0,
        /*191-*/ 0,
        /*192-*/ NavCmd192_DO_REPOSITION,
        /*193-*/ 0,
        /*194-*/ 0,
        /*195-*/ 0,
        /*196-*/ 0,
        /*197-*/ 0,
        /*198-*/ 0,
        /*199-*/ 0,
        /*200-*/ 0,
        /*201-*/ 0,
        /*202-*/ 0,
        /*203-*/ 0,
        /*204-*/ 0,
        /*205-*/ 0,
        /*206-*/ 0,
        /*207-*/ 0,
        /*208-*/ 0,
        /*209-*/ 0,
        /*210-*/ 0,
        /*211-*/ 0,
        /*212-*/ 0,
        /*213-*/ 0,
        /*214-*/ 0,
        /*215-*/ 0,
        /*216-*/ 0,
        /*217-*/ 0,
        /*218-*/ 0,
        /*219-*/ 0,
        /*220-*/ 0,
        /*221-*/ 0,
        /*222-*/ 0,
        /*223-*/ 0,
        /*224-*/ 0,
        /*225-*/ 0,
        /*226-*/ 0,
        /*227-*/ 0,
        /*228-*/ 0,
        /*229-*/ 0,
        /*230-*/ 0,
        /*231-*/ 0,
        /*232-*/ 0,
        /*233-*/ 0,
        /*234-*/ 0,
        /*235-*/ 0,
        /*236-*/ 0,
        /*237-*/ 0,
        /*238-*/ 0,
        /*239-*/ 0,
        /*240-*/ 0,
        /*241-*/ 0,
        /*242-*/ 0,
        /*243-*/ 0,
        /*244-*/ 0,
        /*245-*/ 0,
        /*246-*/ 0,
        /*247-*/ 0,
        /*248-*/ 0,
        /*249-*/ 0,
        /*250-*/ 0,
        /*251-*/ 0,
        /*252-*/ 0,
        /*253-*/ 0,
        /*254-*/ 0,
        /*255-*/ 0,
        /*256-*/ 0,
        /*257-*/ 0,
        /*258-*/ 0,
        /*259-*/ 0,
        /*260-*/ 0,
        /*261-*/ 0,
        /*262-*/ 0,
        /*263-*/ 0,
        /*264-*/ 0,
        /*265-*/ 0,
        /*266-*/ 0,
        /*267-*/ 0,
        /*268-*/ 0,
        /*269-*/ 0,
        /*270-*/ 0,
        /*271-*/ 0,
        /*272-*/ 0,
        /*273-*/ 0,
        /*274-*/ 0,
        /*275-*/ 0,
        /*276-*/ 0,
        /*277-*/ 0,
        /*278-*/ 0,
        /*279-*/ 0,
        /*280-*/ 0,
        /*281-*/ 0,
        /*282-*/ 0,
        /*283-*/ 0,
        /*284-*/ 0,
        /*285-*/ 0,
        /*286-*/ 0,
        /*287-*/ 0,
        /*288-*/ 0,
        /*289-*/ 0,
        /*290-*/ 0,
        /*291-*/ 0,
        /*292-*/ 0,
        /*293-*/ 0,
        /*294-*/ 0,
        /*295-*/ 0,
        /*296-*/ 0,
        /*297-*/ 0,
        /*298-*/ 0,
        /*299-*/ 0,
        /*300-*/ 0,
};
const uint16_t NavCmdProcess_Count = sizeof(NavCmdProcess) / sizeof(void *);

bool check_NavCmd(uint16_t cmd, double freq, uint8_t frame, double params[])
{
    // 无此指令返回错误
    if (cmd >= NavCmdProcess_Count || NavCmdProcess[cmd] == 0)
        return false;
    return true;
}
int32_t Process_NavCmd(uint16_t cmd, double freq, uint8_t frame, double params[], NavCmdInf *inf)
{
    // 无此指令返回错误
    if (cmd >= NavCmdProcess_Count || NavCmdProcess[cmd] == 0)
        return -100;

    return NavCmdProcess[cmd](freq, frame, params, inf);
}