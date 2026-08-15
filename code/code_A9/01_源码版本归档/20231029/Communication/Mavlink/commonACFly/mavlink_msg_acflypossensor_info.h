#pragma once
// MESSAGE ACFlyPosSensor_INFO PACKING

#define MAVLINK_MSG_ID_ACFlyPosSensor_INFO 206


typedef struct __mavlink_acflypossensor_info_t {
 uint64_t time_usec; /*< [us] Timestamp. The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 double posE; /*<  Position East in meters*/
 double posN; /*<  Position North in meters*/
 double posU; /*<  Position Up in meters*/
 float velE; /*<  Velocity East in m/s*/
 float velN; /*<  Velocity North in m/s*/
 float velU; /*<  Velocity Up in m/s*/
 int32_t Lat; /*<  latitude 1e7*/
 int32_t Lon; /*<  lontitude 1e7*/
 float delay; /*<  Sensor observation delay in seconds.*/
 float trustXY; /*<  Sensor trust in meters in XY direction*/
 float trustZ; /*<  Sensor trust in meters in Z direction*/
 char sensor_name[16]; /*<  position sensor name*/
 int8_t ind; /*<  Position sensor index.*/
 uint8_t type; /*<  Position sensor type.*/
 uint8_t DataFrame; /*<  Velocity data frame 0:ENU 1:bodyFLU bit7:sensor available.*/
 uint8_t DataType; /*<  Position sensor data type.*/
 uint8_t additionInfo1[4]; /*<  Addition info according to sensor type.*/
 float additionInfo2[8]; /*<  Addition info according to sensor type.*/
} mavlink_acflypossensor_info_t;

#define MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN 120
#define MAVLINK_MSG_ID_ACFlyPosSensor_INFO_MIN_LEN 84
#define MAVLINK_MSG_ID_206_LEN 120
#define MAVLINK_MSG_ID_206_MIN_LEN 84

#define MAVLINK_MSG_ID_ACFlyPosSensor_INFO_CRC 232
#define MAVLINK_MSG_ID_206_CRC 232

#define MAVLINK_MSG_ACFlyPosSensor_INFO_FIELD_SENSOR_NAME_LEN 16
#define MAVLINK_MSG_ACFlyPosSensor_INFO_FIELD_ADDITIONINFO1_LEN 4
#define MAVLINK_MSG_ACFlyPosSensor_INFO_FIELD_ADDITIONINFO2_LEN 8

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACFlyPosSensor_INFO { \
    206, \
    "ACFlyPosSensor_INFO", \
    19, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_acflypossensor_info_t, time_usec) }, \
         { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 16, 64, offsetof(mavlink_acflypossensor_info_t, sensor_name) }, \
         { "ind", NULL, MAVLINK_TYPE_INT8_T, 0, 80, offsetof(mavlink_acflypossensor_info_t, ind) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 81, offsetof(mavlink_acflypossensor_info_t, type) }, \
         { "DataFrame", NULL, MAVLINK_TYPE_UINT8_T, 0, 82, offsetof(mavlink_acflypossensor_info_t, DataFrame) }, \
         { "DataType", NULL, MAVLINK_TYPE_UINT8_T, 0, 83, offsetof(mavlink_acflypossensor_info_t, DataType) }, \
         { "posE", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_acflypossensor_info_t, posE) }, \
         { "posN", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_acflypossensor_info_t, posN) }, \
         { "posU", NULL, MAVLINK_TYPE_DOUBLE, 0, 24, offsetof(mavlink_acflypossensor_info_t, posU) }, \
         { "velE", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_acflypossensor_info_t, velE) }, \
         { "velN", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_acflypossensor_info_t, velN) }, \
         { "velU", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_acflypossensor_info_t, velU) }, \
         { "Lat", NULL, MAVLINK_TYPE_INT32_T, 0, 44, offsetof(mavlink_acflypossensor_info_t, Lat) }, \
         { "Lon", NULL, MAVLINK_TYPE_INT32_T, 0, 48, offsetof(mavlink_acflypossensor_info_t, Lon) }, \
         { "delay", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_acflypossensor_info_t, delay) }, \
         { "trustXY", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_acflypossensor_info_t, trustXY) }, \
         { "trustZ", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_acflypossensor_info_t, trustZ) }, \
         { "additionInfo1", NULL, MAVLINK_TYPE_UINT8_T, 4, 84, offsetof(mavlink_acflypossensor_info_t, additionInfo1) }, \
         { "additionInfo2", NULL, MAVLINK_TYPE_FLOAT, 8, 88, offsetof(mavlink_acflypossensor_info_t, additionInfo2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACFlyPosSensor_INFO { \
    "ACFlyPosSensor_INFO", \
    19, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_acflypossensor_info_t, time_usec) }, \
         { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 16, 64, offsetof(mavlink_acflypossensor_info_t, sensor_name) }, \
         { "ind", NULL, MAVLINK_TYPE_INT8_T, 0, 80, offsetof(mavlink_acflypossensor_info_t, ind) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 81, offsetof(mavlink_acflypossensor_info_t, type) }, \
         { "DataFrame", NULL, MAVLINK_TYPE_UINT8_T, 0, 82, offsetof(mavlink_acflypossensor_info_t, DataFrame) }, \
         { "DataType", NULL, MAVLINK_TYPE_UINT8_T, 0, 83, offsetof(mavlink_acflypossensor_info_t, DataType) }, \
         { "posE", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_acflypossensor_info_t, posE) }, \
         { "posN", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_acflypossensor_info_t, posN) }, \
         { "posU", NULL, MAVLINK_TYPE_DOUBLE, 0, 24, offsetof(mavlink_acflypossensor_info_t, posU) }, \
         { "velE", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_acflypossensor_info_t, velE) }, \
         { "velN", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_acflypossensor_info_t, velN) }, \
         { "velU", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_acflypossensor_info_t, velU) }, \
         { "Lat", NULL, MAVLINK_TYPE_INT32_T, 0, 44, offsetof(mavlink_acflypossensor_info_t, Lat) }, \
         { "Lon", NULL, MAVLINK_TYPE_INT32_T, 0, 48, offsetof(mavlink_acflypossensor_info_t, Lon) }, \
         { "delay", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_acflypossensor_info_t, delay) }, \
         { "trustXY", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_acflypossensor_info_t, trustXY) }, \
         { "trustZ", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_acflypossensor_info_t, trustZ) }, \
         { "additionInfo1", NULL, MAVLINK_TYPE_UINT8_T, 4, 84, offsetof(mavlink_acflypossensor_info_t, additionInfo1) }, \
         { "additionInfo2", NULL, MAVLINK_TYPE_FLOAT, 8, 88, offsetof(mavlink_acflypossensor_info_t, additionInfo2) }, \
         } \
}
#endif

/**
 * @brief Pack a acflypossensor_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp. The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param sensor_name  position sensor name
 * @param ind  Position sensor index.
 * @param type  Position sensor type.
 * @param DataFrame  Velocity data frame 0:ENU 1:bodyFLU bit7:sensor available.
 * @param DataType  Position sensor data type.
 * @param posE  Position East in meters
 * @param posN  Position North in meters
 * @param posU  Position Up in meters
 * @param velE  Velocity East in m/s
 * @param velN  Velocity North in m/s
 * @param velU  Velocity Up in m/s
 * @param Lat  latitude 1e7
 * @param Lon  lontitude 1e7
 * @param delay  Sensor observation delay in seconds.
 * @param trustXY  Sensor trust in meters in XY direction
 * @param trustZ  Sensor trust in meters in Z direction
 * @param additionInfo1  Addition info according to sensor type.
 * @param additionInfo2  Addition info according to sensor type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acflypossensor_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, const char *sensor_name, int8_t ind, uint8_t type, uint8_t DataFrame, uint8_t DataType, double posE, double posN, double posU, float velE, float velN, float velU, int32_t Lat, int32_t Lon, float delay, float trustXY, float trustZ, const uint8_t *additionInfo1, const float *additionInfo2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_double(buf, 8, posE);
    _mav_put_double(buf, 16, posN);
    _mav_put_double(buf, 24, posU);
    _mav_put_float(buf, 32, velE);
    _mav_put_float(buf, 36, velN);
    _mav_put_float(buf, 40, velU);
    _mav_put_int32_t(buf, 44, Lat);
    _mav_put_int32_t(buf, 48, Lon);
    _mav_put_float(buf, 52, delay);
    _mav_put_float(buf, 56, trustXY);
    _mav_put_float(buf, 60, trustZ);
    _mav_put_int8_t(buf, 80, ind);
    _mav_put_uint8_t(buf, 81, type);
    _mav_put_uint8_t(buf, 82, DataFrame);
    _mav_put_uint8_t(buf, 83, DataType);
    _mav_put_char_array(buf, 64, sensor_name, 16);
    _mav_put_uint8_t_array(buf, 84, additionInfo1, 4);
    _mav_put_float_array(buf, 88, additionInfo2, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN);
#else
    mavlink_acflypossensor_info_t packet;
    packet.time_usec = time_usec;
    packet.posE = posE;
    packet.posN = posN;
    packet.posU = posU;
    packet.velE = velE;
    packet.velN = velN;
    packet.velU = velU;
    packet.Lat = Lat;
    packet.Lon = Lon;
    packet.delay = delay;
    packet.trustXY = trustXY;
    packet.trustZ = trustZ;
    packet.ind = ind;
    packet.type = type;
    packet.DataFrame = DataFrame;
    packet.DataType = DataType;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*16);
    mav_array_memcpy(packet.additionInfo1, additionInfo1, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.additionInfo2, additionInfo2, sizeof(float)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFlyPosSensor_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_MIN_LEN, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_CRC);
}

/**
 * @brief Pack a acflypossensor_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp. The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param sensor_name  position sensor name
 * @param ind  Position sensor index.
 * @param type  Position sensor type.
 * @param DataFrame  Velocity data frame 0:ENU 1:bodyFLU bit7:sensor available.
 * @param DataType  Position sensor data type.
 * @param posE  Position East in meters
 * @param posN  Position North in meters
 * @param posU  Position Up in meters
 * @param velE  Velocity East in m/s
 * @param velN  Velocity North in m/s
 * @param velU  Velocity Up in m/s
 * @param Lat  latitude 1e7
 * @param Lon  lontitude 1e7
 * @param delay  Sensor observation delay in seconds.
 * @param trustXY  Sensor trust in meters in XY direction
 * @param trustZ  Sensor trust in meters in Z direction
 * @param additionInfo1  Addition info according to sensor type.
 * @param additionInfo2  Addition info according to sensor type.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acflypossensor_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,const char *sensor_name,int8_t ind,uint8_t type,uint8_t DataFrame,uint8_t DataType,double posE,double posN,double posU,float velE,float velN,float velU,int32_t Lat,int32_t Lon,float delay,float trustXY,float trustZ,const uint8_t *additionInfo1,const float *additionInfo2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_double(buf, 8, posE);
    _mav_put_double(buf, 16, posN);
    _mav_put_double(buf, 24, posU);
    _mav_put_float(buf, 32, velE);
    _mav_put_float(buf, 36, velN);
    _mav_put_float(buf, 40, velU);
    _mav_put_int32_t(buf, 44, Lat);
    _mav_put_int32_t(buf, 48, Lon);
    _mav_put_float(buf, 52, delay);
    _mav_put_float(buf, 56, trustXY);
    _mav_put_float(buf, 60, trustZ);
    _mav_put_int8_t(buf, 80, ind);
    _mav_put_uint8_t(buf, 81, type);
    _mav_put_uint8_t(buf, 82, DataFrame);
    _mav_put_uint8_t(buf, 83, DataType);
    _mav_put_char_array(buf, 64, sensor_name, 16);
    _mav_put_uint8_t_array(buf, 84, additionInfo1, 4);
    _mav_put_float_array(buf, 88, additionInfo2, 8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN);
#else
    mavlink_acflypossensor_info_t packet;
    packet.time_usec = time_usec;
    packet.posE = posE;
    packet.posN = posN;
    packet.posU = posU;
    packet.velE = velE;
    packet.velN = velN;
    packet.velU = velU;
    packet.Lat = Lat;
    packet.Lon = Lon;
    packet.delay = delay;
    packet.trustXY = trustXY;
    packet.trustZ = trustZ;
    packet.ind = ind;
    packet.type = type;
    packet.DataFrame = DataFrame;
    packet.DataType = DataType;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*16);
    mav_array_memcpy(packet.additionInfo1, additionInfo1, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.additionInfo2, additionInfo2, sizeof(float)*8);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFlyPosSensor_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_MIN_LEN, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_CRC);
}

/**
 * @brief Encode a acflypossensor_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param acflypossensor_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acflypossensor_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_acflypossensor_info_t* acflypossensor_info)
{
    return mavlink_msg_acflypossensor_info_pack(system_id, component_id, msg, acflypossensor_info->time_usec, acflypossensor_info->sensor_name, acflypossensor_info->ind, acflypossensor_info->type, acflypossensor_info->DataFrame, acflypossensor_info->DataType, acflypossensor_info->posE, acflypossensor_info->posN, acflypossensor_info->posU, acflypossensor_info->velE, acflypossensor_info->velN, acflypossensor_info->velU, acflypossensor_info->Lat, acflypossensor_info->Lon, acflypossensor_info->delay, acflypossensor_info->trustXY, acflypossensor_info->trustZ, acflypossensor_info->additionInfo1, acflypossensor_info->additionInfo2);
}

/**
 * @brief Encode a acflypossensor_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param acflypossensor_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acflypossensor_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_acflypossensor_info_t* acflypossensor_info)
{
    return mavlink_msg_acflypossensor_info_pack_chan(system_id, component_id, chan, msg, acflypossensor_info->time_usec, acflypossensor_info->sensor_name, acflypossensor_info->ind, acflypossensor_info->type, acflypossensor_info->DataFrame, acflypossensor_info->DataType, acflypossensor_info->posE, acflypossensor_info->posN, acflypossensor_info->posU, acflypossensor_info->velE, acflypossensor_info->velN, acflypossensor_info->velU, acflypossensor_info->Lat, acflypossensor_info->Lon, acflypossensor_info->delay, acflypossensor_info->trustXY, acflypossensor_info->trustZ, acflypossensor_info->additionInfo1, acflypossensor_info->additionInfo2);
}

/**
 * @brief Send a acflypossensor_info message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp. The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param sensor_name  position sensor name
 * @param ind  Position sensor index.
 * @param type  Position sensor type.
 * @param DataFrame  Velocity data frame 0:ENU 1:bodyFLU bit7:sensor available.
 * @param DataType  Position sensor data type.
 * @param posE  Position East in meters
 * @param posN  Position North in meters
 * @param posU  Position Up in meters
 * @param velE  Velocity East in m/s
 * @param velN  Velocity North in m/s
 * @param velU  Velocity Up in m/s
 * @param Lat  latitude 1e7
 * @param Lon  lontitude 1e7
 * @param delay  Sensor observation delay in seconds.
 * @param trustXY  Sensor trust in meters in XY direction
 * @param trustZ  Sensor trust in meters in Z direction
 * @param additionInfo1  Addition info according to sensor type.
 * @param additionInfo2  Addition info according to sensor type.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_acflypossensor_info_send(mavlink_channel_t chan, uint64_t time_usec, const char *sensor_name, int8_t ind, uint8_t type, uint8_t DataFrame, uint8_t DataType, double posE, double posN, double posU, float velE, float velN, float velU, int32_t Lat, int32_t Lon, float delay, float trustXY, float trustZ, const uint8_t *additionInfo1, const float *additionInfo2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_double(buf, 8, posE);
    _mav_put_double(buf, 16, posN);
    _mav_put_double(buf, 24, posU);
    _mav_put_float(buf, 32, velE);
    _mav_put_float(buf, 36, velN);
    _mav_put_float(buf, 40, velU);
    _mav_put_int32_t(buf, 44, Lat);
    _mav_put_int32_t(buf, 48, Lon);
    _mav_put_float(buf, 52, delay);
    _mav_put_float(buf, 56, trustXY);
    _mav_put_float(buf, 60, trustZ);
    _mav_put_int8_t(buf, 80, ind);
    _mav_put_uint8_t(buf, 81, type);
    _mav_put_uint8_t(buf, 82, DataFrame);
    _mav_put_uint8_t(buf, 83, DataType);
    _mav_put_char_array(buf, 64, sensor_name, 16);
    _mav_put_uint8_t_array(buf, 84, additionInfo1, 4);
    _mav_put_float_array(buf, 88, additionInfo2, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyPosSensor_INFO, buf, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_MIN_LEN, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_CRC);
#else
    mavlink_acflypossensor_info_t packet;
    packet.time_usec = time_usec;
    packet.posE = posE;
    packet.posN = posN;
    packet.posU = posU;
    packet.velE = velE;
    packet.velN = velN;
    packet.velU = velU;
    packet.Lat = Lat;
    packet.Lon = Lon;
    packet.delay = delay;
    packet.trustXY = trustXY;
    packet.trustZ = trustZ;
    packet.ind = ind;
    packet.type = type;
    packet.DataFrame = DataFrame;
    packet.DataType = DataType;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*16);
    mav_array_memcpy(packet.additionInfo1, additionInfo1, sizeof(uint8_t)*4);
    mav_array_memcpy(packet.additionInfo2, additionInfo2, sizeof(float)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyPosSensor_INFO, (const char *)&packet, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_MIN_LEN, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_CRC);
#endif
}

/**
 * @brief Send a acflypossensor_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_acflypossensor_info_send_struct(mavlink_channel_t chan, const mavlink_acflypossensor_info_t* acflypossensor_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_acflypossensor_info_send(chan, acflypossensor_info->time_usec, acflypossensor_info->sensor_name, acflypossensor_info->ind, acflypossensor_info->type, acflypossensor_info->DataFrame, acflypossensor_info->DataType, acflypossensor_info->posE, acflypossensor_info->posN, acflypossensor_info->posU, acflypossensor_info->velE, acflypossensor_info->velN, acflypossensor_info->velU, acflypossensor_info->Lat, acflypossensor_info->Lon, acflypossensor_info->delay, acflypossensor_info->trustXY, acflypossensor_info->trustZ, acflypossensor_info->additionInfo1, acflypossensor_info->additionInfo2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyPosSensor_INFO, (const char *)acflypossensor_info, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_MIN_LEN, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_acflypossensor_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, const char *sensor_name, int8_t ind, uint8_t type, uint8_t DataFrame, uint8_t DataType, double posE, double posN, double posU, float velE, float velN, float velU, int32_t Lat, int32_t Lon, float delay, float trustXY, float trustZ, const uint8_t *additionInfo1, const float *additionInfo2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_double(buf, 8, posE);
    _mav_put_double(buf, 16, posN);
    _mav_put_double(buf, 24, posU);
    _mav_put_float(buf, 32, velE);
    _mav_put_float(buf, 36, velN);
    _mav_put_float(buf, 40, velU);
    _mav_put_int32_t(buf, 44, Lat);
    _mav_put_int32_t(buf, 48, Lon);
    _mav_put_float(buf, 52, delay);
    _mav_put_float(buf, 56, trustXY);
    _mav_put_float(buf, 60, trustZ);
    _mav_put_int8_t(buf, 80, ind);
    _mav_put_uint8_t(buf, 81, type);
    _mav_put_uint8_t(buf, 82, DataFrame);
    _mav_put_uint8_t(buf, 83, DataType);
    _mav_put_char_array(buf, 64, sensor_name, 16);
    _mav_put_uint8_t_array(buf, 84, additionInfo1, 4);
    _mav_put_float_array(buf, 88, additionInfo2, 8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyPosSensor_INFO, buf, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_MIN_LEN, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_CRC);
#else
    mavlink_acflypossensor_info_t *packet = (mavlink_acflypossensor_info_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->posE = posE;
    packet->posN = posN;
    packet->posU = posU;
    packet->velE = velE;
    packet->velN = velN;
    packet->velU = velU;
    packet->Lat = Lat;
    packet->Lon = Lon;
    packet->delay = delay;
    packet->trustXY = trustXY;
    packet->trustZ = trustZ;
    packet->ind = ind;
    packet->type = type;
    packet->DataFrame = DataFrame;
    packet->DataType = DataType;
    mav_array_memcpy(packet->sensor_name, sensor_name, sizeof(char)*16);
    mav_array_memcpy(packet->additionInfo1, additionInfo1, sizeof(uint8_t)*4);
    mav_array_memcpy(packet->additionInfo2, additionInfo2, sizeof(float)*8);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyPosSensor_INFO, (const char *)packet, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_MIN_LEN, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE ACFlyPosSensor_INFO UNPACKING


/**
 * @brief Get field time_usec from acflypossensor_info message
 *
 * @return [us] Timestamp. The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_acflypossensor_info_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field sensor_name from acflypossensor_info message
 *
 * @return  position sensor name
 */
static inline uint16_t mavlink_msg_acflypossensor_info_get_sensor_name(const mavlink_message_t* msg, char *sensor_name)
{
    return _MAV_RETURN_char_array(msg, sensor_name, 16,  64);
}

/**
 * @brief Get field ind from acflypossensor_info message
 *
 * @return  Position sensor index.
 */
static inline int8_t mavlink_msg_acflypossensor_info_get_ind(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  80);
}

/**
 * @brief Get field type from acflypossensor_info message
 *
 * @return  Position sensor type.
 */
static inline uint8_t mavlink_msg_acflypossensor_info_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  81);
}

/**
 * @brief Get field DataFrame from acflypossensor_info message
 *
 * @return  Velocity data frame 0:ENU 1:bodyFLU bit7:sensor available.
 */
static inline uint8_t mavlink_msg_acflypossensor_info_get_DataFrame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  82);
}

/**
 * @brief Get field DataType from acflypossensor_info message
 *
 * @return  Position sensor data type.
 */
static inline uint8_t mavlink_msg_acflypossensor_info_get_DataType(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  83);
}

/**
 * @brief Get field posE from acflypossensor_info message
 *
 * @return  Position East in meters
 */
static inline double mavlink_msg_acflypossensor_info_get_posE(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field posN from acflypossensor_info message
 *
 * @return  Position North in meters
 */
static inline double mavlink_msg_acflypossensor_info_get_posN(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Get field posU from acflypossensor_info message
 *
 * @return  Position Up in meters
 */
static inline double mavlink_msg_acflypossensor_info_get_posU(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  24);
}

/**
 * @brief Get field velE from acflypossensor_info message
 *
 * @return  Velocity East in m/s
 */
static inline float mavlink_msg_acflypossensor_info_get_velE(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field velN from acflypossensor_info message
 *
 * @return  Velocity North in m/s
 */
static inline float mavlink_msg_acflypossensor_info_get_velN(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field velU from acflypossensor_info message
 *
 * @return  Velocity Up in m/s
 */
static inline float mavlink_msg_acflypossensor_info_get_velU(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field Lat from acflypossensor_info message
 *
 * @return  latitude 1e7
 */
static inline int32_t mavlink_msg_acflypossensor_info_get_Lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  44);
}

/**
 * @brief Get field Lon from acflypossensor_info message
 *
 * @return  lontitude 1e7
 */
static inline int32_t mavlink_msg_acflypossensor_info_get_Lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  48);
}

/**
 * @brief Get field delay from acflypossensor_info message
 *
 * @return  Sensor observation delay in seconds.
 */
static inline float mavlink_msg_acflypossensor_info_get_delay(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field trustXY from acflypossensor_info message
 *
 * @return  Sensor trust in meters in XY direction
 */
static inline float mavlink_msg_acflypossensor_info_get_trustXY(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field trustZ from acflypossensor_info message
 *
 * @return  Sensor trust in meters in Z direction
 */
static inline float mavlink_msg_acflypossensor_info_get_trustZ(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field additionInfo1 from acflypossensor_info message
 *
 * @return  Addition info according to sensor type.
 */
static inline uint16_t mavlink_msg_acflypossensor_info_get_additionInfo1(const mavlink_message_t* msg, uint8_t *additionInfo1)
{
    return _MAV_RETURN_uint8_t_array(msg, additionInfo1, 4,  84);
}

/**
 * @brief Get field additionInfo2 from acflypossensor_info message
 *
 * @return  Addition info according to sensor type.
 */
static inline uint16_t mavlink_msg_acflypossensor_info_get_additionInfo2(const mavlink_message_t* msg, float *additionInfo2)
{
    return _MAV_RETURN_float_array(msg, additionInfo2, 8,  88);
}

/**
 * @brief Decode a acflypossensor_info message into a struct
 *
 * @param msg The message to decode
 * @param acflypossensor_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_acflypossensor_info_decode(const mavlink_message_t* msg, mavlink_acflypossensor_info_t* acflypossensor_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    acflypossensor_info->time_usec = mavlink_msg_acflypossensor_info_get_time_usec(msg);
    acflypossensor_info->posE = mavlink_msg_acflypossensor_info_get_posE(msg);
    acflypossensor_info->posN = mavlink_msg_acflypossensor_info_get_posN(msg);
    acflypossensor_info->posU = mavlink_msg_acflypossensor_info_get_posU(msg);
    acflypossensor_info->velE = mavlink_msg_acflypossensor_info_get_velE(msg);
    acflypossensor_info->velN = mavlink_msg_acflypossensor_info_get_velN(msg);
    acflypossensor_info->velU = mavlink_msg_acflypossensor_info_get_velU(msg);
    acflypossensor_info->Lat = mavlink_msg_acflypossensor_info_get_Lat(msg);
    acflypossensor_info->Lon = mavlink_msg_acflypossensor_info_get_Lon(msg);
    acflypossensor_info->delay = mavlink_msg_acflypossensor_info_get_delay(msg);
    acflypossensor_info->trustXY = mavlink_msg_acflypossensor_info_get_trustXY(msg);
    acflypossensor_info->trustZ = mavlink_msg_acflypossensor_info_get_trustZ(msg);
    mavlink_msg_acflypossensor_info_get_sensor_name(msg, acflypossensor_info->sensor_name);
    acflypossensor_info->ind = mavlink_msg_acflypossensor_info_get_ind(msg);
    acflypossensor_info->type = mavlink_msg_acflypossensor_info_get_type(msg);
    acflypossensor_info->DataFrame = mavlink_msg_acflypossensor_info_get_DataFrame(msg);
    acflypossensor_info->DataType = mavlink_msg_acflypossensor_info_get_DataType(msg);
    mavlink_msg_acflypossensor_info_get_additionInfo1(msg, acflypossensor_info->additionInfo1);
    mavlink_msg_acflypossensor_info_get_additionInfo2(msg, acflypossensor_info->additionInfo2);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN? msg->len : MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN;
        memset(acflypossensor_info, 0, MAVLINK_MSG_ID_ACFlyPosSensor_INFO_LEN);
    memcpy(acflypossensor_info, _MAV_PAYLOAD(msg), len);
#endif
}
