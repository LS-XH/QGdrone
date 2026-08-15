#pragma once
// MESSAGE ACFly_RegeisterPosSensor PACKING

#define MAVLINK_MSG_ID_ACFly_RegeisterPosSensor 208


typedef struct __mavlink_acfly_regeisterpossensor_t {
 float delay; /*<  Sensor observation delay in seconds.*/
 float trustXY; /*<  Sensor trust in meters in XY direction*/
 float trustZ; /*<  Sensor trust in meters in Z direction*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 char sensor_name[16]; /*<  position sensor name*/
 int8_t ind; /*<  Position sensor index.*/
 uint8_t type; /*<  Position sensor type.*/
 uint8_t DataFrame; /*<  0:position in ENU velocity in ENU   1:position in ENU velocity in bodyFLU   4:position in SLAM frame velocity in ENU   5:position in SLAM frame velocity in bodyFLU   bit7:sensor available.   bit6:1 to Unregister sensor.*/
 uint8_t DataType; /*<  Position sensor data type.*/
} mavlink_acfly_regeisterpossensor_t;

#define MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN 34
#define MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_MIN_LEN 34
#define MAVLINK_MSG_ID_208_LEN 34
#define MAVLINK_MSG_ID_208_MIN_LEN 34

#define MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_CRC 49
#define MAVLINK_MSG_ID_208_CRC 49

#define MAVLINK_MSG_ACFly_RegeisterPosSensor_FIELD_SENSOR_NAME_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACFly_RegeisterPosSensor { \
    208, \
    "ACFly_RegeisterPosSensor", \
    10, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_acfly_regeisterpossensor_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_acfly_regeisterpossensor_t, target_component) }, \
         { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 16, 14, offsetof(mavlink_acfly_regeisterpossensor_t, sensor_name) }, \
         { "ind", NULL, MAVLINK_TYPE_INT8_T, 0, 30, offsetof(mavlink_acfly_regeisterpossensor_t, ind) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_acfly_regeisterpossensor_t, type) }, \
         { "DataFrame", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_acfly_regeisterpossensor_t, DataFrame) }, \
         { "DataType", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_acfly_regeisterpossensor_t, DataType) }, \
         { "delay", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_acfly_regeisterpossensor_t, delay) }, \
         { "trustXY", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_acfly_regeisterpossensor_t, trustXY) }, \
         { "trustZ", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_acfly_regeisterpossensor_t, trustZ) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACFly_RegeisterPosSensor { \
    "ACFly_RegeisterPosSensor", \
    10, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_acfly_regeisterpossensor_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_acfly_regeisterpossensor_t, target_component) }, \
         { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 16, 14, offsetof(mavlink_acfly_regeisterpossensor_t, sensor_name) }, \
         { "ind", NULL, MAVLINK_TYPE_INT8_T, 0, 30, offsetof(mavlink_acfly_regeisterpossensor_t, ind) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_acfly_regeisterpossensor_t, type) }, \
         { "DataFrame", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_acfly_regeisterpossensor_t, DataFrame) }, \
         { "DataType", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_acfly_regeisterpossensor_t, DataType) }, \
         { "delay", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_acfly_regeisterpossensor_t, delay) }, \
         { "trustXY", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_acfly_regeisterpossensor_t, trustXY) }, \
         { "trustZ", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_acfly_regeisterpossensor_t, trustZ) }, \
         } \
}
#endif

/**
 * @brief Pack a acfly_regeisterpossensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param sensor_name  position sensor name
 * @param ind  Position sensor index.
 * @param type  Position sensor type.
 * @param DataFrame  0:position in ENU velocity in ENU   1:position in ENU velocity in bodyFLU   4:position in SLAM frame velocity in ENU   5:position in SLAM frame velocity in bodyFLU   bit7:sensor available.   bit6:1 to Unregister sensor.
 * @param DataType  Position sensor data type.
 * @param delay  Sensor observation delay in seconds.
 * @param trustXY  Sensor trust in meters in XY direction
 * @param trustZ  Sensor trust in meters in Z direction
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acfly_regeisterpossensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, const char *sensor_name, int8_t ind, uint8_t type, uint8_t DataFrame, uint8_t DataType, float delay, float trustXY, float trustZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN];
    _mav_put_float(buf, 0, delay);
    _mav_put_float(buf, 4, trustXY);
    _mav_put_float(buf, 8, trustZ);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_int8_t(buf, 30, ind);
    _mav_put_uint8_t(buf, 31, type);
    _mav_put_uint8_t(buf, 32, DataFrame);
    _mav_put_uint8_t(buf, 33, DataType);
    _mav_put_char_array(buf, 14, sensor_name, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN);
#else
    mavlink_acfly_regeisterpossensor_t packet;
    packet.delay = delay;
    packet.trustXY = trustXY;
    packet.trustZ = trustZ;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.ind = ind;
    packet.type = type;
    packet.DataFrame = DataFrame;
    packet.DataType = DataType;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFly_RegeisterPosSensor;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_MIN_LEN, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_CRC);
}

/**
 * @brief Pack a acfly_regeisterpossensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param sensor_name  position sensor name
 * @param ind  Position sensor index.
 * @param type  Position sensor type.
 * @param DataFrame  0:position in ENU velocity in ENU   1:position in ENU velocity in bodyFLU   4:position in SLAM frame velocity in ENU   5:position in SLAM frame velocity in bodyFLU   bit7:sensor available.   bit6:1 to Unregister sensor.
 * @param DataType  Position sensor data type.
 * @param delay  Sensor observation delay in seconds.
 * @param trustXY  Sensor trust in meters in XY direction
 * @param trustZ  Sensor trust in meters in Z direction
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acfly_regeisterpossensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,const char *sensor_name,int8_t ind,uint8_t type,uint8_t DataFrame,uint8_t DataType,float delay,float trustXY,float trustZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN];
    _mav_put_float(buf, 0, delay);
    _mav_put_float(buf, 4, trustXY);
    _mav_put_float(buf, 8, trustZ);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_int8_t(buf, 30, ind);
    _mav_put_uint8_t(buf, 31, type);
    _mav_put_uint8_t(buf, 32, DataFrame);
    _mav_put_uint8_t(buf, 33, DataType);
    _mav_put_char_array(buf, 14, sensor_name, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN);
#else
    mavlink_acfly_regeisterpossensor_t packet;
    packet.delay = delay;
    packet.trustXY = trustXY;
    packet.trustZ = trustZ;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.ind = ind;
    packet.type = type;
    packet.DataFrame = DataFrame;
    packet.DataType = DataType;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFly_RegeisterPosSensor;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_MIN_LEN, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_CRC);
}

/**
 * @brief Encode a acfly_regeisterpossensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param acfly_regeisterpossensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acfly_regeisterpossensor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_acfly_regeisterpossensor_t* acfly_regeisterpossensor)
{
    return mavlink_msg_acfly_regeisterpossensor_pack(system_id, component_id, msg, acfly_regeisterpossensor->target_system, acfly_regeisterpossensor->target_component, acfly_regeisterpossensor->sensor_name, acfly_regeisterpossensor->ind, acfly_regeisterpossensor->type, acfly_regeisterpossensor->DataFrame, acfly_regeisterpossensor->DataType, acfly_regeisterpossensor->delay, acfly_regeisterpossensor->trustXY, acfly_regeisterpossensor->trustZ);
}

/**
 * @brief Encode a acfly_regeisterpossensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param acfly_regeisterpossensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acfly_regeisterpossensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_acfly_regeisterpossensor_t* acfly_regeisterpossensor)
{
    return mavlink_msg_acfly_regeisterpossensor_pack_chan(system_id, component_id, chan, msg, acfly_regeisterpossensor->target_system, acfly_regeisterpossensor->target_component, acfly_regeisterpossensor->sensor_name, acfly_regeisterpossensor->ind, acfly_regeisterpossensor->type, acfly_regeisterpossensor->DataFrame, acfly_regeisterpossensor->DataType, acfly_regeisterpossensor->delay, acfly_regeisterpossensor->trustXY, acfly_regeisterpossensor->trustZ);
}

/**
 * @brief Send a acfly_regeisterpossensor message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param sensor_name  position sensor name
 * @param ind  Position sensor index.
 * @param type  Position sensor type.
 * @param DataFrame  0:position in ENU velocity in ENU   1:position in ENU velocity in bodyFLU   4:position in SLAM frame velocity in ENU   5:position in SLAM frame velocity in bodyFLU   bit7:sensor available.   bit6:1 to Unregister sensor.
 * @param DataType  Position sensor data type.
 * @param delay  Sensor observation delay in seconds.
 * @param trustXY  Sensor trust in meters in XY direction
 * @param trustZ  Sensor trust in meters in Z direction
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_acfly_regeisterpossensor_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, const char *sensor_name, int8_t ind, uint8_t type, uint8_t DataFrame, uint8_t DataType, float delay, float trustXY, float trustZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN];
    _mav_put_float(buf, 0, delay);
    _mav_put_float(buf, 4, trustXY);
    _mav_put_float(buf, 8, trustZ);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_int8_t(buf, 30, ind);
    _mav_put_uint8_t(buf, 31, type);
    _mav_put_uint8_t(buf, 32, DataFrame);
    _mav_put_uint8_t(buf, 33, DataType);
    _mav_put_char_array(buf, 14, sensor_name, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor, buf, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_MIN_LEN, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_CRC);
#else
    mavlink_acfly_regeisterpossensor_t packet;
    packet.delay = delay;
    packet.trustXY = trustXY;
    packet.trustZ = trustZ;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.ind = ind;
    packet.type = type;
    packet.DataFrame = DataFrame;
    packet.DataType = DataType;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor, (const char *)&packet, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_MIN_LEN, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_CRC);
#endif
}

/**
 * @brief Send a acfly_regeisterpossensor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_acfly_regeisterpossensor_send_struct(mavlink_channel_t chan, const mavlink_acfly_regeisterpossensor_t* acfly_regeisterpossensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_acfly_regeisterpossensor_send(chan, acfly_regeisterpossensor->target_system, acfly_regeisterpossensor->target_component, acfly_regeisterpossensor->sensor_name, acfly_regeisterpossensor->ind, acfly_regeisterpossensor->type, acfly_regeisterpossensor->DataFrame, acfly_regeisterpossensor->DataType, acfly_regeisterpossensor->delay, acfly_regeisterpossensor->trustXY, acfly_regeisterpossensor->trustZ);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor, (const char *)acfly_regeisterpossensor, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_MIN_LEN, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_acfly_regeisterpossensor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, const char *sensor_name, int8_t ind, uint8_t type, uint8_t DataFrame, uint8_t DataType, float delay, float trustXY, float trustZ)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, delay);
    _mav_put_float(buf, 4, trustXY);
    _mav_put_float(buf, 8, trustZ);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_int8_t(buf, 30, ind);
    _mav_put_uint8_t(buf, 31, type);
    _mav_put_uint8_t(buf, 32, DataFrame);
    _mav_put_uint8_t(buf, 33, DataType);
    _mav_put_char_array(buf, 14, sensor_name, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor, buf, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_MIN_LEN, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_CRC);
#else
    mavlink_acfly_regeisterpossensor_t *packet = (mavlink_acfly_regeisterpossensor_t *)msgbuf;
    packet->delay = delay;
    packet->trustXY = trustXY;
    packet->trustZ = trustZ;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->ind = ind;
    packet->type = type;
    packet->DataFrame = DataFrame;
    packet->DataType = DataType;
    mav_array_memcpy(packet->sensor_name, sensor_name, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor, (const char *)packet, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_MIN_LEN, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_CRC);
#endif
}
#endif

#endif

// MESSAGE ACFly_RegeisterPosSensor UNPACKING


/**
 * @brief Get field target_system from acfly_regeisterpossensor message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_acfly_regeisterpossensor_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field target_component from acfly_regeisterpossensor message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_acfly_regeisterpossensor_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field sensor_name from acfly_regeisterpossensor message
 *
 * @return  position sensor name
 */
static inline uint16_t mavlink_msg_acfly_regeisterpossensor_get_sensor_name(const mavlink_message_t* msg, char *sensor_name)
{
    return _MAV_RETURN_char_array(msg, sensor_name, 16,  14);
}

/**
 * @brief Get field ind from acfly_regeisterpossensor message
 *
 * @return  Position sensor index.
 */
static inline int8_t mavlink_msg_acfly_regeisterpossensor_get_ind(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  30);
}

/**
 * @brief Get field type from acfly_regeisterpossensor message
 *
 * @return  Position sensor type.
 */
static inline uint8_t mavlink_msg_acfly_regeisterpossensor_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field DataFrame from acfly_regeisterpossensor message
 *
 * @return  0:position in ENU velocity in ENU   1:position in ENU velocity in bodyFLU   4:position in SLAM frame velocity in ENU   5:position in SLAM frame velocity in bodyFLU   bit7:sensor available.   bit6:1 to Unregister sensor.
 */
static inline uint8_t mavlink_msg_acfly_regeisterpossensor_get_DataFrame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field DataType from acfly_regeisterpossensor message
 *
 * @return  Position sensor data type.
 */
static inline uint8_t mavlink_msg_acfly_regeisterpossensor_get_DataType(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field delay from acfly_regeisterpossensor message
 *
 * @return  Sensor observation delay in seconds.
 */
static inline float mavlink_msg_acfly_regeisterpossensor_get_delay(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field trustXY from acfly_regeisterpossensor message
 *
 * @return  Sensor trust in meters in XY direction
 */
static inline float mavlink_msg_acfly_regeisterpossensor_get_trustXY(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field trustZ from acfly_regeisterpossensor message
 *
 * @return  Sensor trust in meters in Z direction
 */
static inline float mavlink_msg_acfly_regeisterpossensor_get_trustZ(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a acfly_regeisterpossensor message into a struct
 *
 * @param msg The message to decode
 * @param acfly_regeisterpossensor C-struct to decode the message contents into
 */
static inline void mavlink_msg_acfly_regeisterpossensor_decode(const mavlink_message_t* msg, mavlink_acfly_regeisterpossensor_t* acfly_regeisterpossensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    acfly_regeisterpossensor->delay = mavlink_msg_acfly_regeisterpossensor_get_delay(msg);
    acfly_regeisterpossensor->trustXY = mavlink_msg_acfly_regeisterpossensor_get_trustXY(msg);
    acfly_regeisterpossensor->trustZ = mavlink_msg_acfly_regeisterpossensor_get_trustZ(msg);
    acfly_regeisterpossensor->target_system = mavlink_msg_acfly_regeisterpossensor_get_target_system(msg);
    acfly_regeisterpossensor->target_component = mavlink_msg_acfly_regeisterpossensor_get_target_component(msg);
    mavlink_msg_acfly_regeisterpossensor_get_sensor_name(msg, acfly_regeisterpossensor->sensor_name);
    acfly_regeisterpossensor->ind = mavlink_msg_acfly_regeisterpossensor_get_ind(msg);
    acfly_regeisterpossensor->type = mavlink_msg_acfly_regeisterpossensor_get_type(msg);
    acfly_regeisterpossensor->DataFrame = mavlink_msg_acfly_regeisterpossensor_get_DataFrame(msg);
    acfly_regeisterpossensor->DataType = mavlink_msg_acfly_regeisterpossensor_get_DataType(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN? msg->len : MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN;
        memset(acfly_regeisterpossensor, 0, MAVLINK_MSG_ID_ACFly_RegeisterPosSensor_LEN);
    memcpy(acfly_regeisterpossensor, _MAV_PAYLOAD(msg), len);
#endif
}
