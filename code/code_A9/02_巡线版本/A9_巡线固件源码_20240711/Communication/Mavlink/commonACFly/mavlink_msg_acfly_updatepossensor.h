#pragma once
// MESSAGE ACFly_UpdatePosSensor PACKING

#define MAVLINK_MSG_ID_ACFly_UpdatePosSensor 209

MAVPACKED(
typedef struct __mavlink_acfly_updatepossensor_t {
 double posX; /*<  Position X in m or in Lat*/
 double posY; /*<  Position Y in m or in Lon*/
 double posZ; /*<  Position Z in m*/
 float velX; /*<  Velocity X in m/s*/
 float velY; /*<  Velocity Y in m/s*/
 float velZ; /*<  Velocity Z in m/s*/
 float delay; /*<  Sensor observation delay in seconds. Set to -1 if value not changed.*/
 float trustXY; /*<  Sensor trust in m in XY direction. Set to -1 if value not changed.*/
 float trustZ; /*<  Sensor trust in m in Z direction. Set to -1 if value not changed.*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 int8_t ind; /*<  Position sensor index.*/
 int8_t DataType; /*<  Position sensor data type. Set to -1 if value not changed.*/
 uint8_t reset; /*<  Reset counter should be increased if sensor data jumped(SLAM frame changed).*/
 float AttQuat[4]; /*<  SLAM attitude quaternion(Map to SLAM body, Axis Z must be upwords). Must be sent if position in SLAM frame.*/
}) mavlink_acfly_updatepossensor_t;

#define MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN 69
#define MAVLINK_MSG_ID_ACFly_UpdatePosSensor_MIN_LEN 53
#define MAVLINK_MSG_ID_209_LEN 69
#define MAVLINK_MSG_ID_209_MIN_LEN 53

#define MAVLINK_MSG_ID_ACFly_UpdatePosSensor_CRC 47
#define MAVLINK_MSG_ID_209_CRC 47

#define MAVLINK_MSG_ACFly_UpdatePosSensor_FIELD_ATTQUAT_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACFly_UpdatePosSensor { \
    209, \
    "ACFly_UpdatePosSensor", \
    15, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_acfly_updatepossensor_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_acfly_updatepossensor_t, target_component) }, \
         { "ind", NULL, MAVLINK_TYPE_INT8_T, 0, 50, offsetof(mavlink_acfly_updatepossensor_t, ind) }, \
         { "DataType", NULL, MAVLINK_TYPE_INT8_T, 0, 51, offsetof(mavlink_acfly_updatepossensor_t, DataType) }, \
         { "posX", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_acfly_updatepossensor_t, posX) }, \
         { "posY", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_acfly_updatepossensor_t, posY) }, \
         { "posZ", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_acfly_updatepossensor_t, posZ) }, \
         { "velX", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_acfly_updatepossensor_t, velX) }, \
         { "velY", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_acfly_updatepossensor_t, velY) }, \
         { "velZ", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_acfly_updatepossensor_t, velZ) }, \
         { "delay", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_acfly_updatepossensor_t, delay) }, \
         { "trustXY", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_acfly_updatepossensor_t, trustXY) }, \
         { "trustZ", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_acfly_updatepossensor_t, trustZ) }, \
         { "reset", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_acfly_updatepossensor_t, reset) }, \
         { "AttQuat", NULL, MAVLINK_TYPE_FLOAT, 4, 53, offsetof(mavlink_acfly_updatepossensor_t, AttQuat) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACFly_UpdatePosSensor { \
    "ACFly_UpdatePosSensor", \
    15, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_acfly_updatepossensor_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_acfly_updatepossensor_t, target_component) }, \
         { "ind", NULL, MAVLINK_TYPE_INT8_T, 0, 50, offsetof(mavlink_acfly_updatepossensor_t, ind) }, \
         { "DataType", NULL, MAVLINK_TYPE_INT8_T, 0, 51, offsetof(mavlink_acfly_updatepossensor_t, DataType) }, \
         { "posX", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_acfly_updatepossensor_t, posX) }, \
         { "posY", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_acfly_updatepossensor_t, posY) }, \
         { "posZ", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_acfly_updatepossensor_t, posZ) }, \
         { "velX", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_acfly_updatepossensor_t, velX) }, \
         { "velY", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_acfly_updatepossensor_t, velY) }, \
         { "velZ", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_acfly_updatepossensor_t, velZ) }, \
         { "delay", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_acfly_updatepossensor_t, delay) }, \
         { "trustXY", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_acfly_updatepossensor_t, trustXY) }, \
         { "trustZ", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_acfly_updatepossensor_t, trustZ) }, \
         { "reset", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_acfly_updatepossensor_t, reset) }, \
         { "AttQuat", NULL, MAVLINK_TYPE_FLOAT, 4, 53, offsetof(mavlink_acfly_updatepossensor_t, AttQuat) }, \
         } \
}
#endif

/**
 * @brief Pack a acfly_updatepossensor message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param ind  Position sensor index.
 * @param DataType  Position sensor data type. Set to -1 if value not changed.
 * @param posX  Position X in m or in Lat
 * @param posY  Position Y in m or in Lon
 * @param posZ  Position Z in m
 * @param velX  Velocity X in m/s
 * @param velY  Velocity Y in m/s
 * @param velZ  Velocity Z in m/s
 * @param delay  Sensor observation delay in seconds. Set to -1 if value not changed.
 * @param trustXY  Sensor trust in m in XY direction. Set to -1 if value not changed.
 * @param trustZ  Sensor trust in m in Z direction. Set to -1 if value not changed.
 * @param reset  Reset counter should be increased if sensor data jumped(SLAM frame changed).
 * @param AttQuat  SLAM attitude quaternion(Map to SLAM body, Axis Z must be upwords). Must be sent if position in SLAM frame.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acfly_updatepossensor_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, int8_t ind, int8_t DataType, double posX, double posY, double posZ, float velX, float velY, float velZ, float delay, float trustXY, float trustZ, uint8_t reset, const float *AttQuat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN];
    _mav_put_double(buf, 0, posX);
    _mav_put_double(buf, 8, posY);
    _mav_put_double(buf, 16, posZ);
    _mav_put_float(buf, 24, velX);
    _mav_put_float(buf, 28, velY);
    _mav_put_float(buf, 32, velZ);
    _mav_put_float(buf, 36, delay);
    _mav_put_float(buf, 40, trustXY);
    _mav_put_float(buf, 44, trustZ);
    _mav_put_uint8_t(buf, 48, target_system);
    _mav_put_uint8_t(buf, 49, target_component);
    _mav_put_int8_t(buf, 50, ind);
    _mav_put_int8_t(buf, 51, DataType);
    _mav_put_uint8_t(buf, 52, reset);
    _mav_put_float_array(buf, 53, AttQuat, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN);
#else
    mavlink_acfly_updatepossensor_t packet;
    packet.posX = posX;
    packet.posY = posY;
    packet.posZ = posZ;
    packet.velX = velX;
    packet.velY = velY;
    packet.velZ = velZ;
    packet.delay = delay;
    packet.trustXY = trustXY;
    packet.trustZ = trustZ;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.ind = ind;
    packet.DataType = DataType;
    packet.reset = reset;
    mav_array_memcpy(packet.AttQuat, AttQuat, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFly_UpdatePosSensor;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_MIN_LEN, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_CRC);
}

/**
 * @brief Pack a acfly_updatepossensor message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param ind  Position sensor index.
 * @param DataType  Position sensor data type. Set to -1 if value not changed.
 * @param posX  Position X in m or in Lat
 * @param posY  Position Y in m or in Lon
 * @param posZ  Position Z in m
 * @param velX  Velocity X in m/s
 * @param velY  Velocity Y in m/s
 * @param velZ  Velocity Z in m/s
 * @param delay  Sensor observation delay in seconds. Set to -1 if value not changed.
 * @param trustXY  Sensor trust in m in XY direction. Set to -1 if value not changed.
 * @param trustZ  Sensor trust in m in Z direction. Set to -1 if value not changed.
 * @param reset  Reset counter should be increased if sensor data jumped(SLAM frame changed).
 * @param AttQuat  SLAM attitude quaternion(Map to SLAM body, Axis Z must be upwords). Must be sent if position in SLAM frame.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acfly_updatepossensor_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,int8_t ind,int8_t DataType,double posX,double posY,double posZ,float velX,float velY,float velZ,float delay,float trustXY,float trustZ,uint8_t reset,const float *AttQuat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN];
    _mav_put_double(buf, 0, posX);
    _mav_put_double(buf, 8, posY);
    _mav_put_double(buf, 16, posZ);
    _mav_put_float(buf, 24, velX);
    _mav_put_float(buf, 28, velY);
    _mav_put_float(buf, 32, velZ);
    _mav_put_float(buf, 36, delay);
    _mav_put_float(buf, 40, trustXY);
    _mav_put_float(buf, 44, trustZ);
    _mav_put_uint8_t(buf, 48, target_system);
    _mav_put_uint8_t(buf, 49, target_component);
    _mav_put_int8_t(buf, 50, ind);
    _mav_put_int8_t(buf, 51, DataType);
    _mav_put_uint8_t(buf, 52, reset);
    _mav_put_float_array(buf, 53, AttQuat, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN);
#else
    mavlink_acfly_updatepossensor_t packet;
    packet.posX = posX;
    packet.posY = posY;
    packet.posZ = posZ;
    packet.velX = velX;
    packet.velY = velY;
    packet.velZ = velZ;
    packet.delay = delay;
    packet.trustXY = trustXY;
    packet.trustZ = trustZ;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.ind = ind;
    packet.DataType = DataType;
    packet.reset = reset;
    mav_array_memcpy(packet.AttQuat, AttQuat, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFly_UpdatePosSensor;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_MIN_LEN, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_CRC);
}

/**
 * @brief Encode a acfly_updatepossensor struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param acfly_updatepossensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acfly_updatepossensor_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_acfly_updatepossensor_t* acfly_updatepossensor)
{
    return mavlink_msg_acfly_updatepossensor_pack(system_id, component_id, msg, acfly_updatepossensor->target_system, acfly_updatepossensor->target_component, acfly_updatepossensor->ind, acfly_updatepossensor->DataType, acfly_updatepossensor->posX, acfly_updatepossensor->posY, acfly_updatepossensor->posZ, acfly_updatepossensor->velX, acfly_updatepossensor->velY, acfly_updatepossensor->velZ, acfly_updatepossensor->delay, acfly_updatepossensor->trustXY, acfly_updatepossensor->trustZ, acfly_updatepossensor->reset, acfly_updatepossensor->AttQuat);
}

/**
 * @brief Encode a acfly_updatepossensor struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param acfly_updatepossensor C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acfly_updatepossensor_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_acfly_updatepossensor_t* acfly_updatepossensor)
{
    return mavlink_msg_acfly_updatepossensor_pack_chan(system_id, component_id, chan, msg, acfly_updatepossensor->target_system, acfly_updatepossensor->target_component, acfly_updatepossensor->ind, acfly_updatepossensor->DataType, acfly_updatepossensor->posX, acfly_updatepossensor->posY, acfly_updatepossensor->posZ, acfly_updatepossensor->velX, acfly_updatepossensor->velY, acfly_updatepossensor->velZ, acfly_updatepossensor->delay, acfly_updatepossensor->trustXY, acfly_updatepossensor->trustZ, acfly_updatepossensor->reset, acfly_updatepossensor->AttQuat);
}

/**
 * @brief Send a acfly_updatepossensor message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param ind  Position sensor index.
 * @param DataType  Position sensor data type. Set to -1 if value not changed.
 * @param posX  Position X in m or in Lat
 * @param posY  Position Y in m or in Lon
 * @param posZ  Position Z in m
 * @param velX  Velocity X in m/s
 * @param velY  Velocity Y in m/s
 * @param velZ  Velocity Z in m/s
 * @param delay  Sensor observation delay in seconds. Set to -1 if value not changed.
 * @param trustXY  Sensor trust in m in XY direction. Set to -1 if value not changed.
 * @param trustZ  Sensor trust in m in Z direction. Set to -1 if value not changed.
 * @param reset  Reset counter should be increased if sensor data jumped(SLAM frame changed).
 * @param AttQuat  SLAM attitude quaternion(Map to SLAM body, Axis Z must be upwords). Must be sent if position in SLAM frame.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_acfly_updatepossensor_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, int8_t ind, int8_t DataType, double posX, double posY, double posZ, float velX, float velY, float velZ, float delay, float trustXY, float trustZ, uint8_t reset, const float *AttQuat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN];
    _mav_put_double(buf, 0, posX);
    _mav_put_double(buf, 8, posY);
    _mav_put_double(buf, 16, posZ);
    _mav_put_float(buf, 24, velX);
    _mav_put_float(buf, 28, velY);
    _mav_put_float(buf, 32, velZ);
    _mav_put_float(buf, 36, delay);
    _mav_put_float(buf, 40, trustXY);
    _mav_put_float(buf, 44, trustZ);
    _mav_put_uint8_t(buf, 48, target_system);
    _mav_put_uint8_t(buf, 49, target_component);
    _mav_put_int8_t(buf, 50, ind);
    _mav_put_int8_t(buf, 51, DataType);
    _mav_put_uint8_t(buf, 52, reset);
    _mav_put_float_array(buf, 53, AttQuat, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFly_UpdatePosSensor, buf, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_MIN_LEN, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_CRC);
#else
    mavlink_acfly_updatepossensor_t packet;
    packet.posX = posX;
    packet.posY = posY;
    packet.posZ = posZ;
    packet.velX = velX;
    packet.velY = velY;
    packet.velZ = velZ;
    packet.delay = delay;
    packet.trustXY = trustXY;
    packet.trustZ = trustZ;
    packet.target_system = target_system;
    packet.target_component = target_component;
    packet.ind = ind;
    packet.DataType = DataType;
    packet.reset = reset;
    mav_array_memcpy(packet.AttQuat, AttQuat, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFly_UpdatePosSensor, (const char *)&packet, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_MIN_LEN, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_CRC);
#endif
}

/**
 * @brief Send a acfly_updatepossensor message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_acfly_updatepossensor_send_struct(mavlink_channel_t chan, const mavlink_acfly_updatepossensor_t* acfly_updatepossensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_acfly_updatepossensor_send(chan, acfly_updatepossensor->target_system, acfly_updatepossensor->target_component, acfly_updatepossensor->ind, acfly_updatepossensor->DataType, acfly_updatepossensor->posX, acfly_updatepossensor->posY, acfly_updatepossensor->posZ, acfly_updatepossensor->velX, acfly_updatepossensor->velY, acfly_updatepossensor->velZ, acfly_updatepossensor->delay, acfly_updatepossensor->trustXY, acfly_updatepossensor->trustZ, acfly_updatepossensor->reset, acfly_updatepossensor->AttQuat);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFly_UpdatePosSensor, (const char *)acfly_updatepossensor, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_MIN_LEN, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_acfly_updatepossensor_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, int8_t ind, int8_t DataType, double posX, double posY, double posZ, float velX, float velY, float velZ, float delay, float trustXY, float trustZ, uint8_t reset, const float *AttQuat)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_double(buf, 0, posX);
    _mav_put_double(buf, 8, posY);
    _mav_put_double(buf, 16, posZ);
    _mav_put_float(buf, 24, velX);
    _mav_put_float(buf, 28, velY);
    _mav_put_float(buf, 32, velZ);
    _mav_put_float(buf, 36, delay);
    _mav_put_float(buf, 40, trustXY);
    _mav_put_float(buf, 44, trustZ);
    _mav_put_uint8_t(buf, 48, target_system);
    _mav_put_uint8_t(buf, 49, target_component);
    _mav_put_int8_t(buf, 50, ind);
    _mav_put_int8_t(buf, 51, DataType);
    _mav_put_uint8_t(buf, 52, reset);
    _mav_put_float_array(buf, 53, AttQuat, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFly_UpdatePosSensor, buf, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_MIN_LEN, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_CRC);
#else
    mavlink_acfly_updatepossensor_t *packet = (mavlink_acfly_updatepossensor_t *)msgbuf;
    packet->posX = posX;
    packet->posY = posY;
    packet->posZ = posZ;
    packet->velX = velX;
    packet->velY = velY;
    packet->velZ = velZ;
    packet->delay = delay;
    packet->trustXY = trustXY;
    packet->trustZ = trustZ;
    packet->target_system = target_system;
    packet->target_component = target_component;
    packet->ind = ind;
    packet->DataType = DataType;
    packet->reset = reset;
    mav_array_memcpy(packet->AttQuat, AttQuat, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFly_UpdatePosSensor, (const char *)packet, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_MIN_LEN, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_CRC);
#endif
}
#endif

#endif

// MESSAGE ACFly_UpdatePosSensor UNPACKING


/**
 * @brief Get field target_system from acfly_updatepossensor message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_acfly_updatepossensor_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field target_component from acfly_updatepossensor message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_acfly_updatepossensor_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Get field ind from acfly_updatepossensor message
 *
 * @return  Position sensor index.
 */
static inline int8_t mavlink_msg_acfly_updatepossensor_get_ind(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  50);
}

/**
 * @brief Get field DataType from acfly_updatepossensor message
 *
 * @return  Position sensor data type. Set to -1 if value not changed.
 */
static inline int8_t mavlink_msg_acfly_updatepossensor_get_DataType(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  51);
}

/**
 * @brief Get field posX from acfly_updatepossensor message
 *
 * @return  Position X in m or in Lat
 */
static inline double mavlink_msg_acfly_updatepossensor_get_posX(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Get field posY from acfly_updatepossensor message
 *
 * @return  Position Y in m or in Lon
 */
static inline double mavlink_msg_acfly_updatepossensor_get_posY(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field posZ from acfly_updatepossensor message
 *
 * @return  Position Z in m
 */
static inline double mavlink_msg_acfly_updatepossensor_get_posZ(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Get field velX from acfly_updatepossensor message
 *
 * @return  Velocity X in m/s
 */
static inline float mavlink_msg_acfly_updatepossensor_get_velX(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field velY from acfly_updatepossensor message
 *
 * @return  Velocity Y in m/s
 */
static inline float mavlink_msg_acfly_updatepossensor_get_velY(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field velZ from acfly_updatepossensor message
 *
 * @return  Velocity Z in m/s
 */
static inline float mavlink_msg_acfly_updatepossensor_get_velZ(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field delay from acfly_updatepossensor message
 *
 * @return  Sensor observation delay in seconds. Set to -1 if value not changed.
 */
static inline float mavlink_msg_acfly_updatepossensor_get_delay(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field trustXY from acfly_updatepossensor message
 *
 * @return  Sensor trust in m in XY direction. Set to -1 if value not changed.
 */
static inline float mavlink_msg_acfly_updatepossensor_get_trustXY(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field trustZ from acfly_updatepossensor message
 *
 * @return  Sensor trust in m in Z direction. Set to -1 if value not changed.
 */
static inline float mavlink_msg_acfly_updatepossensor_get_trustZ(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field reset from acfly_updatepossensor message
 *
 * @return  Reset counter should be increased if sensor data jumped(SLAM frame changed).
 */
static inline uint8_t mavlink_msg_acfly_updatepossensor_get_reset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field AttQuat from acfly_updatepossensor message
 *
 * @return  SLAM attitude quaternion(Map to SLAM body, Axis Z must be upwords). Must be sent if position in SLAM frame.
 */
static inline uint16_t mavlink_msg_acfly_updatepossensor_get_AttQuat(const mavlink_message_t* msg, float *AttQuat)
{
    return _MAV_RETURN_float_array(msg, AttQuat, 4,  53);
}

/**
 * @brief Decode a acfly_updatepossensor message into a struct
 *
 * @param msg The message to decode
 * @param acfly_updatepossensor C-struct to decode the message contents into
 */
static inline void mavlink_msg_acfly_updatepossensor_decode(const mavlink_message_t* msg, mavlink_acfly_updatepossensor_t* acfly_updatepossensor)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    acfly_updatepossensor->posX = mavlink_msg_acfly_updatepossensor_get_posX(msg);
    acfly_updatepossensor->posY = mavlink_msg_acfly_updatepossensor_get_posY(msg);
    acfly_updatepossensor->posZ = mavlink_msg_acfly_updatepossensor_get_posZ(msg);
    acfly_updatepossensor->velX = mavlink_msg_acfly_updatepossensor_get_velX(msg);
    acfly_updatepossensor->velY = mavlink_msg_acfly_updatepossensor_get_velY(msg);
    acfly_updatepossensor->velZ = mavlink_msg_acfly_updatepossensor_get_velZ(msg);
    acfly_updatepossensor->delay = mavlink_msg_acfly_updatepossensor_get_delay(msg);
    acfly_updatepossensor->trustXY = mavlink_msg_acfly_updatepossensor_get_trustXY(msg);
    acfly_updatepossensor->trustZ = mavlink_msg_acfly_updatepossensor_get_trustZ(msg);
    acfly_updatepossensor->target_system = mavlink_msg_acfly_updatepossensor_get_target_system(msg);
    acfly_updatepossensor->target_component = mavlink_msg_acfly_updatepossensor_get_target_component(msg);
    acfly_updatepossensor->ind = mavlink_msg_acfly_updatepossensor_get_ind(msg);
    acfly_updatepossensor->DataType = mavlink_msg_acfly_updatepossensor_get_DataType(msg);
    acfly_updatepossensor->reset = mavlink_msg_acfly_updatepossensor_get_reset(msg);
    mavlink_msg_acfly_updatepossensor_get_AttQuat(msg, acfly_updatepossensor->AttQuat);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN? msg->len : MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN;
        memset(acfly_updatepossensor, 0, MAVLINK_MSG_ID_ACFly_UpdatePosSensor_LEN);
    memcpy(acfly_updatepossensor, _MAV_PAYLOAD(msg), len);
#endif
}
