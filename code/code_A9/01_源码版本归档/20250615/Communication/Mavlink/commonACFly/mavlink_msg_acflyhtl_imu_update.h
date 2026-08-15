#pragma once
// MESSAGE ACFlyHTL_IMU_UPDATE PACKING

#define MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE 203


typedef struct __mavlink_acflyhtl_imu_update_t {
 int32_t x; /*<  Sensor x(front axis) data*/
 int32_t y; /*<  Sensor y(left axis) data*/
 int32_t z; /*<  Sensor z(up axis) data*/
 uint16_t imu_type; /*<  Imu sensor type*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
} mavlink_acflyhtl_imu_update_t;

#define MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN 16
#define MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_MIN_LEN 16
#define MAVLINK_MSG_ID_203_LEN 16
#define MAVLINK_MSG_ID_203_MIN_LEN 16

#define MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_CRC 170
#define MAVLINK_MSG_ID_203_CRC 170



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACFlyHTL_IMU_UPDATE { \
    203, \
    "ACFlyHTL_IMU_UPDATE", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_acflyhtl_imu_update_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_acflyhtl_imu_update_t, target_component) }, \
         { "imu_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_acflyhtl_imu_update_t, imu_type) }, \
         { "x", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_acflyhtl_imu_update_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_acflyhtl_imu_update_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_acflyhtl_imu_update_t, z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACFlyHTL_IMU_UPDATE { \
    "ACFlyHTL_IMU_UPDATE", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_acflyhtl_imu_update_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_acflyhtl_imu_update_t, target_component) }, \
         { "imu_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_acflyhtl_imu_update_t, imu_type) }, \
         { "x", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_acflyhtl_imu_update_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_acflyhtl_imu_update_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_acflyhtl_imu_update_t, z) }, \
         } \
}
#endif

/**
 * @brief Pack a acflyhtl_imu_update message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param imu_type  Imu sensor type
 * @param x  Sensor x(front axis) data
 * @param y  Sensor y(left axis) data
 * @param z  Sensor z(up axis) data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_update_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t imu_type, int32_t x, int32_t y, int32_t z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN];
    _mav_put_int32_t(buf, 0, x);
    _mav_put_int32_t(buf, 4, y);
    _mav_put_int32_t(buf, 8, z);
    _mav_put_uint16_t(buf, 12, imu_type);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN);
#else
    mavlink_acflyhtl_imu_update_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.imu_type = imu_type;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_CRC);
}

/**
 * @brief Pack a acflyhtl_imu_update message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param imu_type  Imu sensor type
 * @param x  Sensor x(front axis) data
 * @param y  Sensor y(left axis) data
 * @param z  Sensor z(up axis) data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_update_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t imu_type, int32_t x, int32_t y, int32_t z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN];
    _mav_put_int32_t(buf, 0, x);
    _mav_put_int32_t(buf, 4, y);
    _mav_put_int32_t(buf, 8, z);
    _mav_put_uint16_t(buf, 12, imu_type);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN);
#else
    mavlink_acflyhtl_imu_update_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.imu_type = imu_type;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN);
#endif
}

/**
 * @brief Pack a acflyhtl_imu_update message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param imu_type  Imu sensor type
 * @param x  Sensor x(front axis) data
 * @param y  Sensor y(left axis) data
 * @param z  Sensor z(up axis) data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_update_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint16_t imu_type,int32_t x,int32_t y,int32_t z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN];
    _mav_put_int32_t(buf, 0, x);
    _mav_put_int32_t(buf, 4, y);
    _mav_put_int32_t(buf, 8, z);
    _mav_put_uint16_t(buf, 12, imu_type);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN);
#else
    mavlink_acflyhtl_imu_update_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.imu_type = imu_type;
    packet.target_system = target_system;
    packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_CRC);
}

/**
 * @brief Encode a acflyhtl_imu_update struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param acflyhtl_imu_update C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_update_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_acflyhtl_imu_update_t* acflyhtl_imu_update)
{
    return mavlink_msg_acflyhtl_imu_update_pack(system_id, component_id, msg, acflyhtl_imu_update->target_system, acflyhtl_imu_update->target_component, acflyhtl_imu_update->imu_type, acflyhtl_imu_update->x, acflyhtl_imu_update->y, acflyhtl_imu_update->z);
}

/**
 * @brief Encode a acflyhtl_imu_update struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param acflyhtl_imu_update C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_update_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_acflyhtl_imu_update_t* acflyhtl_imu_update)
{
    return mavlink_msg_acflyhtl_imu_update_pack_chan(system_id, component_id, chan, msg, acflyhtl_imu_update->target_system, acflyhtl_imu_update->target_component, acflyhtl_imu_update->imu_type, acflyhtl_imu_update->x, acflyhtl_imu_update->y, acflyhtl_imu_update->z);
}

/**
 * @brief Encode a acflyhtl_imu_update struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param acflyhtl_imu_update C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_update_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_acflyhtl_imu_update_t* acflyhtl_imu_update)
{
    return mavlink_msg_acflyhtl_imu_update_pack_status(system_id, component_id, _status, msg,  acflyhtl_imu_update->target_system, acflyhtl_imu_update->target_component, acflyhtl_imu_update->imu_type, acflyhtl_imu_update->x, acflyhtl_imu_update->y, acflyhtl_imu_update->z);
}

/**
 * @brief Send a acflyhtl_imu_update message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param imu_type  Imu sensor type
 * @param x  Sensor x(front axis) data
 * @param y  Sensor y(left axis) data
 * @param z  Sensor z(up axis) data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_acflyhtl_imu_update_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t imu_type, int32_t x, int32_t y, int32_t z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN];
    _mav_put_int32_t(buf, 0, x);
    _mav_put_int32_t(buf, 4, y);
    _mav_put_int32_t(buf, 8, z);
    _mav_put_uint16_t(buf, 12, imu_type);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE, buf, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_CRC);
#else
    mavlink_acflyhtl_imu_update_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.imu_type = imu_type;
    packet.target_system = target_system;
    packet.target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE, (const char *)&packet, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_CRC);
#endif
}

/**
 * @brief Send a acflyhtl_imu_update message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_acflyhtl_imu_update_send_struct(mavlink_channel_t chan, const mavlink_acflyhtl_imu_update_t* acflyhtl_imu_update)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_acflyhtl_imu_update_send(chan, acflyhtl_imu_update->target_system, acflyhtl_imu_update->target_component, acflyhtl_imu_update->imu_type, acflyhtl_imu_update->x, acflyhtl_imu_update->y, acflyhtl_imu_update->z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE, (const char *)acflyhtl_imu_update, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_acflyhtl_imu_update_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t imu_type, int32_t x, int32_t y, int32_t z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int32_t(buf, 0, x);
    _mav_put_int32_t(buf, 4, y);
    _mav_put_int32_t(buf, 8, z);
    _mav_put_uint16_t(buf, 12, imu_type);
    _mav_put_uint8_t(buf, 14, target_system);
    _mav_put_uint8_t(buf, 15, target_component);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE, buf, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_CRC);
#else
    mavlink_acflyhtl_imu_update_t *packet = (mavlink_acflyhtl_imu_update_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->imu_type = imu_type;
    packet->target_system = target_system;
    packet->target_component = target_component;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE, (const char *)packet, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_CRC);
#endif
}
#endif

#endif

// MESSAGE ACFlyHTL_IMU_UPDATE UNPACKING


/**
 * @brief Get field target_system from acflyhtl_imu_update message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_acflyhtl_imu_update_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field target_component from acflyhtl_imu_update message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_acflyhtl_imu_update_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field imu_type from acflyhtl_imu_update message
 *
 * @return  Imu sensor type
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_update_get_imu_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field x from acflyhtl_imu_update message
 *
 * @return  Sensor x(front axis) data
 */
static inline int32_t mavlink_msg_acflyhtl_imu_update_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field y from acflyhtl_imu_update message
 *
 * @return  Sensor y(left axis) data
 */
static inline int32_t mavlink_msg_acflyhtl_imu_update_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field z from acflyhtl_imu_update message
 *
 * @return  Sensor z(up axis) data
 */
static inline int32_t mavlink_msg_acflyhtl_imu_update_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Decode a acflyhtl_imu_update message into a struct
 *
 * @param msg The message to decode
 * @param acflyhtl_imu_update C-struct to decode the message contents into
 */
static inline void mavlink_msg_acflyhtl_imu_update_decode(const mavlink_message_t* msg, mavlink_acflyhtl_imu_update_t* acflyhtl_imu_update)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    acflyhtl_imu_update->x = mavlink_msg_acflyhtl_imu_update_get_x(msg);
    acflyhtl_imu_update->y = mavlink_msg_acflyhtl_imu_update_get_y(msg);
    acflyhtl_imu_update->z = mavlink_msg_acflyhtl_imu_update_get_z(msg);
    acflyhtl_imu_update->imu_type = mavlink_msg_acflyhtl_imu_update_get_imu_type(msg);
    acflyhtl_imu_update->target_system = mavlink_msg_acflyhtl_imu_update_get_target_system(msg);
    acflyhtl_imu_update->target_component = mavlink_msg_acflyhtl_imu_update_get_target_component(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN? msg->len : MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN;
        memset(acflyhtl_imu_update, 0, MAVLINK_MSG_ID_ACFlyHTL_IMU_UPDATE_LEN);
    memcpy(acflyhtl_imu_update, _MAV_PAYLOAD(msg), len);
#endif
}
