#pragma once
// MESSAGE ACFlyHTL_IMU_REGISTER PACKING

#define MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER 202


typedef struct __mavlink_acflyhtl_imu_register_t {
 double sensitivity; /*<  Sensor sensitivity*/
 uint16_t imu_type; /*<  Imu sensor type*/
 uint16_t freq; /*<  Sensor frequency(Hz in HTL time).*/
 uint8_t target_system; /*<  System ID*/
 uint8_t target_component; /*<  Component ID*/
 char sensor_name[16]; /*<  position sensor name*/
} mavlink_acflyhtl_imu_register_t;

#define MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN 30
#define MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_MIN_LEN 30
#define MAVLINK_MSG_ID_202_LEN 30
#define MAVLINK_MSG_ID_202_MIN_LEN 30

#define MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_CRC 243
#define MAVLINK_MSG_ID_202_CRC 243

#define MAVLINK_MSG_ACFlyHTL_IMU_REGISTER_FIELD_SENSOR_NAME_LEN 16

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACFlyHTL_IMU_REGISTER { \
    202, \
    "ACFlyHTL_IMU_REGISTER", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_acflyhtl_imu_register_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_acflyhtl_imu_register_t, target_component) }, \
         { "imu_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_acflyhtl_imu_register_t, imu_type) }, \
         { "freq", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_acflyhtl_imu_register_t, freq) }, \
         { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 16, 14, offsetof(mavlink_acflyhtl_imu_register_t, sensor_name) }, \
         { "sensitivity", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_acflyhtl_imu_register_t, sensitivity) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACFlyHTL_IMU_REGISTER { \
    "ACFlyHTL_IMU_REGISTER", \
    6, \
    {  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_acflyhtl_imu_register_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_acflyhtl_imu_register_t, target_component) }, \
         { "imu_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_acflyhtl_imu_register_t, imu_type) }, \
         { "freq", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_acflyhtl_imu_register_t, freq) }, \
         { "sensor_name", NULL, MAVLINK_TYPE_CHAR, 16, 14, offsetof(mavlink_acflyhtl_imu_register_t, sensor_name) }, \
         { "sensitivity", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_acflyhtl_imu_register_t, sensitivity) }, \
         } \
}
#endif

/**
 * @brief Pack a acflyhtl_imu_register message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param imu_type  Imu sensor type
 * @param freq  Sensor frequency(Hz in HTL time).
 * @param sensor_name  position sensor name
 * @param sensitivity  Sensor sensitivity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_register_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t imu_type, uint16_t freq, const char *sensor_name, double sensitivity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN];
    _mav_put_double(buf, 0, sensitivity);
    _mav_put_uint16_t(buf, 8, imu_type);
    _mav_put_uint16_t(buf, 10, freq);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_char_array(buf, 14, sensor_name, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN);
#else
    mavlink_acflyhtl_imu_register_t packet;
    packet.sensitivity = sensitivity;
    packet.imu_type = imu_type;
    packet.freq = freq;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_CRC);
}

/**
 * @brief Pack a acflyhtl_imu_register message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param imu_type  Imu sensor type
 * @param freq  Sensor frequency(Hz in HTL time).
 * @param sensor_name  position sensor name
 * @param sensitivity  Sensor sensitivity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_register_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint8_t target_system, uint8_t target_component, uint16_t imu_type, uint16_t freq, const char *sensor_name, double sensitivity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN];
    _mav_put_double(buf, 0, sensitivity);
    _mav_put_uint16_t(buf, 8, imu_type);
    _mav_put_uint16_t(buf, 10, freq);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_char_array(buf, 14, sensor_name, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN);
#else
    mavlink_acflyhtl_imu_register_t packet;
    packet.sensitivity = sensitivity;
    packet.imu_type = imu_type;
    packet.freq = freq;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN);
#endif
}

/**
 * @brief Pack a acflyhtl_imu_register message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param imu_type  Imu sensor type
 * @param freq  Sensor frequency(Hz in HTL time).
 * @param sensor_name  position sensor name
 * @param sensitivity  Sensor sensitivity
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_register_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t target_system,uint8_t target_component,uint16_t imu_type,uint16_t freq,const char *sensor_name,double sensitivity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN];
    _mav_put_double(buf, 0, sensitivity);
    _mav_put_uint16_t(buf, 8, imu_type);
    _mav_put_uint16_t(buf, 10, freq);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_char_array(buf, 14, sensor_name, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN);
#else
    mavlink_acflyhtl_imu_register_t packet;
    packet.sensitivity = sensitivity;
    packet.imu_type = imu_type;
    packet.freq = freq;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_CRC);
}

/**
 * @brief Encode a acflyhtl_imu_register struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param acflyhtl_imu_register C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_register_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_acflyhtl_imu_register_t* acflyhtl_imu_register)
{
    return mavlink_msg_acflyhtl_imu_register_pack(system_id, component_id, msg, acflyhtl_imu_register->target_system, acflyhtl_imu_register->target_component, acflyhtl_imu_register->imu_type, acflyhtl_imu_register->freq, acflyhtl_imu_register->sensor_name, acflyhtl_imu_register->sensitivity);
}

/**
 * @brief Encode a acflyhtl_imu_register struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param acflyhtl_imu_register C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_register_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_acflyhtl_imu_register_t* acflyhtl_imu_register)
{
    return mavlink_msg_acflyhtl_imu_register_pack_chan(system_id, component_id, chan, msg, acflyhtl_imu_register->target_system, acflyhtl_imu_register->target_component, acflyhtl_imu_register->imu_type, acflyhtl_imu_register->freq, acflyhtl_imu_register->sensor_name, acflyhtl_imu_register->sensitivity);
}

/**
 * @brief Encode a acflyhtl_imu_register struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param acflyhtl_imu_register C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_register_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_acflyhtl_imu_register_t* acflyhtl_imu_register)
{
    return mavlink_msg_acflyhtl_imu_register_pack_status(system_id, component_id, _status, msg,  acflyhtl_imu_register->target_system, acflyhtl_imu_register->target_component, acflyhtl_imu_register->imu_type, acflyhtl_imu_register->freq, acflyhtl_imu_register->sensor_name, acflyhtl_imu_register->sensitivity);
}

/**
 * @brief Send a acflyhtl_imu_register message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system  System ID
 * @param target_component  Component ID
 * @param imu_type  Imu sensor type
 * @param freq  Sensor frequency(Hz in HTL time).
 * @param sensor_name  position sensor name
 * @param sensitivity  Sensor sensitivity
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_acflyhtl_imu_register_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t imu_type, uint16_t freq, const char *sensor_name, double sensitivity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN];
    _mav_put_double(buf, 0, sensitivity);
    _mav_put_uint16_t(buf, 8, imu_type);
    _mav_put_uint16_t(buf, 10, freq);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_char_array(buf, 14, sensor_name, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER, buf, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_CRC);
#else
    mavlink_acflyhtl_imu_register_t packet;
    packet.sensitivity = sensitivity;
    packet.imu_type = imu_type;
    packet.freq = freq;
    packet.target_system = target_system;
    packet.target_component = target_component;
    mav_array_memcpy(packet.sensor_name, sensor_name, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER, (const char *)&packet, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_CRC);
#endif
}

/**
 * @brief Send a acflyhtl_imu_register message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_acflyhtl_imu_register_send_struct(mavlink_channel_t chan, const mavlink_acflyhtl_imu_register_t* acflyhtl_imu_register)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_acflyhtl_imu_register_send(chan, acflyhtl_imu_register->target_system, acflyhtl_imu_register->target_component, acflyhtl_imu_register->imu_type, acflyhtl_imu_register->freq, acflyhtl_imu_register->sensor_name, acflyhtl_imu_register->sensitivity);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER, (const char *)acflyhtl_imu_register, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_acflyhtl_imu_register_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint16_t imu_type, uint16_t freq, const char *sensor_name, double sensitivity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_double(buf, 0, sensitivity);
    _mav_put_uint16_t(buf, 8, imu_type);
    _mav_put_uint16_t(buf, 10, freq);
    _mav_put_uint8_t(buf, 12, target_system);
    _mav_put_uint8_t(buf, 13, target_component);
    _mav_put_char_array(buf, 14, sensor_name, 16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER, buf, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_CRC);
#else
    mavlink_acflyhtl_imu_register_t *packet = (mavlink_acflyhtl_imu_register_t *)msgbuf;
    packet->sensitivity = sensitivity;
    packet->imu_type = imu_type;
    packet->freq = freq;
    packet->target_system = target_system;
    packet->target_component = target_component;
    mav_array_memcpy(packet->sensor_name, sensor_name, sizeof(char)*16);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER, (const char *)packet, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_CRC);
#endif
}
#endif

#endif

// MESSAGE ACFlyHTL_IMU_REGISTER UNPACKING


/**
 * @brief Get field target_system from acflyhtl_imu_register message
 *
 * @return  System ID
 */
static inline uint8_t mavlink_msg_acflyhtl_imu_register_get_target_system(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field target_component from acflyhtl_imu_register message
 *
 * @return  Component ID
 */
static inline uint8_t mavlink_msg_acflyhtl_imu_register_get_target_component(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field imu_type from acflyhtl_imu_register message
 *
 * @return  Imu sensor type
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_register_get_imu_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field freq from acflyhtl_imu_register message
 *
 * @return  Sensor frequency(Hz in HTL time).
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_register_get_freq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field sensor_name from acflyhtl_imu_register message
 *
 * @return  position sensor name
 */
static inline uint16_t mavlink_msg_acflyhtl_imu_register_get_sensor_name(const mavlink_message_t* msg, char *sensor_name)
{
    return _MAV_RETURN_char_array(msg, sensor_name, 16,  14);
}

/**
 * @brief Get field sensitivity from acflyhtl_imu_register message
 *
 * @return  Sensor sensitivity
 */
static inline double mavlink_msg_acflyhtl_imu_register_get_sensitivity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Decode a acflyhtl_imu_register message into a struct
 *
 * @param msg The message to decode
 * @param acflyhtl_imu_register C-struct to decode the message contents into
 */
static inline void mavlink_msg_acflyhtl_imu_register_decode(const mavlink_message_t* msg, mavlink_acflyhtl_imu_register_t* acflyhtl_imu_register)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    acflyhtl_imu_register->sensitivity = mavlink_msg_acflyhtl_imu_register_get_sensitivity(msg);
    acflyhtl_imu_register->imu_type = mavlink_msg_acflyhtl_imu_register_get_imu_type(msg);
    acflyhtl_imu_register->freq = mavlink_msg_acflyhtl_imu_register_get_freq(msg);
    acflyhtl_imu_register->target_system = mavlink_msg_acflyhtl_imu_register_get_target_system(msg);
    acflyhtl_imu_register->target_component = mavlink_msg_acflyhtl_imu_register_get_target_component(msg);
    mavlink_msg_acflyhtl_imu_register_get_sensor_name(msg, acflyhtl_imu_register->sensor_name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN? msg->len : MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN;
        memset(acflyhtl_imu_register, 0, MAVLINK_MSG_ID_ACFlyHTL_IMU_REGISTER_LEN);
    memcpy(acflyhtl_imu_register, _MAV_PAYLOAD(msg), len);
#endif
}
