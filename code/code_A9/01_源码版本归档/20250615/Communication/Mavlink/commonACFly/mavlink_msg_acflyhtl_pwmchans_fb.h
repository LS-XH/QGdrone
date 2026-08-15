#pragma once
// MESSAGE ACFlyHTL_PWMCHANS_FB PACKING

#define MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB 201


typedef struct __mavlink_acflyhtl_pwmchans_fb_t {
 uint16_t pwms[32]; /*<  Pwms in us.*/
} mavlink_acflyhtl_pwmchans_fb_t;

#define MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN 64
#define MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_MIN_LEN 64
#define MAVLINK_MSG_ID_201_LEN 64
#define MAVLINK_MSG_ID_201_MIN_LEN 64

#define MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_CRC 33
#define MAVLINK_MSG_ID_201_CRC 33

#define MAVLINK_MSG_ACFlyHTL_PWMCHANS_FB_FIELD_PWMS_LEN 32

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACFlyHTL_PWMCHANS_FB { \
    201, \
    "ACFlyHTL_PWMCHANS_FB", \
    1, \
    {  { "pwms", NULL, MAVLINK_TYPE_UINT16_T, 32, 0, offsetof(mavlink_acflyhtl_pwmchans_fb_t, pwms) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACFlyHTL_PWMCHANS_FB { \
    "ACFlyHTL_PWMCHANS_FB", \
    1, \
    {  { "pwms", NULL, MAVLINK_TYPE_UINT16_T, 32, 0, offsetof(mavlink_acflyhtl_pwmchans_fb_t, pwms) }, \
         } \
}
#endif

/**
 * @brief Pack a acflyhtl_pwmchans_fb message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pwms  Pwms in us.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acflyhtl_pwmchans_fb_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint16_t *pwms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN];

    _mav_put_uint16_t_array(buf, 0, pwms, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN);
#else
    mavlink_acflyhtl_pwmchans_fb_t packet;

    mav_array_memcpy(packet.pwms, pwms, sizeof(uint16_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_CRC);
}

/**
 * @brief Pack a acflyhtl_pwmchans_fb message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param pwms  Pwms in us.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acflyhtl_pwmchans_fb_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               const uint16_t *pwms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN];

    _mav_put_uint16_t_array(buf, 0, pwms, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN);
#else
    mavlink_acflyhtl_pwmchans_fb_t packet;

    mav_array_memcpy(packet.pwms, pwms, sizeof(uint16_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN);
#endif
}

/**
 * @brief Pack a acflyhtl_pwmchans_fb message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pwms  Pwms in us.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acflyhtl_pwmchans_fb_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint16_t *pwms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN];

    _mav_put_uint16_t_array(buf, 0, pwms, 32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN);
#else
    mavlink_acflyhtl_pwmchans_fb_t packet;

    mav_array_memcpy(packet.pwms, pwms, sizeof(uint16_t)*32);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_CRC);
}

/**
 * @brief Encode a acflyhtl_pwmchans_fb struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param acflyhtl_pwmchans_fb C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acflyhtl_pwmchans_fb_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_acflyhtl_pwmchans_fb_t* acflyhtl_pwmchans_fb)
{
    return mavlink_msg_acflyhtl_pwmchans_fb_pack(system_id, component_id, msg, acflyhtl_pwmchans_fb->pwms);
}

/**
 * @brief Encode a acflyhtl_pwmchans_fb struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param acflyhtl_pwmchans_fb C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acflyhtl_pwmchans_fb_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_acflyhtl_pwmchans_fb_t* acflyhtl_pwmchans_fb)
{
    return mavlink_msg_acflyhtl_pwmchans_fb_pack_chan(system_id, component_id, chan, msg, acflyhtl_pwmchans_fb->pwms);
}

/**
 * @brief Encode a acflyhtl_pwmchans_fb struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param acflyhtl_pwmchans_fb C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acflyhtl_pwmchans_fb_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_acflyhtl_pwmchans_fb_t* acflyhtl_pwmchans_fb)
{
    return mavlink_msg_acflyhtl_pwmchans_fb_pack_status(system_id, component_id, _status, msg,  acflyhtl_pwmchans_fb->pwms);
}

/**
 * @brief Send a acflyhtl_pwmchans_fb message
 * @param chan MAVLink channel to send the message
 *
 * @param pwms  Pwms in us.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_acflyhtl_pwmchans_fb_send(mavlink_channel_t chan, const uint16_t *pwms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN];

    _mav_put_uint16_t_array(buf, 0, pwms, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB, buf, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_CRC);
#else
    mavlink_acflyhtl_pwmchans_fb_t packet;

    mav_array_memcpy(packet.pwms, pwms, sizeof(uint16_t)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB, (const char *)&packet, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_CRC);
#endif
}

/**
 * @brief Send a acflyhtl_pwmchans_fb message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_acflyhtl_pwmchans_fb_send_struct(mavlink_channel_t chan, const mavlink_acflyhtl_pwmchans_fb_t* acflyhtl_pwmchans_fb)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_acflyhtl_pwmchans_fb_send(chan, acflyhtl_pwmchans_fb->pwms);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB, (const char *)acflyhtl_pwmchans_fb, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_acflyhtl_pwmchans_fb_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint16_t *pwms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint16_t_array(buf, 0, pwms, 32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB, buf, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_CRC);
#else
    mavlink_acflyhtl_pwmchans_fb_t *packet = (mavlink_acflyhtl_pwmchans_fb_t *)msgbuf;

    mav_array_memcpy(packet->pwms, pwms, sizeof(uint16_t)*32);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB, (const char *)packet, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_MIN_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_CRC);
#endif
}
#endif

#endif

// MESSAGE ACFlyHTL_PWMCHANS_FB UNPACKING


/**
 * @brief Get field pwms from acflyhtl_pwmchans_fb message
 *
 * @return  Pwms in us.
 */
static inline uint16_t mavlink_msg_acflyhtl_pwmchans_fb_get_pwms(const mavlink_message_t* msg, uint16_t *pwms)
{
    return _MAV_RETURN_uint16_t_array(msg, pwms, 32,  0);
}

/**
 * @brief Decode a acflyhtl_pwmchans_fb message into a struct
 *
 * @param msg The message to decode
 * @param acflyhtl_pwmchans_fb C-struct to decode the message contents into
 */
static inline void mavlink_msg_acflyhtl_pwmchans_fb_decode(const mavlink_message_t* msg, mavlink_acflyhtl_pwmchans_fb_t* acflyhtl_pwmchans_fb)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_acflyhtl_pwmchans_fb_get_pwms(msg, acflyhtl_pwmchans_fb->pwms);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN? msg->len : MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN;
        memset(acflyhtl_pwmchans_fb, 0, MAVLINK_MSG_ID_ACFlyHTL_PWMCHANS_FB_LEN);
    memcpy(acflyhtl_pwmchans_fb, _MAV_PAYLOAD(msg), len);
#endif
}
