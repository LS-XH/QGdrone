#pragma once
// MESSAGE MISSION_CURRENT PACKING

#define MAVLINK_MSG_ID_MISSION_CURRENT 42

MAVPACKED(
typedef struct __mavlink_mission_current_t {
 uint16_t seq; /*<  Sequence*/
 uint16_t resumeSeq; /*<  resume waypoint Sequence*/
 uint16_t flag; /*<  flag*/
 float heading; /*< [deg] Heading angle(0-360),north is 0 deq*/
 float vector_x; /*< [m] waypoint to resume point.*/
 float vector_y; /*< [m] waypoint to resume point.*/
 float vector_z; /*< [m] waypoint to resume point.*/
 float distance; /*< [m] waypoint to resume point distance.*/
 float trigger_interval; /*< [m] trigger interval.*/
 float speed; /*< [m/s] speed.*/
 float radius; /*< [m] radius.*/
}) mavlink_mission_current_t;

#define MAVLINK_MSG_ID_MISSION_CURRENT_LEN 38
#define MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN 2
#define MAVLINK_MSG_ID_42_LEN 38
#define MAVLINK_MSG_ID_42_MIN_LEN 2

#define MAVLINK_MSG_ID_MISSION_CURRENT_CRC 28
#define MAVLINK_MSG_ID_42_CRC 28



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MISSION_CURRENT { \
    42, \
    "MISSION_CURRENT", \
    11, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mission_current_t, seq) }, \
         { "resumeSeq", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_mission_current_t, resumeSeq) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_mission_current_t, flag) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 6, offsetof(mavlink_mission_current_t, heading) }, \
         { "vector_x", NULL, MAVLINK_TYPE_FLOAT, 0, 10, offsetof(mavlink_mission_current_t, vector_x) }, \
         { "vector_y", NULL, MAVLINK_TYPE_FLOAT, 0, 14, offsetof(mavlink_mission_current_t, vector_y) }, \
         { "vector_z", NULL, MAVLINK_TYPE_FLOAT, 0, 18, offsetof(mavlink_mission_current_t, vector_z) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 22, offsetof(mavlink_mission_current_t, distance) }, \
         { "trigger_interval", NULL, MAVLINK_TYPE_FLOAT, 0, 26, offsetof(mavlink_mission_current_t, trigger_interval) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 30, offsetof(mavlink_mission_current_t, speed) }, \
         { "radius", NULL, MAVLINK_TYPE_FLOAT, 0, 34, offsetof(mavlink_mission_current_t, radius) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MISSION_CURRENT { \
    "MISSION_CURRENT", \
    11, \
    {  { "seq", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_mission_current_t, seq) }, \
         { "resumeSeq", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_mission_current_t, resumeSeq) }, \
         { "flag", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_mission_current_t, flag) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 6, offsetof(mavlink_mission_current_t, heading) }, \
         { "vector_x", NULL, MAVLINK_TYPE_FLOAT, 0, 10, offsetof(mavlink_mission_current_t, vector_x) }, \
         { "vector_y", NULL, MAVLINK_TYPE_FLOAT, 0, 14, offsetof(mavlink_mission_current_t, vector_y) }, \
         { "vector_z", NULL, MAVLINK_TYPE_FLOAT, 0, 18, offsetof(mavlink_mission_current_t, vector_z) }, \
         { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 22, offsetof(mavlink_mission_current_t, distance) }, \
         { "trigger_interval", NULL, MAVLINK_TYPE_FLOAT, 0, 26, offsetof(mavlink_mission_current_t, trigger_interval) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 30, offsetof(mavlink_mission_current_t, speed) }, \
         { "radius", NULL, MAVLINK_TYPE_FLOAT, 0, 34, offsetof(mavlink_mission_current_t, radius) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_current message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param seq  Sequence
 * @param resumeSeq  resume waypoint Sequence
 * @param flag  flag
 * @param heading [deg] Heading angle(0-360),north is 0 deq
 * @param vector_x [m] waypoint to resume point.
 * @param vector_y [m] waypoint to resume point.
 * @param vector_z [m] waypoint to resume point.
 * @param distance [m] waypoint to resume point distance.
 * @param trigger_interval [m] trigger interval.
 * @param speed [m/s] speed.
 * @param radius [m] radius.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_current_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t seq, uint16_t resumeSeq, uint16_t flag, float heading, float vector_x, float vector_y, float vector_z, float distance, float trigger_interval, float speed, float radius)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_CURRENT_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint16_t(buf, 2, resumeSeq);
    _mav_put_uint16_t(buf, 4, flag);
    _mav_put_float(buf, 6, heading);
    _mav_put_float(buf, 10, vector_x);
    _mav_put_float(buf, 14, vector_y);
    _mav_put_float(buf, 18, vector_z);
    _mav_put_float(buf, 22, distance);
    _mav_put_float(buf, 26, trigger_interval);
    _mav_put_float(buf, 30, speed);
    _mav_put_float(buf, 34, radius);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#else
    mavlink_mission_current_t packet;
    packet.seq = seq;
    packet.resumeSeq = resumeSeq;
    packet.flag = flag;
    packet.heading = heading;
    packet.vector_x = vector_x;
    packet.vector_y = vector_y;
    packet.vector_z = vector_z;
    packet.distance = distance;
    packet.trigger_interval = trigger_interval;
    packet.speed = speed;
    packet.radius = radius;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_CURRENT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
}

/**
 * @brief Pack a mission_current message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param seq  Sequence
 * @param resumeSeq  resume waypoint Sequence
 * @param flag  flag
 * @param heading [deg] Heading angle(0-360),north is 0 deq
 * @param vector_x [m] waypoint to resume point.
 * @param vector_y [m] waypoint to resume point.
 * @param vector_z [m] waypoint to resume point.
 * @param distance [m] waypoint to resume point distance.
 * @param trigger_interval [m] trigger interval.
 * @param speed [m/s] speed.
 * @param radius [m] radius.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_current_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t seq,uint16_t resumeSeq,uint16_t flag,float heading,float vector_x,float vector_y,float vector_z,float distance,float trigger_interval,float speed,float radius)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_CURRENT_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint16_t(buf, 2, resumeSeq);
    _mav_put_uint16_t(buf, 4, flag);
    _mav_put_float(buf, 6, heading);
    _mav_put_float(buf, 10, vector_x);
    _mav_put_float(buf, 14, vector_y);
    _mav_put_float(buf, 18, vector_z);
    _mav_put_float(buf, 22, distance);
    _mav_put_float(buf, 26, trigger_interval);
    _mav_put_float(buf, 30, speed);
    _mav_put_float(buf, 34, radius);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#else
    mavlink_mission_current_t packet;
    packet.seq = seq;
    packet.resumeSeq = resumeSeq;
    packet.flag = flag;
    packet.heading = heading;
    packet.vector_x = vector_x;
    packet.vector_y = vector_y;
    packet.vector_z = vector_z;
    packet.distance = distance;
    packet.trigger_interval = trigger_interval;
    packet.speed = speed;
    packet.radius = radius;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MISSION_CURRENT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
}

/**
 * @brief Encode a mission_current struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_current C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_current_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_current_t* mission_current)
{
    return mavlink_msg_mission_current_pack(system_id, component_id, msg, mission_current->seq, mission_current->resumeSeq, mission_current->flag, mission_current->heading, mission_current->vector_x, mission_current->vector_y, mission_current->vector_z, mission_current->distance, mission_current->trigger_interval, mission_current->speed, mission_current->radius);
}

/**
 * @brief Encode a mission_current struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_current C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_current_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_current_t* mission_current)
{
    return mavlink_msg_mission_current_pack_chan(system_id, component_id, chan, msg, mission_current->seq, mission_current->resumeSeq, mission_current->flag, mission_current->heading, mission_current->vector_x, mission_current->vector_y, mission_current->vector_z, mission_current->distance, mission_current->trigger_interval, mission_current->speed, mission_current->radius);
}

/**
 * @brief Send a mission_current message
 * @param chan MAVLink channel to send the message
 *
 * @param seq  Sequence
 * @param resumeSeq  resume waypoint Sequence
 * @param flag  flag
 * @param heading [deg] Heading angle(0-360),north is 0 deq
 * @param vector_x [m] waypoint to resume point.
 * @param vector_y [m] waypoint to resume point.
 * @param vector_z [m] waypoint to resume point.
 * @param distance [m] waypoint to resume point distance.
 * @param trigger_interval [m] trigger interval.
 * @param speed [m/s] speed.
 * @param radius [m] radius.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_current_send(mavlink_channel_t chan, uint16_t seq, uint16_t resumeSeq, uint16_t flag, float heading, float vector_x, float vector_y, float vector_z, float distance, float trigger_interval, float speed, float radius)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MISSION_CURRENT_LEN];
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint16_t(buf, 2, resumeSeq);
    _mav_put_uint16_t(buf, 4, flag);
    _mav_put_float(buf, 6, heading);
    _mav_put_float(buf, 10, vector_x);
    _mav_put_float(buf, 14, vector_y);
    _mav_put_float(buf, 18, vector_z);
    _mav_put_float(buf, 22, distance);
    _mav_put_float(buf, 26, trigger_interval);
    _mav_put_float(buf, 30, speed);
    _mav_put_float(buf, 34, radius);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, buf, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#else
    mavlink_mission_current_t packet;
    packet.seq = seq;
    packet.resumeSeq = resumeSeq;
    packet.flag = flag;
    packet.heading = heading;
    packet.vector_x = vector_x;
    packet.vector_y = vector_y;
    packet.vector_z = vector_z;
    packet.distance = distance;
    packet.trigger_interval = trigger_interval;
    packet.speed = speed;
    packet.radius = radius;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, (const char *)&packet, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#endif
}

/**
 * @brief Send a mission_current message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mission_current_send_struct(mavlink_channel_t chan, const mavlink_mission_current_t* mission_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mission_current_send(chan, mission_current->seq, mission_current->resumeSeq, mission_current->flag, mission_current->heading, mission_current->vector_x, mission_current->vector_y, mission_current->vector_z, mission_current->distance, mission_current->trigger_interval, mission_current->speed, mission_current->radius);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, (const char *)mission_current, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#endif
}

#if MAVLINK_MSG_ID_MISSION_CURRENT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_current_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t seq, uint16_t resumeSeq, uint16_t flag, float heading, float vector_x, float vector_y, float vector_z, float distance, float trigger_interval, float speed, float radius)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, seq);
    _mav_put_uint16_t(buf, 2, resumeSeq);
    _mav_put_uint16_t(buf, 4, flag);
    _mav_put_float(buf, 6, heading);
    _mav_put_float(buf, 10, vector_x);
    _mav_put_float(buf, 14, vector_y);
    _mav_put_float(buf, 18, vector_z);
    _mav_put_float(buf, 22, distance);
    _mav_put_float(buf, 26, trigger_interval);
    _mav_put_float(buf, 30, speed);
    _mav_put_float(buf, 34, radius);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, buf, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#else
    mavlink_mission_current_t *packet = (mavlink_mission_current_t *)msgbuf;
    packet->seq = seq;
    packet->resumeSeq = resumeSeq;
    packet->flag = flag;
    packet->heading = heading;
    packet->vector_x = vector_x;
    packet->vector_y = vector_y;
    packet->vector_z = vector_z;
    packet->distance = distance;
    packet->trigger_interval = trigger_interval;
    packet->speed = speed;
    packet->radius = radius;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_CURRENT, (const char *)packet, MAVLINK_MSG_ID_MISSION_CURRENT_MIN_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_LEN, MAVLINK_MSG_ID_MISSION_CURRENT_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_CURRENT UNPACKING


/**
 * @brief Get field seq from mission_current message
 *
 * @return  Sequence
 */
static inline uint16_t mavlink_msg_mission_current_get_seq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field resumeSeq from mission_current message
 *
 * @return  resume waypoint Sequence
 */
static inline uint16_t mavlink_msg_mission_current_get_resumeSeq(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field flag from mission_current message
 *
 * @return  flag
 */
static inline uint16_t mavlink_msg_mission_current_get_flag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field heading from mission_current message
 *
 * @return [deg] Heading angle(0-360),north is 0 deq
 */
static inline float mavlink_msg_mission_current_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  6);
}

/**
 * @brief Get field vector_x from mission_current message
 *
 * @return [m] waypoint to resume point.
 */
static inline float mavlink_msg_mission_current_get_vector_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  10);
}

/**
 * @brief Get field vector_y from mission_current message
 *
 * @return [m] waypoint to resume point.
 */
static inline float mavlink_msg_mission_current_get_vector_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  14);
}

/**
 * @brief Get field vector_z from mission_current message
 *
 * @return [m] waypoint to resume point.
 */
static inline float mavlink_msg_mission_current_get_vector_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  18);
}

/**
 * @brief Get field distance from mission_current message
 *
 * @return [m] waypoint to resume point distance.
 */
static inline float mavlink_msg_mission_current_get_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  22);
}

/**
 * @brief Get field trigger_interval from mission_current message
 *
 * @return [m] trigger interval.
 */
static inline float mavlink_msg_mission_current_get_trigger_interval(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  26);
}

/**
 * @brief Get field speed from mission_current message
 *
 * @return [m/s] speed.
 */
static inline float mavlink_msg_mission_current_get_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  30);
}

/**
 * @brief Get field radius from mission_current message
 *
 * @return [m] radius.
 */
static inline float mavlink_msg_mission_current_get_radius(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  34);
}

/**
 * @brief Decode a mission_current message into a struct
 *
 * @param msg The message to decode
 * @param mission_current C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_current_decode(const mavlink_message_t* msg, mavlink_mission_current_t* mission_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mission_current->seq = mavlink_msg_mission_current_get_seq(msg);
    mission_current->resumeSeq = mavlink_msg_mission_current_get_resumeSeq(msg);
    mission_current->flag = mavlink_msg_mission_current_get_flag(msg);
    mission_current->heading = mavlink_msg_mission_current_get_heading(msg);
    mission_current->vector_x = mavlink_msg_mission_current_get_vector_x(msg);
    mission_current->vector_y = mavlink_msg_mission_current_get_vector_y(msg);
    mission_current->vector_z = mavlink_msg_mission_current_get_vector_z(msg);
    mission_current->distance = mavlink_msg_mission_current_get_distance(msg);
    mission_current->trigger_interval = mavlink_msg_mission_current_get_trigger_interval(msg);
    mission_current->speed = mavlink_msg_mission_current_get_speed(msg);
    mission_current->radius = mavlink_msg_mission_current_get_radius(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MISSION_CURRENT_LEN? msg->len : MAVLINK_MSG_ID_MISSION_CURRENT_LEN;
        memset(mission_current, 0, MAVLINK_MSG_ID_MISSION_CURRENT_LEN);
    memcpy(mission_current, _MAV_PAYLOAD(msg), len);
#endif
}
