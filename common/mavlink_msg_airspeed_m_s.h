#pragma once
// MESSAGE AIRSPEED_M_S PACKING

#define MAVLINK_MSG_ID_AIRSPEED_M_S 229


typedef struct __mavlink_airspeed_m_s_t {
 uint64_t time_usec; /*< [us] Time since system start (microseconds).*/
 float indicated_airspeed_m_s; /*<  Indicated airspeed in m/s.*/
 float true_airspeed_m_s; /*<  True filtered airspeed in m/s.*/
 float air_temperature_celsius; /*<  Air temperature in degrees celsius, -1000 if unknown.*/
 float confidence; /*<  Confidence value from 0 to 1 for this sensor.*/
} mavlink_airspeed_m_s_t;

#define MAVLINK_MSG_ID_AIRSPEED_M_S_LEN 24
#define MAVLINK_MSG_ID_AIRSPEED_M_S_MIN_LEN 24
#define MAVLINK_MSG_ID_229_LEN 24
#define MAVLINK_MSG_ID_229_MIN_LEN 24

#define MAVLINK_MSG_ID_AIRSPEED_M_S_CRC 84
#define MAVLINK_MSG_ID_229_CRC 84



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_AIRSPEED_M_S { \
    229, \
    "AIRSPEED_M_S", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_airspeed_m_s_t, time_usec) }, \
         { "indicated_airspeed_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_airspeed_m_s_t, indicated_airspeed_m_s) }, \
         { "true_airspeed_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_airspeed_m_s_t, true_airspeed_m_s) }, \
         { "air_temperature_celsius", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_airspeed_m_s_t, air_temperature_celsius) }, \
         { "confidence", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_airspeed_m_s_t, confidence) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_AIRSPEED_M_S { \
    "AIRSPEED_M_S", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_airspeed_m_s_t, time_usec) }, \
         { "indicated_airspeed_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_airspeed_m_s_t, indicated_airspeed_m_s) }, \
         { "true_airspeed_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_airspeed_m_s_t, true_airspeed_m_s) }, \
         { "air_temperature_celsius", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_airspeed_m_s_t, air_temperature_celsius) }, \
         { "confidence", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_airspeed_m_s_t, confidence) }, \
         } \
}
#endif

/**
 * @brief Pack a airspeed_m_s message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Time since system start (microseconds).
 * @param indicated_airspeed_m_s  Indicated airspeed in m/s.
 * @param true_airspeed_m_s  True filtered airspeed in m/s.
 * @param air_temperature_celsius  Air temperature in degrees celsius, -1000 if unknown.
 * @param confidence  Confidence value from 0 to 1 for this sensor.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airspeed_m_s_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float indicated_airspeed_m_s, float true_airspeed_m_s, float air_temperature_celsius, float confidence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRSPEED_M_S_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, indicated_airspeed_m_s);
    _mav_put_float(buf, 12, true_airspeed_m_s);
    _mav_put_float(buf, 16, air_temperature_celsius);
    _mav_put_float(buf, 20, confidence);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRSPEED_M_S_LEN);
#else
    mavlink_airspeed_m_s_t packet;
    packet.time_usec = time_usec;
    packet.indicated_airspeed_m_s = indicated_airspeed_m_s;
    packet.true_airspeed_m_s = true_airspeed_m_s;
    packet.air_temperature_celsius = air_temperature_celsius;
    packet.confidence = confidence;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRSPEED_M_S_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRSPEED_M_S;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_AIRSPEED_M_S_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_M_S_LEN, MAVLINK_MSG_ID_AIRSPEED_M_S_CRC);
}

/**
 * @brief Pack a airspeed_m_s message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Time since system start (microseconds).
 * @param indicated_airspeed_m_s  Indicated airspeed in m/s.
 * @param true_airspeed_m_s  True filtered airspeed in m/s.
 * @param air_temperature_celsius  Air temperature in degrees celsius, -1000 if unknown.
 * @param confidence  Confidence value from 0 to 1 for this sensor.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_airspeed_m_s_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float indicated_airspeed_m_s,float true_airspeed_m_s,float air_temperature_celsius,float confidence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRSPEED_M_S_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, indicated_airspeed_m_s);
    _mav_put_float(buf, 12, true_airspeed_m_s);
    _mav_put_float(buf, 16, air_temperature_celsius);
    _mav_put_float(buf, 20, confidence);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_AIRSPEED_M_S_LEN);
#else
    mavlink_airspeed_m_s_t packet;
    packet.time_usec = time_usec;
    packet.indicated_airspeed_m_s = indicated_airspeed_m_s;
    packet.true_airspeed_m_s = true_airspeed_m_s;
    packet.air_temperature_celsius = air_temperature_celsius;
    packet.confidence = confidence;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_AIRSPEED_M_S_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_AIRSPEED_M_S;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_AIRSPEED_M_S_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_M_S_LEN, MAVLINK_MSG_ID_AIRSPEED_M_S_CRC);
}

/**
 * @brief Encode a airspeed_m_s struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param airspeed_m_s C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airspeed_m_s_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_airspeed_m_s_t* airspeed_m_s)
{
    return mavlink_msg_airspeed_m_s_pack(system_id, component_id, msg, airspeed_m_s->time_usec, airspeed_m_s->indicated_airspeed_m_s, airspeed_m_s->true_airspeed_m_s, airspeed_m_s->air_temperature_celsius, airspeed_m_s->confidence);
}

/**
 * @brief Encode a airspeed_m_s struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param airspeed_m_s C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_airspeed_m_s_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_airspeed_m_s_t* airspeed_m_s)
{
    return mavlink_msg_airspeed_m_s_pack_chan(system_id, component_id, chan, msg, airspeed_m_s->time_usec, airspeed_m_s->indicated_airspeed_m_s, airspeed_m_s->true_airspeed_m_s, airspeed_m_s->air_temperature_celsius, airspeed_m_s->confidence);
}

/**
 * @brief Send a airspeed_m_s message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Time since system start (microseconds).
 * @param indicated_airspeed_m_s  Indicated airspeed in m/s.
 * @param true_airspeed_m_s  True filtered airspeed in m/s.
 * @param air_temperature_celsius  Air temperature in degrees celsius, -1000 if unknown.
 * @param confidence  Confidence value from 0 to 1 for this sensor.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_airspeed_m_s_send(mavlink_channel_t chan, uint64_t time_usec, float indicated_airspeed_m_s, float true_airspeed_m_s, float air_temperature_celsius, float confidence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_AIRSPEED_M_S_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, indicated_airspeed_m_s);
    _mav_put_float(buf, 12, true_airspeed_m_s);
    _mav_put_float(buf, 16, air_temperature_celsius);
    _mav_put_float(buf, 20, confidence);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED_M_S, buf, MAVLINK_MSG_ID_AIRSPEED_M_S_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_M_S_LEN, MAVLINK_MSG_ID_AIRSPEED_M_S_CRC);
#else
    mavlink_airspeed_m_s_t packet;
    packet.time_usec = time_usec;
    packet.indicated_airspeed_m_s = indicated_airspeed_m_s;
    packet.true_airspeed_m_s = true_airspeed_m_s;
    packet.air_temperature_celsius = air_temperature_celsius;
    packet.confidence = confidence;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED_M_S, (const char *)&packet, MAVLINK_MSG_ID_AIRSPEED_M_S_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_M_S_LEN, MAVLINK_MSG_ID_AIRSPEED_M_S_CRC);
#endif
}

/**
 * @brief Send a airspeed_m_s message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_airspeed_m_s_send_struct(mavlink_channel_t chan, const mavlink_airspeed_m_s_t* airspeed_m_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_airspeed_m_s_send(chan, airspeed_m_s->time_usec, airspeed_m_s->indicated_airspeed_m_s, airspeed_m_s->true_airspeed_m_s, airspeed_m_s->air_temperature_celsius, airspeed_m_s->confidence);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED_M_S, (const char *)airspeed_m_s, MAVLINK_MSG_ID_AIRSPEED_M_S_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_M_S_LEN, MAVLINK_MSG_ID_AIRSPEED_M_S_CRC);
#endif
}

#if MAVLINK_MSG_ID_AIRSPEED_M_S_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_airspeed_m_s_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float indicated_airspeed_m_s, float true_airspeed_m_s, float air_temperature_celsius, float confidence)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, indicated_airspeed_m_s);
    _mav_put_float(buf, 12, true_airspeed_m_s);
    _mav_put_float(buf, 16, air_temperature_celsius);
    _mav_put_float(buf, 20, confidence);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED_M_S, buf, MAVLINK_MSG_ID_AIRSPEED_M_S_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_M_S_LEN, MAVLINK_MSG_ID_AIRSPEED_M_S_CRC);
#else
    mavlink_airspeed_m_s_t *packet = (mavlink_airspeed_m_s_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->indicated_airspeed_m_s = indicated_airspeed_m_s;
    packet->true_airspeed_m_s = true_airspeed_m_s;
    packet->air_temperature_celsius = air_temperature_celsius;
    packet->confidence = confidence;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIRSPEED_M_S, (const char *)packet, MAVLINK_MSG_ID_AIRSPEED_M_S_MIN_LEN, MAVLINK_MSG_ID_AIRSPEED_M_S_LEN, MAVLINK_MSG_ID_AIRSPEED_M_S_CRC);
#endif
}
#endif

#endif

// MESSAGE AIRSPEED_M_S UNPACKING


/**
 * @brief Get field time_usec from airspeed_m_s message
 *
 * @return [us] Time since system start (microseconds).
 */
static inline uint64_t mavlink_msg_airspeed_m_s_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field indicated_airspeed_m_s from airspeed_m_s message
 *
 * @return  Indicated airspeed in m/s.
 */
static inline float mavlink_msg_airspeed_m_s_get_indicated_airspeed_m_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field true_airspeed_m_s from airspeed_m_s message
 *
 * @return  True filtered airspeed in m/s.
 */
static inline float mavlink_msg_airspeed_m_s_get_true_airspeed_m_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field air_temperature_celsius from airspeed_m_s message
 *
 * @return  Air temperature in degrees celsius, -1000 if unknown.
 */
static inline float mavlink_msg_airspeed_m_s_get_air_temperature_celsius(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field confidence from airspeed_m_s message
 *
 * @return  Confidence value from 0 to 1 for this sensor.
 */
static inline float mavlink_msg_airspeed_m_s_get_confidence(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a airspeed_m_s message into a struct
 *
 * @param msg The message to decode
 * @param airspeed_m_s C-struct to decode the message contents into
 */
static inline void mavlink_msg_airspeed_m_s_decode(const mavlink_message_t* msg, mavlink_airspeed_m_s_t* airspeed_m_s)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    airspeed_m_s->time_usec = mavlink_msg_airspeed_m_s_get_time_usec(msg);
    airspeed_m_s->indicated_airspeed_m_s = mavlink_msg_airspeed_m_s_get_indicated_airspeed_m_s(msg);
    airspeed_m_s->true_airspeed_m_s = mavlink_msg_airspeed_m_s_get_true_airspeed_m_s(msg);
    airspeed_m_s->air_temperature_celsius = mavlink_msg_airspeed_m_s_get_air_temperature_celsius(msg);
    airspeed_m_s->confidence = mavlink_msg_airspeed_m_s_get_confidence(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_AIRSPEED_M_S_LEN? msg->len : MAVLINK_MSG_ID_AIRSPEED_M_S_LEN;
        memset(airspeed_m_s, 0, MAVLINK_MSG_ID_AIRSPEED_M_S_LEN);
    memcpy(airspeed_m_s, _MAV_PAYLOAD(msg), len);
#endif
}
