/**
 * mavlink_decode.h
 *
 * MAVLink v2 Byte-Stream Parser --- Inbound Frames from the Ground Station
 *
 * The parser is fed one byte at a time and holds all of its state in a caller-
 * owned struct. That shape is deliberate: Section 4.2 requires every UART
 * peripheral to be serviced by an interrupt-driven, byte-at-a-time receiver
 * rather than a blocking read, so this must be callable from an ISR context
 * with no allocation, no blocking, and bounded work per byte.
 *
 * Per byte the parser does at most one CRC accumulation and one array store,
 * so the cost of the whole GCS receive path is proportional to the link's
 * actual byte rate --- roughly 5760 bytes/s at 57600 baud --- and not to the
 * 100 Hz control loop.
 *
 * Both MAVLink v2 (0xFD) and legacy v1 (0xFE) frames are accepted. Mission
 * Planner opens a link speaking v1 until it observes a v2 frame from us, so
 * refusing v1 would stall the handshake.
 */

#ifndef MAVLINK_DECODE_H
#define MAVLINK_DECODE_H

#include "mavlink_types.h"

/* ============================================================================
 * Parser State Machine
 * ========================================================================= */

typedef enum {
    MAV_PARSE_IDLE = 0,     /* Hunting for STX */
    MAV_PARSE_LEN,
    MAV_PARSE_INCOMPAT,     /* v2 only */
    MAV_PARSE_COMPAT,       /* v2 only */
    MAV_PARSE_SEQ,
    MAV_PARSE_SYSID,
    MAV_PARSE_COMPID,
    MAV_PARSE_MSGID1,
    MAV_PARSE_MSGID2,       /* v2 only */
    MAV_PARSE_MSGID3,       /* v2 only */
    MAV_PARSE_PAYLOAD,
    MAV_PARSE_CRC1,
    MAV_PARSE_CRC2,
    MAV_PARSE_SIGNATURE     /* v2 signed frames: consume and discard 13 bytes */
} mav_parse_state_t;

/* ============================================================================
 * Link Statistics
 *
 * `packets_lost` is derived from gaps in the sender's sequence number and is
 * the number the GCS-side link quality indicator mirrors. A rising crc_errors
 * count with a healthy byte rate points at radio interference; a rising
 * unknown_msgid count points at a dialect mismatch.
 * ========================================================================= */

typedef struct {
    uint32_t packets_received;
    uint32_t packets_lost;      /* Inferred from sequence-number gaps */
    uint32_t crc_errors;
    uint32_t unknown_msgid;     /* Valid framing, message not in our dialect */
    uint32_t bytes_dropped;     /* Bytes discarded while resynchronising */
} mav_link_stats_t;

/* ============================================================================
 * Parser Instance
 * ========================================================================= */

typedef struct {
    mav_parse_state_t state;
    uint8_t  buffer[MAVLINK_MAX_PAYLOAD_LEN];
    uint8_t  payload_len;       /* LEN field as received */
    uint8_t  payload_index;
    uint8_t  incompat_flags;
    uint8_t  seq;
    uint8_t  sysid;
    uint8_t  compid;
    uint32_t msg_id;
    uint16_t crc;               /* Running CRC over the frame */
    uint16_t crc_received;
    uint8_t  crc_extra;         /* Looked up once the message ID is known */
    uint8_t  signature_index;
    bool     is_v2;
    bool     have_last_seq;
    uint8_t  last_seq;
    mav_link_stats_t stats;
} mav_parser_t;

/* ============================================================================
 * API
 * ========================================================================= */

/** Reset a parser to the idle state and zero its statistics. */
void mav_parser_init(mav_parser_t *p);

/**
 * Feed one received byte.
 *
 * Returns true exactly once per complete, CRC-valid, known message, with
 * `out_msg` filled in. `out_msg->payload` is zero-padded out to the message's
 * full declared length, so the caller may copy it directly into the matching
 * struct from mavlink_types.h without worrying about v2 truncation.
 *
 * Returns false for every other byte, including bytes belonging to a frame
 * that later fails its CRC.
 */
bool mav_parse_byte(mav_parser_t *p, uint8_t byte, mav_message_t *out_msg);

/**
 * Look up a message's CRC_EXTRA seed and full payload length.
 * Returns false if the message ID is outside the dialect this firmware
 * implements --- such frames cannot be CRC-checked and are discarded.
 */
bool mav_msg_info(uint32_t msg_id, uint8_t *out_crc_extra, uint8_t *out_len);

/** Longest payload in the dialect table. Must not exceed
 *  MAVLINK_MAX_DIALECT_PAYLOAD, which the parser uses to bound resynchronisation. */
uint8_t mav_dialect_max_payload(void);

/** Number of message definitions in the dialect table. */
uint16_t mav_dialect_size(void);

/* ============================================================================
 * Typed Payload Accessors
 *
 * Each returns false if the message ID does not match, so a caller may chain
 * them without a prior switch. The payload is already zero-padded, so a
 * truncated MISSION_ITEM_INT with a zero altitude decodes correctly.
 * ========================================================================= */

bool mav_msg_heartbeat(const mav_message_t *m, mav_heartbeat_t *out);
bool mav_msg_command_long(const mav_message_t *m, mav_command_long_t *out);
bool mav_msg_set_mode(const mav_message_t *m, mav_set_mode_t *out);
bool mav_msg_mission_count(const mav_message_t *m, mav_mission_count_t *out);
bool mav_msg_mission_item_int(const mav_message_t *m, mav_mission_item_int_t *out);
bool mav_msg_mission_request_int(const mav_message_t *m, mav_mission_request_int_t *out);
bool mav_msg_mission_request_list(const mav_message_t *m, mav_mission_request_list_t *out);
bool mav_msg_mission_clear_all(const mav_message_t *m, mav_mission_clear_all_t *out);
bool mav_msg_mission_ack(const mav_message_t *m, mav_mission_ack_t *out);
bool mav_msg_param_request_list(const mav_message_t *m, mav_param_request_list_t *out);
bool mav_msg_param_set(const mav_message_t *m, mav_param_set_t *out);
bool mav_msg_request_data_stream(const mav_message_t *m, mav_request_data_stream_t *out);

#endif /* MAVLINK_DECODE_H */
