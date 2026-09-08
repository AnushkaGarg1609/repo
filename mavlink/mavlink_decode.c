/**
 * mavlink_decode.c
 *
 * MAVLink v2 (and legacy v1) byte-stream parser --- implementation.
 *
 * Design notes:
 *
 *   * Bounded work per byte. No loops proportional to frame length except the
 *     final zero-pad, which is a single pass over at most 255 bytes and runs
 *     once per accepted frame, not once per byte.
 *
 *   * Unknown message IDs are consumed and counted, never mis-parsed. Without
 *     a CRC_EXTRA seed there is no way to validate such a frame, so accepting
 *     it would mean trusting an unverified length field.
 *
 *   * A failed CRC returns the parser to IDLE. The next STX in the stream
 *     restarts framing, which is how the parser recovers from a garbled radio
 *     burst without any explicit resynchronisation logic.
 */

#include "mavlink_decode.h"

/* ============================================================================
 * Dialect table --- message ID -> CRC_EXTRA seed and full payload length
 *
 * This is the complete set of messages this firmware understands. Adding a
 * message means adding one row here plus a struct in mavlink_types.h; nothing
 * else in the parser changes.
 * ========================================================================= */

typedef struct {
    uint16_t msg_id;
    uint8_t  crc_extra;
    uint8_t  len;
} mav_msg_info_t;

static const mav_msg_info_t mav_dialect[] = {
    { MAVLINK_MSG_ID_HEARTBEAT,           MAVLINK_CRC_EXTRA_HEARTBEAT,           MAVLINK_MSG_LEN_HEARTBEAT           },
    { MAVLINK_MSG_ID_SYS_STATUS,          MAVLINK_CRC_EXTRA_SYS_STATUS,          MAVLINK_MSG_LEN_SYS_STATUS          },
    { MAVLINK_MSG_ID_SET_MODE,            MAVLINK_CRC_EXTRA_SET_MODE,            MAVLINK_MSG_LEN_SET_MODE            },
    { MAVLINK_MSG_ID_PARAM_REQUEST_LIST,  MAVLINK_CRC_EXTRA_PARAM_REQUEST_LIST,  MAVLINK_MSG_LEN_PARAM_REQUEST_LIST  },
    { MAVLINK_MSG_ID_PARAM_VALUE,         MAVLINK_CRC_EXTRA_PARAM_VALUE,         MAVLINK_MSG_LEN_PARAM_VALUE         },
    { MAVLINK_MSG_ID_PARAM_SET,           MAVLINK_CRC_EXTRA_PARAM_SET,           MAVLINK_MSG_LEN_PARAM_SET           },
    { MAVLINK_MSG_ID_GPS_RAW_INT,         MAVLINK_CRC_EXTRA_GPS_RAW_INT,         MAVLINK_MSG_LEN_GPS_RAW_INT         },
    { MAVLINK_MSG_ID_ATTITUDE,            MAVLINK_CRC_EXTRA_ATTITUDE,            MAVLINK_MSG_LEN_ATTITUDE            },
    { MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MAVLINK_CRC_EXTRA_GLOBAL_POSITION_INT, MAVLINK_MSG_LEN_GLOBAL_POSITION_INT },
    { MAVLINK_MSG_ID_MISSION_CURRENT,     MAVLINK_CRC_EXTRA_MISSION_CURRENT,     MAVLINK_MSG_LEN_MISSION_CURRENT     },
    { MAVLINK_MSG_ID_MISSION_REQUEST_LIST,MAVLINK_CRC_EXTRA_MISSION_REQUEST_LIST,MAVLINK_MSG_LEN_MISSION_REQUEST_LIST},
    { MAVLINK_MSG_ID_MISSION_COUNT,       MAVLINK_CRC_EXTRA_MISSION_COUNT,       MAVLINK_MSG_LEN_MISSION_COUNT       },
    { MAVLINK_MSG_ID_MISSION_CLEAR_ALL,   MAVLINK_CRC_EXTRA_MISSION_CLEAR_ALL,   MAVLINK_MSG_LEN_MISSION_CLEAR_ALL   },
    { MAVLINK_MSG_ID_MISSION_ITEM_REACHED,MAVLINK_CRC_EXTRA_MISSION_ITEM_REACHED,MAVLINK_MSG_LEN_MISSION_ITEM_REACHED},
    { MAVLINK_MSG_ID_MISSION_ACK,         MAVLINK_CRC_EXTRA_MISSION_ACK,         MAVLINK_MSG_LEN_MISSION_ACK         },
    { MAVLINK_MSG_ID_MISSION_REQUEST_INT, MAVLINK_CRC_EXTRA_MISSION_REQUEST_INT, MAVLINK_MSG_LEN_MISSION_REQUEST_INT },
    { MAVLINK_MSG_ID_REQUEST_DATA_STREAM, MAVLINK_CRC_EXTRA_REQUEST_DATA_STREAM, MAVLINK_MSG_LEN_REQUEST_DATA_STREAM },
    { MAVLINK_MSG_ID_MISSION_ITEM_INT,    MAVLINK_CRC_EXTRA_MISSION_ITEM_INT,    MAVLINK_MSG_LEN_MISSION_ITEM_INT    },
    { MAVLINK_MSG_ID_VFR_HUD,             MAVLINK_CRC_EXTRA_VFR_HUD,             MAVLINK_MSG_LEN_VFR_HUD             },
    { MAVLINK_MSG_ID_COMMAND_LONG,        MAVLINK_CRC_EXTRA_COMMAND_LONG,        MAVLINK_MSG_LEN_COMMAND_LONG        },
    { MAVLINK_MSG_ID_COMMAND_ACK,         MAVLINK_CRC_EXTRA_COMMAND_ACK,         MAVLINK_MSG_LEN_COMMAND_ACK         },
    { MAVLINK_MSG_ID_STATUSTEXT,          MAVLINK_CRC_EXTRA_STATUSTEXT,          MAVLINK_MSG_LEN_STATUSTEXT          }
};

#define MAV_DIALECT_SIZE (sizeof(mav_dialect) / sizeof(mav_dialect[0]))

uint8_t mav_dialect_max_payload(void) {
    uint8_t  max = 0;
    size_t   i;
    for (i = 0; i < MAV_DIALECT_SIZE; i++) {
        if (mav_dialect[i].len > max) max = mav_dialect[i].len;
    }
    return max;
}

uint16_t mav_dialect_size(void) {
    return (uint16_t)MAV_DIALECT_SIZE;
}

bool mav_msg_info(uint32_t msg_id, uint8_t *out_crc_extra, uint8_t *out_len) {
    size_t i;
    if (msg_id > 0xFFFFu) return false;
    for (i = 0; i < MAV_DIALECT_SIZE; i++) {
        if (mav_dialect[i].msg_id == (uint16_t)msg_id) {
            if (out_crc_extra) *out_crc_extra = mav_dialect[i].crc_extra;
            if (out_len)       *out_len       = mav_dialect[i].len;
            return true;
        }
    }
    return false;
}

/* ============================================================================
 * Local primitives
 * ========================================================================= */

static void dec_memcpy(void *dst, const void *src, uint8_t n) {
    uint8_t       *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;
    uint8_t        i;
    for (i = 0; i < n; i++) d[i] = s[i];
}

/** Return the parser to IDLE without disturbing statistics. */
static void parser_reset(mav_parser_t *p) {
    p->state           = MAV_PARSE_IDLE;
    p->payload_index   = 0;
    p->signature_index = 0;
}

/** Track sequence-number gaps to estimate link loss, as the GCS does. */
static void account_sequence(mav_parser_t *p, uint8_t seq) {
    if (p->have_last_seq) {
        uint8_t expected = (uint8_t)(p->last_seq + 1u);
        if (seq != expected) {
            p->stats.packets_lost += (uint32_t)(uint8_t)(seq - expected);
        }
    } else {
        p->have_last_seq = true;
    }
    p->last_seq = seq;
}

/* ============================================================================
 * Parser lifecycle
 * ========================================================================= */

void mav_parser_init(mav_parser_t *p) {
    uint16_t i;

    p->state           = MAV_PARSE_IDLE;
    p->payload_len     = 0;
    p->payload_index   = 0;
    p->incompat_flags  = 0;
    p->seq             = 0;
    p->sysid           = 0;
    p->compid          = 0;
    p->msg_id          = 0;
    p->crc             = X25_INIT_CRC;
    p->crc_received    = 0;
    p->crc_extra       = 0;
    p->signature_index = 0;
    p->is_v2           = true;
    p->have_last_seq   = false;
    p->last_seq        = 0;

    p->stats.packets_received = 0;
    p->stats.packets_lost     = 0;
    p->stats.crc_errors       = 0;
    p->stats.unknown_msgid    = 0;
    p->stats.bytes_dropped    = 0;

    for (i = 0; i < MAVLINK_MAX_PAYLOAD_LEN; i++) p->buffer[i] = 0;
}

/* ============================================================================
 * The state machine
 * ========================================================================= */

bool mav_parse_byte(mav_parser_t *p, uint8_t byte, mav_message_t *out_msg) {
    switch (p->state) {

    case MAV_PARSE_IDLE:
        if (byte == MAVLINK_V2_STX || byte == MAVLINK_V1_STX) {
            p->is_v2          = (byte == MAVLINK_V2_STX);
            p->crc            = X25_INIT_CRC;   /* STX itself is not covered */
            p->payload_index  = 0;
            p->incompat_flags = 0;
            p->msg_id         = 0;
            p->state          = MAV_PARSE_LEN;
        } else {
            p->stats.bytes_dropped++;
        }
        return false;

    case MAV_PARSE_LEN:
        /* Reject an impossible length immediately rather than consuming up to
         * 255 bytes of stream on the strength of one noise byte that happened
         * to look like a start marker. See MAVLINK_MAX_DIALECT_PAYLOAD. */
        if (byte > MAVLINK_MAX_DIALECT_PAYLOAD) {
            p->stats.bytes_dropped++;
            /* The rejected byte may itself be the real start marker. */
            if (byte == MAVLINK_V2_STX || byte == MAVLINK_V1_STX) {
                p->is_v2         = (byte == MAVLINK_V2_STX);
                p->crc           = X25_INIT_CRC;
                p->payload_index = 0;
                p->incompat_flags= 0;
                p->msg_id        = 0;
                p->state         = MAV_PARSE_LEN;
            } else {
                parser_reset(p);
            }
            return false;
        }
        p->payload_len = byte;
        p->crc         = crc16_accumulate(p->crc, byte);
        p->state       = p->is_v2 ? MAV_PARSE_INCOMPAT : MAV_PARSE_SEQ;
        return false;

    case MAV_PARSE_INCOMPAT:
        p->incompat_flags = byte;
        p->crc            = crc16_accumulate(p->crc, byte);
        p->state          = MAV_PARSE_COMPAT;
        return false;

    case MAV_PARSE_COMPAT:
        p->crc   = crc16_accumulate(p->crc, byte);
        p->state = MAV_PARSE_SEQ;
        return false;

    case MAV_PARSE_SEQ:
        p->seq   = byte;
        p->crc   = crc16_accumulate(p->crc, byte);
        p->state = MAV_PARSE_SYSID;
        return false;

    case MAV_PARSE_SYSID:
        p->sysid = byte;
        p->crc   = crc16_accumulate(p->crc, byte);
        p->state = MAV_PARSE_COMPID;
        return false;

    case MAV_PARSE_COMPID:
        p->compid = byte;
        p->crc    = crc16_accumulate(p->crc, byte);
        p->state  = MAV_PARSE_MSGID1;
        return false;

    case MAV_PARSE_MSGID1:
        p->msg_id = (uint32_t)byte;
        p->crc    = crc16_accumulate(p->crc, byte);
        if (p->is_v2) {
            p->state = MAV_PARSE_MSGID2;
        } else {
            /* v1 has a single-byte message ID; the payload starts now. */
            p->state = (p->payload_len == 0) ? MAV_PARSE_CRC1 : MAV_PARSE_PAYLOAD;
        }
        return false;

    case MAV_PARSE_MSGID2:
        p->msg_id |= ((uint32_t)byte << 8);
        p->crc     = crc16_accumulate(p->crc, byte);
        p->state   = MAV_PARSE_MSGID3;
        return false;

    case MAV_PARSE_MSGID3:
        p->msg_id |= ((uint32_t)byte << 16);
        p->crc     = crc16_accumulate(p->crc, byte);
        p->state   = (p->payload_len == 0) ? MAV_PARSE_CRC1 : MAV_PARSE_PAYLOAD;
        return false;

    case MAV_PARSE_PAYLOAD:
        /* payload_len can be up to 255, which is exactly the buffer size, so
         * no bounds check can fail here --- but keep one anyway: payload_len
         * comes off the wire and this is the only place it indexes memory. */
        if (p->payload_index < MAVLINK_MAX_PAYLOAD_LEN) {
            p->buffer[p->payload_index] = byte;
        }
        p->payload_index++;
        p->crc = crc16_accumulate(p->crc, byte);
        if (p->payload_index >= p->payload_len) {
            p->state = MAV_PARSE_CRC1;
        }
        return false;

    case MAV_PARSE_CRC1:
        p->crc_received = (uint16_t)byte;
        p->state        = MAV_PARSE_CRC2;
        return false;

    case MAV_PARSE_CRC2: {
        uint8_t full_len = 0;
        uint8_t i;

        p->crc_received |= ((uint16_t)byte << 8);

        /* An unknown message cannot be CRC-checked: no seed exists for it. */
        if (!mav_msg_info(p->msg_id, &p->crc_extra, &full_len)) {
            p->stats.unknown_msgid++;
            parser_reset(p);
            return false;
        }

        /* Fold in CRC_EXTRA and compare. */
        if (crc16_accumulate(p->crc, p->crc_extra) != p->crc_received) {
            p->stats.crc_errors++;
            parser_reset(p);
            return false;
        }

        /* Frame is good. Clear the whole output payload first, then copy only
         * the bytes actually received. That gives the v2 zero-padding for free
         * (senders strip trailing zeros, and Mission Planner strips
         * aggressively) and also guarantees no stale bytes from a previous,
         * longer frame can be read back through a typed accessor.
         * full_len is looked up above purely to validate the dialect; the
         * padding itself needs no length arithmetic. */
        (void)full_len;
        for (i = 0; i < MAVLINK_MAX_PAYLOAD_LEN; i++) {
            out_msg->payload[i] = 0;
        }
        dec_memcpy(out_msg->payload, p->buffer, p->payload_len);

        out_msg->msg_id = p->msg_id;
        out_msg->len    = p->payload_len;
        out_msg->seq    = p->seq;
        out_msg->sysid  = p->sysid;
        out_msg->compid = p->compid;
        out_msg->valid  = true;

        p->stats.packets_received++;
        account_sequence(p, p->seq);

        /* A signed v2 frame carries 13 trailing bytes. We do not verify
         * signatures (Section 14 lists link authentication as future work),
         * but they must be consumed or the next frame will not be found. */
        if (p->is_v2 && (p->incompat_flags & MAVLINK_IFLAG_SIGNED)) {
            p->signature_index = 0;
            p->state           = MAV_PARSE_SIGNATURE;
        } else {
            parser_reset(p);
        }
        return true;
    }

    case MAV_PARSE_SIGNATURE:
        p->signature_index++;
        if (p->signature_index >= MAVLINK_SIGNATURE_LEN) {
            parser_reset(p);
        }
        return false;

    default:
        parser_reset(p);
        return false;
    }
}

/* ============================================================================
 * Typed accessors
 * ========================================================================= */

static bool msg_extract(const mav_message_t *m, uint32_t id, void *out, uint8_t n) {
    if (!m->valid || m->msg_id != id) return false;
    dec_memcpy(out, m->payload, n);
    return true;
}

bool mav_msg_heartbeat(const mav_message_t *m, mav_heartbeat_t *out) {
    return msg_extract(m, MAVLINK_MSG_ID_HEARTBEAT, out, MAVLINK_MSG_LEN_HEARTBEAT);
}

bool mav_msg_command_long(const mav_message_t *m, mav_command_long_t *out) {
    return msg_extract(m, MAVLINK_MSG_ID_COMMAND_LONG, out, MAVLINK_MSG_LEN_COMMAND_LONG);
}

bool mav_msg_set_mode(const mav_message_t *m, mav_set_mode_t *out) {
    return msg_extract(m, MAVLINK_MSG_ID_SET_MODE, out, MAVLINK_MSG_LEN_SET_MODE);
}

bool mav_msg_mission_count(const mav_message_t *m, mav_mission_count_t *out) {
    return msg_extract(m, MAVLINK_MSG_ID_MISSION_COUNT, out, MAVLINK_MSG_LEN_MISSION_COUNT);
}

bool mav_msg_mission_item_int(const mav_message_t *m, mav_mission_item_int_t *out) {
    return msg_extract(m, MAVLINK_MSG_ID_MISSION_ITEM_INT, out, MAVLINK_MSG_LEN_MISSION_ITEM_INT);
}

bool mav_msg_mission_request_int(const mav_message_t *m, mav_mission_request_int_t *out) {
    return msg_extract(m, MAVLINK_MSG_ID_MISSION_REQUEST_INT, out, MAVLINK_MSG_LEN_MISSION_REQUEST_INT);
}

bool mav_msg_mission_request_list(const mav_message_t *m, mav_mission_request_list_t *out) {
    return msg_extract(m, MAVLINK_MSG_ID_MISSION_REQUEST_LIST, out, MAVLINK_MSG_LEN_MISSION_REQUEST_LIST);
}

bool mav_msg_mission_clear_all(const mav_message_t *m, mav_mission_clear_all_t *out) {
    return msg_extract(m, MAVLINK_MSG_ID_MISSION_CLEAR_ALL, out, MAVLINK_MSG_LEN_MISSION_CLEAR_ALL);
}

bool mav_msg_mission_ack(const mav_message_t *m, mav_mission_ack_t *out) {
    return msg_extract(m, MAVLINK_MSG_ID_MISSION_ACK, out, MAVLINK_MSG_LEN_MISSION_ACK);
}

bool mav_msg_param_request_list(const mav_message_t *m, mav_param_request_list_t *out) {
    return msg_extract(m, MAVLINK_MSG_ID_PARAM_REQUEST_LIST, out, MAVLINK_MSG_LEN_PARAM_REQUEST_LIST);
}

bool mav_msg_param_set(const mav_message_t *m, mav_param_set_t *out) {
    return msg_extract(m, MAVLINK_MSG_ID_PARAM_SET, out, MAVLINK_MSG_LEN_PARAM_SET);
}

bool mav_msg_request_data_stream(const mav_message_t *m, mav_request_data_stream_t *out) {
    return msg_extract(m, MAVLINK_MSG_ID_REQUEST_DATA_STREAM, out, MAVLINK_MSG_LEN_REQUEST_DATA_STREAM);
}
