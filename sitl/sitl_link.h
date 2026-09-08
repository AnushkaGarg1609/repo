/**
 * sitl_link.h
 *
 * UDP transport and timing for the software-in-the-loop harness.
 *
 * This is the ONLY file in the project that talks to an operating system.
 * It stands in for exactly one thing on the real board --- the Taurus SDK UART
 * driver that will carry MAVLink over a SiK telemetry radio:
 *
 *     PC (this file)                     Aries V3 (Phase A.9 onward)
 *     ------------------------------     ------------------------------
 *     sitl_link_recv()               ->  taurus_uart_read_byte(UART_GCS)
 *     sitl_link_send()               ->  taurus_uart_write(UART_GCS, ...)
 *     sitl_millis()                  ->  the superloop tick counter (Sec 3.4)
 *
 * Nothing above this file knows which of the two it is running on. That is the
 * point of the exercise: the mavlink/ and core/ code that Mission Planner talks
 * to here is bit-for-bit the code that will run on the board.
 *
 * Wire behaviour matches ArduPilot's SITL so the ground station needs no
 * special configuration: datagrams are sent to 127.0.0.1:14550, and once the
 * ground station replies, its actual source address is adopted as the peer.
 */

#ifndef SITL_LINK_H
#define SITL_LINK_H

#include <stdint.h>
#include <stdbool.h>

/** Default port a ground station listens on for a UDP vehicle connection. */
#define SITL_DEFAULT_GCS_PORT   14550

/** Port this harness binds locally. Mission Planner does not need to know it. */
#define SITL_DEFAULT_LOCAL_PORT 14551

typedef struct sitl_link sitl_link_t;

/**
 * Open the UDP link.
 *
 * @param gcs_host   Ground-station address, typically "127.0.0.1".
 * @param gcs_port   Port the ground station listens on (SITL_DEFAULT_GCS_PORT).
 * @param local_port Port to bind locally; 0 lets the OS choose.
 * @return NULL on failure, with a reason printed to stderr.
 */
sitl_link_t *sitl_link_open(const char *gcs_host, uint16_t gcs_port, uint16_t local_port);

void sitl_link_close(sitl_link_t *link);

/**
 * Read one waiting datagram. Never blocks.
 * @return number of bytes read, or 0 if nothing was waiting.
 *
 * The sender's address is adopted as the reply peer, so a ground station that
 * transmits from an ephemeral port still receives telemetry.
 */
int sitl_link_recv(sitl_link_t *link, uint8_t *buf, int max_len);

/**
 * Send a datagram to the current peer.
 * @return bytes sent, or -1 on error.
 *
 * A datagram may carry several MAVLink frames; receivers treat the payload as
 * a byte stream, exactly as they would a serial link.
 */
int sitl_link_send(sitl_link_t *link, const uint8_t *buf, int len);

/** True once the ground station has sent us at least one packet. */
bool sitl_link_peer_known(const sitl_link_t *link);

/** Monotonic milliseconds since sitl_link_open(). Stands in for the tick counter. */
uint32_t sitl_millis(void);

/** Sleep, for pacing the superloop to wall-clock time. */
void sitl_sleep_ms(uint32_t ms);

#endif /* SITL_LINK_H */
