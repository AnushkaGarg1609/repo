/**
 * sitl_link.c
 *
 * UDP transport and timing --- the only operating-system dependency in the
 * project. Windows (Winsock2) and POSIX are both supported so the harness runs
 * wherever the toolchain does.
 */

#include "sitl_link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
  #include <winsock2.h>
  #include <ws2tcpip.h>
  typedef int socklen_t;
  #define SITL_INVALID_SOCK INVALID_SOCKET
  #define SITL_CLOSE(s)     closesocket(s)
  typedef SOCKET sitl_socket_t;
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  #include <fcntl.h>
  #include <errno.h>
  #include <time.h>
  #define SITL_INVALID_SOCK (-1)
  #define SITL_CLOSE(s)     close(s)
  typedef int sitl_socket_t;
#endif

struct sitl_link {
    sitl_socket_t      sock;
    struct sockaddr_in peer;
    bool               peer_known;
};

/* ============================================================================
 * Timing
 * ========================================================================= */

static uint32_t g_epoch_ms = 0;

static uint32_t raw_millis(void) {
#if defined(_WIN32)
    return (uint32_t)GetTickCount64();
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull);
#endif
}

uint32_t sitl_millis(void) {
    return raw_millis() - g_epoch_ms;
}

void sitl_sleep_ms(uint32_t ms) {
#if defined(_WIN32)
    Sleep(ms);
#else
    struct timespec ts;
    ts.tv_sec  = (time_t)(ms / 1000u);
    ts.tv_nsec = (long)((ms % 1000u) * 1000000ul);
    nanosleep(&ts, NULL);
#endif
}

/* ============================================================================
 * Link lifecycle
 * ========================================================================= */

sitl_link_t *sitl_link_open(const char *gcs_host, uint16_t gcs_port, uint16_t local_port) {
    sitl_link_t *link;
    struct sockaddr_in local;

#if defined(_WIN32)
    WSADATA wsa;
    static bool wsa_started = false;
    if (!wsa_started) {
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            fprintf(stderr, "sitl_link: WSAStartup failed\n");
            return NULL;
        }
        wsa_started = true;
    }
#endif

    link = (sitl_link_t *)calloc(1, sizeof(sitl_link_t));
    if (!link) return NULL;

    link->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (link->sock == SITL_INVALID_SOCK) {
        fprintf(stderr, "sitl_link: could not create UDP socket\n");
        free(link);
        return NULL;
    }

    memset(&local, 0, sizeof(local));
    local.sin_family      = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port        = htons(local_port);

    if (bind(link->sock, (struct sockaddr *)&local, sizeof(local)) != 0) {
        fprintf(stderr, "sitl_link: could not bind local port %u "
                        "(is another SITL instance running?)\n", local_port);
        SITL_CLOSE(link->sock);
        free(link);
        return NULL;
    }

    /* Non-blocking: the superloop polls the socket once per tick and must
     * never stall waiting for a ground station that may not be there. */
#if defined(_WIN32)
    {
        u_long nonblock = 1;
        ioctlsocket(link->sock, FIONBIO, &nonblock);
    }
#else
    {
        int flags = fcntl(link->sock, F_GETFL, 0);
        fcntl(link->sock, F_SETFL, flags | O_NONBLOCK);
    }
#endif

    /* Provisional peer: transmit blind to the ground station's listening port
     * until it answers, at which point its real source address takes over.
     * This is how a ground station set to plain "UDP" discovers a vehicle. */
    memset(&link->peer, 0, sizeof(link->peer));
    link->peer.sin_family = AF_INET;
    link->peer.sin_port   = htons(gcs_port);
    link->peer.sin_addr.s_addr = inet_addr(gcs_host);
    if (link->peer.sin_addr.s_addr == INADDR_NONE) {
        fprintf(stderr, "sitl_link: '%s' is not a dotted-quad address\n", gcs_host);
        SITL_CLOSE(link->sock);
        free(link);
        return NULL;
    }
    link->peer_known = false;

    g_epoch_ms = raw_millis();
    return link;
}

void sitl_link_close(sitl_link_t *link) {
    if (!link) return;
    SITL_CLOSE(link->sock);
    free(link);
#if defined(_WIN32)
    WSACleanup();
#endif
}

/* ============================================================================
 * Transfer
 * ========================================================================= */

int sitl_link_recv(sitl_link_t *link, uint8_t *buf, int max_len) {
    struct sockaddr_in from;
    socklen_t          from_len = sizeof(from);
    int                n;

    n = (int)recvfrom(link->sock, (char *)buf, max_len, 0,
                      (struct sockaddr *)&from, &from_len);
    if (n <= 0) return 0;

    /* Adopt whoever is actually talking to us. Mission Planner sends from an
     * ephemeral port, not from the port it listens on, so replying to the
     * configured port alone would work only by luck. */
    if (!link->peer_known ||
        link->peer.sin_addr.s_addr != from.sin_addr.s_addr ||
        link->peer.sin_port != from.sin_port) {
        link->peer       = from;
        link->peer_known = true;
    }
    return n;
}

int sitl_link_send(sitl_link_t *link, const uint8_t *buf, int len) {
    int n = (int)sendto(link->sock, (const char *)buf, len, 0,
                        (struct sockaddr *)&link->peer, sizeof(link->peer));
    return n;
}

bool sitl_link_peer_known(const sitl_link_t *link) {
    return link->peer_known;
}
