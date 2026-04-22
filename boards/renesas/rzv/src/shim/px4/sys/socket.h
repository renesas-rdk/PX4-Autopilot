/****************************************************************************
 *
 * Copyright (c) 2025 Renesas Electronics Corporation and/or its affiliates
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/**
 * @file sys/socket.h
 * @brief Minimal POSIX socket definitions for Renesas RZ/V2H bare-metal build.
 *        Full socket I/O is proxied to the CA55 Linux side via OpenAMP RPC.
 */

#pragma once

#if defined(__PX4_FREERTOS)
#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t socklen_t;
typedef uint16_t sa_family_t;

struct sockaddr {
    sa_family_t sa_family;
    char        sa_data[14];
};

struct sockaddr_storage {
    sa_family_t ss_family;
    char        __ss_padding[126];
};

struct in_addr {
    uint32_t s_addr;
};

struct sockaddr_in {
    sa_family_t    sin_family;
    uint16_t       sin_port;
    struct in_addr sin_addr;
    char           sin_zero[8];
};

struct in6_addr {
    uint8_t s6_addr[16];
};

struct sockaddr_in6 {
    sa_family_t     sin6_family;
    uint16_t        sin6_port;
    uint32_t        sin6_flowinfo;
    struct in6_addr sin6_addr;
    uint32_t        sin6_scope_id;
};

/* Address families */
#define AF_UNSPEC   0
#define AF_UNIX     1
#define AF_INET     2
#define AF_INET6    10

/* Socket types */
#define SOCK_STREAM    1
#define SOCK_DGRAM     2
#define SOCK_RAW       3
#define SOCK_NONBLOCK  0x800
#define SOCK_CLOEXEC   0x80000

/* Protocol values */
#define IPPROTO_TCP  6
#define IPPROTO_UDP  17

/* setsockopt / getsockopt level */
#define SOL_SOCKET   1

/* Socket options */
#define SO_DEBUG        1
#define SO_REUSEADDR    2
#define SO_TYPE         3
#define SO_ERROR        4
#define SO_DONTROUTE    5
#define SO_BROADCAST    6
#define SO_SNDBUF       7
#define SO_RCVBUF       8
#define SO_KEEPALIVE    9
#define SO_OOBINLINE    10
#define SO_LINGER       13
#define SO_RCVLOWAT     18
#define SO_SNDLOWAT     19
#define SO_RCVTIMEO     20
#define SO_SNDTIMEO     21
#define SO_REUSEPORT    15

/* shutdown() how values */
#define SHUT_RD   0
#define SHUT_WR   1
#define SHUT_RDWR 2

/* send/recv flags */
#define MSG_OOB       0x01
#define MSG_PEEK      0x02
#define MSG_DONTWAIT  0x40
#define MSG_NOSIGNAL  0x4000

/* htons/ntohs helpers (host = little-endian for ARM) */
#ifndef htons
#define htons(x) ((uint16_t)(((x) >> 8) | ((x) << 8)))
#endif
#ifndef ntohs
#define ntohs(x) htons(x)
#endif
#ifndef htonl
#define htonl(x) ((uint32_t)( \
    (((x) & 0xFF000000U) >> 24) | \
    (((x) & 0x00FF0000U) >>  8) | \
    (((x) & 0x0000FF00U) <<  8) | \
    (((x) & 0x000000FFU) << 24)))
#endif
#ifndef ntohl
#define ntohl(x) htonl(x)
#endif

/* Function declarations — implemented in posix_socket_stub.c */
int    socket(int domain, int type, int protocol);
int    bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen);
int    connect(int sockfd, const struct sockaddr *addr, socklen_t addrlen);
int    listen(int sockfd, int backlog);
int    accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen);
int    setsockopt(int sockfd, int level, int optname, const void *optval, socklen_t optlen);
int    getsockopt(int sockfd, int level, int optname, void *optval, socklen_t *optlen);
int    shutdown(int sockfd, int how);
int    getsockname(int sockfd, struct sockaddr *addr, socklen_t *addrlen);
int    getpeername(int sockfd, struct sockaddr *addr, socklen_t *addrlen);

#ifdef __cplusplus
}
#endif
#endif /* __PX4_FREERTOS */
