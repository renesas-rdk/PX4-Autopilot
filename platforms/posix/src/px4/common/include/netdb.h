#pragma once

#if defined(__PX4_FREERTOS)

// Minimal netdb.h stub for FreeRTOS+POSIX
#include <sys/socket.h>

#ifdef __cplusplus
extern "C" {
#endif

struct addrinfo {
    int ai_flags;
    int ai_family;
    int ai_socktype;
    int ai_protocol;
    socklen_t ai_addrlen;
    struct sockaddr *ai_addr;
    char *ai_canonname;
    struct addrinfo *ai_next;
};

static inline int getaddrinfo(const char *node, const char *service,
                             const struct addrinfo *hints,
                             struct addrinfo **res)
{
    (void)node; (void)service; (void)hints; (void)res;
    return -1;
}

static inline void freeaddrinfo(struct addrinfo *res)
{
    (void)res;
}

#ifdef __cplusplus
}
#endif

#else
// On non-FreeRTOS platforms (POSIX/Linux), use the system netdb.h
#include_next <netdb.h>
#endif /* __PX4_FREERTOS */
