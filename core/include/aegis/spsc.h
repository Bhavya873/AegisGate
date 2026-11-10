#ifndef AEGIS_SPSC_H
#define AEGIS_SPSC_H

#include "aegis/types.h"

#include <stdatomic.h>
#include <stdint.h>

/* head and tail are on separate 128-byte lines so the two threads do not share a cache line */
typedef struct {
    _Alignas(128) _Atomic uint32_t head;
    _Alignas(128) _Atomic uint32_t tail;
    _Alignas(128) uint32_t dropped;
    uint32_t mask;
    uint32_t elem_size;
    uint8_t *buf;
} aegis_spsc_t;

aegis_status_t aegis_spsc_init(aegis_spsc_t *q, void *storage, uint32_t capacity,
                               uint32_t elem_size);
bool aegis_spsc_push(aegis_spsc_t *q, const void *elem);
bool aegis_spsc_pop(aegis_spsc_t *q, void *elem);

#endif
