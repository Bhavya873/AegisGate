#include "aegis/spsc.h"

#include <string.h>

aegis_status_t aegis_spsc_init(aegis_spsc_t *q, void *storage, uint32_t capacity,
                                uint32_t elem_size)
{
    if (q == NULL || storage == NULL || elem_size == 0u || capacity < 2u) {
        return AEGIS_E_ARG;
    }
    if ((capacity & (capacity - 1u)) != 0u) {
        return AEGIS_E_RANGE;
    }
    atomic_store_explicit(&q->head, 0u, memory_order_relaxed);
    atomic_store_explicit(&q->tail, 0u, memory_order_relaxed);
    q->dropped = 0u;
    q->mask = capacity - 1u;
    q->elem_size = elem_size;
    q->buf = (uint8_t *)storage;
    return AEGIS_OK;
}

bool aegis_spsc_push(aegis_spsc_t *q, const void *elem)
{
    uint32_t head;
    uint32_t tail;
    uint32_t cap;
    uint32_t index;

    if (q == NULL || elem == NULL) {
        return false;
    }
    head = atomic_load_explicit(&q->head, memory_order_relaxed);
    tail = atomic_load_explicit(&q->tail, memory_order_acquire);
    cap = q->mask + 1u;
    if (head - tail >= cap) {
        /* full: drop the sample, never wait */
        q->dropped += 1u;
        return false;
    }
    index = head & q->mask;
    memcpy(q->buf + (size_t)index * (size_t)q->elem_size, elem, (size_t)q->elem_size);
    atomic_store_explicit(&q->head, head + 1u, memory_order_release);
    return true;
}

bool aegis_spsc_pop(aegis_spsc_t *q, void *elem)
{
    uint32_t tail;
    uint32_t head;
    uint32_t index;

    if (q == NULL || elem == NULL) {
        return false;
    }
    tail = atomic_load_explicit(&q->tail, memory_order_relaxed);
    head = atomic_load_explicit(&q->head, memory_order_acquire);
    if (head == tail) {
        return false;
    }
    index = tail & q->mask;
    memcpy(elem, q->buf + (size_t)index * (size_t)q->elem_size, (size_t)q->elem_size);
    atomic_store_explicit(&q->tail, tail + 1u, memory_order_release);
    return true;
}
