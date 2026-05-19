#ifndef FADEC_H
#define FADEC_H

// rtdp_ringbuf.h
#include <stdint.h>
#include <stdatomic.h>
#include <string.h>

#define N_DEVICES       8
#define RING_CAPACITY   1024   // must be power of 2
#define RING_MASK       (RING_CAPACITY - 1)

typedef struct {
    uint8_t  device_id;
    uint64_t timestamp_ns;     // from clock_gettime(CLOCK_MONOTONIC)
    int32_t  channels[4];      // 4 channels, sign-extended to 32-bit
    uint8_t  raw[13];          // raw SPI bytes: [0]=0xEB sync header, [1..12]=channel data
} __attribute__((packed)) sample_frame_t;

typedef struct {
    sample_frame_t  frames[RING_CAPACITY];
    atomic_uint_fast64_t head;  // written by acquisition core (core 1)
    uint8_t _pad[64 - sizeof(atomic_uint_fast64_t)]; // prevent false sharing
    atomic_uint_fast64_t tail;  // written by consumer core (core 2)
    uint8_t _pad2[64 - sizeof(atomic_uint_fast64_t)];
} __attribute__((aligned(64))) spsc_ring_t;

static inline int ring_push(spsc_ring_t* r, const sample_frame_t* f)
{
    uint64_t head = atomic_load_explicit(&r->head, memory_order_relaxed);
    uint64_t tail = atomic_load_explicit(&r->tail, memory_order_acquire);
    if ((head - tail) >= RING_CAPACITY) return -1; // full, drop frame
    r->frames[head & RING_MASK] = *f;
    atomic_store_explicit(&r->head, head + 1, memory_order_release);
    return 0;
}

static inline int ring_pop(spsc_ring_t* r, sample_frame_t* f)
{
    uint64_t tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
    uint64_t head = atomic_load_explicit(&r->head, memory_order_acquire);
    if (tail == head) return -1; // empty
    *f = r->frames[tail & RING_MASK];
    atomic_store_explicit(&r->tail, tail + 1, memory_order_release);
    return 0;
}

#endif