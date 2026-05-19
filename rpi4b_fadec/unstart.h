#ifndef UNSTART_H
#define UNSTART_H

#include "fadec.h"

#include <signal.h>
#include <stdint.h>
#include <stdatomic.h>

#define UNSTART_MAX_PORTS 6u
#define UNSTART_MAX_MINOR_FRAMES 48u
#define UNSTART_TIMING_LOG_CAPACITY 262144u
#define UNSTART_TIMING_LOG_PATH "unstart_timing.dat"

/*  
    * ////////////////////////
    * unstart_timing_record_t STRUCT
    * ////////////////////////
    * A single record of timing data for one captured frame.  The "user" fields
    * are available for arbitrary use by user processing code in unstart_on_frame().
*/
typedef struct {
    uint32_t record_index;
    uint32_t device_id;
    uint32_t call_us;
    uint32_t compute_us;
    int32_t user0;
    int32_t user1;
    int32_t user2;
    int32_t user3;
} unstart_timing_record_t;


typedef struct {
    uint64_t epoch_ns;
    uint32_t slot_period_ns;
    uint8_t slot_count;
    uint8_t slots[UNSTART_MAX_MINOR_FRAMES];
    uint8_t weights[UNSTART_MAX_PORTS];
} unstart_schedule_t;

typedef struct {
    unsigned int port_count;
    uint32_t spi_hz;
    uint32_t frame_bytes;
    uint32_t cs_setup_delay_ns;
} unstart_config_t;

typedef struct {
    unsigned int port_count;
    uint32_t spi_hz;
    uint32_t frame_bytes;
    uint32_t cs_setup_delay_ns;
    atomic_uint_fast32_t desired_weight[UNSTART_MAX_PORTS];
    atomic_uint_fast32_t active_schedule_index;
    atomic_uint_fast32_t trigger_seen;
    atomic_uint_fast32_t capture_active;
    atomic_uint_fast32_t capture_finished;
    atomic_uint_fast32_t capture_count;
    atomic_uint_fast32_t capture_overflow;
    atomic_uint_fast64_t published_generation;
    atomic_uint_fast64_t trigger_epoch_ns;
    atomic_uint_fast64_t capture_deadline_ns;
    atomic_uint_fast64_t observed_frames[UNSTART_MAX_PORTS];
    unstart_schedule_t schedules[2];
    unstart_timing_record_t timing_records[UNSTART_TIMING_LOG_CAPACITY];
} unstart_runtime_t;

typedef struct {
    unstart_runtime_t *runtime;
    volatile sig_atomic_t *running;
} unstart_thread_args_t;

void unstart_init(unstart_runtime_t *runtime,
                  const unstart_config_t *config,
                  uint64_t epoch_ns);

void *unstart_thread(void *arg);

void unstart_set_port_weight(unstart_runtime_t *runtime,
                             unsigned int port_index,
                             uint8_t weight);

unsigned int unstart_order_ready_ports(const unstart_runtime_t *runtime,
                                       uint64_t now_ns,
                                       uint32_t ready_mask,
                                       uint8_t *ordered_ports,
                                       unsigned int ordered_capacity);

/*
 * Processing hook called by fadec once a frame has already been read,
 * header-checked and decoded. User processing code should read values from
 * frame->channels[] and frame->raw[] here, rather than touching the SPI path.
 */
void unstart_on_frame(unstart_runtime_t *runtime,
                      const sample_frame_t *frame);

int unstart_save_timing_log(const unstart_runtime_t *runtime);

#endif