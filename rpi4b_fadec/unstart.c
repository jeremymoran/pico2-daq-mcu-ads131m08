#define _GNU_SOURCE

#include "unstart.h"

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <gpiod.h>

#define UNSTART_HOST_MARGIN_NS 20000u
#define UNSTART_MAX_WEIGHT 8u
#define UNSTART_GPIO_CHIP "/dev/gpiochip0"
#define UNSTART_CONSUMER "unstart"
#define UNSTART_EXT_TRIG_GPIO 6u
#define UNSTART_CAPTURE_DURATION_NS 2000000000ULL
#define UNSTART_LOOP_NS 10000000LL

/*
 * unstart.c is intentionally the policy layer, not the transport layer.
 * fadec.c still owns DRDY, CS, SPI and the lock-free ring. This file owns:
 *   1. building the current major/minor-frame schedule
 *   2. reordering ready ports according to that schedule
 *   3. exposing a safe hook where user processing code can inspect decoded data
 *
 * The important separation is that any experimental processing code should be
 * added to unstart_on_frame() below, after FADEC has already produced a valid
 * sample_frame_t. That keeps the GPIO/SPI hot path deterministic.
 */

static uint64_t unstart_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static struct gpiod_line_request *unstart_request_trigger_line(struct gpiod_chip *chip,
                                                               unsigned int offset)
{
    struct gpiod_line_settings  *settings = gpiod_line_settings_new();
    struct gpiod_line_config    *line_cfg = gpiod_line_config_new();
    struct gpiod_request_config *req_cfg  = gpiod_request_config_new();
    struct gpiod_line_request   *request  = NULL;

    if (!settings || !line_cfg || !req_cfg) {
        errno = ENOMEM;
        goto out;
    }

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);
    gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_DOWN);
    gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings);
    gpiod_request_config_set_consumer(req_cfg, UNSTART_CONSUMER);
    request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

out:
    if (settings) gpiod_line_settings_free(settings);
    if (line_cfg) gpiod_line_config_free(line_cfg);
    if (req_cfg)  gpiod_request_config_free(req_cfg);
    return request;
}

static void unstart_begin_capture(unstart_runtime_t *runtime, uint64_t trigger_ns)
{
    atomic_store_explicit(&runtime->trigger_epoch_ns, trigger_ns, memory_order_relaxed);
    atomic_store_explicit(&runtime->capture_deadline_ns,
                          trigger_ns + UNSTART_CAPTURE_DURATION_NS,
                          memory_order_relaxed);
    atomic_store_explicit(&runtime->capture_count, 0u, memory_order_relaxed);
    atomic_store_explicit(&runtime->capture_overflow, 0u, memory_order_relaxed);
    atomic_store_explicit(&runtime->capture_finished, 0u, memory_order_relaxed);
    atomic_store_explicit(&runtime->capture_active, 1u, memory_order_release);
    atomic_store_explicit(&runtime->trigger_seen, 1u, memory_order_release);
}

static int unstart_check_trigger(unstart_runtime_t *runtime,
                                 struct gpiod_line_request *trigger_req,
                                 struct gpiod_edge_event_buffer *event_buf)
{
    int ready;

    if (!trigger_req || !event_buf) return 0;
    if (atomic_load_explicit(&runtime->trigger_seen, memory_order_acquire)) return 0;

    ready = gpiod_line_request_wait_edge_events(trigger_req, UNSTART_LOOP_NS);
    if (ready <= 0) return ready;

    ready = gpiod_line_request_read_edge_events(trigger_req, event_buf, 1);
    if (ready <= 0) return ready;

    unstart_begin_capture(runtime, unstart_now_ns());
    printf("[unstart] EXT_TRIG detected on GPIO %u; timing capture started\n",
           UNSTART_EXT_TRIG_GPIO);
    fflush(stdout);
    return ready;
}

/*
 * Convert RTDP transport timing into one scheduler slot duration.
 * A single minor frame should be large enough to cover one 13-byte SPI read,
 * CS setup time and a conservative host-side margin.
 */
static uint32_t unstart_slot_period_ns(const unstart_runtime_t *runtime)
{
    uint64_t wire_ns;

    if (!runtime->spi_hz) return runtime->cs_setup_delay_ns + UNSTART_HOST_MARGIN_NS;

    wire_ns = ((uint64_t)runtime->frame_bytes * 8ULL * 1000000000ULL)
            / (uint64_t)runtime->spi_hz;

    return (uint32_t)(wire_ns + runtime->cs_setup_delay_ns + UNSTART_HOST_MARGIN_NS);
}

static void unstart_seed_initial_weights(unstart_runtime_t *runtime)
{
    for (unsigned int port = 0; port < runtime->port_count; port++)
        atomic_store_explicit(&runtime->desired_weight[port], 1u, memory_order_relaxed);
}

static unsigned int unstart_load_weights(const unstart_runtime_t *runtime,
                                         uint8_t *weights)
{
    unsigned int total = 0;

    for (unsigned int port = 0; port < runtime->port_count; port++) {
        uint32_t weight = atomic_load_explicit(&runtime->desired_weight[port],
                                               memory_order_relaxed);

        if (weight > UNSTART_MAX_WEIGHT) weight = UNSTART_MAX_WEIGHT;

        weights[port] = (uint8_t)weight;
        total += (unsigned int)weights[port];
    }

    if (total > UNSTART_MAX_MINOR_FRAMES) {
        total = 0;
        for (unsigned int port = 0; port < runtime->port_count; port++) {
            weights[port] = weights[port] ? 1u : 0u;
            total += (unsigned int)weights[port];
        }
    }

    return total;
}

/*
 * Build one complete major frame from the current per-port weights.
 * Example: weights [2,1,1,0,0,0] will produce a four-slot major frame where
 * port 0 appears twice and ports 1 and 2 appear once each.
 *
 * The double-buffered schedule lets the unstart thread publish a new frame
 * atomically without forcing acq_thread() to take a lock.
 */
static void unstart_build_schedule(unstart_runtime_t *runtime, uint64_t epoch_ns)
{
    uint8_t weights[UNSTART_MAX_PORTS] = {0};
    unsigned int total = unstart_load_weights(runtime, weights);
    unsigned int next_index = 1u - atomic_load_explicit(&runtime->active_schedule_index,
                                                         memory_order_relaxed);
    unstart_schedule_t *schedule = &runtime->schedules[next_index];
    int current[UNSTART_MAX_PORTS] = {0};

    memset(schedule, 0, sizeof(*schedule));
    schedule->epoch_ns = epoch_ns;
    schedule->slot_period_ns = unstart_slot_period_ns(runtime);

    for (unsigned int port = 0; port < runtime->port_count; port++)
        schedule->weights[port] = weights[port];

    if (!total) {
        atomic_store_explicit(&runtime->active_schedule_index, next_index,
                              memory_order_release);
        atomic_fetch_add_explicit(&runtime->published_generation, 1u,
                                  memory_order_relaxed);
        return;
    }

    schedule->slot_count = (uint8_t)total;

    for (unsigned int slot = 0; slot < total; slot++) {
        int best_score = 0;
        unsigned int best_port = 0;
        int have_candidate = 0;

        for (unsigned int port = 0; port < runtime->port_count; port++) {
            if (!weights[port]) continue;
            current[port] += (int)weights[port];
            if (!have_candidate || current[port] > best_score) {
                best_score = current[port];
                best_port = port;
                have_candidate = 1;
            }
        }

        schedule->slots[slot] = (uint8_t)best_port;
        current[best_port] -= (int)total;
    }

    atomic_store_explicit(&runtime->active_schedule_index, next_index,
                          memory_order_release);
    atomic_fetch_add_explicit(&runtime->published_generation, 1u,
                              memory_order_relaxed);
}

void unstart_init(unstart_runtime_t *runtime,
                  const unstart_config_t *config,
                  uint64_t epoch_ns)
{
    memset(runtime, 0, sizeof(*runtime));

    runtime->port_count = config->port_count;
    if (runtime->port_count > UNSTART_MAX_PORTS)
        runtime->port_count = UNSTART_MAX_PORTS;

    runtime->spi_hz = config->spi_hz;
    runtime->frame_bytes = config->frame_bytes;
    runtime->cs_setup_delay_ns = config->cs_setup_delay_ns;

    for (unsigned int port = 0; port < UNSTART_MAX_PORTS; port++) {
        atomic_init(&runtime->desired_weight[port], 0u);
        atomic_init(&runtime->observed_frames[port], 0u);
    }
    atomic_init(&runtime->active_schedule_index, 0u);
    atomic_init(&runtime->trigger_seen, 0u);
    atomic_init(&runtime->capture_active, 0u);
    atomic_init(&runtime->capture_finished, 0u);
    atomic_init(&runtime->capture_count, 0u);
    atomic_init(&runtime->capture_overflow, 0u);
    atomic_init(&runtime->published_generation, 0u);
    atomic_init(&runtime->trigger_epoch_ns, 0u);
    atomic_init(&runtime->capture_deadline_ns, 0u);

    unstart_seed_initial_weights(runtime);
    unstart_build_schedule(runtime, epoch_ns);
}

void *unstart_thread(void *arg)
{
    unstart_thread_args_t *ctx = (unstart_thread_args_t *)arg;
    uint8_t last_weights[UNSTART_MAX_PORTS] = {0};
    cpu_set_t cpuset;
    struct timespec pause = { .tv_sec = 0, .tv_nsec = UNSTART_LOOP_NS };
    struct gpiod_chip *chip = NULL;
    struct gpiod_line_request *trigger_req = NULL;
    struct gpiod_edge_event_buffer *event_buf = NULL;

    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    chip = gpiod_chip_open(UNSTART_GPIO_CHIP);
    if (!chip) {
        perror("[unstart] gpiod_chip_open");
    } else {
        trigger_req = unstart_request_trigger_line(chip, UNSTART_EXT_TRIG_GPIO);
        if (!trigger_req) {
            fprintf(stderr, "[unstart] Cannot request EXT_TRIG GPIO %u: %s\n",
                    UNSTART_EXT_TRIG_GPIO, strerror(errno));
        } else {
            event_buf = gpiod_edge_event_buffer_new(1);
            if (!event_buf)
                perror("[unstart] gpiod_edge_event_buffer_new");
        }
    }

    while (*ctx->running) {
        uint8_t weights[UNSTART_MAX_PORTS] = {0};
        uint64_t deadline_ns;
        uint64_t now_ns;

        if (!atomic_load_explicit(&ctx->runtime->trigger_seen, memory_order_acquire)) {
            int trigger_status = unstart_check_trigger(ctx->runtime, trigger_req, event_buf);
            if (trigger_status < 0) {
                fprintf(stderr, "[unstart] EXT_TRIG wait failed: %s\n", strerror(errno));
                nanosleep(&pause, NULL);
            }
        } else {
            nanosleep(&pause, NULL);
        }

        /*
         * Today the scheduler only rebuilds when port weights change.
         * That gives you a stable major frame until your own control code
         * calls unstart_set_port_weight().
         */
        (void)unstart_load_weights(ctx->runtime, weights);

        if (memcmp(weights, last_weights, sizeof(weights)) != 0) {
            memcpy(last_weights, weights, sizeof(weights));
            unstart_build_schedule(ctx->runtime, unstart_now_ns());
        }

        if (!atomic_load_explicit(&ctx->runtime->capture_active, memory_order_acquire))
            continue;

        deadline_ns = atomic_load_explicit(&ctx->runtime->capture_deadline_ns,
                                           memory_order_relaxed);
        now_ns = unstart_now_ns();
        if (deadline_ns && now_ns >= deadline_ns) {
            atomic_store_explicit(&ctx->runtime->capture_active, 0u, memory_order_release);
            atomic_store_explicit(&ctx->runtime->capture_finished, 1u, memory_order_release);
            *ctx->running = 0;
            printf("[unstart] capture window complete; shutting down\n");
            fflush(stdout);
        }
    }

    if (event_buf) gpiod_edge_event_buffer_free(event_buf);
    if (trigger_req) gpiod_line_request_release(trigger_req);
    if (chip) gpiod_chip_close(chip);

    return NULL;
}

void unstart_set_port_weight(unstart_runtime_t *runtime,
                             unsigned int port_index,
                             uint8_t weight)
{
    if (!runtime) return;
    if (port_index >= runtime->port_count) return;

    atomic_store_explicit(&runtime->desired_weight[port_index], weight,
                          memory_order_relaxed);
}

unsigned int unstart_order_ready_ports(const unstart_runtime_t *runtime,
                                       uint64_t now_ns,
                                       uint32_t ready_mask,
                                       uint8_t *ordered_ports,
                                       unsigned int ordered_capacity)
{
    unsigned int count = 0;
    unsigned int index = atomic_load_explicit(&runtime->active_schedule_index,
                                              memory_order_acquire);
    const unstart_schedule_t *schedule = &runtime->schedules[index];
    uint32_t remaining = ready_mask;

    /*
     * FADEC passes in a bitmask of ports whose DRDY lines are currently ready.
     * This function does not perform any bus activity. It only decides order.
     *
     * ordered_ports[] comes back as a list of port indices in the order FADEC
     * should service them. If only one bit is set, the result is trivial. If
     * multiple ports are ready, the current minor-frame position breaks ties.
     */
    if (schedule->slot_count && schedule->slot_period_ns) {
        uint64_t elapsed_ns = (now_ns >= schedule->epoch_ns)
                            ? (now_ns - schedule->epoch_ns) : 0u;
        unsigned int start = (unsigned int)((elapsed_ns / schedule->slot_period_ns)
                          % (uint64_t)schedule->slot_count);

        for (unsigned int offset = 0; offset < schedule->slot_count; offset++) {
            unsigned int slot = (start + offset) % schedule->slot_count;
            unsigned int port = schedule->slots[slot];
            uint32_t bit = 1u << port;

            if (!(remaining & bit)) continue;
            if (count >= ordered_capacity) break;

            ordered_ports[count++] = (uint8_t)port;
            remaining &= ~bit;
        }
    }

    for (unsigned int port = 0; port < runtime->port_count && remaining; port++) {
        uint32_t bit = 1u << port;

        if (!(remaining & bit)) continue;
        if (count >= ordered_capacity) break;

        ordered_ports[count++] = (uint8_t)port;
        remaining &= ~bit;
    }

    return count;
}

/*
 * This is the user-facing processing hook.
 *
 * By the time execution gets here:
 *   - FADEC has already read the SPI frame from the selected port
 *   - the 0xEB header has already been validated
 *   - the four 24-bit samples have already been sign-extended into int32_t
 *
 * That means your own code can safely process decoded numbers directly from:
 *   frame->device_id      which RTDP port produced the frame
 *   frame->timestamp_ns   when FADEC saw the DRDY event
 *   frame->channels[0..3] the four decoded sample values
 *   frame->raw[0..12]     the original RTDP bytes, if you need raw access
 *
 * A typical starting point for your own logic is:
 *   int32_t ch0 = frame->channels[0];
 *   int32_t ch1 = frame->channels[1];
 *   ... run filtering / thresholding / derived calculations here ...
 *
 * Keep heavy or experimental processing here in unstart rather than inside
 * acq_thread(). That preserves FADEC as the bulletproof bus layer.
 */
void unstart_on_frame(unstart_runtime_t *runtime,
                      const sample_frame_t *frame)
{
    uint64_t call_ns;
    uint64_t trigger_epoch_ns;
    uint64_t deadline_ns;
    uint64_t user_start_ns;
    uint64_t user_end_ns;
    uint32_t call_us;
    uint32_t compute_us;
    uint32_t record_index;
    int32_t user0 = 0;
    int32_t user1 = 0;
    int32_t user2 = 0;
    int32_t user3 = 0;

    if (!runtime) return;
    if (!frame) return;
    if (frame->device_id >= runtime->port_count) return;

    atomic_fetch_add_explicit(&runtime->observed_frames[frame->device_id], 1u,
                              memory_order_relaxed);

    if (!atomic_load_explicit(&runtime->capture_active, memory_order_acquire))
        return;

    call_ns = unstart_now_ns();
    trigger_epoch_ns = atomic_load_explicit(&runtime->trigger_epoch_ns,
                                            memory_order_relaxed);
    deadline_ns = atomic_load_explicit(&runtime->capture_deadline_ns,
                                       memory_order_relaxed);

    if (!trigger_epoch_ns) return;
    if (deadline_ns && call_ns >= deadline_ns) return;

    call_us = (uint32_t)((call_ns - trigger_epoch_ns) / 1000ULL);
    user_start_ns = unstart_now_ns();

    /*
     * ========================= USER CODE STARTS HERE =========================
     *
        * Anything you want written to unstart_timing.dat must be copied into one
        * or more of these four local variables before this block ends:
        *
        *   user0
        *   user1
        *   user2
        *   user3
        *
        * Those four values are written to the output file on every logged record.
        * If you do not assign them, they stay at 0 for that record.
        *
        * Concrete example:
        *   int32_t sample0 = frame->channels[0];
        *   int32_t sample1 = frame->channels[1];
        *   int32_t delta01 = sample0 - sample1;
        *
        *   user0 = sample0;   // saved in the 'user0' column
        *   user1 = sample1;   // saved in the 'user1' column
        *   user2 = delta01;   // saved in the 'user2' column
        *   user3 = call_us;   // or any derived/debug value you want to track
     *
     * If you want per-port state, keep it in unstart_runtime_t and index it
     * by frame->device_id.
     *
     * The timing log measures this user region only:
     *   call_us    = microseconds from EXT_TRIG to callback entry
     *   compute_us = approximate microseconds spent in this user block
     *
     * Example re-schedule request:
     *   If a processed value says port 0 now needs more attention, ask the
     *   scheduler to give it more minor-frame slots:
     *
     *     if (frame->channels[0] > SOME_THRESHOLD)
     *         unstart_set_port_weight(runtime, 0, 2);
     *
     *   That does not change the bus order immediately inside this function.
     *   It updates runtime->desired_weight[], and the unstart thread notices
     *   the new weight and rebuilds the published major frame on its next pass.
     *
     *   Likewise, to reduce attention later:
     *
     *     unstart_set_port_weight(runtime, 0, 1);
     *
     *   Use weight 0 to remove a port from the planned schedule entirely.
     *   FADEC still owns the actual SPI reads; unstart only changes priority.
    *
    * Another very common pattern is to log threshold decisions directly:
    *
    *   if (frame->channels[0] > 1000) {
    *       user0 = frame->channels[0];
    *       user1 = 1; // flag meaning "threshold crossed"
    *   }
     */




    // END OF USER CODE; LOGGING SCRIPT
    user_end_ns = unstart_now_ns();
    
    compute_us = (uint32_t)((user_end_ns - user_start_ns) / 1000ULL);
    record_index = (uint32_t)atomic_fetch_add_explicit(&runtime->capture_count,
                                                       1u,
                                                       memory_order_relaxed);

    if (record_index < UNSTART_TIMING_LOG_CAPACITY) {
        unstart_timing_record_t *record = &runtime->timing_records[record_index];

        record->record_index = record_index;
        record->device_id = (uint32_t)frame->device_id;
        record->call_us = call_us;
        record->compute_us = compute_us;
        record->user0 = user0;
        record->user1 = user1;
        record->user2 = user2;
        record->user3 = user3;
    } else {
        atomic_fetch_add_explicit(&runtime->capture_overflow, 1u,
                                  memory_order_relaxed);
    }
}

/*
    * Save the timing log to a text file. Each line corresponds to one captured frame.
*/
int unstart_save_timing_log(const unstart_runtime_t *runtime)
{
    FILE *fp;
    uint32_t capture_count;
    uint32_t overflow_count;

    if (!runtime) return -1;
    if (!atomic_load_explicit(&runtime->trigger_seen, memory_order_acquire))
        return 0;

    fp = fopen(UNSTART_TIMING_LOG_PATH, "w");
    if (!fp) return -1;

    capture_count = (uint32_t)atomic_load_explicit(&runtime->capture_count,
                                                   memory_order_relaxed);
    overflow_count = (uint32_t)atomic_load_explicit(&runtime->capture_overflow,
                                                    memory_order_relaxed);
    if (capture_count > UNSTART_TIMING_LOG_CAPACITY)
        capture_count = UNSTART_TIMING_LOG_CAPACITY;

    fprintf(fp, "record_index device_id call_us compute_us user0 user1 user2 user3\n");
    for (uint32_t i = 0; i < capture_count; i++) {
        const unstart_timing_record_t *record = &runtime->timing_records[i];

        fprintf(fp, "%u %u %u %u %d %d %d %d\n",
                record->record_index,
                record->device_id,
                record->call_us,
                record->compute_us,
                record->user0,
                record->user1,
                record->user2,
                record->user3);
    }

    if (overflow_count)
        fprintf(fp, "# overflow_count %u\n", overflow_count);

    fclose(fp);
    return 1;
}