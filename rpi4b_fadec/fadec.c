/*
 * fadec.c
 * FADEC Real-Time Data Port (RTDP) master — Raspberry Pi 4B
 *
 * PURPOSE
 * ///////
 * This program runs on a Raspberry Pi 4B and acts as the "master" controller
 * that reads live ADC (analogue-to-digital converter) data from up to six
 * RP2350 slave microcontroller boards.  Each slave board contains an
 * ADS131M08 8-channel 24-bit ADC and streams its most recent sample set
 * over an RS-485 serial bus using the SPI protocol.
 *
 * HOW THE HARDWARE TALKS
 * //////////////////////
 * 1. Each slave asserts its DATA_RDY (DRDY) GPIO pin HIGH when a fresh
 *    16-byte sample frame is ready to be read.
 * 2. This program detects that rising edge and immediately pulls the
 *    slave's Chip Select (CS) line LOW (CS is "active low" — LOW means
 *    "talk to me").
 * 3. The Pi then clocks 13 bytes out of the SPI bus.  The slave puts the
 *    data on MISO; the Pi ignores MOSI (we send zeros).
 * 4. After the transfer the Pi releases CS (back to HIGH) so the bus is
 *    free for the next slave.
 * 5. Byte 0 is a 0xEB sync header.  The remaining 12 bytes contain four
 *    24-bit signed integers (big-endian, MSB first), sign-extended to
 *    int32_t on decode.  Frames whose header byte is not 0xEB are dropped.
 *
 * TWO-THREAD ARCHITECTURE
 * ///////////////////////
 * The program uses two POSIX threads (pthreads) pinned to isolated CPU
 * cores so they never interfere with each other or with the Linux kernel:
 *
 *   Thread        Core   Priority           Job
 *   ///////////   ////   ////////////////   ////////////////////////////
 *   acq_thread     1     SCHED_FIFO / 99    Waits for DRDY edges, does
 *                        (highest RT)        SPI reads, pushes frames
 *                                            into the ring buffer.
 *   proc_thread    2     SCHED_FIFO / 50    Pops frames from the ring
 *                        (lower RT)          buffer, decodes them, prints
 *                                            results and statistics.
 *
 * The ring buffer (defined in fadec.h) is a lock-free single-producer /
 * single-consumer queue that lets the two threads exchange data without
 * any mutexes (locks), which would introduce unpredictable latency.
 *
 * REQUIRED BOOT OPTION (isolate cores 1 and 2 from Linux scheduling)
 * ///////////////////////////////////////////////////////////////////
 * Add to /boot/firmware/cmdline.txt (all on one line):
 *   isolcpus=1,2 nohz_full=1,2 rcu_nocbs=1,2
 *
 *   core 0  Linux kernel, IRQs, housekeeping  (untouched)
 *   core 1  acq_thread — DRDY polling + SPI   (real-time, isolated)
 *   core 2  proc_thread — data processing     (real-time, isolated)
 *   core 3  spare / Linux overflow / logging  (untouched)
 *
 * HARDWARE PIN MAPPING (Raspberry Pi 4B GPIO header)
 * ///////////////////////////////////////////////////
 *   Signal   GPIO#   Direction   Notes
 *   ///////  //////  //////////  //////////////////////////////////////
 *   DRDY1    GPIO  2   INPUT     ADC board 1 data-ready  [WIRED]
 *   DRDY2    GPIO  3   INPUT     ADC board 2 data-ready
 *   DRDY3    GPIO 26   INPUT     ADC board 3 data-ready
 *   DRDY4    GPIO 23   INPUT     ADC board 4 data-ready
 *   DRDY5    GPIO 22   INPUT     ADC board 5 data-ready
 *   DRDY6    GPIO 12   INPUT     ADC board 6 data-ready
 *   MISO     GPIO  9   INPUT     SPI0 data IN  (slave sends to Pi)
 *   CLK      GPIO 11   OUTPUT    SPI0 clock    (Pi drives, idle HIGH)
 *   CS1      GPIO 13   OUTPUT    Chip-select board 1  [WIRED]
 *   CS2      GPIO 19   OUTPUT    Chip-select board 2
 *   CS3      GPIO 16   OUTPUT    Chip-select board 3
 *   CS4      GPIO 18   OUTPUT    Chip-select board 4
 *   CS5      GPIO 20   OUTPUT    Chip-select board 5
 *   CS6      GPIO 21   OUTPUT    Chip-select board 6
 *   DE       GPIO 24   OUTPUT    RS-485 ENABLE CLOCK OUTPUT (IMPORTANT)
 *   REN      GPIO 25   OUTPUT    RS-485 RECEIVER ENABLE (IMPORTANT)
 *
 * SPI PROTOCOL DETAILS
 * ////////////////////
 *   - Mode 3: clock idles HIGH (CPOL=1), data sampled on falling edge (CPHA=1)
 *   - Maximum clock speed: 10 MHz (supported by RP2350 RTDP firmware)
 *   - Frame size: 13 bytes = 1 sync header (0xEB) + 4 channels x 3 bytes (24-bit signed, big-endian)
 *   - MOSI (Pi→slave) bytes are ignored by the slave; we send zeros
 *
 * BUILD
 * /////
 *   make
 *
 * RUN
 * ///
 *   sudo ./fadec          <- sudo needed for RT scheduling and GPIO access
 *   (or: add your user to the 'gpio' and 'spi' groups; run as root for RT)
 *
 * OUTPUT
 * //////
 *   One decoded line per received SPI frame showing channel values.
 *   Per-second frame rate summary printed every second.
 *   Press Ctrl+C to stop and see final totals.
 */

/*
 * _GNU_SOURCE tells the C library to expose Linux-specific extensions
 * such as pthread_setaffinity_np (the "np" means "non-portable").
 * Must be defined before any #include.
 */
#define _GNU_SOURCE

/*
 * fadec.h contains the ring buffer type (spsc_ring_t) and the sample
 * frame type (sample_frame_t) shared between the two threads.
 */
#include "fadec.h"
#include "unstart.h"
#include "../rtdp.h"

/* STANDARD C LIBRARY HEADERS */
#include <stdio.h>      /* printf, fprintf, perror                     */
#include <stdlib.h>     /* EXIT_SUCCESS, EXIT_FAILURE                  */
#include <stdint.h>     /* uint8_t, uint32_t, uint64_t, int32_t, etc. */
#include <stdatomic.h>  /* atomic_uint_fast64_t, atomic_fetch_add, etc.*/
#include <string.h>     /* memset, strerror                            */
#include <signal.h>     /* signal(), SIGINT, SIGTERM                   */
#include <errno.h>      /* errno — the last error code from syscalls   */
#include <time.h>       /* clock_gettime(), CLOCK_MONOTONIC            */
#include <fcntl.h>      /* open(), O_RDWR                              */
#include <unistd.h>     /* close(), pause()                            */

/* POSIX / LINUX HEADERS */
#include <pthread.h>         /* pthreads — POSIX threads                */
#include <sched.h>           /* CPU affinity, SCHED_FIFO, sched_yield() */
#include <sys/mman.h>        /* mlockall() — pin pages into RAM          */
#include <sys/ioctl.h>       /* ioctl() — device control calls           */
#include <linux/spi/spidev.h>/* SPI ioctl constants (SPI_MODE_3, etc.)   */
#include <gpiod.h>           /* libgpiod v2 — GPIO control without sysfs */


/* //////////////////////////////////////////////////////////////////////////
 * HARDWARE CONSTANTS
 * //////////////////////////////////////////////////////////////////////////
 *
 * These #define and static array values encode the physical wiring of the
 * board.  If you rewire anything, update these values and recompile.
 * Nothing else in this file needs to change.
 */

/*
 * N_SLAVES  — total number of ADC slave boards the software supports.
 *             Currently only slave 1 is physically wired, but the software
 *             monitors all six DRDY lines and will read any that fire.
 * N_CHANNELS — number of ADC channels per frame (fixed by the RTDP spec).
 * GPIO_CHIP  — the Linux device file that represents the Pi's GPIO hardware.
 * CONSUMER   — a name string attached to our GPIO requests; shows up in
 *             'gpioinfo' output so you can see what owns each line.
 */
#define N_SLAVES        6
#define N_CHANNELS      4
#define GPIO_CHIP       "/dev/gpiochip0"
#define CONSUMER        "fadec"

/*
 * DRDY_GPIO[]
 * ///////////
 * The GPIO pin number (BCM numbering) connected to each slave's DATA_RDY
 * output.  Index 0 = slave 1, index 1 = slave 2, etc.
 *
 * These pins are configured as INPUTS with a pull-down resistor enabled in
 * software.  The pull-down ensures that any pin NOT physically wired reads
 * LOW and never generates spurious "data ready" events.
 */
static const unsigned int DRDY_GPIO[N_SLAVES] = {
     2,   /* DRDY1 — GPIO  2  [PHYSICALLY WIRED] */
     3,   /* DRDY2 — GPIO  3                     */
    26,   /* DRDY3 — GPIO 26                     */
    23,   /* DRDY4 — GPIO 23                     */
    22,   /* DRDY5 — GPIO 22                     */
    12,   /* DRDY6 — GPIO 12                     */
};

/*
 * DRDY_LABEL[]
 * ////////////
 * Human-readable strings used in log output so you can see which slave
 * triggered an event.  The asterisk (*) marks the currently wired line.
 */
static const char * const DRDY_LABEL[N_SLAVES] = {
    "DRDY1(GPIO2)*",  /* slave 1 — wired */
    "DRDY2(GPIO3)",
    "DRDY3(GPIO26)",
    "DRDY4(GPIO23)",
    "DRDY5(GPIO22)",
    "DRDY6(GPIO12)",
};

/*
 * CS_GPIO[]
 * /////////
 * The GPIO pin number used as Chip Select for each slave.
 * CS is active-LOW: pulling it LOW selects the slave for SPI communication;
 * pulling it HIGH deselects it (idle state).
 *
 * A value of 0 means no GPIO has been assigned yet for that slave.
 * If the code sees CS_GPIO[i] == 0 it skips the SPI read for slave i and
 * only counts the DRDY edge in the statistics.
 */
static const unsigned int CS_GPIO[N_SLAVES] = {
    13,   /* CS1 — GPIO 13  [PHYSICALLY WIRED] */
    19,   /* CS2 — GPIO 19                     */
    16,   /* CS3 — GPIO 16                     */
    18,   /* CS4 — GPIO 18                     */
    20,   /* CS5 — GPIO 20                     */
    21,   /* CS6 — GPIO 21                     */
};

/*
 * RS-485 BUS CONTROL PINS
 * ///////////////////////
 * The ISL83491 RS-485 transceiver has two enable pins:
 *
 *   DE  (Driver Enable)    HIGH = driver active (A/B differential pair driven).
 *                          The Pi's SPI CLK is wired to DI, so DE must be HIGH
 *                          for the clock to reach the RP2350 over the bus.
 *
 *   /RE (Receiver Enable, active LOW)  LOW = Pi's receiver is active.
 *                          The RP2350 slave data output arrives at RO, so /RE
 *                          must stay LOW so the Pi can read the SPI MISO data.
 *
 * DE is held HIGH and /RE is held LOW at startup and never changed during the run.
 */
#define GPIO_DE   24u   /* RS-485 DE  pin — hold HIGH (Pi drives CLK via DI)  */
#define GPIO_REN  25u   /* RS-485 /RE pin — hold LOW  (receiver always on)    */

/*
 * SPI CONFIGURATION
 * /////////////////
 * SPI_DEVICE  — the Linux device file for SPI bus 0, chip-select 0.
 *               We use SPI_NO_CS to disable the kernel's built-in CS
 *               toggling and drive CS ourselves via GPIO.
 * SPI_HZ      — clock frequency for the RTDP transport.
 * FRAME_BYTES — each read fetches exactly 12 bytes: 4 channels × 3 bytes.
 * EVT_BATCH   — how many GPIO edge events to drain in one go per wakeup.
 *               64 is generous; in practice we rarely see more than 1-2
 *               at a time, but batching avoids extra syscall overhead.
 */
#define SPI_DEVICE   "/dev/spidev0.0"
#define SPI_HZ       RTDP_SPI_HZ /* shared with Pico RTDP firmware          */
#define FRAME_BYTES  13u         /* 1 sync header (0xEB) + 4 × 24-bit = 13 bytes */
#define CS_SETUP_DELAY_NS 5000u  /* CS low to first clock; shared-bus turnaround margin */
#define EVT_BATCH    64          /* edge events per batch read */


/* //////////////////////////////////////////////////////////////////////////
 * SHARED STATE BETWEEN THREADS
 * //////////////////////////////////////////////////////////////////////////
 *
 * These variables are accessed by both threads.  Access is safe because:
 *   - g_running uses volatile sig_atomic_t — the signal handler writes it
 *     and the main/acq threads read it; volatile prevents compiler caching.
 *   - g_ring uses lock-free atomic operations (see fadec.h).
 *   - g_event_count etc. use C11 atomics so individual increments are safe.
 */

/*
 * g_running
 * /////////
 * Set to 1 at startup.  The signal handler (Ctrl+C / SIGTERM) sets it to 0.
 * Both threads check this flag in their loops to know when to exit cleanly.
 * 'volatile' tells the compiler not to cache this in a register — the value
 * can change at any time from the signal handler running asynchronously.
 * 'sig_atomic_t' is guaranteed to be readable/writable atomically on this
 * platform, which is required for signal-handler safety.
 */
static volatile sig_atomic_t g_running = 1;

/*
 * g_ring
 * //////
 * The lock-free ring (circular) buffer.  acq_thread pushes frames in;
 * proc_thread pops them out.  Defined in fadec.h.
 * "Lock-free" means neither thread ever waits for the other — if the buffer
 * is full the producer drops the frame rather than blocking.
 */
static spsc_ring_t g_ring;

/*
 * DIAGNOSTIC COUNTERS
 * ///////////////////
 * Written by acq_thread, read (approximately) by proc_thread for printing.
 * We use C11 atomic types so that increments from acq_thread are visible to
 * proc_thread without needing a mutex.  'memory_order_relaxed' is used for
 * these counters because exact ordering does not matter — we only need each
 * increment to be atomic (indivisible), not to synchronise other memory.
 */
static atomic_uint_fast64_t g_event_count[N_SLAVES]; /* total DRDY rising edges seen  */
static atomic_uint_fast64_t g_frame_count[N_SLAVES]; /* frames successfully SPI-read  */
static atomic_uint_fast64_t g_err_count[N_SLAVES];   /* SPI ioctl errors              */
static atomic_uint_fast64_t g_hdr_err_count[N_SLAVES]; /* frames dropped: bad header byte */
static volatile uint8_t     g_last_bad_frame[N_SLAVES][FRAME_BYTES]; /* full last bad frame */

/*
 * g_start_ns
 * //////////
 * Monotonic timestamp (nanoseconds) captured just before the acquisition
 * thread is spawned.  Used in the final summary to compute the average
 * data rate over the entire run: frames / elapsed_seconds = Hz.
 * Written once in main(), read once after both threads have joined.
 * No atomics needed — the pthread_join() calls act as a full memory
 * barrier, guaranteeing main() sees the write before reading it again.
 */
static uint64_t g_start_ns;
static atomic_uint_fast64_t g_dropped;               /* frames dropped (ring was full)*/
static unstart_runtime_t g_unstart;


/* //////////////////////////////////////////////////////////////////////////
 * UTILITY FUNCTIONS
 * //////////////////////////////////////////////////////////////////////////
 */

/*
 * sig_handler()
 * /////////////
 * Called by the OS when the user presses Ctrl+C (SIGINT) or the process
 * receives SIGTERM.  Setting g_running = 0 tells all loops to exit cleanly.
 * The (void)s suppresses a compiler warning about the unused parameter.
 */
static void sig_handler(int s) { (void)s; g_running = 0; }

/*
 * now_ns()
 * ////////
 * Returns the current value of the monotonic clock in nanoseconds.
 * CLOCK_MONOTONIC never jumps backwards (unlike wall-clock time), making it
 * ideal for measuring intervals and timestamping events.
 * The return value is an unsigned 64-bit integer — it can hold ~585 years
 * of nanoseconds before overflowing.
 */
static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    /* tv_sec is whole seconds; tv_nsec is the sub-second remainder in ns.
     * Multiply seconds by 1,000,000,000 and add the nanoseconds part. */
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/* //////////////////////////////////////////////////////////////////////////
 * GPIO HELPER FUNCTIONS (libgpiod v2 API)
 * //////////////////////////////////////////////////////////////////////////
 *
 * libgpiod is the modern Linux API for controlling GPIO pins from userspace.
 * It replaces the old /sys/class/gpio interface.
 *
 * In libgpiod v2 every GPIO operation requires three objects:
 *   gpiod_line_settings   — what you want the pin to do (input/output/edge)
 *   gpiod_line_config     — which pins get those settings
 *   gpiod_request_config  — metadata (consumer name) for the request
 * After a successful request you get a gpiod_line_request handle to use
 * for reading/writing.  All three temporary objects are freed immediately
 * after the request is made.
 */

/*
 * request_output_line()
 * /////////////////////
 * Requests ownership of a single GPIO pin and configures it as an OUTPUT,
 * driven to 'init' (GPIOD_LINE_VALUE_ACTIVE = HIGH, _INACTIVE = LOW).
 *
 * Used for CS lines (initial HIGH = deasserted) and RS-485 control lines
 * (DE and /RE, initial LOW).
 *
 * Returns a line_request handle on success, NULL on failure (errno is set).
 */
static struct gpiod_line_request *request_output_line(struct gpiod_chip *chip,
                                                       unsigned int offset,
                                                       enum gpiod_line_value init)
{
    /* Allocate the three configuration objects */
    struct gpiod_line_settings  *ls  = gpiod_line_settings_new();
    struct gpiod_line_config    *lc  = gpiod_line_config_new();
    struct gpiod_request_config *rc  = gpiod_request_config_new();
    struct gpiod_line_request   *req = NULL;

    /* If any allocation failed, errno is already set to ENOMEM */
    if (!ls || !lc || !rc) { errno = ENOMEM; goto out; }

    /* Configure: direction = output, initial driven value = init */
    gpiod_line_settings_set_direction(ls, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(ls, init);

    /* Apply those settings to the one pin at 'offset' */
    gpiod_line_config_add_line_settings(lc, &offset, 1, ls);

    /* Tag the request with our consumer name (visible in 'gpioinfo') */
    gpiod_request_config_set_consumer(rc, CONSUMER);

    /* Make the actual kernel request — this is the syscall */
    req = gpiod_chip_request_lines(chip, rc, lc);

out:
    /* Always free the temporary config objects regardless of success/failure */
    if (ls) gpiod_line_settings_free(ls);
    if (lc) gpiod_line_config_free(lc);
    if (rc) gpiod_request_config_free(rc);
    return req; /* NULL on failure */
}

/*
 * request_drdy_lines()
 * ////////////////////
 * Requests all N_SLAVES DRDY pins at once as INPUTS with RISING-EDGE
 * detection and an internal PULL-DOWN resistor.
 *
 * RISING EDGE means the kernel records an event only when the pin
 * transitions from LOW to HIGH (0→1) — exactly when a slave signals
 * "new data ready".
 *
 * PULL-DOWN means any pin not physically wired to anything is held at
 * a known LOW level by an internal resistor, preventing random noise
 * from generating spurious "data ready" events.
 *
 * By requesting all 6 DRDY lines in a single call we get a single file
 * descriptor that wakes up when ANY of them fire — much cheaper than
 * polling 6 separate file descriptors with select/poll/epoll.
 */
static struct gpiod_line_request *request_drdy_lines(struct gpiod_chip *chip)
{
    struct gpiod_line_settings  *ls  = gpiod_line_settings_new();
    struct gpiod_line_config    *lc  = gpiod_line_config_new();
    struct gpiod_request_config *rc  = gpiod_request_config_new();
    struct gpiod_line_request   *req = NULL;

    if (!ls || !lc || !rc) { errno = ENOMEM; goto out; }

    gpiod_line_settings_set_direction(ls, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(ls, GPIOD_LINE_EDGE_RISING);
    gpiod_line_settings_set_bias(ls, GPIOD_LINE_BIAS_PULL_DOWN);

    /* Apply settings to all N_SLAVES pins listed in DRDY_GPIO[] */
    gpiod_line_config_add_line_settings(lc, DRDY_GPIO, N_SLAVES, ls);
    gpiod_request_config_set_consumer(rc, CONSUMER);

    req = gpiod_chip_request_lines(chip, rc, lc);
out:
    if (ls) gpiod_line_settings_free(ls);
    if (lc) gpiod_line_config_free(lc);
    if (rc) gpiod_request_config_free(rc);
    return req;
}


/* //////////////////////////////////////////////////////////////////////////
 * ACQUISITION THREAD — CORE 1, SCHED_FIFO PRIORITY 99
 * //////////////////////////////////////////////////////////////////////////
 *
 * This is the real-time "hot path" of the program.  Its only jobs are:
 *   1. Wait for a DRDY rising edge (blocking, zero CPU when idle).
 *   2. Assert CS for the slave that fired.
 *   3. Perform a 16-byte SPI read.
 *   4. Deassert CS.
 *   5. Decode the raw bytes into four int32_t channel values.
 *   6. Push the decoded frame into the ring buffer for proc_thread.
 *   7. Loop back to step 1.
 *
 * REAL-TIME DESIGN RULES FOLLOWED HERE
 * /////////////////////////////////////
 * - Pinned to core 1 (isolated from Linux) via pthread_setaffinity_np.
 * - SCHED_FIFO/99: the highest possible real-time priority.  Linux will
 *   never preempt this thread for any normal kernel activity on core 1.
 * - Stack prefaulted before the hot loop so the first iteration never
 *   takes a page-fault (page faults can stall execution for microseconds).
 * - mlockall(MCL_CURRENT) pins this thread's pages into RAM so the OS
 *   cannot swap them out.
 * - No dynamic memory allocation (malloc/free) inside the hot loop.
 * - SPI transfer structs pre-allocated once before the loop.
 */
static void *acq_thread(void *arg)
{
    (void)arg; /* unused — pthread_create requires this signature */

    /* //////////////////////////////////////////////////////////////////
     * STEP 1: CPU AFFINITY — PIN THIS THREAD TO CORE 1
     * //////////////////////////////////////////////////////////////////
     * cpu_set_t is a bitmask of CPU cores.  We clear it (CPU_ZERO),
     * then set bit 1 (CPU_SET(1,...)) meaning "allowed to run on core 1
     * only".  pthread_setaffinity_np applies the mask to this thread.
     * If it fails (e.g. core 1 doesn't exist), we warn and carry on.
     */
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0)
        perror("[acq] setaffinity (continuing)");

    /* //////////////////////////////////////////////////////////////////
     * STEP 2: REAL-TIME SCHEDULING — SCHED_FIFO PRIORITY 99
     * //////////////////////////////////////////////////////////////////
     * SCHED_FIFO is a real-time scheduling policy.  Once this thread is
     * running, Linux will not preempt it for any lower-priority task on
     * the same core.  Priority 99 is the maximum.
     * Requires root (sudo) or the CAP_SYS_NICE capability.
     * Without it, acquisition still works but may suffer occasional
     * scheduling jitter from the Linux kernel.
     */
    struct sched_param sp = { .sched_priority = 99 };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        perror("[acq] SCHED_FIFO/99 (run as root for RT; continuing without)");

    /* //////////////////////////////////////////////////////////////////
     * STEP 3: MEMORY LOCKING — PREVENT PAGE FAULTS IN THE HOT LOOP
     * //////////////////////////////////////////////////////////////////
     * First we touch every byte of a large stack array.  This forces the
     * OS to allocate real physical memory pages for the stack NOW, before
     * the hot loop starts.  Without this, the first access to deep stack
     * levels would trigger a "page fault" — a kernel interrupt that can
     * cost 10-100 µs of latency.
     *
     * Then mlockall(MCL_CURRENT) tells the OS to pin all of this thread's
     * currently mapped memory pages into RAM — they cannot be paged out
     * to disk while the program is running.  If the OS tried to swap a
     * page mid-loop, the resulting page fault would destroy timing.
     *
     * We call mlockall HERE (inside the thread) rather than in main()
     * because calling it in main with MCL_FUTURE causes pthread_create to
     * fail: it can't lock the new thread's 8 MB stack under the default
     * low per-process memory lock limit (RLIMIT_MEMLOCK).
     */
    uint8_t stack_prefault[65536];
    memset(stack_prefault, 0, sizeof(stack_prefault));
    if (mlockall(MCL_CURRENT) < 0)
        perror("[acq] mlockall (continuing -- may see latency spikes)");

    /* //////////////////////////////////////////////////////////////////
     * HARDWARE INITIALISATION
     * //////////////////////////////////////////////////////////////////
     * Declare handles for all the hardware resources we'll open.
     * Initialised to NULL / -1 so the cleanup label (acq_cleanup:) at the
     * bottom can safely check "did we open this?" before releasing it.
     */
    struct gpiod_chip              *chip     = NULL; /* GPIO chip handle     */
    struct gpiod_line_request      *drdy_req = NULL; /* DRDY input lines     */
    struct gpiod_line_request      *cs_req[N_SLAVES];/* CS output per slave  */
    struct gpiod_line_request      *de_req   = NULL; /* RS-485 DE line       */
    struct gpiod_line_request      *ren_req  = NULL; /* RS-485 /RE line      */
    struct gpiod_edge_event_buffer *evt_buf  = NULL; /* edge event buffer    */
    int                             spi_fd   = -1;   /* SPI device fd        */
    volatile uint32_t              *gpio_regs = NULL; /* BCM2711 register map */

    memset(cs_req, 0, sizeof(cs_req)); /* initialise all CS handles to NULL */

    /* OPEN THE GPIO CHIP
     * //////////////////
     * /dev/gpiochip0 is the main GPIO controller on the Pi 4B.
     * gpiod_chip_open() returns a handle we pass to all subsequent GPIO
     * requests.  If this fails, we can't control any GPIO at all.
     */
    chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip) {
        perror("gpiod_chip_open");
        goto acq_cleanup;
    }

    /* RS-485 DRIVER ENABLE (DE) — HOLD HIGH
     * ////////////////////////////////////////
     * The Pi's SPI CLK (GPIO 11) is wired to the ISL83491 DI pin.
     * DE must be HIGH to enable the line driver so the clock signal
     * is transmitted over the differential pair to the RP2350 slave.
     * GPIOD_LINE_VALUE_ACTIVE = logical HIGH on this output pin.
     */
    de_req = request_output_line(chip, GPIO_DE, GPIOD_LINE_VALUE_ACTIVE);
    if (!de_req) {
        fprintf(stderr, "Cannot request DE GPIO %u: %s\n",
                GPIO_DE, strerror(errno));
        goto acq_cleanup;
    }

    /* RS-485 RECEIVER ENABLE (/RE) — HOLD LOW
     * /////////////////////////////////////////
     * The /RE pin is active-LOW: LOW means the receiver is enabled.
     * We keep it LOW so the Pi can always receive data from the bus.
     */
    ren_req = request_output_line(chip, GPIO_REN, GPIOD_LINE_VALUE_INACTIVE);
    if (!ren_req) {
        fprintf(stderr, "Cannot request REN GPIO %u: %s\n",
                GPIO_REN, strerror(errno));
        goto acq_cleanup;
    }

    /* CHIP SELECT LINES — ONE PER SLAVE, INITIAL STATE HIGH (DEASSERTED)
     * ////////////////////////////////////////////////////////////////////
     * CS is active-LOW: we idle HIGH (deasserted = slave not selected).
     * We only request GPIO lines for slaves that have a CS pin assigned
     * (CS_GPIO[i] != 0).  Unassigned slaves will be DRDY-counted only.
     * GPIOD_LINE_VALUE_ACTIVE = logical HIGH on this output pin.
     */
    for (int i = 0; i < N_SLAVES; i++) {
        if (CS_GPIO[i] == 0) continue; /* skip unassigned slaves */
        cs_req[i] = request_output_line(chip, CS_GPIO[i],
                                        GPIOD_LINE_VALUE_ACTIVE);
        if (!cs_req[i]) {
            fprintf(stderr, "Cannot request CS%d GPIO %u: %s\n",
                    i + 1, CS_GPIO[i], strerror(errno));
            goto acq_cleanup;
        }
        printf("[acq] CS%d  GPIO %-2u  OK\n", i + 1, CS_GPIO[i]);
    }

    /* ALL 6 DRDY INPUT LINES — RISING EDGE, PULL-DOWN
     * /////////////////////////////////////////////////
     * A single request for all six lines.  The kernel will wake us up
     * whenever ANY of them transitions LOW→HIGH.
     */
    drdy_req = request_drdy_lines(chip);
    if (!drdy_req) {
        fprintf(stderr, "Cannot request DRDY lines: %s\n", strerror(errno));
        goto acq_cleanup;
    }

    /* evt_buf is declared for cleanup safety but is not used in the hot loop.
     * The mmap polling approach (see below) replaces the libgpiod event system. */

    /* SPI DEVICE INITIALISATION
     * //////////////////////////
     * Open the SPI device file.  On the Pi 4B with spidev enabled,
     * /dev/spidev0.0 = SPI bus 0, hardware CE0.
     *
     * We configure:
     *   SPI_MODE_3   — CPOL=1 (clock idle high), CPHA=1 (sample on 2nd edge)
     *   SPI_NO_CS    — disable kernel CE0 toggling; we drive CS via GPIO
     *   8 bits/word  — standard byte-oriented transfers
     *   10 MHz       — RP2350 RTDP maximum supported clock
     *
     * SPI_IOC_WR_MODE32 is used (not SPI_IOC_WR_MODE) because SPI_NO_CS
     * lives in bit 6 which doesn't fit in a uint8_t.  Using a uint32_t
     * avoids silent truncation of the flag.
     */
    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) {
        perror("open " SPI_DEVICE);
        goto acq_cleanup;
    }
    {
        uint32_t mode  = SPI_MODE_3 | SPI_NO_CS;
        uint8_t  bits  = 8;
        uint32_t speed = SPI_HZ;
        if (ioctl(spi_fd, SPI_IOC_WR_MODE32,        &mode)  < 0 ||
            ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits)  < 0 ||
            ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ,  &speed) < 0) {
            perror("SPI ioctl config");
            goto acq_cleanup;
        }
    }

    /* PRE-ALLOCATE SPI TRANSFER DESCRIPTORS
     * ///////////////////////////////////////
     * The Linux spidev kernel driver accepts an array of spi_ioc_transfer
     * structs via ioctl(SPI_IOC_MESSAGE).  Each struct describes one
     * transfer: where the TX and RX buffers are, how many bytes, speed, etc.
     *
     * We fill these in ONCE before the loop and reuse them every iteration.
     * Allocating inside the loop would call malloc on every DRDY event —
     * malloc can block briefly and has unpredictable latency.
     *
     * tx_zero[] is a static zero-filled buffer used as dummy TX data.
     * The slave ignores everything the Pi sends; we only care about RX.
     * 'static' means it lives in BSS (zero-initialised data segment),
     * not on the stack — important because FRAME_BYTES × N_SLAVES could
     * overflow a small stack if we had many slaves.
     */
    static const uint8_t tx_zero[FRAME_BYTES]; /* zero-initialised by default */
    uint8_t rx[N_SLAVES][FRAME_BYTES];          /* receive buffers, one per slave */
    struct spi_ioc_transfer xfer[N_SLAVES];
    memset(xfer, 0, sizeof(xfer));
    for (int i = 0; i < N_SLAVES; i++) {
        /* tx_buf and rx_buf are uint64_t in the kernel ABI (pointer-as-int) */
        xfer[i].tx_buf        = (uint64_t)(uintptr_t)tx_zero;
        xfer[i].rx_buf        = (uint64_t)(uintptr_t)rx[i];
        xfer[i].len           = FRAME_BYTES;   /* 13 bytes per read          */
        xfer[i].speed_hz      = SPI_HZ;        /* 10 MHz                     */
        xfer[i].bits_per_word = 8;             /* 8-bit words                */
        /* cs_change = 0: do not toggle CS between words (we handle it) */
    }

    printf("[acq] SPI %s  Mode3  %u Hz  %u B/frame\n[acq] DRDY lines: GPIO",
           SPI_DEVICE, SPI_HZ, FRAME_BYTES);
    for (int i = 0; i < N_SLAVES; i++)
        printf(" %u", DRDY_GPIO[i]);
    printf("\n[acq] waiting for DRDY events...\n");
    fflush(stdout); /* flush stdout so the message appears immediately */

    /* //////////////////////////////////////////////////////////////////
     * DIRECT GPIO REGISTER MAPPING — THE KEY TO HIGH-SPEED OPERATION
     * //////////////////////////////////////////////////////////////////
     *
     * WHY THE PREVIOUS LIBGPIOD APPROACH WAS SLOW (~800 Hz observed,
     * 32000 Hz expected):
     *
     * Problem 1 — CS toggling via gpiod_line_request_set_value():
     *   This function makes a write() syscall to /dev/gpiochip0.
     *   On Pi 4 each call takes ~50 µs.  With two calls per frame
     *   (assert + deassert) that alone caps throughput at ~10,000 Hz
     *   before the SPI transfer is even counted.
     *
     * Problem 2 — DRDY detection via gpiod edge events:
     *   gpiod_line_request_wait_edge_events() uses poll() on the GPIO
     *   character device.  The kernel's event FIFO has a small default
     *   capacity.  At 32 kHz it fills in ~500 µs.  If our loop takes
     *   longer than that, events are silently dropped — we were seeing
     *   only ~2,100 out of ~86,000 expected events (97.5% loss).
     *
     * FIX — direct memory-mapped register access:
     *   /dev/gpiomem exposes the BCM2711 GPIO hardware registers directly.
     *   Reading or writing a register is a single 32-bit AXI bus operation
     *   that takes < 1 ns — no syscall, no kernel round-trip.
     *
     *   We use three registers (all at byte offsets from the GPIO base,
     *   converted to uint32_t array indices by dividing by 4):
     *
     *   GPLEV0  (offset 0x34) — read all pin levels simultaneously
     *   GPSET0  (offset 0x1C) — write 1 to a bit to drive that pin HIGH
     *   GPCLR0  (offset 0x28) — write 1 to a bit to drive that pin LOW
     *
     *   For GPSET0/GPCLR0, bits you write 0 to are ignored (other pins
     *   are not affected), so we can safely set/clear individual pins
     *   with a single 32-bit write.
     *
     * COMPATIBILITY NOTE:
     *   We still use libgpiod to configure all GPIO directions and biases
     *   at startup (input with pull-down for DRDY, output for CS/DE/REN).
     *   The libgpiod handles are kept open so the kernel driver maintains
     *   those configurations.  We simply bypass the gpiod API for the
     *   actual hot-path read/write operations.
     *
     *   mmap offset must be 0 — /dev/gpiomem maps the GPIO register block
     *   starting from register 0; GPIO_BASE (0xFE200000) is a physical
     *   address, not a file offset, and must not be used here.
     */

    /* BCM2711 (Pi 4B) GPIO register word-offsets (byte_offset / 4) */
#define _GPLEV0  (0x34 / 4)   /* Level register:   read  pin states   */
#define _GPSET0  (0x1C / 4)   /* Set   register:   write 1 = pin HIGH */
#define _GPCLR0  (0x28 / 4)   /* Clear register:   write 1 = pin LOW  */

    {
        int mem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
        if (mem_fd < 0) {
            perror("open /dev/gpiomem");
            goto acq_cleanup;
        }
        gpio_regs = (volatile uint32_t *)mmap(NULL, 4096,
                        PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, 0);
        close(mem_fd);
        if (gpio_regs == MAP_FAILED) {
            perror("mmap /dev/gpiomem");
            gpio_regs = NULL;
            goto acq_cleanup;
        }
    }

    /* PRE-COMPUTE PER-SLAVE BITMASKS
     * /////////////////////////////////
     * For each slave, pre-compute a single uint32_t with exactly one bit
     * set — the bit corresponding to that slave's GPIO pin number.
     * Writing this mask to GPSET0 drives the pin HIGH; writing it to
     * GPCLR0 drives it LOW.  AND-ing it with GPLEV0 tests the pin level.
     * Pre-computing avoids repeated bit-shifts inside the hot loop.
     */
    uint32_t cs_bit[N_SLAVES];    /* CS GPIO bitmask per slave   */
    uint32_t drdy_bit[N_SLAVES];  /* DRDY GPIO bitmask per slave */
    for (int i = 0; i < N_SLAVES; i++) {
        /* CS_GPIO[i] == 0 means not assigned; leave bitmask as 0 */
        cs_bit[i]   = CS_GPIO[i] ? (1u << CS_GPIO[i]) : 0u;
        drdy_bit[i] = (1u << DRDY_GPIO[i]);
    }

    /* ENSURE ALL CS LINES START DEASSERTED (HIGH)
     * //////////////////////////////////////////////
     * Write all CS bits to GPSET0 in one register write.
     * The hardware ignores the 0-bits; only the 1-bits are affected.
     */
    {
        uint32_t cs_all = 0;
        for (int i = 0; i < N_SLAVES; i++) cs_all |= cs_bit[i];
        if (cs_all) gpio_regs[_GPSET0] = cs_all;
    }

    /* Snapshot the current DRDY register state so the edge-detect logic
     * has a valid "previous" value before the very first iteration. */
    uint32_t drdy_prev = gpio_regs[_GPLEV0];

    /* //////////////////////////////////////////////////////////////////
     * HOT LOOP — THIS RUNS CONTINUOUSLY UNTIL CTRL+C
     * //////////////////////////////////////////////////////////////////
     * Design goal: minimum latency between DRDY rising edge and SPI read.
     *
     * DRDY EDGE DETECTION — software edge detect on the raw register:
     *   1. Read GPLEV0 into drdy_now.
     *   2. XOR with drdy_prev: bits that CHANGED are set.
     *   3. AND with drdy_now: of the changed bits, only those now HIGH
     *      are rising edges (0 -> 1 transitions).
     *   4. Store drdy_now as drdy_prev for the next iteration.
     *
     * If no edges are detected we emit an ARM "yield" hint and spin.
     * "yield" (the ARM instruction, not sched_yield() the syscall) is
     * a micro-architectural hint that tells the CPU pipeline it is in
     * a spin-wait loop, allowing the processor to de-prioritise
     * speculative execution and save power — it does NOT give up the
     * core to the scheduler.  __asm__ volatile("yield") is used (not
     * plain asm) because -std=c11 does not expose the asm keyword, but
     * __asm__ is a GCC built-in always available regardless of -std.
     */
    uint8_t ready_ports[N_SLAVES];

    while (g_running) {
        uint32_t drdy_now  = gpio_regs[_GPLEV0];
        uint32_t new_edges = (drdy_now ^ drdy_prev) & drdy_now; /* rising edges */
        drdy_prev = drdy_now;

        if (!new_edges) {
            __asm__ volatile("yield"); /* ARM spin-wait hint, not a syscall */
            continue;
        }

        /* SERVICE READY SLAVES USING THE CURRENT MAJOR/MINOR-FRAME ORDER. */
        uint32_t ready_mask = 0;
        unsigned int ready_count;

        for (int dev = 0; dev < N_SLAVES; dev++) {
            if (new_edges & drdy_bit[dev])
                ready_mask |= (1u << dev);
        }

        ready_count = unstart_order_ready_ports(&g_unstart, now_ns(), ready_mask,
                                                ready_ports, N_SLAVES);

        for (unsigned int ready = 0; ready < ready_count; ready++) {
            int dev = (int)ready_ports[ready];

            /* Timestamp at the moment we detect the edge.
             * Using now_ns() here (CLOCK_MONOTONIC) rather than a
             * kernel event timestamp because we are no longer using
             * the kernel's event system. */
            uint64_t ts = now_ns();

            /* Count the DRDY edge for statistics */
            atomic_fetch_add_explicit(&g_event_count[dev], 1,
                                      memory_order_relaxed);

            /* No CS assigned for this slave — count edge and skip SPI */
            if (!cs_bit[dev]) continue;

            /* ASSERT CS — SINGLE 32-BIT REGISTER WRITE, ZERO SYSCALL
             * //////////////////////////////////////////////////////////
             * Writing cs_bit[dev] to GPCLR0 drives that GPIO LOW.
             * Other GPIO pins are unaffected (writing 0 has no effect).
             * This takes < 1 ns — contrast with ~50 µs for gpiod API. */
            gpio_regs[_GPCLR0] = cs_bit[dev];

            /* CS-SETUP DELAY — wait for RP2350 SPI receiver to enable
             * //////////////////////////////////////////////////////////
             * The RP2350 SPI slave needs ~1 µs after CS assertion before
             * it can drive MISO with the correct first byte.  Without this
             * delay the Pi clocks the first byte before the slave has loaded
             * its TX register, so byte 0 is undefined/stale — the 0xEB sync
             * header is never seen.
             * Busy-wait via now_ns() (vDSO clock_gettime — no syscall,
             * no scheduler involvement) to keep latency deterministic. */
            { uint64_t _t0 = now_ns(); while (now_ns() - _t0 < CS_SETUP_DELAY_NS) __asm__ volatile("nop"); }

            /* SPI READ — 13 BYTES
             * /////////////////////
             * ioctl is still a syscall (~15-20 µs kernel overhead) but
             * is unavoidable for userspace SPI.  The actual wire time
             * is 10.4 µs at 10 MHz (13 bytes x 8 bits / 10e6 bps). */
            int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer[dev]);

            /* DEASSERT CS — SINGLE 32-BIT REGISTER WRITE
             * /////////////////////////////////////////////
             * Done immediately after the transfer regardless of success. */
            gpio_regs[_GPSET0] = cs_bit[dev];

            if (ret < 0) {
                atomic_fetch_add_explicit(&g_err_count[dev], 1,
                                          memory_order_relaxed);
                continue;
            }

            /* VALIDATE SYNC HEADER AND DECODE 4 x 24-bit signed BIG-ENDIAN → int32_t
             * ////////////////////////////////////////////////////////////////////////
             * Byte 0 is the 0xEB sync header added by the RTDP firmware for
             * packet synchronisation.  Drop the frame if it is missing.
             * Channels occupy bytes [1 + c*3 .. 1 + c*3 + 2], MSB first.
             * Sign-extend bit 23 into bits 31-24.
             */
            if (rx[dev][0] != 0xEB) {
                /* Capture full frame so proc_thread can print it */
                for (unsigned int _b = 0; _b < FRAME_BYTES; _b++)
                    g_last_bad_frame[dev][_b] = rx[dev][_b];
                atomic_fetch_add_explicit(&g_hdr_err_count[dev], 1,
                                          memory_order_relaxed);
                continue;
            }
            sample_frame_t frame;
            frame.device_id    = (uint8_t)dev;
            frame.timestamp_ns = ts;
            memcpy(frame.raw, rx[dev], FRAME_BYTES);
            for (int c = 0; c < N_CHANNELS; c++) {
                const uint8_t *b = &rx[dev][1 + c * 3];
                int32_t raw = ((int32_t)b[0] << 16)
                            | ((int32_t)b[1] <<  8)
                            |  (int32_t)b[2];
                /* Sign-extend from 24-bit to 32-bit */
                if (raw & (1 << 23)) raw |= (int32_t)0xFF000000;
                frame.channels[c] = raw;
            }

            /* PUSH TO RING BUFFER — lock-free, no blocking */
            if (ring_push(&g_ring, &frame) < 0)
                atomic_fetch_add_explicit(&g_dropped, 1,
                                          memory_order_relaxed);
            else
                atomic_fetch_add_explicit(&g_frame_count[dev], 1,
                                          memory_order_relaxed);
        }
    } /* end while(g_running) */

    /* //////////////////////////////////////////////////////////////////
     * CLEANUP — RELEASE ALL HARDWARE RESOURCES
     * //////////////////////////////////////////////////////////////////
     * 'goto acq_cleanup' is used above so that error paths also reach
     * here and release everything that was successfully opened.
     * The checks (if (x)) prevent double-frees on partial initialisation.
     */
acq_cleanup:
    if (gpio_regs && gpio_regs != MAP_FAILED)
        munmap((void *)gpio_regs, 4096); /* release the memory-mapped GPIO region */
    if (spi_fd >= 0)  close(spi_fd);
    if (evt_buf)      gpiod_edge_event_buffer_free(evt_buf);
    if (drdy_req)     gpiod_line_request_release(drdy_req);
    for (int i = 0; i < N_SLAVES; i++)
        if (cs_req[i]) gpiod_line_request_release(cs_req[i]);
    if (de_req)  gpiod_line_request_release(de_req);
    if (ren_req) gpiod_line_request_release(ren_req);
    if (chip)    gpiod_chip_close(chip);
    return NULL;
}


/* //////////////////////////////////////////////////////////////////////////
 * PROCESSING THREAD — CORE 2, SCHED_FIFO PRIORITY 50
 * //////////////////////////////////////////////////////////////////////////
 *
 * This thread consumes frames from the ring buffer and handles all output:
 *   - Prints decoded channel values for every frame received.
 *   - Prints a per-slave frame rate summary once per second.
 *
 * It runs at a lower RT priority than acq_thread (50 vs 99) so that if
 * both cores are fully loaded, the acquisition thread wins.  In practice
 * they run on different cores (1 and 2) so priority rarely matters.
 *
 * The thread exits when g_running == 0 AND the ring buffer is empty,
 * ensuring all frames buffered before Ctrl+C are printed before quitting.
 */
static void *proc_thread(void *arg)
{
    (void)arg;

    /* PIN TO CORE 2 */
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0)
        perror("[proc] setaffinity (continuing)");

    /* REAL-TIME PRIORITY 50 — LOWER THAN ACQ (99) BUT STILL RT */
    struct sched_param sp = { .sched_priority = 50 };
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        perror("[proc] SCHED_FIFO/50 (run as root for RT; continuing without)");

    /* State for the 1-second rate report */
    uint64_t last_stats_ns = now_ns();
    uint64_t interval_frames[N_SLAVES]; /* frames received in current 1-s window */
    memset(interval_frames, 0, sizeof(interval_frames));
    /* Snapshots of cumulative acq_thread counters at the last report boundary.
     * Subtracting the snapshot from the current value gives the per-interval delta. */
    uint64_t prev_event[N_SLAVES];
    uint64_t prev_err[N_SLAVES];
    memset(prev_event, 0, sizeof(prev_event));
    memset(prev_err,   0, sizeof(prev_err));

    sample_frame_t frame; /* temporary storage for one popped frame */

    while (1) {

        /* TRY TO POP A FRAME FROM THE RING BUFFER
         * //////////////////////////////////////////
         * ring_pop() is non-blocking and lock-free.
         * Returns 0 on success (frame is filled), -1 if the ring is empty.
         */
        if (ring_pop(&g_ring, &frame) < 0) {

            /* Ring is empty — check if it's time to shut down */
            if (!g_running) break; /* acq_thread has stopped and ring is drained */

            /* ONCE-PER-SECOND RATE REPORT
             * /////////////////////////////
             * Check elapsed time.  If a full second has passed, print
             * how many frames per slave we processed in that interval.
             */
            uint64_t now = now_ns();
            if (now - last_stats_ns >= 1000000000ULL) { /* 1,000,000,000 ns = 1 s */
                double dt = (double)(now - last_stats_ns) / 1e9; /* actual elapsed seconds */
                last_stats_ns = now;

                int any = 0; /* did any slave send data this interval? */
                for (int i = 0; i < N_SLAVES; i++) {
                    /* Sample cumulative acq_thread counters and compute deltas */
                    uint64_t ev      = atomic_load_explicit(&g_event_count[i],   memory_order_relaxed);
                    uint64_t spi_err = atomic_load_explicit(&g_err_count[i],     memory_order_relaxed);
                    uint64_t hdr_err = atomic_load_explicit(&g_hdr_err_count[i], memory_order_relaxed);
                    uint64_t d_ev      = ev      - prev_event[i];
                    uint64_t d_spi_err = spi_err - prev_err[i];
                    prev_event[i] = ev;
                    prev_err[i]   = spi_err;

                    if (interval_frames[i] || d_ev) {
                        if (!any) printf("[rate] ");
                        if (hdr_err > 0) {
                            printf("slave%d: %.0f fr/s  evts=%llu spi_errs=%llu hdr_errs=%llu\n",
                                   i + 1, (double)interval_frames[i] / dt,
                                   (unsigned long long)d_ev,
                                   (unsigned long long)d_spi_err,
                                   (unsigned long long)hdr_err);
                            printf("  last_bad_frame:");
                            for (unsigned int _b = 0; _b < FRAME_BYTES; _b++)
                                printf(" %02X", (unsigned)g_last_bad_frame[i][_b]);
                            printf("\n");
                        } else
                            printf("slave%d: %.0f fr/s  evts=%llu spi_errs=%llu  ",
                                   i + 1, (double)interval_frames[i] / dt,
                                   (unsigned long long)d_ev,
                                   (unsigned long long)d_spi_err);
                        any = 1;
                    }
                }
                /* Print "no activity" if nothing came in, so the user
                 * knows the program is still running. */
                printf(any ? "\n" : "[rate] no DRDY activity\n");
                fflush(stdout);
                memset(interval_frames, 0, sizeof(interval_frames));
            }

            /* YIELD THE CPU BRIEFLY
             * ///////////////////////
             * The ring is empty so there is nothing to do right now.
             * sched_yield() asks the scheduler to run any other runnable
             * thread before returning here.  This prevents this thread
             * from spinning at 100% CPU while waiting for the next frame.
             * On an isolated core with no other runnable threads it
             * returns immediately, which is fine.
             */
            sched_yield();
            continue;
        }

        /* WE HAVE A FRAME — UPDATE STATISTICS AND PRINT IT */
        int dev = (int)frame.device_id;

        unstart_on_frame(&g_unstart, &frame);

        if (dev >= 0 && dev < N_SLAVES)
            interval_frames[dev]++; /* count for rate report */

        /*
         * Print raw bytes for debugging.
         * Format:  [<timestamp ns>] <DRDY label>  raw: EB 00 27 0F ...
         */
        const char *label = (dev >= 0 && dev < N_SLAVES) ? DRDY_LABEL[dev] : "?";
        printf("[%llu ns] %-22s  raw:",
               (unsigned long long)frame.timestamp_ns, label);
        for (unsigned int b = 0; b < FRAME_BYTES; b++)
            printf(" %02X", frame.raw[b]);
        printf("\n");
        /* Note: no fflush here — stdio buffers lines automatically when
         * stdout is a terminal.  Add fflush(stdout) if piping to a file. */
    }

    return NULL;
}


/* //////////////////////////////////////////////////////////////////////////
 * MAIN — PROGRAM ENTRY POINT
 * //////////////////////////////////////////////////////////////////////////
 *
 * main() does very little itself:
 *   1. Installs signal handlers for clean shutdown.
 *   2. Initialises shared data structures.
 *   3. Spawns the two worker threads.
 *   4. Sleeps in a light wait loop until shutdown is requested.
 *   5. Joins both threads to wait for them to finish.
 *   6. Prints a final summary.
 *
 * All real work happens in acq_thread and proc_thread.
 */
int main(void)
{
    /* INSTALL SIGNAL HANDLERS
     * ////////////////////////
     * When the user presses Ctrl+C the OS sends SIGINT.
     * When the system or another process asks us to quit it sends SIGTERM.
     * sig_handler() sets g_running = 0, which both threads check in their
     * loops to initiate a clean shutdown.
     */
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    /* INITIALISE SHARED DATA
     * ///////////////////////
     * Zero the ring buffer struct (sets head and tail counters to 0,
     * meaning the ring is empty).
     * Initialise all atomic counters to zero using atomic_init(), which
     * is the correct portable way to initialise C11 atomics (as opposed
     * to just assigning 0, which is not guaranteed to be atomic-safe on
     * all compilers).
     */
    memset(&g_ring, 0, sizeof(g_ring));
    for (int i = 0; i < N_SLAVES; i++) {
        atomic_init(&g_event_count[i],   0);
        atomic_init(&g_frame_count[i],   0);
        atomic_init(&g_err_count[i],     0);
        atomic_init(&g_hdr_err_count[i], 0);
        memset((void *)g_last_bad_frame[i], 0, FRAME_BYTES);
    }
    atomic_init(&g_dropped, 0);

    unstart_config_t unstart_cfg = {
        .port_count = N_SLAVES,
        .spi_hz = SPI_HZ,
        .frame_bytes = FRAME_BYTES,
        .cs_setup_delay_ns = CS_SETUP_DELAY_NS,
    };

    unstart_init(&g_unstart, &unstart_cfg, now_ns());

    printf("FADEC RTDP master\n");
    printf("cores: acq=1 (FIFO/99)  proc=2 (FIFO/50)  unstart=3\n\n");

    /* SPAWN THE TWO WORKER THREADS
     * //////////////////////////////
     * pthread_create() starts a thread running the given function.
     * We store the thread handle (pthread_t) so we can join it later.
     *
     * proc_thread is started first so it is ready to consume frames
     * before acq_thread starts producing them.  Order matters here:
     * if acq_thread started first and immediately filled the ring,
     * proc_thread would drop frames during its startup phase.
     */
    pthread_t t_acq, t_proc, t_unstart;
    int unstart_started = 0;
    unstart_thread_args_t unstart_args = {
        .runtime = &g_unstart,
        .running = &g_running,
    };

    /* Stamp the start time immediately before spawning threads so the
     * elapsed-time denominator in the final rate calculation is accurate. */
    g_start_ns = now_ns();

    if (pthread_create(&t_proc, NULL, proc_thread, NULL) != 0) {
        perror("pthread_create proc");
        return EXIT_FAILURE;
    }
    if (pthread_create(&t_unstart, NULL, unstart_thread, &unstart_args) != 0) {
        perror("pthread_create unstart (continuing with fixed schedule)");
    } else {
        unstart_started = 1;
    }
    if (pthread_create(&t_acq, NULL, acq_thread, NULL) != 0) {
        perror("pthread_create acq");
        /* Signal proc_thread to exit since acq_thread won't start */
        g_running = 0;
        if (unstart_started)
            pthread_join(t_unstart, NULL);
        pthread_join(t_proc, NULL);
        return EXIT_FAILURE;
    }

    /* WAIT FOR SHUTDOWN
     * //////////////////
     * Signals still request shutdown via sig_handler(), but unstart may also
     * stop the program after its timed capture window. A short sleep loop lets
     * main observe either path without touching the acquisition hot loop.
     */
    while (g_running) {
        struct timespec pause = { .tv_sec = 0, .tv_nsec = 50000000L };
        nanosleep(&pause, NULL);
    }

    /* INITIATE SHUTDOWN AND WAIT FOR BOTH THREADS
     * /////////////////////////////////////////////
     * Set g_running = 0 again (redundant but explicit) so threads that
     * might have missed the signal handler's write see it here.
     * pthread_join() blocks until the named thread returns from its
     * function — this guarantees all cleanup code in those threads
     * has completed before we print the summary.
     */
    g_running = 0;
    pthread_join(t_acq,  NULL);
    pthread_join(t_proc, NULL);
    if (unstart_started)
        pthread_join(t_unstart, NULL);

    /* FINAL SUMMARY
     * //////////////
     * Compute the total elapsed run time and use it to derive the average
     * data rate for each slave:
     *
     *   rate (Hz) = frames_received / elapsed_seconds
     *
     * We report both raw Hz and kHz for readability.  At 32 kHz you would
     * expect to see approximately 32000 Hz / 32.0 kHz per active slave.
     *
     * Note: elapsed time starts from just before the threads were spawned,
     * so the first few frames during thread startup are included, which
     * makes the average very slightly lower than the true steady-state rate.
     * For a multi-second run the difference is negligible.
     */
    double elapsed_s = (double)(now_ns() - g_start_ns) / 1e9;

    printf("\n-- Summary (elapsed: %.3f s) ----------------------------------\n",
           elapsed_s);
    printf("  %-22s  %8s  %8s  %8s  %8s  %8s\n",
           "Slave", "DRDY evts", "Frames", "Errors",
           "Avg rate", "(kHz)");
    printf("  %-22s  %8s  %8s  %8s  %8s  %8s\n",
           "----------------------",
           "--------", "--------", "--------",
           "--------", "--------");
    for (int i = 0; i < N_SLAVES; i++) {
        uint64_t frames = atomic_load(&g_frame_count[i]);
        /* Avoid division by zero on a zero-length run */
        double rate_hz  = (elapsed_s > 0.0) ? (double)frames / elapsed_s : 0.0;
        double rate_khz = rate_hz / 1000.0;
        printf("  %-22s  %8llu  %8llu  %8llu  %7.1f Hz  %6.3f kHz\n",
               DRDY_LABEL[i],
               (unsigned long long)atomic_load(&g_event_count[i]),
               (unsigned long long)frames,
               (unsigned long long)atomic_load(&g_err_count[i]),
               rate_hz, rate_khz);
        uint64_t hdr_errs = atomic_load(&g_hdr_err_count[i]);
        if (hdr_errs) {
            printf("    hdr_errs: %llu, last bad frame bytes:\n    ",
                   (unsigned long long)hdr_errs);
            for (unsigned int _b = 0; _b < FRAME_BYTES; _b++)
                printf(" %02X", (unsigned)g_last_bad_frame[i][_b]);
            printf("\n");
        }
    }
    /* g_dropped counts frames the ring buffer was too full to accept */
    printf("  %-22s  dropped (ring full): %llu\n",
           "", (unsigned long long)atomic_load(&g_dropped));

    {
        int unstart_save = unstart_save_timing_log(&g_unstart);
        if (unstart_save < 0)
            perror("[unstart] save timing log");
        else if (unstart_save > 0)
            printf("[unstart] timing log saved to %s\n", UNSTART_TIMING_LOG_PATH);
    }

    return EXIT_SUCCESS;
}
