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
 * 3. The Pi then clocks 16 bytes out of the SPI bus.  The slave puts the
 *    data on MISO; the Pi ignores MOSI (we send zeros).
 * 4. After the transfer the Pi releases CS (back to HIGH) so the bus is
 *    free for the next slave.
 * 5. The 16 bytes contain four 32-bit signed integers (big-endian byte
 *    order, meaning the most-significant byte comes first).
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
 *   DE       GPIO 24   OUTPUT    RS-485 driver enable (HIGH=transmit)
 *                                Keep LOW — Pi only receives, never transmits
 *   REN      GPIO 25   OUTPUT    RS-485 receiver enable (active LOW)
 *                                Keep LOW — receiver is always on
 *
 * SPI PROTOCOL DETAILS
 * ////////////////////
 *   - Mode 3: clock idles HIGH (CPOL=1), data sampled on falling edge (CPHA=1)
 *   - Maximum clock speed: 10 MHz (supported by RP2350 RTDP firmware)
 *   - Frame size: 16 bytes = 4 channels x 4 bytes (int32_t, big-endian)
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
 *   DE  (Driver Enable)    HIGH = Pi is transmitting on the bus.
 *                          We NEVER transmit, so we keep this LOW.
 *
 *   /RE (Receiver Enable, active LOW)  LOW = Pi's receiver is active.
 *                          We ALWAYS want to receive, so we keep this LOW.
 *
 * Both pins are set LOW at startup and never changed during the run.
 */
#define GPIO_DE   24u   /* RS-485 DE  pin — keep LOW (Pi never drives bus) */
#define GPIO_REN  25u   /* RS-485 /RE pin — keep LOW (receiver always on)  */

/*
 * SPI CONFIGURATION
 * /////////////////
 * SPI_DEVICE  — the Linux device file for SPI bus 0, chip-select 0.
 *               We use SPI_NO_CS to disable the kernel's built-in CS
 *               toggling and drive CS ourselves via GPIO.
 * SPI_HZ      — clock frequency.  The RP2350 RTDP firmware supports 10 MHz.
 * FRAME_BYTES — each read fetches exactly 16 bytes: 4 channels × 4 bytes.
 * EVT_BATCH   — how many GPIO edge events to drain in one go per wakeup.
 *               64 is generous; in practice we rarely see more than 1-2
 *               at a time, but batching avoids extra syscall overhead.
 */
#define SPI_DEVICE   "/dev/spidev0.0"
#define SPI_HZ       10000000u   /* 10 MHz                     */
#define FRAME_BYTES  16u         /* 4 × int32_t = 16 bytes     */
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
static atomic_uint_fast64_t g_dropped;               /* frames dropped (ring was full)*/


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

/*
 * gpio_to_slave()
 * ///////////////
 * Given a GPIO pin number from an edge event, return the 0-based slave index
 * (0 = slave 1, 1 = slave 2, …).  Returns -1 if the GPIO is not in our
 * DRDY_GPIO table (should never happen, but we handle it defensively).
 */
static int gpio_to_slave(unsigned int gpio)
{
    for (int i = 0; i < N_SLAVES; i++)
        if (DRDY_GPIO[i] == gpio) return i;
    return -1; /* unknown GPIO — ignore */
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

    /* RS-485 DRIVER ENABLE (DE) — HOLD LOW
     * //////////////////////////////////////
     * The ISL83491 transceiver's DE pin must be LOW to keep the Pi in
     * receive-only mode.  We set it LOW at startup and never change it.
     * GPIOD_LINE_VALUE_INACTIVE = logical LOW on this output pin.
     */
    de_req = request_output_line(chip, GPIO_DE, GPIOD_LINE_VALUE_INACTIVE);
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

    /* EDGE EVENT BUFFER
     * //////////////////
     * Pre-allocate a buffer that can hold up to EVT_BATCH edge events.
     * We pass this to gpiod_line_request_read_edge_events() in the loop
     * to drain all pending events in one call without allocating memory.
     */
    evt_buf = gpiod_edge_event_buffer_new(EVT_BATCH);
    if (!evt_buf) {
        fprintf(stderr, "Cannot allocate event buffer\n");
        goto acq_cleanup;
    }

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
        xfer[i].len           = FRAME_BYTES;   /* 16 bytes per read          */
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
     * HOT LOOP — THIS RUNS CONTINUOUSLY UNTIL CTRL+C
     * //////////////////////////////////////////////////////////////////
     * Design goal: minimum latency between DRDY rising edge and SPI read.
     * Every operation here has been chosen to avoid syscalls, allocations,
     * and unpredictable delays.
     */
    while (g_running) {

        /* WAIT FOR A DRDY EDGE (BLOCKING)
         * /////////////////////////////////
         * gpiod_line_request_wait_edge_events() sleeps (yields the CPU)
         * until at least one edge event is pending on any of the 6 DRDY
         * lines, OR until the 1-second timeout expires.
         *
         * The 1-second timeout lets us re-check g_running so the thread
         * exits cleanly after Ctrl+C even if no slave is connected.
         *
         * Return values:
         *   > 0  at least one event is ready to read
         *   = 0  timeout expired with no events
         *   < 0  error (EINTR = interrupted by a signal, e.g. Ctrl+C)
         */
        int r = gpiod_line_request_wait_edge_events(drdy_req, 1000000000LL);
        if (r < 0) {
            if (errno == EINTR) continue; /* signal received — loop and check g_running */
            perror("wait_edge_events");
            break; /* unexpected error — exit thread */
        }
        if (r == 0) continue; /* timeout — loop and check g_running */

        /* DRAIN ALL PENDING EDGE EVENTS IN ONE BATCH
         * ////////////////////////////////////////////
         * After the wait returns > 0 there may be more than one event
         * queued (e.g. two slaves fired almost simultaneously).
         * Read up to EVT_BATCH events at once into the pre-allocated buffer.
         */
        int n = gpiod_line_request_read_edge_events(drdy_req, evt_buf,
                                                    EVT_BATCH);
        if (n < 0) { perror("read_edge_events"); continue; }

        /* PROCESS EACH EVENT */
        for (int ei = 0; ei < n; ei++) {

            /*
             * Get a pointer to the ei-th event in the buffer.
             * Cast away const because the installed libgpiod headers on
             * this system declare the getter functions without 'const' on
             * the parameter, so passing a const pointer produces a
             * -Wdiscarded-qualifiers warning.  The data is not modified.
             */
            struct gpiod_edge_event *ev =
                (struct gpiod_edge_event *)
                gpiod_edge_event_buffer_get_event(evt_buf, (unsigned long)ei);

            /*
             * Extract the GPIO line offset (pin number) that fired and
             * the hardware timestamp in nanoseconds (captured by the
             * kernel at the moment the edge was detected — more accurate
             * than calling clock_gettime() here).
             */
            unsigned int gpio = gpiod_edge_event_get_line_offset(ev);
            uint64_t     ts   = gpiod_edge_event_get_timestamp_ns(ev);

            /* Convert GPIO number to slave index (0-based) */
            int dev = gpio_to_slave(gpio);
            if (dev < 0) continue; /* unknown GPIO, should never happen */

            /* Count the DRDY edge regardless of whether we SPI-read it */
            atomic_fetch_add_explicit(&g_event_count[dev], 1,
                                      memory_order_relaxed);

            /* If this slave has no CS pin assigned, we cannot do an SPI
             * read — just record the event and move on. */
            if (cs_req[dev] == NULL) continue;

            /* ASSERT CS — SELECT THE SLAVE (DRIVE LOW)
             * //////////////////////////////////////////
             * GPIOD_LINE_VALUE_INACTIVE = LOW on an active-low CS pin.
             * The slave sees CS go LOW and prepares to clock out its frame.
             */
            gpiod_line_request_set_value(cs_req[dev], CS_GPIO[dev],
                                         GPIOD_LINE_VALUE_INACTIVE);

            /* SPI READ — 16 BYTES FROM THE SLAVE
             * /////////////////////////////////////
             * ioctl(SPI_IOC_MESSAGE(1)) sends and receives one spi_ioc_transfer.
             * The slave drives MISO with 16 bytes of ADC data while we clock
             * 16 zero bytes out on MOSI (which the slave ignores).
             * At 10 MHz this transfer takes approximately 12.8 µs.
             */
            int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer[dev]);

            /* DEASSERT CS — RELEASE THE SLAVE (DRIVE HIGH)
             * ///////////////////////////////////////////////
             * Done immediately after the transfer regardless of success/failure.
             * Leaving CS asserted would lock the bus and block other slaves.
             */
            gpiod_line_request_set_value(cs_req[dev], CS_GPIO[dev],
                                         GPIOD_LINE_VALUE_ACTIVE);

            if (ret < 0) {
                /* SPI transfer failed — count the error, skip this frame */
                atomic_fetch_add_explicit(&g_err_count[dev], 1,
                                          memory_order_relaxed);
                continue;
            }

            /* DECODE 4 × int32_t BIG-ENDIAN FROM rx[dev][]
             * ///////////////////////////////////////////////
             * The slave sends data in big-endian byte order (most significant
             * byte first).  Standard x86/ARM processors use little-endian
             * (least significant byte first), so we must swap manually.
             *
             * For channel c, the 4 bytes at positions [c*4 .. c*4+3] are:
             *   b[0] = bits 31-24 (most significant)
             *   b[1] = bits 23-16
             *   b[2] = bits 15- 8
             *   b[3] = bits  7- 0 (least significant)
             *
             * We shift and OR them into a uint32_t then cast to int32_t
             * to preserve the sign bit for negative values.
             */
            sample_frame_t frame;
            frame.device_id    = (uint8_t)dev;
            frame.timestamp_ns = ts; /* kernel-captured hardware timestamp */
            for (int c = 0; c < N_CHANNELS; c++) {
                const uint8_t *b = &rx[dev][c * 4];
                frame.channels[c] = (int32_t)(  ((uint32_t)b[0] << 24)
                                               | ((uint32_t)b[1] << 16)
                                               | ((uint32_t)b[2] <<  8)
                                               |  (uint32_t)b[3]);
            }

            /* PUSH THE DECODED FRAME INTO THE RING BUFFER
             * //////////////////////////////////////////////
             * ring_push() is lock-free (no mutex).  If the ring is full
             * (proc_thread is not keeping up), the frame is dropped and
             * g_dropped is incremented so we can diagnose it later.
             */
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
                    if (interval_frames[i]) {
                        if (!any) printf("[rate] ");
                        printf("slave%d: %.0f fr/s  ",
                               i + 1, (double)interval_frames[i] / dt);
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
        if (dev >= 0 && dev < N_SLAVES)
            interval_frames[dev]++; /* count for rate report */

        /*
         * Print the decoded frame.  Format:
         *   [<timestamp ns>] <DRDY label>  ch0=<val> ch1=<val> ch2=<val> ch3=<val>
         *
         * %-22s: left-align the label in a 22-character field so columns line up.
         * %-11d: left-align each channel value in an 11-character field.
         * %llu: unsigned long long — required for uint64_t with printf on Linux.
         */
        printf("[%llu ns] %-22s"
               "  ch0=%-11d ch1=%-11d ch2=%-11d ch3=%-11d\n",
               (unsigned long long)frame.timestamp_ns,
               (dev >= 0 && dev < N_SLAVES) ? DRDY_LABEL[dev] : "?",
               frame.channels[0], frame.channels[1],
               frame.channels[2], frame.channels[3]);
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
 *   4. Sleeps (pause()) until Ctrl+C.
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
        atomic_init(&g_event_count[i], 0);
        atomic_init(&g_frame_count[i], 0);
        atomic_init(&g_err_count[i],   0);
    }
    atomic_init(&g_dropped, 0);

    printf("FADEC RTDP master\n");
    printf("cores: acq=1 (FIFO/99)  proc=2 (FIFO/50)\n\n");

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
    pthread_t t_acq, t_proc;

    if (pthread_create(&t_proc, NULL, proc_thread, NULL) != 0) {
        perror("pthread_create proc");
        return EXIT_FAILURE;
    }
    if (pthread_create(&t_acq, NULL, acq_thread, NULL) != 0) {
        perror("pthread_create acq");
        /* Signal proc_thread to exit since acq_thread won't start */
        g_running = 0;
        pthread_join(t_proc, NULL);
        return EXIT_FAILURE;
    }

    /* WAIT FOR CTRL+C / SIGTERM
     * //////////////////////////
     * pause() suspends the calling thread until any signal is received.
     * When sig_handler() fires it sets g_running = 0 and returns.
     * pause() then returns (with errno == EINTR) and the while condition
     * is checked — if g_running is now 0, we fall through to cleanup.
     */
    while (g_running) pause();

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

    /* FINAL SUMMARY
     * //////////////
     * Read and print all atomic counters.
     * atomic_load() is used (not direct read) to ensure we see the final
     * values written by the now-stopped threads.
     */
    printf("\n-- Summary ---------------------------------------------------\n");
    for (int i = 0; i < N_SLAVES; i++) {
        printf("  %-22s  events: %-8llu  frames: %-8llu  errors: %llu\n",
               DRDY_LABEL[i],
               (unsigned long long)atomic_load(&g_event_count[i]),
               (unsigned long long)atomic_load(&g_frame_count[i]),
               (unsigned long long)atomic_load(&g_err_count[i]));
    }
    /* g_dropped counts frames the ring buffer was too full to accept */
    printf("  dropped (ring full): %llu\n",
           (unsigned long long)atomic_load(&g_dropped));

    return EXIT_SUCCESS;
}
