/*
 * myact_ctl.c  -  Minimal IgH EtherCAT master application for MyActuator
 *                 RMD-X (EtherCAT "E" variants) on the Cerebellum board.
 *
 * Modes (CiA 402 cyclic synchronous, as documented by MyActuator):
 *   csp-hold  : hold the position the motor had when enabled
 *   csp-sine  : smooth 0 -> amp -> 0 cosine move, repeated (position)
 *   csv       : ramp to a constant velocity (counts/s)
 *   cst       : ramp to a constant torque   (per-mille of rated current)
 *
 * Safety features: everything is armed with zero/current-position targets,
 * ramped, limit-checked, and ramped down on Ctrl-C.  Use --monitor first.
 *
 * Build:  make            (see Makefile)
 * Run  :  sudo ./myact_ctl --monitor
 *         sudo ./myact_ctl --mode csp-sine --amp 5000 --period 4
 *
 * CHANGES vs. previous version:
 *   - 0x6072 (Max Torque) is part of the drive's default RxPDO 0x1600.
 *     It is now registered and written every cycle (= --tq-limit, or 0 in
 *     --monitor).  Previously it stayed 0 in the process image.
 *   - 0x603F (Error Code) is read (optional) and shown in the monitor.
 *   - Mode-of-operation is not written in --monitor mode (strictly passive).
 *   - --max-tq-sdo removed (PDO value would overwrite it every cycle).
 *
 * ASSUMPTIONS you must verify on your drive (see bringup.sh):
 *   - the default PDO mapping contains the entries listed in ENT[] below
 *     (otherwise build with -DUSE_CUSTOM_PDO and check 0x1600/0x1A00)
 *   - DC AssignActivate (--dc-assign) matches the ESI file
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <signal.h>
#include <sched.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include <stdatomic.h>
#include <sys/mman.h>
#include <ecrt.h>

#define MAX_AX        16
#define NSEC          1000000000LL
#define COUNTS_PER_REV 131072.0   /* 17-bit output encoder (verify!) */

/* ---------------- CiA 402 constants ---------------- */
#define CW_SHUTDOWN   0x0006
#define CW_SWITCH_ON  0x0007
#define CW_ENABLE     0x000F
#define CW_FAULT_RST  0x0080
#define SW_FAULT      0x0008

enum { OPM_CSP = 8, OPM_CSV = 9, OPM_CST = 10 };

/* ---------------- PDO entries we want ---------------- */
enum { E_CW, E_MODE, E_TPOS, E_TVEL, E_TTQ, E_MAXTQ,
       E_SW, E_MODED, E_APOS, E_AVEL, E_ATQ, E_ERR, E_N };

static const struct { uint16_t idx; uint8_t sub; const char *name; } ENT[E_N] = {
    {0x6040, 0, "controlword"},     {0x6060, 0, "mode_of_operation"},
    {0x607A, 0, "target_position"}, {0x60FF, 0, "target_velocity"},
    {0x6071, 0, "target_torque"},   {0x6072, 0, "max_torque"},
    {0x6041, 0, "statusword"},      {0x6061, 0, "mode_display"},
    {0x6064, 0, "actual_position"}, {0x606C, 0, "actual_velocity"},
    {0x6077, 0, "actual_torque"},   {0x603F, 0, "error_code"},
};

#ifdef USE_CUSTOM_PDO
/* Only use if the drive really supports configurable PDOs at 0x1600/0x1A00.
 * Mirrors the drive's default mapping (order matters for the drive).
 * Verify with:  ethercat sdos -p 0  |  grep -E "0x1600|0x1A00|0x1C12|0x1C13" */
static ec_pdo_entry_info_t rx_entries[] = {
    {0x6040, 0, 16}, {0x607A, 0, 32}, {0x60FF, 0, 32}, {0x6071, 0, 16},
    {0x6072, 0, 16}, {0x6060, 0, 8},  {0x2FFD, 0, 8}};
static ec_pdo_entry_info_t tx_entries[] = {
    {0x6041, 0, 16}, {0x6064, 0, 32}, {0x606C, 0, 32}, {0x6077, 0, 16},
    {0x603F, 0, 16}, {0x6061, 0, 8},  {0x2FFE, 0, 8}};
static ec_pdo_info_t pdos[] = {{0x1600, 7, rx_entries}, {0x1A00, 7, tx_entries}};
static ec_sync_info_t syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, pdos + 1, EC_WD_DISABLE},
    {0xff}};
#endif

/* ---------------- per-axis value lists ("a" or "a,b,c") ---------------- */
typedef struct { double v[MAX_AX]; int n; } list_t;

static void parse_list(list_t *l, const char *s)
{
    char buf[256];
    strncpy(buf, s, sizeof buf - 1); buf[sizeof buf - 1] = 0;
    l->n = 0;
    for (char *tok = strtok(buf, ","); tok && l->n < MAX_AX; tok = strtok(NULL, ","))
        l->v[l->n++] = atof(tok);
    if (l->n == 0) { l->v[0] = 0; l->n = 1; }
}
static double lget(const list_t *l, int i) { return l->n == 1 ? l->v[0] : l->v[i]; }

/* ---------------- configuration (command line) ---------------- */
typedef enum { PRF_HOLD, PRF_SINE, PRF_MOVE } prf_t;
static struct {
    int      mode_n;          /* number of --mode entries (0 = default csp-hold) */
    int      m_op[MAX_AX];    /* 8/9/10, per entry */
    prf_t    m_prf[MAX_AX];
    int      monitor;         /* no enable, only observe */
    int      nslaves;         /* 0 = all found */
    long     cycle_us;
    list_t   amp;             /* counts, per axis */
    double   period;          /* s */
    list_t   delta;           /* counts, csp-move relative target, per axis */
    double   move_time;       /* s, csp-move duration */
    list_t   vel;             /* counts/s, per axis */
    double   vel_accel;       /* counts/s^2 */
    double   vel_limit;       /* counts/s */
    list_t   tq;              /* per-mille of rated, per axis */
    double   tq_rate;         /* per-mille per s */
    list_t   tq_limit;        /* per-mille; also written to 0x6072 (max torque), per axis */
    list_t   cpr;             /* encoder counts per output revolution, per axis */
    int      expect;          /* required slave count, 0 = don't check */
    double   max_step;        /* counts per cycle (CSP slew limit) */
    list_t   duration;        /* s in RUN per axis, 0 = until Ctrl-C */
    int      use_dc;
    unsigned dc_assign;
    int      dc_shift;
    int      reset_fault;
    int      rt_prio;
    int      cpu;
    const char *log_path;     /* CSV log file, "auto" = timestamped name, NULL = no log */
    int      log_every;       /* log every Nth cycle */
    int      quiet;           /* no live terminal display */
} cfg = {
    .monitor = 0, .nslaves = 0,
    .cycle_us = 1000, .amp = {.n = 1}, .period = 4.0, .delta = {.n = 1}, .move_time = 3.0, .vel = {.n = 1}, .vel_accel = 131072,
    .vel_limit = 262144, .tq = {.n = 1}, .tq_rate = 200, .tq_limit = {.v = {200}, .n = 1}, .cpr = {.v = {COUNTS_PER_REV}, .n = 1}, .expect = 0,
    .max_step = 200, .duration = {.n = 1}, .use_dc = 1, .dc_assign = 0x0300,
    .dc_shift = 0, .reset_fault = 0, .rt_prio = 80, .cpu = -1,
    .log_path = NULL, .log_every = 10, .quiet = 0,
};

/* ---------------- state ---------------- */
typedef enum { PH_WAIT_OP, PH_MONITOR, PH_PREPARE, PH_ENABLING, PH_RUN,
               PH_STOPPING, PH_DONE, PH_FAULT } phase_t;
static const char *PH_NAME[] = {"WAIT_OP", "MONITOR", "PREPARE", "ENABLING",
                                "RUN", "STOPPING", "DONE", "FAULT"};

typedef struct {
    ec_slave_config_t *sc;
    uint32_t vendor, product;
    char     name[64];
    int      off[E_N];       /* byte offsets in domain, -1 = not mapped */
    phase_t  phase;
    unsigned cnt;            /* generic per-phase cycle counter */
    unsigned run_cycles;
    int32_t  pos0;
    double   tpos_d, tvel_d, ttq_d;
    double   amp, delta, vel, tq, tq_limit, cpr, duration;   /* per-axis settings */
    int      opmode;         /* 8/9/10, per axis */
    prf_t    profile;
    /* snapshot for the monitor thread */
    volatile uint16_t s_sw, s_err;
    volatile int32_t  s_pos, s_vel;
    volatile int16_t  s_tq;
    volatile int8_t   s_mode;
    volatile int32_t  s_tpos, s_tvel;   /* last commanded targets (for the log) */
    volatile int16_t  s_ttq;
} axis_t;

static axis_t   ax[MAX_AX];
static int      n_ax;
static ec_master_t *master;
static ec_domain_t *domain;
static uint8_t *dom_data;
static volatile sig_atomic_t g_stop;
static volatile int g_comm_bad, g_master_al, g_resp, g_link;
static volatile long long g_lat_max_ns;
static double cycle_s;

static void on_sig(int s) { (void)s; g_stop = 1; }

static int ramp(double *x, double target, double step)
{
    double d = target - *x;
    if (fabs(d) <= step) { *x = target; return 1; }
    *x += (d > 0) ? step : -step;
    return 0;
}

/* ---------------- one axis, one cycle ---------------- */
static void axis_step(axis_t *a, uint8_t *d, int slave_op, int stop_req)
{
    uint16_t sw = 0, err = 0; int32_t apos = 0, avel = 0; int16_t atq = 0; int8_t md = 0;
    uint16_t cw = CW_SHUTDOWN;
    int *o = a->off;

    if (o[E_SW]   >= 0) sw   = EC_READ_U16(d + o[E_SW]);
    if (o[E_APOS] >= 0) apos = EC_READ_S32(d + o[E_APOS]);
    if (o[E_AVEL] >= 0) avel = EC_READ_S32(d + o[E_AVEL]);
    if (o[E_ATQ]  >= 0) atq  = EC_READ_S16(d + o[E_ATQ]);
    if (o[E_MODED]>= 0) md   = EC_READ_S8 (d + o[E_MODED]);
    if (o[E_ERR]  >= 0) err  = EC_READ_U16(d + o[E_ERR]);
    a->s_sw = sw; a->s_pos = apos; a->s_vel = avel; a->s_tq = atq; a->s_mode = md;
    a->s_err = err;

    /* Defaults every cycle: safe targets (current pos, zero vel, zero torque) */
    int32_t tpos = apos; int32_t tvel = 0; int16_t ttq = 0;
    uint16_t s = sw & 0x006F;

    switch (a->phase) {
    case PH_WAIT_OP:
        cw = 0x0000;
        if (slave_op) {
            a->phase = cfg.monitor ? PH_MONITOR : PH_PREPARE;
            a->cnt = 0;
        }
        break;

    case PH_MONITOR:
        cw = 0x0000;                 /* never enable in monitor mode */
        break;

    case PH_PREPARE:                 /* mode is written every cycle; wait for echo */
        cw = CW_SHUTDOWN;
        a->cnt++;
        if (a->cnt > 50 && (o[E_MODED] < 0 || md == a->opmode)) {
            a->phase = PH_ENABLING; a->cnt = 0;
        } else if (a->cnt > 3000) {
            fprintf(stderr, "axis: mode_display never became %d\n", a->opmode);
            a->phase = PH_FAULT; a->cnt = 0; g_stop = 1;
        }
        break;

    case PH_ENABLING:
        a->cnt++;
        if (sw & SW_FAULT) {
            if (cfg.reset_fault && a->cnt < 2000)
                cw = ((a->cnt / 50) & 1) ? CW_FAULT_RST : 0x0000; /* edge on bit7 */
            else { a->phase = PH_FAULT; a->cnt = 0; g_stop = 1; }
            break;
        }
        if ((sw & 0x004F) == 0x0040)      cw = CW_SHUTDOWN;      /* switch-on disabled */
        else if (s == 0x0021)             cw = CW_SWITCH_ON;     /* ready to switch on */
        else if (s == 0x0023)             cw = CW_ENABLE;        /* switched on        */
        else if (s == 0x0027) {                                   /* operation enabled  */
            cw = CW_ENABLE;
            a->pos0 = apos; a->tpos_d = apos;
            a->tvel_d = 0; a->ttq_d = 0; a->run_cycles = 0;
            a->phase = PH_RUN;
        } else                            cw = CW_SHUTDOWN;
        if (a->phase == PH_ENABLING && a->cnt > 5000) {
            fprintf(stderr, "axis: enable sequence timed out (sw=0x%04x)\n", sw);
            a->phase = PH_FAULT; a->cnt = 0; g_stop = 1;
        }
        break;

    case PH_RUN: {
        if (s != 0x0027 || (sw & SW_FAULT)) {      /* lost operation-enabled */
            a->phase = PH_FAULT; a->cnt = 0; g_stop = 1; break;
        }
        cw = CW_ENABLE;
        if (a->duration > 0 && a->run_cycles * cycle_s >= a->duration) stop_req = 1;  /* this axis' time is up */
        if (stop_req) { a->phase = PH_STOPPING; a->cnt = 0; }
        double t = a->run_cycles++ * cycle_s;
        if (a->opmode == OPM_CSP) {
            double off = 0.0;
            if (a->profile == PRF_SINE)
                off = a->amp * 0.5 * (1.0 - cos(2.0 * M_PI * t / cfg.period));
            else if (a->profile == PRF_MOVE)   /* smooth cosine ease, then hold at target */
                off = (t >= cfg.move_time) ? a->delta
                    : a->delta * 0.5 * (1.0 - cos(M_PI * t / cfg.move_time));
            double tgt = a->pos0 + off;
            if (fabs(tgt - a->tpos_d) > cfg.max_step) {
                fprintf(stderr, "axis: CSP step %.1f > max_step %.1f, aborting\n",
                        fabs(tgt - a->tpos_d), cfg.max_step);
                a->phase = PH_STOPPING; a->cnt = 0; g_stop = 1;
            } else a->tpos_d = tgt;
            tpos = (int32_t)llround(a->tpos_d);
        } else if (a->opmode == OPM_CSV) {
            ramp(&a->tvel_d, a->vel, cfg.vel_accel * cycle_s);
            tvel = (int32_t)llround(a->tvel_d);
        } else {
            ramp(&a->ttq_d, a->tq, cfg.tq_rate * cycle_s);
            ttq = (int16_t)llround(a->ttq_d);
        }
        break;
    }

    case PH_STOPPING: {
        int done = 1;
        cw = CW_ENABLE;
        if (a->opmode == OPM_CSP) {
            tpos = (int32_t)llround(a->tpos_d);        /* hold last target */
        } else if (a->opmode == OPM_CSV) {
            done = ramp(&a->tvel_d, 0, cfg.vel_accel * cycle_s);
            tvel = (int32_t)llround(a->tvel_d);
        } else {
            done = ramp(&a->ttq_d, 0, cfg.tq_rate * cycle_s);
            ttq = (int16_t)llround(a->ttq_d);
        }
        if (done) {
            cw = CW_SHUTDOWN;
            if (++a->cnt > 100) a->phase = PH_DONE;
        }
        if (sw & SW_FAULT) { a->phase = PH_FAULT; a->cnt = 0; }
        break;
    }

    case PH_FAULT:
        a->cnt++;
        cw = cfg.reset_fault ? (((a->cnt / 50) & 1) ? CW_FAULT_RST : 0) : 0x0000;
        if (a->cnt > 200) a->phase = PH_DONE;
        break;

    case PH_DONE:
        cw = CW_SHUTDOWN;
        break;
    }

    a->s_tpos = tpos; a->s_tvel = tvel; a->s_ttq = ttq;

    /* ---- write outputs ---- */
    if (o[E_CW] >= 0) EC_WRITE_U16(d + o[E_CW], cw);

    /* Mode of operation: not written at all in monitor mode (passive) */
    if (o[E_MODE] >= 0 && !cfg.monitor)
        EC_WRITE_S8(d + o[E_MODE], (int8_t)a->opmode);

    /* Max torque (0x6072) is part of the RxPDO: it MUST be written every
     * cycle, otherwise the drive sees 0 and will not produce torque.
     * Zero in monitor mode. */
    if (o[E_MAXTQ] >= 0)
        EC_WRITE_U16(d + o[E_MAXTQ], cfg.monitor ? 0 : (uint16_t)a->tq_limit);

    if (a->opmode == OPM_CSP && o[E_TPOS] >= 0) EC_WRITE_S32(d + o[E_TPOS], tpos);
    if (a->opmode == OPM_CSV && o[E_TVEL] >= 0) EC_WRITE_S32(d + o[E_TVEL], tvel);
    if (a->opmode == OPM_CST && o[E_TTQ]  >= 0) EC_WRITE_S16(d + o[E_TTQ],  ttq);
}

/* ---------------- monitor thread (non-RT; all printing lives here) ---------------- */
static void *monitor_thread(void *arg)
{
    (void)arg;
    for (;;) {
        int all_done = 1;
        printf("\033[2J\033[H");
        printf("link=%d  master AL=0x%x  slaves=%d  comm_bad=%d  lat_max=%lld us\n",
               g_link, g_master_al, g_resp, g_comm_bad, g_lat_max_ns / 1000);
        for (int i = 0; i < n_ax; i++) {
            axis_t *a = &ax[i];
            printf("[%d] %-10s sw=0x%04x err=0x%04x md=%2d pos=%11d (%8.2f deg) "
                   "vel=%8d (%7.2f rpm) tq=%5.1f%%\n",
                   i, PH_NAME[a->phase], a->s_sw, a->s_err, a->s_mode, a->s_pos,
                   a->s_pos * 360.0 / a->cpr, a->s_vel,
                   a->s_vel * 60.0 / a->cpr, a->s_tq / 10.0);
            if (a->phase != PH_DONE) all_done = 0;
        }
        fflush(stdout);
        if (all_done) break;
        usleep(250000);
    }
    return NULL;
}

/* ---------------- CSV logger: RT loop pushes into a lock-free ring, a normal
 * priority thread writes the file, so disk I/O can never delay the cycle ---------------- */
#define LOG_RING 65536              /* samples, must be a power of two */
typedef struct {
    double   t;
    int      axis, phase, md, comm_bad;
    unsigned sw, err;
    int32_t  apos, cpos, avel, cvel;
    int16_t  atq, ctq;
} log_sample_t;

static log_sample_t g_ring[LOG_RING];
static atomic_ulong g_lh, g_lt;      /* head (producer) / tail (consumer) */
static atomic_int   g_log_done;
static unsigned long g_log_dropped;
static FILE *g_logf;
static char  g_log_name[160];

static void log_push(double t, int i)
{
    axis_t *a = &ax[i];
    unsigned long h  = atomic_load_explicit(&g_lh, memory_order_relaxed);
    unsigned long tl = atomic_load_explicit(&g_lt, memory_order_acquire);
    if (h - tl >= LOG_RING) { g_log_dropped++; return; }
    log_sample_t *s = &g_ring[h & (LOG_RING - 1)];
    s->t = t; s->axis = i; s->phase = (int)a->phase; s->md = a->s_mode; s->comm_bad = g_comm_bad;
    s->sw = a->s_sw; s->err = a->s_err;
    s->apos = a->s_pos; s->cpos = a->s_tpos; s->avel = a->s_vel; s->cvel = a->s_tvel;
    s->atq = a->s_tq; s->ctq = a->s_ttq;
    atomic_store_explicit(&g_lh, h + 1, memory_order_release);
}

static void *log_thread(void *arg)
{
    (void)arg;
    for (;;) {
        unsigned long tl = atomic_load_explicit(&g_lt, memory_order_relaxed);
        unsigned long h  = atomic_load_explicit(&g_lh, memory_order_acquire);
        if (tl == h) {
            if (atomic_load(&g_log_done)) {
                if (tl == atomic_load_explicit(&g_lh, memory_order_acquire)) break;
                continue;
            }
            fflush(g_logf);
            usleep(20000);
            continue;
        }
        while (tl != h) {
            const log_sample_t *s = &g_ring[tl & (LOG_RING - 1)];
            double cpr = ax[s->axis].cpr;
            fprintf(g_logf, "%.4f,%d,%s,0x%04x,0x%04x,%d,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%d\n",
                    s->t, s->axis, PH_NAME[s->phase], s->sw, s->err, s->md,
                    s->apos, s->cpos, s->avel, s->cvel, s->atq, s->ctq,
                    s->apos * 360.0 / cpr, s->cpos * 360.0 / cpr,
                    s->avel * 60.0 / cpr,  s->cvel * 60.0 / cpr,
                    s->atq / 10.0, s->ctq / 10.0, s->comm_bad);
            tl++;
        }
        atomic_store_explicit(&g_lt, tl, memory_order_release);
    }
    fflush(g_logf);
    return NULL;
}

/* ---------------- helpers ---------------- */
static void usage(const char *p)
{
    fprintf(stderr,
    "usage: %s [options]\n"
    "  --monitor               observe only, never enable the drives\n"
    "  --mode M[,M,...]        M = csp-hold|csp-sine|csp-move|csv|cst; one for all axes,\n"
    "                          or a comma list with one per slave (modes can differ per axis)\n"
    "  --delta COUNTS --move-time S   csp-move: smooth relative move, then hold\n"
    "  --slaves N              use first N slaves (default: all found)\n"
    "  --cycle-us N            cycle time (default 1000)\n"
    "  --amp COUNTS --period S csp-sine amplitude / period\n"
    "  --vel COUNTS_PER_S      csv target   (--vel-accel, --vel-limit)\n"
    "  --tq PERMILLE           cst target   (--tq-rate)\n"
    "  --tq-limit PERMILLE     torque limit; also written to 0x6072 Max Torque\n"
    "                          (default 200 = 20%% of rated)\n"
    "  --max-step COUNTS       CSP slew limit per cycle (default 200)\n"
    "  --duration S[,S,...]    stop each axis S seconds after it starts running (0 = until Ctrl-C);\n"
    "                          one value for all axes or one per slave\n"
    "  --no-dc | --dc-assign 0x0300 | --dc-shift NS\n"
    "  --reset-fault           try bit-7 fault reset during enabling (undocumented!)\n"
    "  --cpr COUNTS            encoder counts per output revolution (default 131072)\n"
    "  --expect N              refuse to run unless exactly N slaves are on the bus\n"
    "  --amp/--delta/--vel/--tq/--tq-limit/--cpr accept one value (all axes) or a\n"
    "  comma list, one per slave in bus order, e.g. --vel 20000,-30000\n"
    "  --log FILE.csv|auto     write a CSV log (auto = myact_log_YYYYmmdd_HHMMSS.csv)\n"
    "  --log-every N           log every Nth cycle (default 10 = 100 Hz at 1 ms)\n"
    "  --quiet                 no live terminal display\n"
    "  --rt-prio N  --cpu N\n", p);
    exit(1);
}

static long long now_ns(clockid_t c)
{ struct timespec t; clock_gettime(c, &t); return t.tv_sec * NSEC + t.tv_nsec; }

static int list_ok(const list_t *l, const char *name)
{
    if (l->n != 1 && l->n < n_ax) {
        fprintf(stderr, "--%s needs 1 value or at least %d (one per slave), got %d\n", name, n_ax, l->n);
        return 0;
    }
    return 1;
}

/* per-axis safety checks, run before anything is enabled */
static int check_axes(void)
{
    for (int i = 0; i < n_ax; i++) {
        axis_t *a = &ax[i];
        if (a->duration < 0) { fprintf(stderr, "axis %d: --duration must be >= 0\n", i); return 1; }
        if (a->cpr <= 0) { fprintf(stderr, "axis %d: --cpr must be > 0\n", i); return 1; }
        if (fabs(a->vel) > cfg.vel_limit) { fprintf(stderr, "axis %d: --vel exceeds --vel-limit\n", i); return 1; }
        if (a->tq_limit < 0 || a->tq_limit > 65535) { fprintf(stderr, "axis %d: --tq-limit out of range (0..65535)\n", i); return 1; }
        if (fabs(a->tq) > a->tq_limit) { fprintf(stderr, "axis %d: --tq exceeds --tq-limit\n", i); return 1; }
        if (a->opmode == OPM_CSP && a->profile == PRF_MOVE) {
            if (cfg.move_time <= 0) { fprintf(stderr, "--move-time must be > 0\n"); return 1; }
            double vpk = fabs(a->delta) * M_PI / (2.0 * cfg.move_time);   /* counts/s peak */
            if (vpk * cycle_s > cfg.max_step) {
                fprintf(stderr, "axis %d: csp-move too fast: peak step %.1f counts/cycle > max_step %.1f "
                        "(increase --move-time or reduce --delta)\n", i, vpk * cycle_s, cfg.max_step);
                return 1;
            }
        }
        if (a->opmode == OPM_CSP && a->profile == PRF_SINE) {
            if (cfg.period <= 0) { fprintf(stderr, "--period must be > 0\n"); return 1; }
            double vpk = fabs(a->amp) * M_PI / cfg.period;                  /* counts/s peak */
            if (vpk * cycle_s > cfg.max_step) {
                fprintf(stderr, "axis %d: csp-sine too fast: peak step %.1f counts/cycle > max_step %.1f\n",
                        i, vpk * cycle_s, cfg.max_step);
                return 1;
            }
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        const char *k = argv[i];
#define NEXT (++i < argc ? argv[i] : (usage(argv[0]), ""))
        if (!strcmp(k, "--monitor")) cfg.monitor = 1;
        else if (!strcmp(k, "--mode")) {
            char mb[256];
            strncpy(mb, NEXT, sizeof mb - 1); mb[sizeof mb - 1] = 0;
            cfg.mode_n = 0;
            for (char *m = strtok(mb, ","); m && cfg.mode_n < MAX_AX; m = strtok(NULL, ",")) {
                int n = cfg.mode_n;
                if (!strcmp(m, "csp-hold"))      { cfg.m_op[n] = OPM_CSP; cfg.m_prf[n] = PRF_HOLD; }
                else if (!strcmp(m, "csp-sine")) { cfg.m_op[n] = OPM_CSP; cfg.m_prf[n] = PRF_SINE; }
                else if (!strcmp(m, "csp-move")) { cfg.m_op[n] = OPM_CSP; cfg.m_prf[n] = PRF_MOVE; }
                else if (!strcmp(m, "csv"))      { cfg.m_op[n] = OPM_CSV; cfg.m_prf[n] = PRF_HOLD; }
                else if (!strcmp(m, "cst"))      { cfg.m_op[n] = OPM_CST; cfg.m_prf[n] = PRF_HOLD; }
                else usage(argv[0]);
                cfg.mode_n++;
            }
            if (cfg.mode_n == 0) usage(argv[0]);
        }
        else if (!strcmp(k, "--slaves"))      cfg.nslaves = atoi(NEXT);
        else if (!strcmp(k, "--cycle-us"))    cfg.cycle_us = atol(NEXT);
        else if (!strcmp(k, "--amp"))         parse_list(&cfg.amp, NEXT);
        else if (!strcmp(k, "--period"))      cfg.period = atof(NEXT);
        else if (!strcmp(k, "--delta"))       parse_list(&cfg.delta, NEXT);
        else if (!strcmp(k, "--move-time"))   cfg.move_time = atof(NEXT);
        else if (!strcmp(k, "--vel"))         parse_list(&cfg.vel, NEXT);
        else if (!strcmp(k, "--vel-accel"))   cfg.vel_accel = atof(NEXT);
        else if (!strcmp(k, "--vel-limit"))   cfg.vel_limit = atof(NEXT);
        else if (!strcmp(k, "--tq"))          parse_list(&cfg.tq, NEXT);
        else if (!strcmp(k, "--tq-rate"))     cfg.tq_rate = atof(NEXT);
        else if (!strcmp(k, "--tq-limit"))    parse_list(&cfg.tq_limit, NEXT);
        else if (!strcmp(k, "--max-step"))    cfg.max_step = atof(NEXT);
        else if (!strcmp(k, "--duration"))    parse_list(&cfg.duration, NEXT);
        else if (!strcmp(k, "--no-dc"))       cfg.use_dc = 0;
        else if (!strcmp(k, "--dc-assign"))   cfg.dc_assign = strtoul(NEXT, NULL, 0);
        else if (!strcmp(k, "--dc-shift"))    cfg.dc_shift = atoi(NEXT);
        else if (!strcmp(k, "--reset-fault")) cfg.reset_fault = 1;
        else if (!strcmp(k, "--log"))         cfg.log_path = NEXT;
        else if (!strcmp(k, "--log-every"))   cfg.log_every = atoi(NEXT);
        else if (!strcmp(k, "--quiet"))       cfg.quiet = 1;
        else if (!strcmp(k, "--rt-prio"))     cfg.rt_prio = atoi(NEXT);
        else if (!strcmp(k, "--cpu"))         cfg.cpu = atoi(NEXT);
        else if (!strcmp(k, "--cpr"))         parse_list(&cfg.cpr, NEXT);
        else if (!strcmp(k, "--expect"))      cfg.expect = atoi(NEXT);
        else usage(argv[0]);
    }
    if (cfg.cycle_us < 250) { fprintf(stderr, "cycle too short\n"); return 1; }
    cycle_s = cfg.cycle_us * 1e-6;
    unsigned long long cycle_ns = (unsigned long long)cfg.cycle_us * 1000ULL;

    if (cfg.log_every < 1) { fprintf(stderr, "--log-every must be >= 1\n"); return 1; }
    if (cfg.log_path) {
        const char *path = cfg.log_path;
        if (!strcmp(path, "auto")) {
            time_t tt = time(NULL); struct tm tmv; localtime_r(&tt, &tmv);
            strftime(g_log_name, sizeof g_log_name, "myact_log_%Y%m%d_%H%M%S.csv", &tmv);
            path = g_log_name;
        }
        g_logf = fopen(path, "w");
        if (!g_logf) { fprintf(stderr, "cannot open log file %s: %s\n", path, strerror(errno)); return 1; }
        fprintf(g_logf, "time_s,axis,phase,statusword,error_code,mode_display,act_pos,cmd_pos,act_vel,cmd_vel,"
                        "act_torque_pm,cmd_torque_pm,pos_deg,cmd_pos_deg,vel_rpm,cmd_vel_rpm,torque_pct,"
                        "cmd_torque_pct,comm_bad\n");
        printf("logging to %s (every %d cycles)\n", path, cfg.log_every);
        snprintf(g_log_name, sizeof g_log_name, "%s", path);
    }

    mlockall(MCL_CURRENT | MCL_FUTURE);

    /* ---- 1. request master, wait for the bus scan ---- */
    master = ecrt_request_master(0);
    if (!master) { fprintf(stderr, "cannot request master 0 (is the ethercat service running / another app holding it?)\n"); return 1; }

    ec_master_info_t mi;
    for (int t = 0; t < 200; t++) {
        if (ecrt_master(master, &mi) == 0 && !mi.scan_busy && mi.slave_count > 0) break;
        usleep(50000);
    }
    if (ecrt_master(master, &mi) || !mi.link_up || mi.slave_count == 0) {
        fprintf(stderr, "no link or no slaves found\n"); ecrt_release_master(master); return 1;
    }
    n_ax = mi.slave_count;
    if (cfg.nslaves > 0 && cfg.nslaves < n_ax) n_ax = cfg.nslaves;
    if (n_ax > MAX_AX) n_ax = MAX_AX;

    if (cfg.expect > 0 && (int)mi.slave_count != cfg.expect) {
        fprintf(stderr, "expected %d slaves but found %u\n", cfg.expect, mi.slave_count);
        ecrt_release_master(master); return 1;
    }
    if (!list_ok(&cfg.amp, "amp") || !list_ok(&cfg.delta, "delta") || !list_ok(&cfg.vel, "vel") ||
        !list_ok(&cfg.tq, "tq") || !list_ok(&cfg.tq_limit, "tq-limit") || !list_ok(&cfg.cpr, "cpr") ||
        !list_ok(&cfg.duration, "duration")) {
        ecrt_release_master(master); return 1;
    }
    if (cfg.mode_n > 1 && cfg.mode_n < n_ax) {
        fprintf(stderr, "--mode needs 1 entry or at least %d (one per slave), got %d\n", n_ax, cfg.mode_n);
        ecrt_release_master(master); return 1;
    }
    for (int i = 0; i < n_ax; i++) {
        int mi_ = (cfg.mode_n == 0) ? -1 : (cfg.mode_n == 1 ? 0 : i);
        ax[i].opmode  = (mi_ < 0) ? OPM_CSP  : cfg.m_op[mi_];
        ax[i].profile = (mi_ < 0) ? PRF_HOLD : cfg.m_prf[mi_];
        ax[i].amp = lget(&cfg.amp, i);        ax[i].delta = lget(&cfg.delta, i);
        ax[i].vel = lget(&cfg.vel, i);        ax[i].tq = lget(&cfg.tq, i);
        ax[i].tq_limit = lget(&cfg.tq_limit, i);
        ax[i].cpr = lget(&cfg.cpr, i);
        ax[i].duration = lget(&cfg.duration, i);
    }
    if (check_axes()) { ecrt_release_master(master); return 1; }

    domain = ecrt_master_create_domain(master);
    if (!domain) { fprintf(stderr, "domain create failed\n"); return 1; }

    /* ---- 2. configure every slave ---- */
    for (int i = 0; i < n_ax; i++) {
        axis_t *a = &ax[i];
        ec_slave_info_t si;
        if (ecrt_master_get_slave(master, i, &si)) { fprintf(stderr, "get_slave(%d) failed\n", i); return 1; }
        a->vendor = si.vendor_id; a->product = si.product_code;
        snprintf(a->name, sizeof a->name, "%s", si.name);
        printf("slave %d: \"%s\" vendor=0x%08x product=0x%08x rev=0x%08x\n",
               i, si.name, si.vendor_id, si.product_code, si.revision_number);

        a->sc = ecrt_master_slave_config(master, 0, i, a->vendor, a->product);
        if (!a->sc) { fprintf(stderr, "slave_config(%d) failed\n", i); return 1; }
#ifdef USE_CUSTOM_PDO
        if (ecrt_slave_config_pdos(a->sc, EC_END, syncs)) { fprintf(stderr, "config_pdos failed\n"); return 1; }
#endif
        for (int e = 0; e < E_N; e++) {
            unsigned bp = 0;
            int r = ecrt_slave_config_reg_pdo_entry(a->sc, ENT[e].idx, ENT[e].sub, domain, &bp);
            a->off[e] = (r >= 0 && bp == 0) ? r : -1;
        }
        /* mandatory entries per mode (max torque is needed to produce any motion) */
        int ok = a->off[E_CW] >= 0 && a->off[E_SW] >= 0;
        if (a->opmode == OPM_CSP) ok = ok && a->off[E_TPOS] >= 0 && a->off[E_APOS] >= 0;
        if (a->opmode == OPM_CSV) ok = ok && a->off[E_TVEL] >= 0;
        if (a->opmode == OPM_CST) ok = ok && a->off[E_TTQ]  >= 0;
        if (!ok) {
            fprintf(stderr, "slave %d: required PDO entries are not in its current mapping.\n"
                    "  run: ethercat pdos -p %d   and compare with ENT[] / try -DUSE_CUSTOM_PDO\n", i, i);
            for (int e = 0; e < E_N; e++)
                fprintf(stderr, "    %-18s 0x%04x : %s\n", ENT[e].name, ENT[e].idx,
                        a->off[e] >= 0 ? "mapped" : "MISSING");
            ecrt_release_master(master); return 1;
        }
        if (a->off[E_MAXTQ] < 0 && !cfg.monitor)
            fprintf(stderr, "warning: slave %d has no 0x6072 in PDO; max torque is whatever the drive's SDO holds\n", i);
        /* mode not in PDO -> fall back to a one-time SDO write at PREOP->SAFEOP */
        if (a->off[E_MODE] < 0 && !cfg.monitor)
            ecrt_slave_config_sdo8(a->sc, 0x6060, 0, (uint8_t)a->opmode);
        if (cfg.use_dc)
            ecrt_slave_config_dc(a->sc, cfg.dc_assign, (uint32_t)cycle_ns, cfg.dc_shift, 0, 0);
        a->phase = PH_WAIT_OP;
    }

    /* ---- 3. activate ---- */
    ecrt_master_application_time(master, now_ns(CLOCK_MONOTONIC));
    if (ecrt_master_activate(master)) { fprintf(stderr, "activate failed\n"); return 1; }
    dom_data = ecrt_domain_data(domain);
    if (!dom_data) { fprintf(stderr, "no domain data\n"); return 1; }

    /* ---- 4. real-time setup ---- */
    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);
    struct sched_param sp = { .sched_priority = cfg.rt_prio };
    if (sched_setscheduler(0, SCHED_FIFO, &sp))
        fprintf(stderr, "warning: SCHED_FIFO failed (%s) - run as root for real-time\n", strerror(errno));
    if (cfg.cpu >= 0) {
        cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(cfg.cpu, &cs);
        sched_setaffinity(0, sizeof cs, &cs);
    }
    pthread_t mon; pthread_attr_t pa; pthread_attr_init(&pa);
    pthread_attr_setinheritsched(&pa, PTHREAD_EXPLICIT_SCHED);
    struct sched_param sp0 = { .sched_priority = 0 };
    pthread_attr_setschedpolicy(&pa, SCHED_OTHER);
    pthread_attr_setschedparam(&pa, &sp0);
    int have_mon = 0, have_log = 0; pthread_t lth;
    if (!cfg.quiet) { pthread_create(&mon, &pa, monitor_thread, NULL); have_mon = 1; }
    if (g_logf)     { pthread_create(&lth, &pa, log_thread, NULL);     have_log = 1; }

    /* ---- 5. cyclic loop ---- */
    struct timespec wake; clock_gettime(CLOCK_MONOTONIC, &wake);
    int bus_up = 0;             /* set once all slaves were OP with a complete working counter */
    unsigned long cyc = 0;
    long long t_log0 = now_ns(CLOCK_MONOTONIC);
    int all_done = 0;

    while (!all_done) {
        wake.tv_nsec += cycle_ns;
        while (wake.tv_nsec >= NSEC) { wake.tv_nsec -= NSEC; wake.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wake, NULL);
        long long lat = now_ns(CLOCK_MONOTONIC) - ((long long)wake.tv_sec * NSEC + wake.tv_nsec);
        if (lat > g_lat_max_ns) g_lat_max_ns = lat;

        ecrt_master_receive(master);
        ecrt_domain_process(domain);

        ec_domain_state_t ds; ecrt_domain_state(domain, &ds);
        if (ds.wc_state == EC_WC_COMPLETE) g_comm_bad = 0; else g_comm_bad++;

        if ((cyc % 100) == 0) {
            ec_master_state_t ms; ecrt_master_state(master, &ms);
            g_master_al = ms.al_states; g_resp = ms.slaves_responding; g_link = ms.link_up;
        }

        /* The working counter is incomplete while the master walks the slaves
         * PREOP->SAFEOP->OP, so comm loss only counts once the bus has been up. */
        if (!bus_up) {
            int all_op = (ds.wc_state == EC_WC_COMPLETE);
            for (int i = 0; i < n_ax; i++) {
                ec_slave_config_state_t st0; ecrt_slave_config_state(ax[i].sc, &st0);
                if (!st0.operational) all_op = 0;
            }
            if (all_op) bus_up = 1;
        }
        if (bus_up && g_comm_bad > 50 && !cfg.monitor) g_stop = 1;  /* lost the bus: start stopping */

        all_done = 1;
        for (int i = 0; i < n_ax; i++) {
            ec_slave_config_state_t st; ecrt_slave_config_state(ax[i].sc, &st);
            int op = st.operational && ds.wc_state == EC_WC_COMPLETE;
            if (cfg.monitor && g_stop) ax[i].phase = PH_DONE;
            axis_step(&ax[i], dom_data, op, g_stop);
            if (ax[i].phase != PH_DONE) all_done = 0;
        }

        if (g_logf && bus_up && (cyc % (unsigned long)cfg.log_every) == 0) {
            double tl_ = (now_ns(CLOCK_MONOTONIC) - t_log0) * 1e-9;
            for (int i = 0; i < n_ax; i++) log_push(tl_, i);
        }

        ecrt_domain_queue(domain);
        if (cfg.use_dc) {
            ecrt_master_application_time(master, now_ns(CLOCK_MONOTONIC));
            ecrt_master_sync_reference_clock(master);
            ecrt_master_sync_slave_clocks(master);
        }
        ecrt_master_send(master);
        cyc++;
    }

    if (have_mon) pthread_join(mon, NULL);
    atomic_store(&g_log_done, 1);
    if (have_log) pthread_join(lth, NULL);
    if (g_logf) {
        fclose(g_logf);
        printf("\nlog saved: %s", g_log_name);
        if (g_log_dropped) printf("  (WARNING: %lu samples dropped)", g_log_dropped);
        printf("\n");
    }
    ecrt_release_master(master);       /* slaves drop to INIT/PREOP; SM watchdog protects meanwhile */
    printf("\ndone.\n");
    return 0;
}
