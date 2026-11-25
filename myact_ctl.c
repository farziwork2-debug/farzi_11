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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
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
typedef enum { PRF_HOLD, PRF_SINE, PRF_MOVE, PRF_UDP, PRF_UDPX } prf_t;
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
    int      sdo_auto;        /* probe the standard CiA 402 readings (current, DC link voltage, ...) */
    const char *sdo_spec;     /* extra objects to log: IDX:SUB:TYPE:NAME,... */
    int      udp_port;        /* csp-udp: UDP port to listen on */
    int      udp_timeout_ms;  /* csp-udp: stop if no valid packet for this long */
    double   udp_tau_ms;      /* csp-udp: target low-pass time constant, 0 = off */
    list_t   scale;           /* csp-udp: motor-output rad per joint rad, sign included, per axis */
    list_t   limit_deg;       /* csp-udp: max travel from start position, +/- deg, per axis */
    const char *udp_from;     /* csp-udp: only accept packets from this IPv4 address */
    int      udp_on_timeout_hold;   /* 1: no packets -> hold position and wait; 0: stop all motors */
    double   udp_hold_max_s;        /* with hold: give up and stop after this many seconds without packets */
} cfg = {
    .monitor = 0, .nslaves = 0,
    .cycle_us = 1000, .amp = {.n = 1}, .period = 4.0, .delta = {.n = 1}, .move_time = 3.0, .vel = {.n = 1}, .vel_accel = 131072,
    .vel_limit = 262144, .tq = {.n = 1}, .tq_rate = 200, .tq_limit = {.v = {200}, .n = 1}, .cpr = {.v = {COUNTS_PER_REV}, .n = 1}, .expect = 0,
    .max_step = 200, .duration = {.n = 1}, .use_dc = 1, .dc_assign = 0x0300,
    .dc_shift = 0, .reset_fault = 0, .rt_prio = 80, .cpu = -1,
    .log_path = NULL, .log_every = 10, .quiet = 0, .sdo_auto = 0, .sdo_spec = NULL,
    .udp_port = 5005, .udp_timeout_ms = 200, .udp_tau_ms = 15, .udp_on_timeout_hold = 0, .udp_hold_max_s = 60,
    .scale = {.v = {1}, .n = 1}, .limit_deg = {.v = {30}, .n = 1}, .udp_from = NULL,
};

/* ---------------- state ---------------- */
typedef enum { PH_WAIT_OP, PH_MONITOR, PH_PREPARE, PH_ENABLING, PH_RUN,
               PH_STOPPING, PH_DONE, PH_FAULT } phase_t;
static const char *PH_NAME[] = {"WAIT_OP", "MONITOR", "PREPARE", "ENABLING",
                                "RUN", "STOPPING", "DONE", "FAULT"};

#define MAX_SDO 16
typedef struct { uint16_t idx; uint8_t sub; char type; int size; char name[32]; } sdo_def_t;
static sdo_def_t g_sdo[MAX_SDO];     /* extra objects read via SDO and logged (union over all axes) */
static int g_nsdo;

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
    int      idx, need_align, aligned;   /* csp-udp state */
    double   scale, limit_counts, q0, pos_ref, q_enable;
    int      umode, pend, sw_state, sw_cnt;   /* 'udp' mode: active / pending logical mode, switch state */
    /* snapshot for the monitor thread */
    volatile uint16_t s_sw, s_err;
    volatile int32_t  s_pos, s_vel;
    volatile int16_t  s_tq;
    volatile int8_t   s_mode;
    volatile int32_t  s_tpos, s_tvel;   /* last commanded targets (for the log) */
    volatile int16_t  s_ttq;
    /* extra readings via SDO, polled in the background */
    ec_sdo_request_t *sreq[MAX_SDO];
    int               sup[MAX_SDO];        /* object readable on this axis */
    volatile long long sval[MAX_SDO];
    volatile int      svalid[MAX_SDO];
    int               nsup, scur;
    unsigned long     serr;
    long long         rated_ma, rated_mnm; /* 0x6075 / 0x6076 if the drive has them, else -1 */
    volatile int      s_online, s_al;
} axis_t;

static axis_t   ax[MAX_AX];
#define IS_UDP(a) ((a)->profile == PRF_UDP || (a)->profile == PRF_UDPX)
static int      n_ax;
static ec_master_t *master;
static ec_domain_t *domain;
static uint8_t *dom_data;
static volatile sig_atomic_t g_stop;
static volatile int g_comm_bad, g_master_al, g_resp, g_link;
static volatile long long g_lat_max_ns;
static long long g_lat_win_ns;                 /* worst wake-up latency since the last log sample */
static volatile int g_wc, g_wcs;               /* domain working counter and its state */
static int g_argc; static char **g_argv;
static double cycle_s;

/* ---- csp-udp bridge state (touched only by the real-time loop) ---- */
#define UDP_MAGIC 0x3241594DU          /* bytes 'M' 'Y' 'A' '2' */
static int    g_usock = -1;
static double g_uq[MAX_AX];            /* latest joint targets from the simulation, rad */
static int    g_uhave, g_ufresh, g_ureply, g_ustale_msg;
static double g_uv[MAX_AX], g_utau[MAX_AX];   /* latest velocity (rad/s) and torque (% rated) targets */
static uint8_t g_umode[MAX_AX];               /* latest requested mode: 0 hold, 1 pos, 2 vel, 3 torque */
static long long g_ulast_ns;
static struct sockaddr_in g_usrc;
static volatile unsigned long g_urx, g_ubad;   /* valid / rejected packet counters (shown in the monitor) */
static volatile uint32_t g_usrc_ip;            /* source of the most recent packet, rejected or not */
static volatile int g_ulen, g_un;              /* its size and joint count */
static const char * volatile g_ureason = "nothing received yet";
static struct in_addr g_uallow;
static int    g_uallow_set;

static void on_sig(int s) { (void)s; g_stop = 1; }

static int ramp(double *x, double target, double step)
{
    double d = target - *x;
    if (fabs(d) <= step) { *x = target; return 1; }
    *x += (d > 0) ? step : -step;
    return 0;
}

/* ---------------- 'udp' mode: the laptop picks the mode and the target every packet ----------------
 * Logical modes: 0 hold position, 1 position, 2 velocity, 3 torque.
 * Switching between drive modes (CSP <-> CSV <-> CST) never disables the motor: the old mode is ramped
 * to neutral, 0x6060 is changed, the drive's echo is awaited, and the new mode starts from neutral. */
enum { UM_HOLD = 0, UM_POS = 1, UM_VEL = 2, UM_TQ = 3 };
static int um_drive_mode(int um) { return um == UM_VEL ? OPM_CSV : um == UM_TQ ? OPM_CST : OPM_CSP; }

static void udpx_run(axis_t *a, int32_t apos, int8_t md, int32_t *tpos, int32_t *tvel, int16_t *ttq)
{
    const int    i = a->idx;
    const double k = a->scale * a->cpr / (2.0 * M_PI);       /* counts per joint rad */
    const double margin = a->cpr * 2.0 / 360.0;              /* 2 deg beyond --limit-deg */

    /* travel limit around the start pose, in every mode (velocity/torque would otherwise run away) */
    if (a->limit_counts > 0 && fabs((double)apos - a->pos0) > a->limit_counts + margin && !g_stop) {   /* 0 = no limit */
        fprintf(stderr, "axis %d: travel limit (+/-%.1f deg from start) exceeded, stopping all axes\n",
                i, a->limit_counts * 360.0 / a->cpr);
        g_stop = 1;
    }
    /* torque mode has no speed loop. The governor in UM_TQ fades torque out near --vel-limit; this is the
     * backstop if the shaft is driven faster anyway (external force): stop everything at 1.5 x --vel-limit */
    if (cfg.vel_limit > 0 && a->opmode == OPM_CST && fabs((double)a->s_vel) > 1.5 * cfg.vel_limit && !g_stop) {   /* 0 = no limit */
        fprintf(stderr, "axis %d: speed %.0f counts/s is far above --vel-limit (%.0f) in torque mode, stopping all axes\n",
                i, (double)a->s_vel, cfg.vel_limit);
        g_stop = 1;
    }
    if (a->need_align) {                                     /* hold until the first fresh packet */
        if (g_ufresh) {
            a->q0 = g_uq[i]; a->q_enable = a->q0; a->pos_ref = a->tpos_d;
            a->need_align = 0; a->aligned = 1; a->umode = UM_HOLD;
        }
        *tpos = (int32_t)llround(a->tpos_d);
        return;
    }
    int want = g_ufresh ? (int)g_umode[i] : (cfg.udp_on_timeout_hold ? (int)UM_HOLD : a->umode);

    switch (a->sw_state) {
    case 0:                                                  /* normal operation */
        if (want != a->umode) {
            if (um_drive_mode(want) == a->opmode) {          /* hold <-> position: same drive mode */
                a->q0 = g_uq[i]; a->pos_ref = a->tpos_d; a->umode = want;
            } else { a->pend = want; a->sw_state = 1; a->sw_cnt = 0; }
        }
        break;
    case 1: {                                                /* ramp old mode to neutral, then settle */
        int done = 1;
        if (a->opmode == OPM_CSV)      done = ramp(&a->tvel_d, 0, cfg.vel_accel * cycle_s);
        else if (a->opmode == OPM_CST) done = ramp(&a->ttq_d, 0, cfg.tq_rate * cycle_s);
        if (done && ++a->sw_cnt > 100) {
            a->opmode = um_drive_mode(a->pend);              /* 0x6060 is written from a->opmode */
            a->tpos_d = apos; a->tvel_d = 0; a->ttq_d = 0;
            a->sw_state = 2; a->sw_cnt = 0;
        }
        break; }
    case 2:                                                  /* wait for the drive to echo the new mode */
        a->tpos_d = apos;
        if (md == a->opmode) {
            if (++a->sw_cnt > 20) {
                a->umode = a->pend; a->q0 = g_uq[i]; a->pos_ref = apos; a->tpos_d = apos; a->sw_state = 0;
            }
        } else if (++a->sw_cnt > 3000) {
            fprintf(stderr, "axis %d: drive never echoed mode %d, stopping all axes\n", i, a->opmode);
            g_stop = 1;
        }
        break;
    }

    if (a->sw_state != 0) {                                  /* switching: neutral targets only */
        if (a->opmode == OPM_CSP)      *tpos = (int32_t)llround(a->tpos_d);
        else if (a->opmode == OPM_CSV) *tvel = (int32_t)llround(a->tvel_d);
        else                           *ttq  = (int16_t)llround(a->ttq_d);
        return;
    }

    switch (a->umode) {
    case UM_HOLD:
        *tpos = (int32_t)llround(a->tpos_d);
        break;
    case UM_POS: {
        double tgt = g_ufresh ? a->pos_ref + k * (g_uq[i] - a->q0) : a->tpos_d;
        if (a->limit_counts > 0) {
            if (tgt > a->pos0 + a->limit_counts) tgt = a->pos0 + a->limit_counts;
            if (tgt < a->pos0 - a->limit_counts) tgt = a->pos0 - a->limit_counts;
        }
        double alpha = (cfg.udp_tau_ms > 0) ? 1.0 - exp(-cycle_s * 1000.0 / cfg.udp_tau_ms) : 1.0;
        double step = (tgt - a->tpos_d) * alpha;
        if (step >  cfg.max_step) step =  cfg.max_step;
        if (step < -cfg.max_step) step = -cfg.max_step;
        a->tpos_d += step;
        *tpos = (int32_t)llround(a->tpos_d);
        break; }
    case UM_VEL: {
        double tgt = g_ufresh ? k * g_uv[i] : 0.0;           /* counts/s; no fresh packet -> 0 */
        if (cfg.vel_limit > 0) {
            if (tgt >  cfg.vel_limit) tgt =  cfg.vel_limit;
            if (tgt < -cfg.vel_limit) tgt = -cfg.vel_limit;
        }
        ramp(&a->tvel_d, tgt, cfg.vel_accel * cycle_s);
        *tvel = (int32_t)llround(a->tvel_d);
        break; }
    case UM_TQ: {
        double tgt = g_ufresh ? ((k < 0) ? -1.0 : 1.0) * g_utau[i] * 10.0 : 0.0;   /* per-mille */
        if (tgt >  a->tq_limit) tgt =  a->tq_limit;
        if (tgt < -a->tq_limit) tgt = -a->tq_limit;
        if (tgt != 0.0) {
            /* governors: torque that would push further along the direction of motion fades to 0 as the
             * speed approaches --vel-limit (from 80 %), and as the shaft approaches the end of its allowed
             * travel (last 15 %). Torque that slows the shaft down is never reduced. */
            double sg = (tgt > 0) ? 1.0 : -1.0, f = 1.0;
            double ov = sg * (double)a->s_vel, v1 = cfg.vel_limit, v0 = 0.8 * cfg.vel_limit;
            if (cfg.vel_limit > 0) { if (ov >= v1) f = 0.0; else if (ov > v0) f = (v1 - ov) / (v1 - v0); }
            double op = sg * ((double)apos - a->pos0), p1 = a->limit_counts, p0 = 0.85 * a->limit_counts, fp = 1.0;
            if (a->limit_counts > 0) { if (op >= p1) fp = 0.0; else if (op > p0) fp = (p1 - op) / (p1 - p0); }
            if (fp < f) f = fp;
            tgt *= f;
        }
        ramp(&a->ttq_d, tgt, cfg.tq_rate * cycle_s);
        *ttq = (int16_t)llround(a->ttq_d);
        break; }
    }
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
            a->need_align = 1; a->aligned = 0;
            a->umode = 0; a->sw_state = 0; a->pos_ref = apos;
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
        if (a->profile == PRF_UDPX) {
            udpx_run(a, apos, md, &tpos, &tvel, &ttq);
        } else if (a->opmode == OPM_CSP && a->profile == PRF_UDP) {
            /* follow the simulation: relative to the pose at enable, filtered and rate limited */
            if (g_ufresh) {
                if (a->need_align) { a->q0 = g_uq[a->idx]; a->q_enable = a->q0; a->need_align = 0; a->aligned = 1; }
                double want = a->pos0 + a->scale * (g_uq[a->idx] - a->q0) * a->cpr / (2.0 * M_PI);
                if (a->limit_counts > 0) {
                    if (want > a->pos0 + a->limit_counts) want = a->pos0 + a->limit_counts;
                    if (want < a->pos0 - a->limit_counts) want = a->pos0 - a->limit_counts;
                }
                double alpha = (cfg.udp_tau_ms > 0) ? 1.0 - exp(-cycle_s * 1000.0 / cfg.udp_tau_ms) : 1.0;
                double step = (want - a->tpos_d) * alpha;
                if (step >  cfg.max_step) step =  cfg.max_step;
                if (step < -cfg.max_step) step = -cfg.max_step;
                a->tpos_d += step;
            }
            tpos = (int32_t)llround(a->tpos_d);      /* no fresh packet: hold */
        } else if (a->opmode == OPM_CSP) {
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
        if (g_usock >= 0) {
            struct timespec tn; clock_gettime(CLOCK_MONOTONIC, &tn);
            long long age = g_uhave ? ((long long)tn.tv_sec * NSEC + tn.tv_nsec - g_ulast_ns) / 1000000 : -1;
            char ip[INET_ADDRSTRLEN] = "-"; struct in_addr ia; ia.s_addr = g_usrc_ip;
            if (g_usrc_ip) inet_ntop(AF_INET, &ia, ip, sizeof ip);
            printf("udp: valid=%lu rejected=%lu last_from=%s len=%d (need %d) joints=%d (need %d) last_valid_age=%lld ms | %s\n",
                   g_urx, g_ubad, ip, g_ulen, 12 + MAX_AX + 24 * MAX_AX, g_un, n_ax, age, g_ureason);
        }
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
    double   lat_us;
    int      online, al, wc, wcs, resp;
    unsigned sdo_mask;
    long long sdo[MAX_SDO];
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
    s->lat_us = g_lat_win_ns / 1000.0; s->online = a->s_online; s->al = a->s_al;
    s->wc = g_wc; s->wcs = g_wcs; s->resp = g_resp;
    s->sdo_mask = 0;
    for (int k = 0; k < g_nsdo; k++)
        if (a->sup[k] && a->svalid[k]) { s->sdo_mask |= 1u << k; s->sdo[k] = a->sval[k]; }
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
            fprintf(g_logf, "%.4f,%d,%s,0x%04x,0x%04x,%d,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%d",
                    s->t, s->axis, PH_NAME[s->phase], s->sw, s->err, s->md,
                    s->apos, s->cpos, s->avel, s->cvel, s->atq, s->ctq,
                    s->apos * 360.0 / cpr, s->cpos * 360.0 / cpr,
                    s->avel * 60.0 / cpr,  s->cvel * 60.0 / cpr,
                    s->atq / 10.0, s->ctq / 10.0, s->comm_bad);
            fprintf(g_logf, ",%.1f,%d,%d,%d,%d,%d", s->lat_us, s->online, s->al, s->wc, s->wcs, s->resp);
            for (int k = 0; k < g_nsdo; k++) {
                if (s->sdo_mask & (1u << k)) fprintf(g_logf, ",%lld", s->sdo[k]);
                else fputc(',', g_logf);                       /* not readable / not read yet */
            }
            fputc('\n', g_logf);
            tl++;
        }
        atomic_store_explicit(&g_lt, tl, memory_order_release);
    }
    fflush(g_logf);
    return NULL;
}

/* ---------------- extra readings via SDO (current, DC link voltage, ...) ---------------- */
static const struct { uint16_t idx; uint8_t sub; char type; int size; const char *name; } SDO_AUTO[] = {
    {0x6078, 0, 's', 2, "current_actual"},    /* CiA 402: per-mille of rated current */
    {0x6079, 0, 'u', 4, "dc_link_mv"},        /* CiA 402: DC link circuit voltage, mV */
    {0x60F4, 0, 's', 4, "following_error"},   /* counts */
    {0x6062, 0, 's', 4, "pos_demand"},
    {0x606B, 0, 's', 4, "vel_demand"},
    {0x6074, 0, 's', 2, "torque_demand"},
    {0x1001, 0, 'u', 1, "error_register"},
};

static void sdo_add(uint16_t idx, uint8_t sub, char type, int size, const char *name)
{
    for (int k = 0; k < g_nsdo; k++)
        if (g_sdo[k].idx == idx && g_sdo[k].sub == sub) return;
    if (g_nsdo >= MAX_SDO) { fprintf(stderr, "too many --sdo objects (max %d), ignoring 0x%04x\n", MAX_SDO, idx); return; }
    g_sdo[g_nsdo].idx = idx; g_sdo[g_nsdo].sub = sub; g_sdo[g_nsdo].type = type; g_sdo[g_nsdo].size = size;
    snprintf(g_sdo[g_nsdo].name, sizeof g_sdo[g_nsdo].name, "%s", name);
    g_nsdo++;
}

/* spec: IDX:SUB:TYPE:NAME[,IDX:SUB:TYPE:NAME...]  e.g. 0x2001:0:u16:bus_voltage_raw   TYPE = u8 s8 u16 s16 u32 s32 */
static int sdo_parse_spec(const char *spec)
{
    char buf[512]; snprintf(buf, sizeof buf, "%s", spec);
    char *save1 = NULL;
    for (char *item = strtok_r(buf, ",", &save1); item; item = strtok_r(NULL, ",", &save1)) {
        char *f[4] = {0}; int n = 0; char *save2 = NULL;
        for (char *t = strtok_r(item, ":", &save2); t && n < 4; t = strtok_r(NULL, ":", &save2)) f[n++] = t;
        if (n != 4) { fprintf(stderr, "bad --sdo entry (need IDX:SUB:TYPE:NAME): %s\n", item); return 1; }
        unsigned long idx = strtoul(f[0], NULL, 0), sub = strtoul(f[1], NULL, 0);
        char type = f[2][0]; int size = atoi(f[2] + 1) / 8;
        if ((type != 'u' && type != 's') || (size != 1 && size != 2 && size != 4) || idx > 0xFFFF || sub > 0xFF) {
            fprintf(stderr, "bad --sdo type/index in: %s:%s:%s:%s\n", f[0], f[1], f[2], f[3]); return 1;
        }
        for (char *c = f[3]; *c; c++) if (!((*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') || (*c >= '0' && *c <= '9') || *c == '_')) *c = '_';
        sdo_add((uint16_t)idx, (uint8_t)sub, type, size, f[3]);
    }
    return 0;
}

static long long sdo_decode(const uint8_t *b, char type, int size)
{
    if (size == 1) return type == 's' ? (long long)(int8_t)b[0] : (long long)b[0];
    if (size == 2) { uint16_t v = (uint16_t)(b[0] | (b[1] << 8)); return type == 's' ? (long long)(int16_t)v : (long long)v; }
    uint32_t v = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    return type == 's' ? (long long)(int32_t)v : (long long)v;
}

static int sdo_read_once(int slave, uint16_t idx, uint8_t sub, char type, int size, long long *out)
{
    uint8_t buf[8]; size_t rs = 0; uint32_t abort_code = 0;
    if (ecrt_master_sdo_upload(master, (uint16_t)slave, idx, sub, buf, sizeof buf, &rs, &abort_code)) return 0;
    if ((int)rs != size) return 0;
    *out = sdo_decode(buf, type, size);
    return 1;
}

/* before activation: find out which objects each drive really has, create background read requests */
static void probe_sdos(void)
{
    for (int i = 0; i < n_ax; i++) {
        axis_t *a = &ax[i];
        a->rated_ma = a->rated_mnm = -1;
        long long v;
        if (sdo_read_once(i, 0x6075, 0, 'u', 4, &v)) a->rated_ma = v;      /* motor rated current, mA */
        if (sdo_read_once(i, 0x6076, 0, 'u', 4, &v)) a->rated_mnm = v;     /* motor rated torque, mNm */
        printf("axis %d: rated current %s, rated torque %s\n", i,
               a->rated_ma >= 0 ? "read" : "n/a", a->rated_mnm >= 0 ? "read" : "n/a");
        if (a->rated_ma >= 0)  printf("        0x6075 rated current = %lld mA\n", a->rated_ma);
        if (a->rated_mnm >= 0) printf("        0x6076 rated torque  = %lld mNm\n", a->rated_mnm);
        a->nsup = 0; a->scur = 0;
        for (int k = 0; k < g_nsdo; k++) {
            a->sup[k] = 0; a->svalid[k] = 0;
            if (!sdo_read_once(i, g_sdo[k].idx, g_sdo[k].sub, g_sdo[k].type, g_sdo[k].size, &v)) continue;
            a->sreq[k] = ecrt_slave_config_create_sdo_request(a->sc, g_sdo[k].idx, g_sdo[k].sub, (size_t)g_sdo[k].size);
            if (!a->sreq[k]) continue;
            ecrt_sdo_request_timeout(a->sreq[k], 500);
            a->sup[k] = 1; a->sval[k] = v; a->svalid[k] = 1; a->nsup++;
        }
        printf("axis %d: logging %d of %d extra objects:", i, a->nsup, g_nsdo);
        for (int k = 0; k < g_nsdo; k++) if (a->sup[k]) printf(" %s", g_sdo[k].name);
        printf("\n");
        for (int k = 0; k < g_nsdo; k++)
            if (!a->sup[k]) { a->scur = k; break; }
        while (a->nsup && !a->sup[a->scur]) a->scur = (a->scur + 1) % g_nsdo;
    }
}

/* one request in flight per axis, cycling through the readable objects (called every cycle) */
static void sdo_poll(axis_t *a)
{
    if (a->nsup == 0) return;
    ec_sdo_request_t *r = a->sreq[a->scur];
    ec_request_state_t st = ecrt_sdo_request_state(r);
    if (st == EC_REQUEST_UNUSED) { ecrt_sdo_request_read(r); return; }
    if (st == EC_REQUEST_BUSY) return;
    if (st == EC_REQUEST_SUCCESS) {
        a->sval[a->scur] = sdo_decode(ecrt_sdo_request_data(r), g_sdo[a->scur].type, g_sdo[a->scur].size);
        a->svalid[a->scur] = 1;
    } else {
        a->svalid[a->scur] = 0; a->serr++;
    }
    do { a->scur = (a->scur + 1) % g_nsdo; } while (!a->sup[a->scur]);
    ecrt_sdo_request_read(a->sreq[a->scur]);
}

/* log file: '#' meta lines (settings used, rated values read from the drives), then the column header */
static void log_header(void)
{
    time_t tt = time(NULL); char ts[64]; struct tm tmv; localtime_r(&tt, &tmv);
    strftime(ts, sizeof ts, "%Y-%m-%d %H:%M:%S", &tmv);
    fprintf(g_logf, "# program=myact_ctl\n# time=%s\n# command=", ts);
    for (int i = 0; i < g_argc; i++) fprintf(g_logf, "%s%s", i ? " " : "", g_argv[i]);
    fprintf(g_logf, "\n# axes=%d\n# cycle_us=%ld\n# cpr=", n_ax, cfg.cycle_us);
    for (int i = 0; i < n_ax; i++) fprintf(g_logf, "%s%.0f", i ? "," : "", ax[i].cpr);
    fprintf(g_logf, "\n# rated_current_mA=");
    for (int i = 0; i < n_ax; i++) fprintf(g_logf, "%s%lld", i ? "," : "", ax[i].rated_ma);
    fprintf(g_logf, "\n# rated_torque_mNm=");
    for (int i = 0; i < n_ax; i++) fprintf(g_logf, "%s%lld", i ? "," : "", ax[i].rated_mnm);
    fprintf(g_logf, "\n# tq_limit_permille=");
    for (int i = 0; i < n_ax; i++) fprintf(g_logf, "%s%.0f", i ? "," : "", ax[i].tq_limit);
    fprintf(g_logf, "\n");
    for (int k = 0; k < g_nsdo; k++)
        fprintf(g_logf, "# sdo_%s=0x%04x:%u:%c%d\n", g_sdo[k].name, g_sdo[k].idx, g_sdo[k].sub, g_sdo[k].type, g_sdo[k].size * 8);
    fprintf(g_logf, "time_s,axis,phase,statusword,error_code,mode_display,act_pos,cmd_pos,act_vel,cmd_vel,"
                    "act_torque_pm,cmd_torque_pm,pos_deg,cmd_pos_deg,vel_rpm,cmd_vel_rpm,torque_pct,"
                    "cmd_torque_pct,comm_bad,lat_us_max,slave_online,al_state,working_counter,wc_state,slaves_responding");
    for (int k = 0; k < g_nsdo; k++) fprintf(g_logf, ",sdo_%s", g_sdo[k].name);
    fprintf(g_logf, "\n");
}

/* ---------------- UDP bridge: simulation (laptop) -> motors ---------------- */
typedef struct __attribute__((packed)) {      /* laptop -> board, 412 bytes, little endian */
    uint32_t magic, seq, n;
    uint8_t  mode[MAX_AX];                     /* 0 hold, 1 position, 2 velocity, 3 torque (udp mode only) */
    double   q[MAX_AX];                        /* position targets, rad            (modes hold/position) */
    double   v[MAX_AX];                        /* velocity targets, rad/s          (mode velocity) */
    double   tau[MAX_AX];                      /* torque targets, % of rated       (mode torque) */
} udp_req_t;
typedef struct __attribute__((packed)) {      /* board -> laptop, 460 bytes */
    uint32_t magic, seq, n;
    double   q[MAX_AX], qd[MAX_AX], tq[MAX_AX];  /* rad, rad/s, % rated torque (q = NaN until aligned) */
    uint16_t sw[MAX_AX];
    uint8_t  phase[MAX_AX];
    uint8_t  mode[MAX_AX];                     /* active logical mode, 255 while switching */
} udp_rep_t;
static uint32_t g_useq;

static int udp_open(int port)
{
    g_usock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_usock < 0) { perror("socket"); return -1; }
    struct sockaddr_in a; memset(&a, 0, sizeof a);
    a.sin_family = AF_INET; a.sin_addr.s_addr = htonl(INADDR_ANY); a.sin_port = htons((uint16_t)port);
    if (bind(g_usock, (struct sockaddr *)&a, sizeof a)) { perror("bind"); return -1; }
    return 0;
}

static void udp_poll(long long now)
{
    for (;;) {
        udp_req_t rq; struct sockaddr_in src; socklen_t sl = sizeof src;
        ssize_t n = recvfrom(g_usock, &rq, sizeof rq, MSG_DONTWAIT, (struct sockaddr *)&src, &sl);
        if (n < 0) return;                                   /* nothing (more) waiting */
        g_usrc_ip = src.sin_addr.s_addr; g_ulen = (int)n;
        if (g_uallow_set && src.sin_addr.s_addr != g_uallow.s_addr) {
            g_ubad++; g_ureason = "REJECTED: source IP differs from --udp-from"; continue; }
        if (n != (ssize_t)sizeof rq) { g_ubad++; g_ureason = "REJECTED: wrong packet size"; continue; }
        if (rq.magic != UDP_MAGIC)   { g_ubad++; g_ureason = "REJECTED: wrong magic (not a myact packet)"; continue; }
        g_un = (int)rq.n;
        if ((int)rq.n != n_ax)       { g_ubad++; g_ureason = "REJECTED: joint count in packet != number of motors"; continue; }
        int ok = 1;
        for (int i = 0; i < n_ax; i++)
            if (!isfinite(rq.q[i]) || fabs(rq.q[i]) > 1000.0 || !isfinite(rq.v[i]) || fabs(rq.v[i]) > 1000.0 ||
                !isfinite(rq.tau[i]) || fabs(rq.tau[i]) > 1000.0 || rq.mode[i] > 3) ok = 0;
        if (!ok) { g_ubad++; g_ureason = "REJECTED: bad value (non-finite, huge, or unknown mode)"; continue; }
        g_urx++; g_ureason = "ok";
        for (int i = 0; i < n_ax; i++) { g_uq[i] = rq.q[i]; g_uv[i] = rq.v[i]; g_utau[i] = rq.tau[i]; g_umode[i] = rq.mode[i]; }
        g_useq = rq.seq; g_uhave = 1; g_ulast_ns = now; g_usrc = src; g_ureply = 1;
    }
}

static void udp_reply(void)
{
    udp_rep_t r; memset(&r, 0, sizeof r);
    r.magic = UDP_MAGIC; r.seq = g_useq; r.n = (uint32_t)n_ax;
    for (int i = 0; i < MAX_AX; i++) r.q[i] = r.qd[i] = r.tq[i] = NAN;
    for (int i = 0; i < n_ax; i++) {
        axis_t *a = &ax[i];
        double k = (a->scale * a->cpr) / (2.0 * M_PI);        /* counts per joint rad */
        if (IS_UDP(a) && a->aligned && k != 0.0) {
            r.q[i]  = a->q_enable + ((double)a->s_pos - (double)a->pos0) / k;
            r.qd[i] = (double)a->s_vel / k;
        }
        r.tq[i] = ((a->scale < 0) ? -1.0 : 1.0) * a->s_tq / 10.0;   /* same sign convention as the commands */
        r.sw[i] = a->s_sw; r.phase[i] = (uint8_t)a->phase;
        r.mode[i] = (a->profile == PRF_UDPX) ? (uint8_t)(a->sw_state ? 255 : a->umode) : 1;
    }
    sendto(g_usock, &r, sizeof r, MSG_DONTWAIT, (struct sockaddr *)&g_usrc, sizeof g_usrc);
}

/* ---------------- helpers ---------------- */
static void usage(const char *p)
{
    fprintf(stderr,
    "usage: %s [options]\n"
    "  --monitor               observe only, never enable the drives\n"
    "  --mode M[,M,...]        M = csp-hold|csp-sine|csp-move|csp-udp|udp|csv|cst; one for all axes,\n"
    "                          or a comma list with one per slave (modes can differ per axis)\n"
    "  --delta COUNTS --move-time S   csp-move: smooth relative move, then hold\n"
    "  --slaves N              use first N slaves (default: all found)\n"
    "  --cycle-us N            cycle time (default 1000)\n"
    "  --amp COUNTS --period S csp-sine amplitude / period\n"
    "  --vel COUNTS_PER_S      csv target   (--vel-accel; --vel-limit caps speed in csv and udp mode, 0 = no speed limit)\n"
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
    "  csp-udp: follow joint angles from a simulation over UDP (positions relative to the pose at enable)\n"
    "  udp:     same, and the laptop also chooses hold/position/velocity/torque and their targets live\n"
    "  --udp-port P            UDP port (default 5005)\n"
    "  --udp-timeout-ms N      stop all motors if no valid packet for N ms (default 200)\n"
    "  --udp-tau-ms N          target low-pass time constant, 0 = off (default 15)\n"
    "  --udp-on-timeout stop|hold  no packets: stop+disable all motors (default) or hold position and wait\n"
    "  --udp-hold-max-s S      with hold: stop after S seconds without packets (default 60)\n"
    "  --scale S[,S,...]       motor rad per joint rad, sign included (default 1)\n"
    "  --limit-deg D[,D,...]   max travel +/- from the start pose (default 30; 0 = no travel limit at all)\n"
    "  --udp-from IP           only accept packets from this IPv4 address\n"
    "  --log FILE.csv|auto     write a CSV log (auto = myact_log_YYYYmmdd_HHMMSS.csv)\n"
    "  --log-every N           log every Nth cycle (default 10 = 100 Hz at 1 ms)\n"
    "  --quiet                 no live terminal display\n"
    "  --sdo-auto              also log motor current, DC link voltage, following error, ... if the drive has them\n"
    "  --sdo IDX:SUB:TYPE:NAME[,...]  also log other drive objects, e.g. 0x2001:0:u16:temp_raw (TYPE u8 s8 u16 s16 u32 s32)\n"
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
        if (IS_UDP(a)) {
            if (a->scale == 0 || fabs(a->scale) > 100) { fprintf(stderr, "axis %d: --scale must be non-zero and <= 100\n", i); return 1; }
            if (a->limit_counts < 0) { fprintf(stderr, "axis %d: --limit-deg must be >= 0 (0 = no limit)\n", i); return 1; }
            if (cfg.udp_timeout_ms < 10 || cfg.udp_tau_ms < 0 || cfg.udp_port < 1 || cfg.udp_port > 65535) {
                fprintf(stderr, "bad --udp-port / --udp-timeout-ms (>=10) / --udp-tau-ms (>=0)\n"); return 1;
            }
        }
        if (cfg.vel_limit > 0 && fabs(a->vel) > cfg.vel_limit) { fprintf(stderr, "axis %d: --vel exceeds --vel-limit\n", i); return 1; }
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
    g_argc = argc; g_argv = argv;
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
                else if (!strcmp(m, "csp-udp"))  { cfg.m_op[n] = OPM_CSP; cfg.m_prf[n] = PRF_UDP; }
                else if (!strcmp(m, "udp"))      { cfg.m_op[n] = OPM_CSP; cfg.m_prf[n] = PRF_UDPX; }
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
        else if (!strcmp(k, "--udp-port"))       cfg.udp_port = atoi(NEXT);
        else if (!strcmp(k, "--udp-timeout-ms")) cfg.udp_timeout_ms = atoi(NEXT);
        else if (!strcmp(k, "--udp-tau-ms"))     cfg.udp_tau_ms = atof(NEXT);
        else if (!strcmp(k, "--udp-on-timeout")) {
            const char *v = NEXT;
            if (!strcmp(v, "hold")) cfg.udp_on_timeout_hold = 1;
            else if (!strcmp(v, "stop")) cfg.udp_on_timeout_hold = 0;
            else usage(argv[0]);
        }
        else if (!strcmp(k, "--udp-hold-max-s")) cfg.udp_hold_max_s = atof(NEXT);
        else if (!strcmp(k, "--scale"))          parse_list(&cfg.scale, NEXT);
        else if (!strcmp(k, "--limit-deg"))      parse_list(&cfg.limit_deg, NEXT);
        else if (!strcmp(k, "--udp-from")) {
            cfg.udp_from = NEXT;
            if (inet_pton(AF_INET, cfg.udp_from, &g_uallow) != 1) usage(argv[0]);
            g_uallow_set = 1;
        }
        else if (!strcmp(k, "--log"))         cfg.log_path = NEXT;
        else if (!strcmp(k, "--log-every"))   cfg.log_every = atoi(NEXT);
        else if (!strcmp(k, "--quiet"))       cfg.quiet = 1;
        else if (!strcmp(k, "--sdo-auto"))    cfg.sdo_auto = 1;
        else if (!strcmp(k, "--sdo"))         cfg.sdo_spec = NEXT;
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
        printf("logging to %s (every %d cycles)\n", path, cfg.log_every);
        if (path != g_log_name) snprintf(g_log_name, sizeof g_log_name, "%s", path);   /* "auto" already is g_log_name */
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
        !list_ok(&cfg.duration, "duration") ||
        !list_ok(&cfg.scale, "scale") || !list_ok(&cfg.limit_deg, "limit-deg")) {
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
        ax[i].idx = i;
        ax[i].scale = lget(&cfg.scale, i);
        ax[i].limit_counts = lget(&cfg.limit_deg, i) / 360.0 * ax[i].cpr;
    }
    if (check_axes()) { ecrt_release_master(master); return 1; }
    int need_udp = 0;
    for (int i = 0; i < n_ax; i++) if (IS_UDP(&ax[i])) need_udp = 1;
    if (need_udp) {
        if (udp_open(cfg.udp_port)) { ecrt_release_master(master); return 1; }
        printf("csp-udp: listening on UDP port %d, expecting %d joint values per packet\n", cfg.udp_port, n_ax);
    }

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
        if (a->profile == PRF_UDPX) ok = ok && a->off[E_TVEL] >= 0 && a->off[E_TTQ] >= 0;
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

    /* ---- 2b. extra readings via SDO: probe what each drive has, then write the log header ---- */
    if (cfg.sdo_auto)
        for (unsigned k = 0; k < sizeof SDO_AUTO / sizeof SDO_AUTO[0]; k++)
            sdo_add(SDO_AUTO[k].idx, SDO_AUTO[k].sub, SDO_AUTO[k].type, SDO_AUTO[k].size, SDO_AUTO[k].name);
    if (cfg.sdo_spec && sdo_parse_spec(cfg.sdo_spec)) { ecrt_release_master(master); return 1; }
    probe_sdos();
    if (g_logf) log_header();

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
        if (lat > g_lat_win_ns) g_lat_win_ns = lat;

        ecrt_master_receive(master);
        ecrt_domain_process(domain);

        ec_domain_state_t ds; ecrt_domain_state(domain, &ds);
        g_wc = (int)ds.working_counter; g_wcs = (int)ds.wc_state;
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

        if (g_usock >= 0) {
            long long un = now_ns(CLOCK_MONOTONIC);
            udp_poll(un);
            g_ufresh = g_uhave && (un - g_ulast_ns) < (long long)cfg.udp_timeout_ms * 1000000LL;
            if (bus_up && !cfg.monitor && !g_stop) {
                int armed = 0;
                for (int i = 0; i < n_ax; i++)
                    if (IS_UDP(&ax[i]) && ax[i].aligned && ax[i].phase == PH_RUN) armed = 1;
                if (armed && !g_ufresh) {
                    double idle = (un - g_ulast_ns) * 1e-9;
                    if (!cfg.udp_on_timeout_hold) {
                        fprintf(stderr, "UDP watchdog: no valid packet for %d ms, stopping all axes\n", cfg.udp_timeout_ms);
                        g_stop = 1;
                    } else if (idle > cfg.udp_hold_max_s) {
                        fprintf(stderr, "UDP: no packets for %.0f s, giving up and stopping all axes\n", idle);
                        g_stop = 1;
                    } else if (!g_ustale_msg) {
                        g_ustale_msg = 1;
                        fprintf(stderr, "UDP: no valid packet for %d ms - HOLDING position (motors stay enabled); "
                                "resumes when packets return, stops after %.0f s\n", cfg.udp_timeout_ms, cfg.udp_hold_max_s);
                    }
                } else if (g_ufresh && g_ustale_msg) {
                    g_ustale_msg = 0;
                    fprintf(stderr, "UDP: packets resumed\n");
                }
            }
        }

        all_done = 1;
        for (int i = 0; i < n_ax; i++) {
            ec_slave_config_state_t st; ecrt_slave_config_state(ax[i].sc, &st);
            ax[i].s_online = st.online; ax[i].s_al = st.al_state;
            if (bus_up) sdo_poll(&ax[i]);
            int op = st.operational && ds.wc_state == EC_WC_COMPLETE;
            if (cfg.monitor && g_stop) ax[i].phase = PH_DONE;
            axis_step(&ax[i], dom_data, op, g_stop);
            if (ax[i].phase != PH_DONE) all_done = 0;
        }

        if (g_usock >= 0 && g_ureply) { udp_reply(); g_ureply = 0; }

        if (g_logf && bus_up && (cyc % (unsigned long)cfg.log_every) == 0) {
            double tl_ = (now_ns(CLOCK_MONOTONIC) - t_log0) * 1e-9;
            for (int i = 0; i < n_ax; i++) log_push(tl_, i);
            g_lat_win_ns = 0;
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
