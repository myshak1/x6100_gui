/*
 * bt_dev.c - known Bluetooth devices.
 *
 * Finding out which devices are connected is the awkward part.
 * "bluetoothctl devices Connected" is a newer form that BlueZ 5.65 on
 * this radio does not understand - it returns nothing at all rather
 * than an error - so the detection walks a chain and uses whichever
 * link answers:
 *
 *   1. bluetoothctl devices Connected   one call, newer BlueZ only
 *   2. hcitool con                      one call, deprecated but cheap
 *   3. bluetoothctl info <addr>         one call per device, last resort
 *
 * The third is capped at INFO_MAX devices; a dozen forks every couple
 * of seconds is more than this processor has to spare.
 *
 * The device list itself also needs filtering. bluetoothctl remembers
 * everything an inquiry ever saw, most of it nameless passers-by whose
 * "name" is just the address with dashes. Those are dropped.
 */

/* posix_openpt, grantpt, unlockpt, ptsname */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "bt_dev.h"

#include <pthread.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#define SCAN_SECONDS 12
#define INFO_MAX     6

/* Pairing a device that is not bonded yet (see do_pair_connect): wait
 * this long for it to show up in the inquiry. */
#define PAIR_FIND_S   30

typedef enum {
    CMD_NONE = 0,
    CMD_REFRESH,
    CMD_CONNECT,
    CMD_DISCONNECT,
    CMD_SCAN,
    CMD_FORGET,
} cmd_t;

static struct {
    pthread_t       thread;
    pthread_mutex_t lock;
    pthread_cond_t  cond;
    bool            thread_valid;
    bool            quit;

    cmd_t           cmd;
    char            cmd_addr[18];

    bt_dev_t        list[BT_DEV_MAX];
    int             count;
    bool            scanning;
    char            activity[40];
} S = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .cond = PTHREAD_COND_INITIALIZER,
};

/* ------------------------------------------------------------------ */

static void set_activity(const char *what)
{
    pthread_mutex_lock(&S.lock);
    snprintf(S.activity, sizeof(S.activity), "%s", what);
    pthread_mutex_unlock(&S.lock);
}

/* Only addresses that came from bluetoothctl output reach this, but the
 * check is cheap and keeps anything odd out of a shell command line. */
static bool addr_ok(const char *addr)
{
    int i;

    if (addr == NULL || strlen(addr) != 17) {
        return false;
    }
    for (i = 0; i < 17; i++) {
        char c = addr[i];

        if ((i % 3) == 2) {
            if (c != ':') {
                return false;
            }
        } else if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') ||
                     (c >= 'a' && c <= 'f'))) {
            return false;
        }
    }
    return true;
}

/* "Device AA:BB:CC:DD:EE:FF Some Name" */
static bool parse_device_line(const char *line, char *addr, size_t addr_sz,
                              char *name, size_t name_sz)
{
    const char *p;
    const char *sp;
    char       *nl;

    if (strncmp(line, "Device ", 7) != 0) {
        return false;
    }
    p = line + 7;
    sp = strchr(p, ' ');
    if (sp == NULL || (size_t)(sp - p) != 17) {
        return false;
    }

    snprintf(addr, addr_sz, "%.17s", p);
    if (!addr_ok(addr)) {
        return false;
    }

    snprintf(name, name_sz, "%s", sp + 1);
    nl = strchr(name, '\n');
    if (nl != NULL) {
        *nl = '\0';
    }
    return true;
}

/* An inquiry leaves behind every device that ever answered, and the
 * nameless ones come back as their own address with dashes. Nobody
 * wants to scroll past those to reach their headphones. */
static bool name_is_placeholder(const char *addr, const char *name)
{
    int i;

    if (strlen(name) != 17) {
        return false;
    }
    for (i = 0; i < 17; i++) {
        if ((i % 3) == 2) {
            if (name[i] != '-') {
                return false;
            }
        } else if (name[i] != addr[i]) {
            return false;
        }
    }
    return true;
}

/* Fills in the connected flags. Returns false when this method could
 * not tell us anything, so the caller can try the next one. */
static bool mark_connected_bluetoothctl(bt_dev_t *list, int n)
{
    FILE *p = popen("bluetoothctl devices Connected 2>/dev/null", "r");
    char  line[256];
    bool  any = false;
    int   i;

    if (p == NULL) {
        return false;
    }
    while (fgets(line, sizeof(line), p) != NULL) {
        char addr[18];
        char name[32];

        if (!parse_device_line(line, addr, sizeof(addr), name, sizeof(name))) {
            continue;
        }
        any = true;
        for (i = 0; i < n; i++) {
            if (strcmp(list[i].addr, addr) == 0) {
                list[i].connected = true;
            }
        }
    }
    pclose(p);
    return any;
}

/* "> ACL 50:C2:ED:96:B6:72 handle 1 state 1 lm PERIPHERAL" */
static bool mark_connected_hcitool(bt_dev_t *list, int n)
{
    FILE *p = popen("hcitool con 2>/dev/null", "r");
    char  line[256];
    bool  ran = false;
    int   i;

    if (p == NULL) {
        return false;
    }
    while (fgets(line, sizeof(line), p) != NULL) {
        ran = true;
        for (i = 0; i < n; i++) {
            if (list[i].addr[0] != '\0' &&
                strstr(line, list[i].addr) != NULL) {
                list[i].connected = true;
            }
        }
    }
    if (pclose(p) != 0) {
        return false;
    }
    return ran;
}

static void mark_connected_per_device(bt_dev_t *list, int n)
{
    char cmd[96];
    char reply[128];
    int  i;

    for (i = 0; i < n && i < INFO_MAX; i++) {
        FILE *p;

        snprintf(cmd, sizeof(cmd),
                 "bluetoothctl info %.17s 2>/dev/null | "
                 "grep -m1 '^\tConnected:'",
                 list[i].addr);
        p = popen(cmd, "r");
        if (p == NULL) {
            continue;
        }
        if (fgets(reply, sizeof(reply), p) != NULL &&
            strstr(reply, "yes") != NULL) {
            list[i].connected = true;
        }
        pclose(p);
    }
}

static void addr_to_underscores(const char *addr, char *out, size_t out_sz)
{
    size_t i;

    snprintf(out, out_sz, "%.17s", addr);
    for (i = 0; i < out_sz && out[i] != '\0'; i++) {
        if (out[i] == ':') {
            out[i] = '_';
        }
    }
}

static void do_refresh(void)
{
    bt_dev_t tmp[BT_DEV_MAX];
    int      n = 0;
    FILE    *p;
    char     line[256];
    int      i;

    memset(tmp, 0, sizeof(tmp));

    p = popen("bluetoothctl devices 2>/dev/null", "r");
    if (p != NULL) {
        while (n < BT_DEV_MAX && fgets(line, sizeof(line), p) != NULL) {
            if (!parse_device_line(line, tmp[n].addr, sizeof(tmp[n].addr),
                                   tmp[n].name, sizeof(tmp[n].name))) {
                continue;
            }
            if (name_is_placeholder(tmp[n].addr, tmp[n].name)) {
                continue;
            }
            n++;
        }
        pclose(p);
    }

    if (!mark_connected_bluetoothctl(tmp, n)) {
        if (!mark_connected_hcitool(tmp, n)) {
            mark_connected_per_device(tmp, n);
        }
    }

    /* A device can play audio only once PulseAudio has made a card for
     * it, which is a better test than guessing from the name. */
    p = popen("pactl list short cards 2>/dev/null", "r");
    if (p != NULL) {
        while (fgets(line, sizeof(line), p) != NULL) {
            for (i = 0; i < n; i++) {
                char under[24];

                addr_to_underscores(tmp[i].addr, under, sizeof(under));
                if (strstr(line, under) != NULL) {
                    tmp[i].audio = true;
                }
            }
        }
        pclose(p);
    }

    pthread_mutex_lock(&S.lock);
    memcpy(S.list, tmp, sizeof(tmp));
    S.count = n;
    pthread_mutex_unlock(&S.lock);
}

/* Has this device been bonded before? Connecting an unpaired device
 * fails with br-connection-unknown, which says nothing useful to
 * anybody. */
static bool is_paired(const char *addr)
{
    char  cmd[96];
    char  reply[128];
    FILE *p;
    bool  paired = false;

    snprintf(cmd, sizeof(cmd),
             "bluetoothctl info %.17s 2>/dev/null | grep -m1 '^\tPaired:'",
             addr);
    p = popen(cmd, "r");
    if (p == NULL) {
        return false;
    }
    if (fgets(reply, sizeof(reply), p) != NULL &&
        strstr(reply, "yes") != NULL) {
        paired = true;
    }
    pclose(p);
    return paired;
}

/* Runs a command, returns true when its output contains want. */
static bool run_expect(const char *cmd, const char *want)
{
    char  line[200];
    FILE *p = popen(cmd, "r");
    bool  hit = false;

    if (p == NULL) {
        return false;
    }
    while (fgets(line, sizeof(line), p) != NULL) {
        if (strstr(line, want) != NULL) {
            hit = true;
        }
    }
    pclose(p);
    return hit;
}

/* ---- Interactive bluetoothctl on a pseudo-terminal ------------------- */

/*
 * Pairing a new device goes through ONE interactive bluetoothctl, typed
 * to over a pseudo-terminal - exactly what an operator does over SSH,
 * which is what worked on the radio:
 *
 *     scan on / (device shows up) / pair / trust / scan off / connect
 *
 * One-shot "bluetoothctl pair" calls did not: each is a new D-Bus client,
 * the device seen by the Scan button is only "temporary" in bluez once
 * that inquiry has ended, and a failed attempt makes bluez drop it -
 * the headset vanished from the list. Here the inquiry, the pairing and
 * the agent all live in the same client until the job is done.
 *
 * A pty rather than pipes so bluetoothctl runs as it does for a person
 * (readline, line-buffered output); its echo of our commands does no
 * harm, nothing below waits for text that a command line contains.
 */

/* Everything the session says and everything we type goes here, so a
 * failed pairing can be read afterwards over SSH. Overwritten each time. */
#define PAIR_LOG "/tmp/bt_pair.log"

typedef struct {
    pid_t  pid;
    int    fd;
    FILE  *log;
    int    esc;          /* escape-sequence filter state, see ctl_filter */
    size_t len;
    char   buf[4096];
} ctl_t;

static long now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long)ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static bool ctl_open(ctl_t *c)
{
    char  slave[64];
    char *name;
    int   m;

    c->pid = -1;
    c->fd = -1;
    c->len = 0;
    c->buf[0] = '\0';
    c->esc = 0;
    c->log = fopen(PAIR_LOG, "w");

    m = posix_openpt(O_RDWR | O_NOCTTY);
    if (m < 0) {
        return false;
    }
    if (grantpt(m) != 0 || unlockpt(m) != 0 || (name = ptsname(m)) == NULL) {
        close(m);
        return false;
    }
    snprintf(slave, sizeof(slave), "%s", name);

    c->pid = fork();
    if (c->pid < 0) {
        close(m);
        return false;
    }
    if (c->pid == 0) {
        /* Child: only async-signal-safe calls until exec. */
        int s;

        setsid();
        s = open(slave, O_RDWR);
        if (s < 0) {
            _exit(127);
        }
        dup2(s, STDIN_FILENO);
        dup2(s, STDOUT_FILENO);
        dup2(s, STDERR_FILENO);
        if (s > STDERR_FILENO) {
            close(s);
        }
        close(m);
        execlp("bluetoothctl", "bluetoothctl", (char *)NULL);
        _exit(127);
    }

    c->fd = m;
    return true;
}

static void ctl_send(ctl_t *c, const char *line)
{
    size_t n = strlen(line);
    ssize_t w;

    if (c->log != NULL) {
        fprintf(c->log, "\n>>> %s", line);
        fflush(c->log);
    }

    while (n > 0) {
        w = write(c->fd, line, n);
        if (w <= 0) {
            return;
        }
        line += w;
        n -= (size_t)w;
    }
}

static void ctl_clear(ctl_t *c)
{
    c->len = 0;
    c->buf[0] = '\0';
}

/* A marker in the transcript, so the log says what we waited for. */
static void ctl_note(ctl_t *c, const char *what)
{
    if (c->log != NULL) {
        fprintf(c->log, "\n=== %s\n", what);
        fflush(c->log);
    }
}

/* Appends raw pty output to c->buf as plain text: colour codes, cursor
 * and prompt-redraw sequences (ESC [ ... final, ESC ] ... BEL, ESC x),
 * carriage returns and readline's prompt markers are dropped, so a
 * search for "50:C2:..." cannot miss because readline or a colour code
 * put bytes in the middle of a line. State survives across reads. */
static void ctl_filter(ctl_t *c, const char *in, size_t n)
{
    size_t i;

    for (i = 0; i < n; i++) {
        unsigned char ch = (unsigned char)in[i];

        switch (c->esc) {
            case 1:                          /* after ESC */
                c->esc = (ch == '[') ? 2 : (ch == ']') ? 3 : 0;
                continue;
            case 2:                          /* CSI: until 0x40..0x7e */
                if (ch >= 0x40 && ch <= 0x7e) {
                    c->esc = 0;
                }
                continue;
            case 3:                          /* OSC: until BEL */
                if (ch == 0x07) {
                    c->esc = 0;
                }
                continue;
            default:
                break;
        }
        if (ch == 0x1b) {
            c->esc = 1;
            continue;
        }
        if (ch < 0x20 && ch != '\n') {
            continue;
        }
        if (c->len < sizeof(c->buf) - 1) {
            c->buf[c->len++] = (char)ch;
        }
    }
    c->buf[c->len] = '\0';
}

/* strstr for "x|y|z": any of the alternatives. */
static bool ctl_has(const char *buf, const char *pat)
{
    char        one[64];
    const char *bar;

    while (pat != NULL && *pat != '\0') {
        bar = strchr(pat, '|');
        snprintf(one, sizeof(one), "%.*s",
                 (int)(bar != NULL ? (size_t)(bar - pat) : strlen(pat)), pat);
        if (one[0] != '\0' && strstr(buf, one) != NULL) {
            return true;
        }
        pat = (bar != NULL) ? bar + 1 : NULL;
    }
    return false;
}

/* Waits for text a (returns 1) or b (returns 2) in the output, 0 on
 * timeout or when bluetoothctl went away.
 *
 * The deadline is wall time: during an inquiry bluetoothctl prints an
 * RSSI change every moment, so counting only idle polls (the first
 * version) never timed out and the window sat on "connecting..."
 * forever.
 *
 * bluez may ask the session's agent to confirm a passkey or authorize a
 * service ("... (yes/no):"); nobody can type on the radio, so it is
 * answered yes here - the same Just Works the operator chose by
 * pressing Connect. */
static int ctl_wait(ctl_t *c, const char *a, const char *b, int timeout_ms)
{
    long deadline = now_ms() + timeout_ms;

    for (;;) {
        struct pollfd pfd = { .fd = c->fd, .events = POLLIN };
        ssize_t       r;
        char         *q;
        long          left;

        if (ctl_has(c->buf, a)) {
            return 1;
        }
        if (ctl_has(c->buf, b)) {
            return 2;
        }
        left = deadline - now_ms();
        if (left <= 0) {
            return 0;
        }
        if (poll(&pfd, 1, left > 200 ? 200 : (int)left) <= 0) {
            continue;
        }
        if (c->len > sizeof(c->buf) - 1100) {   /* room for one read */
            /* Keep the tail; everything we look for is recent. */
            size_t keep = 1024;

            memmove(c->buf, c->buf + c->len - keep, keep);
            c->len = keep;
            c->buf[c->len] = '\0';
        }
        {
            char   raw[1024];
            size_t old = c->len;

            r = read(c->fd, raw, sizeof(raw));
            if (r <= 0) {
                return 0;    /* EIO once the child has exited */
            }
            ctl_filter(c, raw, (size_t)r);
            if (c->log != NULL) {
                fwrite(c->buf + old, 1, c->len - old, c->log);
                fflush(c->log);
            }
        }

        q = strstr(c->buf, "(yes/no)");
        if (q != NULL) {
            memcpy(q, "(yes/ok)", 8);     /* answer each prompt once */
            ctl_send(c, "yes\n");
        }
    }
}

static void ctl_close(ctl_t *c)
{
    int i;

    if (c->fd >= 0) {
        ctl_send(c, "quit\n");
    }
    if (c->pid > 0) {
        for (i = 0; i < 20; i++) {
            if (waitpid(c->pid, NULL, WNOHANG) == c->pid) {
                c->pid = -1;
                break;
            }
            usleep(100000);
        }
        if (c->pid > 0) {
            kill(c->pid, SIGKILL);
            waitpid(c->pid, NULL, 0);
            c->pid = -1;
        }
    }
    if (c->fd >= 0) {
        close(c->fd);
        c->fd = -1;
    }
    if (c->log != NULL) {
        fclose(c->log);
        c->log = NULL;
    }
}

typedef enum {
    PAIR_OK = 0,
    PAIR_NOT_FOUND,
    PAIR_FAILED,
    PAIR_CONNECT_FAILED,
} pair_result_t;

/* Pair, trust and connect a device that is not bonded yet. */
static pair_result_t do_pair_connect(const char *addr)
{
    static ctl_t c;
    char         line[64];
    int          r;

    set_activity("searching...");
    if (!ctl_open(&c)) {
        if (c.log != NULL) {
            fclose(c.log);
            c.log = NULL;
        }
        return PAIR_FAILED;
    }

    /* Ready once the agent is in (it is what answers the Just Works
     * exchange); carry on after a few seconds regardless. */
    (void)ctl_wait(&c, "Agent registered", NULL, 4000);
    /* Let the start-up chatter pass (it may list cached devices, ours
     * among them) so only what the inquiry finds counts below. */
    (void)ctl_wait(&c, NULL, NULL, 1000);

    ctl_clear(&c);
    ctl_send(&c, "scan on\n");
    /* [NEW] Device <addr> ... for a fresh find, [CHG] Device <addr>
     * RSSI ... for one bluez already had. Either way it is in range and
     * answering. Our own commands do not contain the address yet. */
    snprintf(line, sizeof(line), "waiting for %s (%d s)", addr, PAIR_FIND_S);
    ctl_note(&c, line);
    /* The address alone: bluetoothctl's own line layout does not matter
     * then. Not seen in time is not fatal - bluez may still have the
     * device from the window's Scan - "pair" itself will tell. */
    if (ctl_wait(&c, addr, NULL, PAIR_FIND_S * 1000) != 1) {
        ctl_note(&c, "not seen by the inquiry, trying pair anyway");
    }

    set_activity("pairing...");
    ctl_clear(&c);
    snprintf(line, sizeof(line), "pair %s\n", addr);
    ctl_send(&c, line);
    r = ctl_wait(&c, "Pairing successful|AlreadyExists",
                 "Failed to pair|not available", 30000);
    if (r != 1) {
        bool missing = (strstr(c.buf, "not available") != NULL);

        ctl_note(&c, missing ? "result: not found" : "result: pair failed");
        ctl_send(&c, "scan off\n");
        ctl_close(&c);
        return missing ? PAIR_NOT_FOUND : PAIR_FAILED;
    }

    ctl_clear(&c);
    snprintf(line, sizeof(line), "trust %s\n", addr);
    ctl_send(&c, line);
    (void)ctl_wait(&c, "trust succeeded", "Failed", 5000);

    /* The inquiry slows paging down; it has done its job. */
    ctl_clear(&c);
    ctl_send(&c, "scan off\n");
    (void)ctl_wait(&c, "Discovery stopped", NULL, 3000);

    set_activity("connecting...");
    ctl_clear(&c);
    snprintf(line, sizeof(line), "connect %s\n", addr);
    ctl_send(&c, line);
    r = ctl_wait(&c, "Connection successful", "Failed to connect|not available",
                 25000);
    ctl_note(&c, (r == 1) ? "result: connected" : "result: connect failed");

    ctl_close(&c);
    return (r == 1) ? PAIR_OK : PAIR_CONNECT_FAILED;
}

static void do_connect(const char *addr, bool connect)
{
    char cmd[128];
    int  rc = 0;

    if (!addr_ok(addr)) {
        return;
    }

    set_activity(connect ? "connecting..." : "disconnecting...");

    if (connect) {
        /* Pair first when needed. The button says Connect because that
         * is what the operator wants; pairing is plumbing. It needs the
         * headset in pairing mode, and when that is missing the state
         * line says so instead of the device silently vanishing. */
        if (!is_paired(addr)) {
            pair_result_t pr = do_pair_connect(addr);

            sleep(1);
            do_refresh();
            switch (pr) {
                case PAIR_OK:
                    /* PulseAudio makes its card on the connection
                     * event; look again in a moment. */
                    sleep(2);
                    do_refresh();
                    set_activity("");
                    break;
                case PAIR_NOT_FOUND:
                    set_activity("Not found");
                    break;
                case PAIR_CONNECT_FAILED:
                    set_activity("Connect failed");
                    break;
                default:
                    set_activity("Pairing failed");
                    break;
            }
            return;
        }
        set_activity("connecting...");

        /* Trust as well, so it comes back on its own next time without
         * anyone having to open this window. */
        snprintf(cmd, sizeof(cmd), "bluetoothctl trust %s >/dev/null 2>&1",
                 addr);
        rc = system(cmd);
        (void)rc;

        snprintf(cmd, sizeof(cmd), "bluetoothctl connect %s 2>&1", addr);
        if (!run_expect(cmd, "Connection successful")) {
            /* Typical cause: the headset refuses (busy with two other
             * sources, or it lost its key for us -> Forget, pair again). */
            sleep(1);
            do_refresh();
            set_activity("Connect failed");
            return;
        }
    } else {
        snprintf(cmd, sizeof(cmd),
                 "bluetoothctl disconnect %s >/dev/null 2>&1", addr);
        rc = system(cmd);
        (void)rc;
    }

    /* PulseAudio only notices a device when the connection event
     * arrives, so give it a moment before looking for its card. */
    sleep(2);
    do_refresh();
    set_activity("");
}

/* Drop the pairing on our side. Needed when the headset has lost or
 * replaced its key for this radio (reset, paired elsewhere): bluez
 * keeps offering the old key, the headset refuses, and nothing short of
 * forgetting it and pairing afresh with the headset in pairing mode
 * gets out of that. */
static void do_forget(const char *addr)
{
    char cmd[128];
    int  rc;

    if (!addr_ok(addr)) {
        return;
    }

    set_activity("removing...");
    snprintf(cmd, sizeof(cmd),
             "bluetoothctl disconnect %s >/dev/null 2>&1", addr);
    rc = system(cmd);
    snprintf(cmd, sizeof(cmd),
             "bluetoothctl remove %s >/dev/null 2>&1", addr);
    rc = system(cmd);
    (void)rc;

    do_refresh();
    set_activity("");
}

static void do_scan(void)
{
    char cmd[96];
    int  rc;

    pthread_mutex_lock(&S.lock);
    S.scanning = true;
    pthread_mutex_unlock(&S.lock);
    set_activity("scanning...");

    snprintf(cmd, sizeof(cmd),
             "bluetoothctl --timeout %d scan on >/dev/null 2>&1",
             SCAN_SECONDS);
    rc = system(cmd);
    (void)rc;

    do_refresh();

    pthread_mutex_lock(&S.lock);
    S.scanning = false;
    pthread_mutex_unlock(&S.lock);
    set_activity("");
}

static void *worker(void *arg)
{
    (void)arg;

    for (;;) {
        cmd_t cmd;
        char  addr[18];

        pthread_mutex_lock(&S.lock);
        while (!S.quit && S.cmd == CMD_NONE) {
            pthread_cond_wait(&S.cond, &S.lock);
        }
        if (S.quit) {
            pthread_mutex_unlock(&S.lock);
            break;
        }
        cmd = S.cmd;
        S.cmd = CMD_NONE;
        snprintf(addr, sizeof(addr), "%s", S.cmd_addr);
        pthread_mutex_unlock(&S.lock);

        switch (cmd) {
            case CMD_REFRESH:
                do_refresh();
                break;
            case CMD_CONNECT:
                do_connect(addr, true);
                break;
            case CMD_DISCONNECT:
                do_connect(addr, false);
                break;
            case CMD_SCAN:
                do_scan();
                break;
            case CMD_FORGET:
                do_forget(addr);
                break;
            default:
                break;
        }
    }

    return NULL;
}

static void ensure_thread(void)
{
    bool start = false;

    pthread_mutex_lock(&S.lock);
    if (!S.thread_valid) {
        S.quit = false;
        start = true;
    }
    pthread_mutex_unlock(&S.lock);

    if (start && pthread_create(&S.thread, NULL, worker, NULL) == 0) {
        S.thread_valid = true;
    }
}

static void post(cmd_t cmd, const char *addr)
{
    ensure_thread();

    pthread_mutex_lock(&S.lock);
    /* A scan holds the worker for a dozen seconds; do not pile requests
     * behind it, the timer would queue one every second. */
    if (S.scanning && cmd == CMD_REFRESH) {
        pthread_mutex_unlock(&S.lock);
        return;
    }
    /* There is one slot. The timer's refresh must not overwrite a
     * connect, disconnect or forget the worker has not picked up yet
     * (it may be busy for a few seconds with the previous refresh). */
    if (cmd == CMD_REFRESH && S.cmd != CMD_NONE) {
        pthread_mutex_unlock(&S.lock);
        return;
    }
    S.cmd = cmd;
    if (addr != NULL) {
        snprintf(S.cmd_addr, sizeof(S.cmd_addr), "%s", addr);
    }
    pthread_cond_signal(&S.cond);
    pthread_mutex_unlock(&S.lock);
}

/* ------------------------------------------------------------------ */

void bt_dev_init(void)
{
    ensure_thread();
    post(CMD_REFRESH, NULL);
}

void bt_dev_deinit(void)
{
    if (!S.thread_valid) {
        return;
    }

    pthread_mutex_lock(&S.lock);
    S.quit = true;
    pthread_cond_signal(&S.cond);
    pthread_mutex_unlock(&S.lock);

    pthread_join(S.thread, NULL);
    S.thread_valid = false;
}

void bt_dev_refresh(void)
{
    post(CMD_REFRESH, NULL);
}

int bt_dev_count(void)
{
    int n;

    pthread_mutex_lock(&S.lock);
    n = S.count;
    pthread_mutex_unlock(&S.lock);
    return n;
}

bool bt_dev_get(int idx, bt_dev_t *out)
{
    bool ok = false;

    if (out == NULL) {
        return false;
    }

    pthread_mutex_lock(&S.lock);
    if (idx >= 0 && idx < S.count) {
        *out = S.list[idx];
        ok = true;
    }
    pthread_mutex_unlock(&S.lock);
    return ok;
}

void bt_dev_connect(const char *addr)
{
    post(CMD_CONNECT, addr);
}

void bt_dev_disconnect(const char *addr)
{
    post(CMD_DISCONNECT, addr);
}

void bt_dev_forget(const char *addr)
{
    post(CMD_FORGET, addr);
}

void bt_dev_scan(void)
{
    post(CMD_SCAN, NULL);
}

bool bt_dev_scanning(void)
{
    bool s;

    pthread_mutex_lock(&S.lock);
    s = S.scanning;
    pthread_mutex_unlock(&S.lock);
    return s;
}

const char *bt_dev_activity(void)
{
    return S.activity;
}
