/*
 * bt_ctl.c - power and discoverability for the built-in Bluetooth adapter.
 *
 * Bring-up is done by /usr/bin/bt_up.sh (rootfs overlay) rather than in
 * C: the sequence is a pile of small tools with timing between them, and
 * a shell runs it the way it was tested at a prompt. Status is read straight from the kernel with hci_devinfo(),
 * not by parsing hciconfig output.
 */

#include "bt_ctl.h"
#include "bt_spp.h"

#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#define BT_CONF     "/mnt/bt.conf"
#define BT_UP       "/usr/bin/bt_up.sh"
#define BT_DOWN     "/usr/bin/bt_down.sh"
/*
 * RFCOMM channel for the WSJT-X frames. x6100_gui listens on
 * channel 1 itself (CAT over Bluetooth, cat/cat.cpp) and bt_start.sh
 * publishes that as the Serial Port record, so ours moved to 2. bt_up.sh
 * publishes it under the Dial-up Networking class (UUID 0x1103) so the
 * phone can find it by UUID without knowing the channel number - two
 * records of the same Serial Port class could not be told apart.
 */
#define BT_CHANNEL  2

static struct {
    pthread_t       thread;
    pthread_mutex_t lock;
    pthread_cond_t  cond;
    bool            thread_valid;
    bool            quit;
    bool            want_on;
    bool            pending;

    bt_ctl_state_t  state;
    char            addr[18];
    char            name[32];
} S = {
    .lock  = PTHREAD_MUTEX_INITIALIZER,
    .cond  = PTHREAD_COND_INITIALIZER,
    .state = BT_CTL_OFF,
    .name  = "X6100",
};

/* ------------------------------------------------------------------ */

static void conf_load(void)
{
    FILE *f = fopen(BT_CONF, "r");
    char  line[128];
    bool  enabled = false;

    if (f == NULL) {
        return;
    }

    while (fgets(line, sizeof(line), f) != NULL) {
        char *nl = strchr(line, '\n');

        if (nl) {
            *nl = '\0';
        }
        if (strncmp(line, "enabled=", 8) == 0) {
            enabled = (atoi(line + 8) != 0);
        } else if (strncmp(line, "name=", 5) == 0 && line[5] != '\0') {
            snprintf(S.name, sizeof(S.name), "%.31s", line + 5);
        }
    }
    fclose(f);

    if (enabled) {
        S.want_on = true;
        S.pending = true;
    }
}

static void conf_save(bool enabled)
{
    FILE *f = fopen(BT_CONF, "w");

    if (f == NULL) {
        return;
    }
    fprintf(f, "# Bluetooth SPP bridge for FT8/FT4 packets\n");
    fprintf(f, "enabled=%d\n", enabled ? 1 : 0);
    fprintf(f, "name=%s\n", S.name);
    fclose(f);
}

/* Runs a script the way a shell would and waits for it. Called only from
 * the worker thread, never from the UI thread. */
static bool run_script(const char *path, const char *arg)
{
    char cmd[256];
    int  rc;

    snprintf(cmd, sizeof(cmd), "%s '%s' %d >/dev/null 2>&1", path, arg, BT_CHANNEL);
    rc = system(cmd);
    return (rc == 0);
}

/* ------------------------------------------------------------------ */

static bool adapter_info(char *addr, size_t addr_len, bool *discoverable)
{
    struct hci_dev_info di;

    memset(&di, 0, sizeof(di));
    di.dev_id = 0;

    if (hci_devinfo(0, &di) < 0) {
        return false;
    }
    if (!hci_test_bit(HCI_UP, &di.flags)) {
        return false;
    }
    if (addr != NULL && addr_len >= 18) {
        ba2str(&di.bdaddr, addr);
    }
    if (discoverable != NULL) {
        *discoverable = hci_test_bit(HCI_ISCAN, &di.flags) != 0;
    }
    return true;
}

static void *worker(void *arg)
{
    (void)arg;

    for (;;) {
        bool on;

        pthread_mutex_lock(&S.lock);
        while (!S.quit && !S.pending) {
            pthread_cond_wait(&S.cond, &S.lock);
        }
        if (S.quit) {
            pthread_mutex_unlock(&S.lock);
            break;
        }
        S.pending = false;
        on = S.want_on;
        S.state = on ? BT_CTL_STARTING : BT_CTL_STOPPING;
        pthread_mutex_unlock(&S.lock);

        if (on) {
            bool ok = run_script(BT_UP, S.name);
            char addr[18] = "";

            if (ok) {
                ok = adapter_info(addr, sizeof(addr), NULL);
            }
            if (ok) {
                ok = bt_spp_start(BT_CHANNEL);
            }

            pthread_mutex_lock(&S.lock);
            if (S.pending) {
                /* The user changed their mind while we were starting.
                 * Leave the new request to the next pass. */
                pthread_mutex_unlock(&S.lock);
                continue;
            }
            snprintf(S.addr, sizeof(S.addr), "%s", addr);
            S.state = ok ? BT_CTL_ON : BT_CTL_FAILED;
            pthread_mutex_unlock(&S.lock);
            conf_save(ok);
        } else {
            bt_spp_stop();
            run_script(BT_DOWN, S.name);

            pthread_mutex_lock(&S.lock);
            S.addr[0] = '\0';
            if (!S.pending) {
                S.state = BT_CTL_OFF;
            }
            pthread_mutex_unlock(&S.lock);
            conf_save(false);
        }
    }

    return NULL;
}

/* ------------------------------------------------------------------ */

/* ft8_udp_init() runs only when the FT8 window opens, and the Bluetooth
 * window may well be opened first, so every entry point makes sure the
 * worker thread exists; starting it twice is a no-op. */
static void ensure_thread(void)
{
    bool start = false;

    pthread_mutex_lock(&S.lock);
    if (!S.thread_valid) {
        conf_load();
        S.quit = false;
        start = true;
    }
    pthread_mutex_unlock(&S.lock);

    if (!start) {
        return;
    }

    if (pthread_create(&S.thread, NULL, worker, NULL) == 0) {
        S.thread_valid = true;
        pthread_mutex_lock(&S.lock);
        if (S.pending) {
            pthread_cond_signal(&S.cond);
        }
        pthread_mutex_unlock(&S.lock);
    }
}

void bt_ctl_init(void)
{
    ensure_thread();
}

void bt_ctl_deinit(void)
{
    if (!S.thread_valid) {
        return;
    }

    bt_spp_stop();

    pthread_mutex_lock(&S.lock);
    S.quit = true;
    pthread_cond_signal(&S.cond);
    pthread_mutex_unlock(&S.lock);

    pthread_join(S.thread, NULL);
    S.thread_valid = false;
}

void bt_ctl_request(bool on)
{
    ensure_thread();

    pthread_mutex_lock(&S.lock);
    S.want_on = on;
    S.pending = true;
    pthread_cond_signal(&S.cond);
    pthread_mutex_unlock(&S.lock);
}

bt_ctl_state_t bt_ctl_state(void)
{
    bt_ctl_state_t st;

    ensure_thread();

    pthread_mutex_lock(&S.lock);
    st = S.state;
    pthread_mutex_unlock(&S.lock);
    return st;
}

const char *bt_ctl_addr(void)
{
    return S.addr;
}

const char *bt_ctl_name(void)
{
    return S.name;
}

void bt_ctl_set_name(const char *name)
{
    if (name == NULL || name[0] == '\0') {
        return;
    }
    pthread_mutex_lock(&S.lock);
    snprintf(S.name, sizeof(S.name), "%s", name);
    pthread_mutex_unlock(&S.lock);
    conf_save(bt_ctl_state() == BT_CTL_ON);
}

bool bt_ctl_discoverable(void)
{
    bool disc = false;

    if (!adapter_info(NULL, 0, &disc)) {
        return false;
    }
    return disc;
}
