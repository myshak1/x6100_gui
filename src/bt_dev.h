/*
 * bt_dev.h - known Bluetooth devices, for the table in dialog_bt.
 *
 * Every call here shells out to bluetoothctl and each one can take a
 * noticeable fraction of a second, so the work happens on a thread and
 * the dialog reads a cached snapshot.
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BT_DEV_MAX 12

typedef struct {
    char addr[18];
    char name[32];
    bool connected;
    bool audio;      /* PulseAudio has a card for it, so it can play */
} bt_dev_t;

void bt_dev_init(void);
void bt_dev_deinit(void);

/* Asynchronous refresh of the snapshot. Call from the dialog timer. */
void bt_dev_refresh(void);

int  bt_dev_count(void);
bool bt_dev_get(int idx, bt_dev_t *out);

void bt_dev_connect(const char *addr);
void bt_dev_disconnect(const char *addr);

/* Disconnect and remove the pairing ("bluetoothctl remove"). The next
 * Connect pairs again, so the headset must be in pairing mode then. */
void bt_dev_forget(const char *addr);

/* Runs an inquiry for a fixed period, then refreshes the snapshot. */
void bt_dev_scan(void);
bool bt_dev_scanning(void);

/* What the worker is doing, "" when idle. For the status panel. */
const char *bt_dev_activity(void);

#ifdef __cplusplus
}
#endif
