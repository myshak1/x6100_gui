/*
 * bt_dev.h - known Bluetooth devices, for the table in dialog_bt.
 *
 * Everything goes to BlueZ over D-Bus on a worker thread (pairing can
 * take many seconds), and the window reads a cached snapshot.
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
    bool audio;      /* headset or speaker the radio can play to (never a phone) */
} bt_dev_t;

void bt_dev_init(void);
void bt_dev_deinit(void);

/* Asynchronous refresh of the snapshot. Call from the dialog timer. */
void bt_dev_refresh(void);

int  bt_dev_count(void);
bool bt_dev_get(int idx, bt_dev_t *out);

/* One device at a time: Connect blocks every other paired device
 * (BlueZ "Blocked", shown as "(off)") and lets this one in. A headset is
 * then connected (paired first when needed); a phone only unblocked, it
 * connects SPP from its app. Disconnect blocks the device, so a phone
 * cannot come straight back. */
void bt_dev_connect(const char *addr);
void bt_dev_disconnect(const char *addr);

/* Disconnect and remove the pairing (Adapter1.RemoveDevice). The next
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
