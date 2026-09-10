/* emsxpico — the C ABI of libmsxpico.
 *
 * One firmware instance per loaded library image. Bus calls are synchronous:
 * each is one complete Z80 cycle. After a REBOOT or HALTED event the instance
 * is dead; the host unloads the library, loads a fresh copy and calls
 * msxpico_init again. See docs/superpowers/specs/2026-09-09-phase-1-1-firmware-on-host-design.md §5. */
#ifndef MSXPICO_H
#define MSXPICO_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MSXPICO_ABI_VERSION 1u

typedef void (*msxpico_log_fn)(int level, const char *msg, void *user);

typedef struct msxpico_config {
    uint32_t struct_size;          /* sizeof(msxpico_config), for compatible evolution */
    const char *flash_image_path;  /* required; 16 MiB; opened read/write */
    const char *sd_image_path;     /* NULL: no card (phase 2 adds the driver) */
    msxpico_log_fn log;            /* NULL: stderr */
    void *log_user;
} msxpico_config;

typedef enum msxpico_event_kind {
    MSXPICO_EVENT_NONE = 0,
    MSXPICO_EVENT_REBOOT = 1,      /* watchdog_reboot / reset_usb_boot */
    MSXPICO_EVENT_HALTED = 2       /* firmware stuck in a fatal loop, or a fake-SDK failure */
} msxpico_event_kind;

typedef struct msxpico_event {
    msxpico_event_kind kind;
    uint32_t scratch[4];           /* watchdog scratch registers at the reboot */
} msxpico_event;

enum {
    MSXPICO_OK = 0,
    MSXPICO_ERR_CONFIG = -1,       /* bad struct_size or missing path */
    MSXPICO_ERR_FLASH = -2,        /* image missing, wrong size, or cannot map at the fixed address */
    MSXPICO_ERR_HALTED = -3,       /* the firmware did not reach the bus (see the log and poll_event) */
    MSXPICO_ERR_STALE = -4         /* init after shutdown in the same image: load a fresh copy */
};

uint32_t msxpico_abi_version(void);
int      msxpico_init(const msxpico_config *cfg);
void     msxpico_shutdown(void);

int      msxpico_read(uint16_t addr);               /* 0..255, or -1: bus not driven */
void     msxpico_write(uint16_t addr, uint8_t data);
int      msxpico_read_io(uint16_t port);            /* full 16-bit I/O address */
void     msxpico_write_io(uint16_t port, uint8_t data);

int      msxpico_poll_event(msxpico_event *out);    /* 1 if an event was pending */
size_t   msxpico_pull_samples(int16_t *stereo, size_t frames); /* 0 until phase 3 */

#ifdef __cplusplus
}
#endif
#endif
