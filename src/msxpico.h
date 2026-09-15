/* emsxpico — the C ABI of libmsxpico.
 *
 * One firmware instance per loaded library image. Bus calls are synchronous:
 * each is one complete Z80 cycle. After a REBOOT or HALTED event the instance
 * is dead; the host unloads the library, loads a fresh copy and calls
 * msxpico_init again. See docs/superpowers/specs/2026-09-09-phase-1-1-firmware-on-host-design.md §5,
 * and docs/superpowers/specs/2026-09-14-phase-4-connectivity-design.md §4 for version 4. */
#ifndef MSXPICO_H
#define MSXPICO_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MSXPICO_ABI_VERSION 4u

/* The levels msxpico_log_fn is called with. Version 1 left them unnamed and
 * every host hardcoded the fake SDK's values; they are these. */
enum {
    MSXPICO_LOG_DEBUG = 0,
    MSXPICO_LOG_INFO = 1,
    MSXPICO_LOG_WARN = 2,
    MSXPICO_LOG_ERROR = 3
};

typedef void (*msxpico_log_fn)(int level, const char *msg, void *user);

typedef struct msxpico_config {
    uint32_t struct_size;          /* sizeof(msxpico_config), for compatible evolution */
    const char *flash_image_path;  /* required; 16 MiB; opened read/write */
    const char *sd_image_path;     /* NULL: no card (phase 2 adds the driver) */
    msxpico_log_fn log;            /* NULL: stderr */
    void *log_user;
    /* The watchdog scratch registers this instance starts with.
     *
     * On the chip these are hardware registers that survive a warm reset,
     * which is what the firmware uses them for: it writes the reboot reason
     * and the firmware image to start into them and then resets, and the
     * bootloader reads them back on the way up. They are cleared only by
     * losing power.
     *
     * A host instance is a library image, and a reset is a new image, so that
     * lifetime has to come from the host: pass the `scratch` of the REBOOT
     * event that ended the previous instance, and all zeroes for a cold start.
     * A host that passes zeroes every time makes the firmware forget across
     * every reboot -- which is how the menu's FM toggle used to switch image
     * for exactly one reboot and be undone by the next. */
    uint32_t scratch[4];
    /* The ESP8266 model's Unix socket (version 4). NULL: no module -- the
     * firmware finds WIFI_RX low and hides every Wi-Fi feature, as on a board
     * without the ESP. A path nobody listens on is the same, logged once.
     * Connected in msxpico_init, before the firmware starts, because the
     * firmware reads the pin once at boot. */
    const char *esp8266_socket_path;
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
/* Sound, as the cartridge's I2S output carries it: interleaved left/right,
 * int16 (the top 16 of the DAC's 24 bits), exactly `frames` frames written
 * to `stereo`. Every frame is one step of the firmware's sound interrupt, so
 * the caller's cadence is the audio's clock: call it from the emulated
 * machine's mixer, for the samples emulated time has produced. May be called
 * from a thread other than the bus's. Returns `frames`; 0 only for a NULL
 * buffer. */
size_t   msxpico_pull_samples(int16_t *stereo, size_t frames);
/* The firmware's current output sample rate in Hz, derived from the I2S
 * divider it programmed (version 3). Valid once msxpico_init has returned;
 * changes while the firmware plays a WAV or MP3 at the file's rate and
 * returns to the fixed rate afterwards. May be called from any thread. */
uint32_t msxpico_sample_rate(void);

#ifdef __cplusplus
}
#endif
#endif
