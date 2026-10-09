#ifndef USB_CONTROLLERS_H
#define USB_CONTROLLERS_H

#include "pad_types.h"
#include <stdint.h>
#include <stddef.h>

typedef enum {
    CTRL_UNKNOWN = 0,
    CTRL_NINTENDO_SWITCH_PRO,
    CTRL_SONY_DS4,
    CTRL_SONY_DS3,
    CTRL_XBOX_XINPUT,
    CTRL_8BITDO_ADAPTER,
    CTRL_GENERIC_HID,
    CTRL_GAMESIR            /* GameSir (VID 0x3537): GIP, XInput or GameSir HID protocol, chosen by descriptors */
} usb_controller_type_t;

#define USB_VID_GAMESIR 0x3537

usb_controller_type_t usb_identify_controller(uint16_t vid, uint16_t pid, const char **name_out);
int usb_init_controller_handshake(int fd, usb_controller_type_t type);
int usb_parse_input_report(usb_controller_type_t type, const uint8_t *data, size_t len, pad_state_t *st);
uint8_t axis16_to_u8(int16_t val, int invert_y);

/* Strict parsers: return 1 and overwrite *st only for a genuine input-state
 * packet, 0 (leaving *st untouched) for anything else. They never fall back
 * to the generic HID guesser, so status/keep-alive packets cannot inject
 * phantom presses. */
int usb_parse_x360_report(const uint8_t *data, size_t len, pad_state_t *st);
int usb_parse_gamesir_hid_report(const uint8_t *data, size_t len, pad_state_t *st);

/* Xbox 360 wireless receiver channel (interface FF/5D/81), framed like xpad's
 * xpad360w_process_packet. LINK_LOST also sets *st to neutral. */
typedef enum {
    X360W_PKT_UNKNOWN = 0,
    X360W_PKT_STATE,       /* pad data decoded into *st */
    X360W_PKT_STATUS,      /* battery, trailer, announce, "pad connected" */
    X360W_PKT_LINK_LOST    /* the pad switched off or left range */
} x360w_pkt_t;
x360w_pkt_t usb_parse_x360w_report(const uint8_t *data, size_t len, pad_state_t *st);

/* Builds the GameSir HID "full report" mode-switch output report (A2 01 00, padded).
 * Returns the number of bytes written (64) or 0 when the buffer is too small. */
size_t usb_gamesir_build_mode_switch(uint8_t *out, size_t cap);

#endif /* USB_CONTROLLERS_H */
