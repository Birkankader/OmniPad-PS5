#ifndef USB_DESC_H
#define USB_DESC_H

#include <stdint.h>
#include <stddef.h>

/* Pure USB configuration-descriptor helpers (no I/O, host-testable). */

#define USB_DESC_MAX_IFACES 16

typedef enum {
    USB_ROUTE_NONE = 0,
    USB_ROUTE_LEGACY,   /* No class hint: first interrupt/bulk interface, VID/PID heuristics */
    USB_ROUTE_GIP,      /* Xbox One / Series "GIP" interface (class FF, subclass 47, proto D0) */
    USB_ROUTE_X360,     /* Xbox 360 "XInput" interface (class FF, subclass 5D) */
    USB_ROUTE_HID       /* HID class interface that is not a boot keyboard/mouse */
} usb_route_kind_t;

typedef struct {
    uint8_t  number;
    uint8_t  alt;
    uint8_t  cls;
    uint8_t  subcls;
    uint8_t  proto;
    uint8_t  ep_in;            /* first interrupt/bulk IN endpoint address, 0 if none */
    uint8_t  ep_out;           /* first interrupt/bulk OUT endpoint address, 0 if none */
    uint16_t ep_in_maxpkt;
    uint16_t ep_out_maxpkt;
    uint16_t hid_report_len;   /* wDescriptorLength of the HID report descriptor, 0 if none */
} usb_iface_info_t;

typedef struct {
    int              n_ifaces;
    usb_iface_info_t ifaces[USB_DESC_MAX_IFACES];
} usb_config_info_t;

typedef struct {
    usb_route_kind_t kind;
    uint8_t          iface;    /* bInterfaceNumber */
    uint8_t          alt;      /* bAlternateSetting */
    uint8_t          proto;    /* bInterfaceProtocol (0x81 = Xbox 360 wireless receiver channel) */
    uint8_t          ep_in;
    uint8_t          ep_out;   /* 0 when the interface has no OUT endpoint */
    uint16_t         in_maxpkt;
    uint16_t         out_maxpkt;
    uint16_t         hid_report_len;
    uint8_t          iface_count;   /* distinct bInterfaceNumber values in the configuration */
    uint8_t          aux_iface;     /* GIP: the other FF/47/D0 interface (audio), 0xff if none */
    uint8_t          aux_has_alt1;  /* aux_iface has an alternate setting 1 */
    /* HID routes, from the report descriptor (usb_desc_refine_hid_route) */
    uint8_t          hid_report_id;     /* report ID of the gamepad input report, 0 = IDs unused */
    uint16_t         hid_input_len;     /* gamepad input report length incl. the ID byte, 0 = unknown */
    uint16_t         hid_max_input_len; /* longest input report of the interface, 0 = unknown */
    uint8_t          hid_ignore_ids[32]; /* bitmap: report IDs of keyboard/mouse/consumer inputs */
} usb_input_route_t;

/* What interface selection and transfer sizing need from a HID report descriptor */
typedef struct {
    int      has_gamepad;        /* Generic Desktop Joystick/Gamepad/Multi-axis application collection */
    int      uses_report_ids;
    uint8_t  gamepad_report_id;
    uint16_t gamepad_input_len;  /* bytes incl. the report ID, 0 if the collection has no input */
    uint16_t max_input_len;      /* longest input report incl. the report ID */
    uint8_t  ignore_ids[32];     /* bitmap: report IDs whose inputs belong to keyboard, mouse
                                  * or consumer collections only (never pad state) */
} usb_hid_info_t;

/* 1 when report ID id belongs to a keyboard/mouse/consumer collection of the route */
static inline int usb_route_hid_id_ignored(const usb_input_route_t *r, uint8_t id)
{
    return (r->hid_ignore_ids[id >> 3] >> (id & 7)) & 1;
}

/* Fetches the HID report descriptor of iface into buf; returns its length or -1. */
typedef int (*usb_hid_fetch_fn)(void *user, const usb_iface_info_t *iface, uint8_t *buf, size_t cap);

/* Parses a full configuration descriptor. Returns the number of interface
 * (alternate setting) entries found, or -1 on malformed input. */
int usb_desc_parse_config(const uint8_t *raw, size_t len, usb_config_info_t *out);

/* Picks the interface that carries gamepad input:
 * GIP > XInput (gamepad protocol first) > HID (non boot keyboard/mouse) > first interrupt IN.
 * Only alternate settings with an interrupt/bulk IN endpoint qualify.
 * Returns 1 and fills route on success, 0 when nothing usable exists. */
int usb_desc_select_input(const usb_config_info_t *cfg, usb_input_route_t *route);

/* Parses a HID report descriptor. Returns 1 on success, 0 on malformed input. */
int usb_hid_parse_report_desc(const uint8_t *d, size_t len, usb_hid_info_t *out);

/* For a HID route, reads the report descriptor of every candidate HID interface
 * and moves the route to the first one with a gamepad collection, filling the
 * hid_* fields. Without one the route keeps its interface and only gets sizes.
 * Returns 1 when a gamepad collection was found. */
int usb_desc_refine_hid_route(const usb_config_info_t *cfg, usb_input_route_t *route,
                              usb_hid_fetch_fn fetch, void *user);

const char *usb_route_name(usb_route_kind_t kind);

#endif /* USB_DESC_H */
