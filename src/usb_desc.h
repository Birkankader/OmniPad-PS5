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
    uint8_t          ep_in;
    uint8_t          ep_out;   /* 0 when the interface has no OUT endpoint */
    uint16_t         in_maxpkt;
    uint16_t         out_maxpkt;
    uint16_t         hid_report_len;
    uint8_t          iface_count;   /* distinct bInterfaceNumber values in the configuration */
} usb_input_route_t;

/* Parses a full configuration descriptor. Returns the number of interface
 * (alternate setting) entries found, or -1 on malformed input. */
int usb_desc_parse_config(const uint8_t *raw, size_t len, usb_config_info_t *out);

/* Picks the interface that carries gamepad input:
 * GIP > XInput (gamepad protocol first) > HID (non boot keyboard/mouse) > first interrupt IN.
 * Only alternate settings with an interrupt/bulk IN endpoint qualify.
 * Returns 1 and fills route on success, 0 when nothing usable exists. */
int usb_desc_select_input(const usb_config_info_t *cfg, usb_input_route_t *route);

const char *usb_route_name(usb_route_kind_t kind);

#endif /* USB_DESC_H */
