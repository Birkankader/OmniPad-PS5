#include "usb_desc.h"

#include <string.h>

#define UDESC_CONFIG    0x02
#define UDESC_INTERFACE 0x04
#define UDESC_ENDPOINT  0x05
#define UDESC_HID       0x21
#define UDESC_REPORT    0x22

#define UE_XFER_MASK    0x03
#define UE_BULK         0x02
#define UE_INTERRUPT    0x03
#define UE_DIR_IN       0x80

int usb_desc_parse_config(const uint8_t *raw, size_t len, usb_config_info_t *out)
{
    if (!raw || !out) return -1;
    memset(out, 0, sizeof(*out));
    if (len < 2) return -1;

    /* Never walk past wTotalLength when the buffer starts with a config descriptor */
    if (len >= 4 && raw[0] >= 4 && raw[1] == UDESC_CONFIG) {
        size_t total = (size_t)raw[2] | ((size_t)raw[3] << 8);
        if (total >= 4 && total < len) len = total;
    }

    usb_iface_info_t *cur = NULL;
    size_t offset = 0;
    while (offset + 2 <= len) {
        size_t entry = raw[offset];
        uint8_t type = raw[offset + 1];
        if (entry < 2 || offset + entry > len) break;
        const uint8_t *d = raw + offset;

        if (type == UDESC_INTERFACE && entry >= 9) {
            if (out->n_ifaces < USB_DESC_MAX_IFACES) {
                cur = &out->ifaces[out->n_ifaces++];
                memset(cur, 0, sizeof(*cur));
                cur->number = d[2];
                cur->alt = d[3];
                cur->cls = d[5];
                cur->subcls = d[6];
                cur->proto = d[7];
            } else {
                cur = NULL; /* table full: ignore the rest instead of corrupting the last entry */
            }
        } else if (type == UDESC_ENDPOINT && entry >= 7 && cur) {
            uint8_t addr = d[2];
            uint8_t xfer = d[3] & UE_XFER_MASK;
            uint16_t maxpkt = (uint16_t)((d[4] | (d[5] << 8)) & 0x07ff);
            if (xfer == UE_INTERRUPT || xfer == UE_BULK) {
                if ((addr & UE_DIR_IN) && cur->ep_in == 0) {
                    cur->ep_in = addr;
                    cur->ep_in_maxpkt = maxpkt;
                } else if (!(addr & UE_DIR_IN) && cur->ep_out == 0) {
                    cur->ep_out = addr;
                    cur->ep_out_maxpkt = maxpkt;
                }
            }
        } else if (type == UDESC_HID && entry >= 9 && cur && cur->cls == 0x03) {
            /* bNumDescriptors at [5], then (bDescriptorType, wDescriptorLength) pairs */
            uint8_t count = d[5];
            for (uint8_t i = 0; i < count && (size_t)(6 + i * 3 + 2) < entry; i++) {
                if (d[6 + i * 3] == UDESC_REPORT) {
                    cur->hid_report_len = (uint16_t)(d[7 + i * 3] | (d[8 + i * 3] << 8));
                    break;
                }
            }
        }
        offset += entry;
    }
    return out->n_ifaces;
}

static int is_gip(const usb_iface_info_t *i)
{
    return i->cls == 0xff && i->subcls == 0x47 && i->proto == 0xd0;
}

static int is_x360(const usb_iface_info_t *i)
{
    return i->cls == 0xff && i->subcls == 0x5d;
}

static int is_x360_gamepad(const usb_iface_info_t *i)
{
    /* 0x01 = wired controller, 0x81 = wireless receiver controller channel */
    return is_x360(i) && (i->proto == 0x01 || i->proto == 0x81);
}

static int is_hid_candidate(const usb_iface_info_t *i)
{
    /* Boot keyboards (1/1) and boot mice (1/2) never carry gamepad input */
    if (i->cls != 0x03) return 0;
    if (i->subcls == 0x01 && (i->proto == 0x01 || i->proto == 0x02)) return 0;
    return 1;
}

static const usb_iface_info_t *first_match(const usb_config_info_t *cfg,
                                           int (*pred)(const usb_iface_info_t *))
{
    for (int i = 0; i < cfg->n_ifaces; i++) {
        const usb_iface_info_t *it = &cfg->ifaces[i];
        if (it->ep_in && (!pred || pred(it))) return it;
    }
    return NULL;
}

int usb_desc_select_input(const usb_config_info_t *cfg, usb_input_route_t *route)
{
    if (!cfg || !route) return 0;
    memset(route, 0, sizeof(*route));

    const usb_iface_info_t *it = NULL;
    usb_route_kind_t kind = USB_ROUTE_NONE;

    if ((it = first_match(cfg, is_gip)) != NULL) {
        kind = USB_ROUTE_GIP;
    } else if ((it = first_match(cfg, is_x360_gamepad)) != NULL ||
               (it = first_match(cfg, is_x360)) != NULL) {
        kind = USB_ROUTE_X360;
    } else if ((it = first_match(cfg, is_hid_candidate)) != NULL) {
        kind = USB_ROUTE_HID;
    } else if ((it = first_match(cfg, NULL)) != NULL) {
        kind = USB_ROUTE_LEGACY;
    }
    if (!it) return 0;

    route->kind = kind;
    route->iface = it->number;
    route->alt = it->alt;
    route->ep_in = it->ep_in;
    route->ep_out = it->ep_out;
    route->in_maxpkt = it->ep_in_maxpkt;
    route->out_maxpkt = it->ep_out_maxpkt;
    route->hid_report_len = it->hid_report_len;
    for (int i = 0; i < cfg->n_ifaces; i++) {
        int seen = 0;
        for (int j = 0; j < i; j++) {
            if (cfg->ifaces[j].number == cfg->ifaces[i].number) { seen = 1; break; }
        }
        if (!seen) route->iface_count++;
    }
    return 1;
}

const char *usb_route_name(usb_route_kind_t kind)
{
    switch (kind) {
    case USB_ROUTE_LEGACY: return "legacy";
    case USB_ROUTE_GIP:    return "GIP";
    case USB_ROUTE_X360:   return "XInput";
    case USB_ROUTE_HID:    return "HID";
    default:               return "none";
    }
}
