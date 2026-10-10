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

static void route_use_iface(usb_input_route_t *route, const usb_iface_info_t *it)
{
    route->iface = it->number;
    route->alt = it->alt;
    route->proto = it->proto;
    route->ep_in = it->ep_in;
    route->ep_out = it->ep_out;
    route->in_maxpkt = it->ep_in_maxpkt;
    route->out_maxpkt = it->ep_out_maxpkt;
    route->hid_report_len = it->hid_report_len;
}

int usb_desc_select_input(const usb_config_info_t *cfg, usb_input_route_t *route)
{
    if (!cfg || !route) return 0;
    memset(route, 0, sizeof(*route));
    route->aux_iface = 0xff;

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
    route_use_iface(route, it);
    for (int i = 0; i < cfg->n_ifaces; i++) {
        const usb_iface_info_t *o = &cfg->ifaces[i];
        int seen = 0;
        for (int j = 0; j < i; j++) {
            if (cfg->ifaces[j].number == o->number) { seen = 1; break; }
        }
        if (!seen) route->iface_count++;
        /* GIP audio interface: xpad and xone put it back to alt 0 before streaming */
        if (kind == USB_ROUTE_GIP && is_gip(o) && o->number != it->number) {
            if (route->aux_iface == 0xff) route->aux_iface = o->number;
            if (o->number == route->aux_iface && o->alt == 1) route->aux_has_alt1 = 1;
        }
    }
    return 1;
}

/* ---- HID report descriptor summary ----
 * Only what interface selection and transfer sizing need: is there a Generic
 * Desktop Joystick/Gamepad application collection, which report ID it uses and
 * how long its input report is. */

#define HID_MAX_PUSH 4

typedef struct {
    uint16_t usage_page;
    uint32_t report_size;
    uint32_t report_count;
    uint8_t  report_id;
} hid_globals_t;

static int is_gamepad_usage(uint32_t usage)
{
    /* Generic Desktop: Joystick, Game Pad, Multi-axis Controller */
    return usage == 0x00010004u || usage == 0x00010005u || usage == 0x00010008u;
}

static int is_non_pad_usage(uint32_t usage)
{
    /* Generic Desktop Mouse, Keyboard, Keypad, System Control; Consumer Control */
    return usage == 0x00010002u || usage == 0x00010006u || usage == 0x00010007u ||
           usage == 0x00010080u || usage == 0x000c0001u;
}

int usb_hid_parse_report_desc(const uint8_t *d, size_t len, usb_hid_info_t *out)
{
    if (!d || !out) return 0;
    memset(out, 0, sizeof(*out));

    uint32_t in_bits[256];           /* input bits per report ID */
    memset(in_bits, 0, sizeof(in_bits));
    hid_globals_t g, stack[HID_MAX_PUSH];
    memset(&g, 0, sizeof(g));
    int sp = 0;
    uint32_t pending_usage = 0;      /* first local Usage (page << 16 | id) since the last main item */
    int have_usage = 0;
    int depth = 0;
    int gamepad_depth = -1;          /* collection depth of the gamepad application, -1 = outside */
    int gamepad_id_set = 0;
    int other_depth = -1;            /* same for a keyboard/mouse/consumer application */
    uint8_t pad_ids[32];             /* report IDs with inputs outside such collections */
    memset(pad_ids, 0, sizeof(pad_ids));

    size_t i = 0;
    while (i < len) {
        uint8_t prefix = d[i++];
        if (prefix == 0xfe) {        /* long item: size, tag, data */
            if (i + 2 > len) return 0;
            size_t n = d[i];
            i += 2;
            if (i + n > len) return 0;
            i += n;
            continue;
        }
        size_t n = prefix & 0x03;
        if (n == 3) n = 4;
        if (i + n > len) return 0;
        uint32_t v = 0;
        for (size_t k = 0; k < n; k++) v |= (uint32_t)d[i + k] << (8 * k);
        i += n;

        uint8_t type = (prefix >> 2) & 0x03;
        uint8_t tag = prefix >> 4;

        if (type == 1) {             /* global */
            switch (tag) {
            case 0x0: g.usage_page = (uint16_t)v; break;
            case 0x7: g.report_size = v; break;
            case 0x8:
                g.report_id = (uint8_t)v;
                out->uses_report_ids = 1;
                break;
            case 0x9: g.report_count = v; break;
            case 0xa: if (sp < HID_MAX_PUSH) stack[sp++] = g; break;
            case 0xb: if (sp > 0) g = stack[--sp]; break;
            default: break;
            }
        } else if (type == 2) {      /* local */
            if (tag == 0x0 && !have_usage) {
                pending_usage = (n == 4) ? v : (((uint32_t)g.usage_page << 16) | (v & 0xffff));
                have_usage = 1;
            }
        } else if (type == 0) {      /* main */
            switch (tag) {
            case 0x8: {              /* Input */
                /* The ID in effect at the collection's first Input: one declared before
                 * it or inside it, never an Output/Feature ID or another collection's */
                if (gamepad_depth >= 0 && !gamepad_id_set) {
                    out->gamepad_report_id = g.report_id;
                    gamepad_id_set = 1;
                }
                if (g.report_id != 0) {
                    uint8_t *map = other_depth >= 0 ? out->ignore_ids : pad_ids;
                    map[g.report_id >> 3] |= (uint8_t)(1u << (g.report_id & 7));
                }
                uint64_t bits = (uint64_t)g.report_size * g.report_count;
                uint64_t total = (uint64_t)in_bits[g.report_id] + bits;
                in_bits[g.report_id] = total > 0xffffu * 8u ? 0xffffu * 8u : (uint32_t)total;
                break;
            }
            case 0xa:                /* Collection */
                if (v == 0x01 && depth == 0 && gamepad_depth < 0 && !out->has_gamepad &&
                    is_gamepad_usage(pending_usage)) {
                    out->has_gamepad = 1;
                    gamepad_depth = depth;
                } else if (v == 0x01 && depth == 0 && is_non_pad_usage(pending_usage)) {
                    other_depth = depth;
                }
                depth++;
                break;
            case 0xc:                /* End Collection */
                if (depth > 0) depth--;
                if (depth == gamepad_depth) gamepad_depth = -1;
                if (depth == other_depth) other_depth = -1;
                break;
            default: break;
            }
            have_usage = 0;
            pending_usage = 0;
        }
    }

    for (int id = 0; id < 256; id++) {
        if (!in_bits[id]) continue;
        uint32_t bytes = (in_bits[id] + 7) / 8 + (id != 0 ? 1u : 0u);
        if (bytes > 0xffff) bytes = 0xffff;
        if (bytes > out->max_input_len) out->max_input_len = (uint16_t)bytes;
    }
    /* An ID that also carries gamepad or vendor input is not ignorable */
    for (int k = 0; k < 32; k++) out->ignore_ids[k] &= (uint8_t)~pad_ids[k];
    if (out->has_gamepad && in_bits[out->gamepad_report_id]) {
        uint32_t bytes = (in_bits[out->gamepad_report_id] + 7) / 8 + (out->gamepad_report_id ? 1u : 0u);
        out->gamepad_input_len = (uint16_t)(bytes > 0xffff ? 0xffff : bytes);
    }
    return 1;
}

int usb_desc_refine_hid_route(const usb_config_info_t *cfg, usb_input_route_t *route,
                              usb_hid_fetch_fn fetch, void *user)
{
    if (!cfg || !route || !fetch || route->kind != USB_ROUTE_HID) return 0;

    uint8_t buf[1024];
    memset(buf, 0, sizeof(buf));
    for (int i = 0; i < cfg->n_ifaces; i++) {
        const usb_iface_info_t *it = &cfg->ifaces[i];
        if (!it->ep_in || !is_hid_candidate(it) || it->hid_report_len == 0) continue;
        int seen = 0;
        for (int j = 0; j < i; j++) {
            if (cfg->ifaces[j].number == it->number) { seen = 1; break; }
        }
        if (seen) continue;

        int n = fetch(user, it, buf, sizeof(buf));
        usb_hid_info_t info;
        if (n <= 0 || !usb_hid_parse_report_desc(buf, (size_t)n, &info)) continue;

        if (info.has_gamepad) {
            route_use_iface(route, it);
            route->hid_report_id = info.uses_report_ids ? info.gamepad_report_id : 0;
            route->hid_input_len = info.gamepad_input_len;
            route->hid_max_input_len = info.max_input_len;
            memcpy(route->hid_ignore_ids, info.ignore_ids, sizeof(route->hid_ignore_ids));
            return 1;
        }
        if (it->number == route->iface) {
            route->hid_max_input_len = info.max_input_len;
            memcpy(route->hid_ignore_ids, info.ignore_ids, sizeof(route->hid_ignore_ids));
        }
    }
    return 0;
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
