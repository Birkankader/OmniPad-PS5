#include "usb_hotplug.h"
#include "usb_controllers.h"
#include "usb_desc.h"
#include "gip.h"
#include "ps5_vpad.h"
#include "log.h"
#include "usb_lifecycle.h"
#include "util.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>

#ifdef __PROSPERO__
#include <dev/usb/usb.h>
#include <dev/usb/usb_ioctl.h>
#include <dev/usb/usb_endian.h>
#include <dev/usb/usbdi.h>
#endif

typedef struct {
    int                   active;
    int                   slot;
    int                   fd;        /* /dev/ugenX.Y control device */
    char                  dev_path[32];
    uint16_t              vid, pid;
    usb_controller_type_t type;
    const char           *name;
    pad_conn_type_t       conn_type;
    usb_lifecycle_t       lifecycle;
    int                   stop_thread;
    int                   use_fs;
    usb_input_route_t     route;     /* interface/endpoints/protocol chosen from descriptors */
    gip_ctx_t             gip;
    int                   gip_reinit_pending; /* a GIP announce asked for the init sequence */
    unsigned              n_hid_filtered;     /* non-pad HID reports dropped (first few are logged) */

#ifdef __PROSPERO__
    struct usb_fs_endpoint endpoints[4];
    uint8_t               in_index;
    uint8_t               out_index;
    int                   has_out;
    void                 *fs_frame[4];
    uint32_t              fs_len[4];
    uint8_t               rx_buf[64];
    uint8_t               tx_buf[64];
    uint32_t              in_xfer_len;  /* IN transfer size, wMaxPacketSize of the endpoint */
    int                   in_pending, in_ready, in_stalled;
    int                   out_pending, out_ready, out_stalled;
#else
    int                   ep_fd[6];
#endif
} usb_slot_device_t;

typedef struct {
    usb_slot_device_t devices[MAX_SLOTS];
    long              last_scan_time;
    int               initialized;
    int               shutting_down;
    pthread_mutex_t   lock;
} usb_hotplug_state_t;

static usb_hotplug_state_t g_usb_hotplug = {
    .lock = PTHREAD_MUTEX_INITIALIZER
};
static pthread_mutex_t g_usb_operation_lock = PTHREAD_MUTEX_INITIALIZER;
static uint32_t s_last_seen[6][21] = {{0}};

static void reset_device(usb_slot_device_t *sdev)
{
    memset(sdev, 0, sizeof(*sdev));
    sdev->slot = -1;
    sdev->fd = -1;
#ifndef __PROSPERO__
    for (int i = 0; i < 6; i++) sdev->ep_fd[i] = -1;
#endif
}

static void close_device_resources(usb_slot_device_t *sdev)
{
#ifdef __PROSPERO__
    if (sdev->use_fs && sdev->fd >= 0) {
        struct usb_fs_uninit uninit;
        memset(&uninit, 0, sizeof(uninit));
        ioctl(sdev->fd, USB_FS_UNINIT, &uninit);
        sdev->use_fs = 0;
    }
#else
    for (int i = 0; i < 6; i++) {
        if (sdev->ep_fd[i] >= 0) {
            close(sdev->ep_fd[i]);
            sdev->ep_fd[i] = -1;
        }
    }
#endif
    if (sdev->fd >= 0) {
        close(sdev->fd);
        sdev->fd = -1;
    }
}

static int worker_stop_requested(usb_slot_device_t *dev)
{
    int stop;
    pthread_mutex_lock(&g_usb_hotplug.lock);
    stop = dev->stop_thread;
    pthread_mutex_unlock(&g_usb_hotplug.lock);
    return stop;
}

static void request_worker_stop(usb_slot_device_t *dev)
{
    pthread_mutex_lock(&g_usb_hotplug.lock);
    dev->stop_thread = 1;
    pthread_mutex_unlock(&g_usb_hotplug.lock);
}

/* Writes len bytes as hex, 32 per log line (log_line truncates at 1 KiB). */
static void log_hex_block(const char *what, int slot, const uint8_t *data, int len)
{
    char line[3 * 32 + 1];
    for (int off = 0; off < len; off += 32) {
        int n = (len - off) < 32 ? (len - off) : 32;
        for (int i = 0; i < n; i++) {
            snprintf(line + i * 3, sizeof(line) - (size_t)i * 3, "%02x ", data[off + i]);
        }
        line[n * 3 - 1] = '\0';
        log_line("usb_hotplug: slot %d %s [%03d] %s", slot, what, off, line);
    }
}

#ifdef __PROSPERO__

static void find_usb_endpoints(int fd, usb_controller_type_t type, uint16_t vid, uint16_t pid, uint8_t *in_addr, uint8_t *out_addr)
{
    *in_addr = 0;
    *out_addr = 0;
    struct usb_gen_descriptor desc;
    uint8_t raw[512];
    memset(&desc, 0, sizeof(desc));
    memset(raw, 0, sizeof(raw));
    desc.ugd_data = raw;
    desc.ugd_maxlen = sizeof(raw);
    if (ioctl(fd, USB_GET_FULL_DESC, &desc) != 0) return;
    int len = desc.ugd_actlen;
    if (len <= 0 || len > (int)sizeof(raw)) len = (int)sizeof(raw);

    uint8_t ins[8], outs[8];
    int in_cnt = 0, out_cnt = 0;
    int offset = 0;
    while (offset + 2 <= len) {
        int entry = raw[offset];
        if (entry < 2 || offset + entry > len) break;
        uint8_t bDescriptorType = raw[offset + 1];
        if (bDescriptorType == 0x05 && entry >= 5) { /* Endpoint descriptor */
            uint8_t ep_addr = raw[offset + 2];
            uint8_t ep_attr = raw[offset + 3] & 0x03;
            if (ep_attr == 0x03 || ep_attr == 0x02) { /* Interrupt or Bulk */
                if ((ep_addr & 0x80) && in_cnt < 8) ins[in_cnt++] = ep_addr;
                else if (!(ep_addr & 0x80) && out_cnt < 8) outs[out_cnt++] = ep_addr;
            }
        }
        offset += entry;
    }

    int is_xbox_one = (type == CTRL_XBOX_XINPUT && vid == 0x045e && pid != 0x028e);

    if (is_xbox_one) {
        /* Only Xbox One / Series X|S (GIP) uses EP 0x82 IN and 0x02 OUT */
        for (int i = 0; i < in_cnt; i++) {
            if (ins[i] == 0x82) { *in_addr = 0x82; break; }
        }
        for (int i = 0; i < out_cnt; i++) {
            if (outs[i] == 0x02) { *out_addr = 0x02; break; }
        }
    } else {
        /* Xbox 360, Machenike G5 Pro, ShanWan, Switch, DS4, DS3 all use EP 0x81 IN */
        for (int i = 0; i < in_cnt; i++) {
            if (ins[i] == 0x81) { *in_addr = 0x81; break; }
        }
        for (int i = 0; i < out_cnt; i++) {
            if (outs[i] == 0x02) { *out_addr = 0x02; break; }
        }
        if (*out_addr == 0) {
            for (int i = 0; i < out_cnt; i++) {
                if (outs[i] == 0x01) { *out_addr = 0x01; break; }
            }
        }
    }

    if (*in_addr == 0 && in_cnt > 0) *in_addr = ins[0];
    if (*out_addr == 0 && out_cnt > 0) *out_addr = outs[0];
}

static int read_config_descriptor(int fd, uint8_t *raw, size_t cap)
{
    struct usb_gen_descriptor desc;
    memset(&desc, 0, sizeof(desc));
    memset(raw, 0, cap);
    desc.ugd_data = raw;
    desc.ugd_maxlen = (uint16_t)cap;
    if (ioctl(fd, USB_GET_FULL_DESC, &desc) != 0) return -1;
    int len = desc.ugd_actlen;
    if (len <= 0 || len > (int)cap) len = (int)cap;
    return len;
}

/* xpad and xone send SET_INTERFACE(audio, alt 0) before streaming ("mandatory for
 * certain third party devices"; the PowerA 20d6:200e reports Guide only after it).
 * USB_SET_ALTINTERFACE skips the request when the alt is already active, which it
 * is after SET_CONFIG, so send it raw, or else go through alt 1 so each switch is
 * real. Must precede USB_FS_INIT: changing the alt setting tears down FS state. */
static void gip_reset_audio_iface(usb_slot_device_t *sdev)
{
    uint8_t aif = sdev->route.aux_iface;
    if (sdev->route.kind != USB_ROUTE_GIP || sdev->route.ep_in == 0 || aif == 0xff) return;

    struct usb_ctl_request req;
    memset(&req, 0, sizeof(req));
    req.ucr_request.bmRequestType = 0x01; /* host to device, standard, interface */
    req.ucr_request.bRequest = 0x0b;      /* SET_INTERFACE */
    USETW(req.ucr_request.wValue, 0);
    USETW(req.ucr_request.wIndex, aif);
    USETW(req.ucr_request.wLength, 0);
    if (ioctl(sdev->fd, USB_DO_REQUEST, &req) == 0) {
        log_line("usb_hotplug: %s GIP audio interface %u -> SET_INTERFACE alt 0 sent",
                 sdev->name, (unsigned)aif);
        return;
    }
    int raw_err = errno; /* EPERM without PRIV_DRIVER (ugen_check_request) */
    if (!sdev->route.aux_has_alt1) {
        log_line("usb_hotplug: %s GIP audio interface %u -> SET_INTERFACE failed (%s), no alt 1",
                 sdev->name, (unsigned)aif, strerror(raw_err));
        return;
    }

    /* uai_interface_index is an index; GIP pads number their interfaces 0, 1 */
    struct usb_alt_interface alt;
    memset(&alt, 0, sizeof(alt));
    alt.uai_interface_index = aif;
    alt.uai_alt_index = 1;
    int r1 = ioctl(sdev->fd, USB_SET_ALTINTERFACE, &alt);
    int e1 = errno;
    alt.uai_alt_index = 0;
    int r0 = ioctl(sdev->fd, USB_SET_ALTINTERFACE, &alt);
    int e0 = errno;
    log_line("usb_hotplug: %s GIP audio interface %u -> SET_INTERFACE %s; alt 1 %s, alt 0 %s",
             sdev->name, (unsigned)aif, strerror(raw_err),
             r1 == 0 ? "ok" : strerror(e1), r0 == 0 ? "ok" : strerror(e0));
}

static int usb_fs_setup_device(usb_slot_device_t *sdev)
{
    /* A protocol-only fallback route (descriptor unreadable) has no endpoints */
    int route_known = (sdev->route.kind != USB_ROUTE_NONE && sdev->route.ep_in != 0);
    int iface = route_known ? sdev->route.iface : 0;
    ioctl(sdev->fd, USB_CLAIM_INTERFACE, &iface);

    int config = 0;
    if (ioctl(sdev->fd, USB_GET_CONFIG, &config) == 0 && (config & 0xff) == 0xff) {
        config = 0;
        ioctl(sdev->fd, USB_SET_CONFIG, &config);
    }

    uint8_t in_addr = 0, out_addr = 0;
    if (route_known) {
        /* Endpoints of the interface that actually carries gamepad input */
        in_addr = sdev->route.ep_in;
        out_addr = sdev->route.ep_out;
    } else {
        find_usb_endpoints(sdev->fd, sdev->type, sdev->vid, sdev->pid, &in_addr, &out_addr);
    }

    int is_xbox_one = (sdev->type == CTRL_XBOX_XINPUT && sdev->vid == 0x045e && sdev->pid != 0x028e);
    if (in_addr == 0) {
        in_addr = is_xbox_one ? 0x82 : 0x81;
    }
    if (out_addr == 0 && !route_known && (sdev->type == CTRL_XBOX_XINPUT)) {
        out_addr = is_xbox_one ? 0x02 : 0x01;
    }

    gip_reset_audio_iface(sdev);

    struct usb_fs_init init;
    memset(sdev->endpoints, 0, sizeof(sdev->endpoints));
    memset(&init, 0, sizeof(init));
    init.pEndpoints = sdev->endpoints;
    init.ep_index_max = 2;

    if (ioctl(sdev->fd, USB_FS_INIT, &init) != 0) {
        if (errno != ENOTTY && errno != ENXIO && errno != EIO && errno != EBADF) {
            log_line("usb_hotplug: USB_FS_INIT failed for %s (%s)", sdev->dev_path, strerror(errno));
        }
        return 0;
    }
    sdev->use_fs = 1;

    sdev->in_index = 0;
    struct usb_fs_open open_in;
    int in_opened = 0;
    uint8_t try_ins[] = { in_addr, 0x81, 0x82, 0x83, 0x84 };
    for (size_t i = 0; i < sizeof(try_ins); i++) {
        if (try_ins[i] == 0) continue;
        /* One transfer per report: a 64-byte buffer on a 32-byte endpoint would merge
         * two full-size reports into one read. A HID report longer than one packet
         * (known from the report descriptor) needs a transfer that spans the packets
         * and ends at the short one. */
        uint32_t want = sizeof(sdev->rx_buf);
        if (i == 0 && route_known && sdev->route.in_maxpkt >= 8 &&
            sdev->route.in_maxpkt <= sizeof(sdev->rx_buf)) {
            want = sdev->route.in_maxpkt;
            if (sdev->route.kind == USB_ROUTE_HID && sdev->route.hid_max_input_len > want) {
                want = sdev->route.hid_max_input_len > sizeof(sdev->rx_buf)
                       ? sizeof(sdev->rx_buf) : sdev->route.hid_max_input_len;
            }
        }
        memset(&open_in, 0, sizeof(open_in));
        open_in.max_bufsize = want;
        open_in.max_frames = 1;
        open_in.ep_index = sdev->in_index;
        open_in.ep_no = try_ins[i];
        if (ioctl(sdev->fd, USB_FS_OPEN, &open_in) == 0) {
            in_addr = try_ins[i];
            in_opened = 1;
            /* The kernel writes back the size it actually allocated */
            sdev->in_xfer_len = (open_in.max_bufsize > 0 && open_in.max_bufsize <= sizeof(sdev->rx_buf))
                                ? open_in.max_bufsize : want;
            break;
        }
    }
    if (!in_opened) {
        log_line("usb_hotplug: USB_FS_OPEN IN failed for %s (%s)", sdev->dev_path, strerror(errno));
        return 0;
    }

    if (route_known && in_addr != sdev->route.ep_in) {
        log_line("usb_hotplug: %s route IN 0x%02x unavailable, fell back to 0x%02x",
                 sdev->name, sdev->route.ep_in, in_addr);
    }

    sdev->has_out = 0;
    uint8_t try_outs[] = { out_addr, 0x02, 0x01, 0x03, 0x04 };
    /* With a known route only the gamepad interface's own OUT endpoint is valid:
     * another interface's OUT (audio, keyboard) would swallow init commands. */
    size_t n_outs = route_known ? 1 : sizeof(try_outs);
    for (size_t i = 0; i < n_outs; i++) {
        if (try_outs[i] == 0) continue;
        sdev->out_index = 1;
        struct usb_fs_open open_out;
        memset(&open_out, 0, sizeof(open_out));
        open_out.max_bufsize = 64;
        open_out.max_frames = 1;
        open_out.ep_index = sdev->out_index;
        open_out.ep_no = try_outs[i];
        if (ioctl(sdev->fd, USB_FS_OPEN, &open_out) == 0) {
            out_addr = try_outs[i];
            sdev->has_out = 1;
            break;
        }
    }

    sdev->in_pending = sdev->in_ready = sdev->in_stalled = 0;
    sdev->out_pending = sdev->out_ready = sdev->out_stalled = 0;

    sdev->fs_frame[0] = sdev->rx_buf;
    sdev->fs_len[0] = sizeof(sdev->rx_buf);
    sdev->endpoints[0].ppBuffer = &sdev->fs_frame[0];
    sdev->endpoints[0].pLength = &sdev->fs_len[0];
    sdev->endpoints[0].nFrames = 1;
    sdev->endpoints[0].flags = USB_FS_FLAG_SINGLE_SHORT_OK;

    sdev->fs_frame[1] = sdev->tx_buf;
    sdev->fs_len[1] = sizeof(sdev->tx_buf);
    sdev->endpoints[1].ppBuffer = &sdev->fs_frame[1];
    sdev->endpoints[1].pLength = &sdev->fs_len[1];
    sdev->endpoints[1].nFrames = 1;
    sdev->endpoints[1].flags = USB_FS_FLAG_SINGLE_SHORT_OK;

    log_line("usb_hotplug: %s claimed via USB_FS (route=%s iface=%u IN=0x%02x/%u, OUT=0x%02x%s)",
             sdev->name, usb_route_name(sdev->route.kind), (unsigned)sdev->route.iface,
             in_addr, (unsigned)sdev->in_xfer_len, out_addr, sdev->has_out ? "" : " unavailable");
    return 1;
}

/* Reaps one finished transfer. USB_FS_COMPLETE drains a single queue shared by
 * every endpoint of the fd and reports which one finished, so an OUT completion
 * must never be read as input (stale rx_buf) nor an IN completion be lost by a send.
 * Returns the endpoint index or -1 (errno set, EBUSY when nothing finished). */
static int usb_fs_reap(usb_slot_device_t *dev)
{
    struct usb_fs_complete complete;
    memset(&complete, 0, sizeof(complete));
    if (ioctl(dev->fd, USB_FS_COMPLETE, &complete) != 0) return -1;
    if (complete.ep_index == dev->in_index) {
        dev->in_pending = 0;
        dev->in_ready = 1;     /* data (if any) is parked in rx_buf until recv takes it */
    } else if (complete.ep_index == dev->out_index) {
        dev->out_pending = 0;
        dev->out_ready = 1;
        /* Whichever loop reaps it, a halted pipe must be cleared by the next send */
        if (dev->endpoints[dev->out_index].status == USB_ERR_STALLED) dev->out_stalled = 1;
    }
    return complete.ep_index;
}

static int usb_fs_take_input(usb_slot_device_t *dev, uint8_t *buffer, uint32_t length)
{
    struct usb_fs_endpoint *ep = &dev->endpoints[dev->in_index];
    dev->in_ready = 0;
    if (ep->status == USB_ERR_NORMAL_COMPLETION || ep->status == USB_ERR_SHORT_XFER) {
        uint32_t n = dev->fs_len[dev->in_index];
        if (n > length) n = length;
        if (n > sizeof(dev->rx_buf)) n = sizeof(dev->rx_buf);
        memcpy(buffer, dev->rx_buf, n);
        return (int)n;
    }
    if (ep->status == USB_ERR_STALLED) dev->in_stalled = 1; /* cleared on the next start */
    return 0;
}

static int usb_fs_recv_packet(usb_slot_device_t *dev, uint8_t *buffer, uint32_t length, int timeout_ms)
{
    if (!dev->use_fs) return -1;
    if (dev->in_ready) return usb_fs_take_input(dev, buffer, length);

    if (!dev->in_pending) {
        struct usb_fs_endpoint *ep = &dev->endpoints[dev->in_index];
        uint32_t ask_len = dev->in_xfer_len ? dev->in_xfer_len : sizeof(dev->rx_buf);
        if (ask_len > length) ask_len = length;
        dev->fs_len[dev->in_index] = ask_len;
        /* No kernel timeout: the transfer stays armed across calls (in_pending) and
         * timeout_ms only bounds this call's wait. A timeout racing a report the
         * device already sent would discard it, and the device never resends it. */
        ep->timeout = 0; /* USB_FS_TIMEOUT_NONE */
        ep->flags = USB_FS_FLAG_SINGLE_SHORT_OK | (dev->in_stalled ? USB_FS_FLAG_CLEAR_STALL : 0);

        struct usb_fs_start start;
        memset(&start, 0, sizeof(start));
        start.ep_index = dev->in_index;
        if (ioctl(dev->fd, USB_FS_START, &start) != 0 && errno != EBUSY) return -1;
        dev->in_pending = 1;
        dev->in_stalled = 0;
    }

    int spins = timeout_ms / 4 + 1;
    while (spins-- > 0) {
        if (usb_fs_reap(dev) >= 0) {
            if (dev->in_ready) return usb_fs_take_input(dev, buffer, length);
            continue; /* an OUT transfer finished: keep waiting for input */
        }
        if (errno == EBUSY || errno == ETIMEDOUT || errno == EAGAIN || errno == EWOULDBLOCK) {
            /* Transfer still in progress or waiting for packet — continue select loop */
        } else {
            return -1;
        }

        struct timeval wait;
        fd_set rset;
        FD_ZERO(&rset);
        FD_SET(dev->fd, &rset);
        wait.tv_sec = 0;
        wait.tv_usec = 4000;
        select(dev->fd + 1, &rset, NULL, NULL, &wait);
    }
    return 0;
}

static int usb_fs_send_packet(usb_slot_device_t *dev, const uint8_t *buffer, uint32_t length, int timeout_ms)
{
    if (!dev->use_fs || !dev->has_out) return -1;
    int spins = timeout_ms / 4 + 1;

    /* An earlier OUT that outlived its caller's wait may still be in flight */
    while (dev->out_pending && spins-- > 0) {
        if (usb_fs_reap(dev) < 0) {
            if (errno != EBUSY) return -1;
            usleep(4000);
        }
    }
    if (dev->out_pending) return -1;

    struct usb_fs_endpoint *ep = &dev->endpoints[dev->out_index];
    uint32_t send_len = length > sizeof(dev->tx_buf) ? sizeof(dev->tx_buf) : length;
    memcpy(dev->tx_buf, buffer, send_len);
    dev->fs_len[dev->out_index] = send_len;
    ep->timeout = (uint16_t)timeout_ms;
    ep->flags = USB_FS_FLAG_SINGLE_SHORT_OK | (dev->out_stalled ? USB_FS_FLAG_CLEAR_STALL : 0);

    struct usb_fs_start start;
    memset(&start, 0, sizeof(start));
    start.ep_index = dev->out_index;
    if (ioctl(dev->fd, USB_FS_START, &start) != 0) return -1;
    dev->out_pending = 1;
    dev->out_ready = 0;
    dev->out_stalled = 0;

    spins = timeout_ms / 4 + 1;
    while (spins-- > 0) {
        if (usb_fs_reap(dev) >= 0) {
            if (!dev->out_ready) continue; /* input arrived meanwhile: parked for recv */
            dev->out_ready = 0;
            if (ep->status == USB_ERR_STALLED) dev->out_stalled = 1;
            return (ep->status == USB_ERR_NORMAL_COMPLETION || ep->status == USB_ERR_SHORT_XFER) ? (int)send_len : -1;
        }
        if (errno != EBUSY) return -1;
        usleep(4000);
    }
    return -1;
}

static void gip_send_init_sequence(usb_slot_device_t *dev, int verbose)
{
    uint8_t pkt[GIP_MAX_PACKET];
    size_t len;
    int sent = 0, failed = 0;

    gip_restart_init(&dev->gip);
    while ((len = gip_next_init_packet(&dev->gip, pkt, sizeof(pkt))) > 0) {
        if (usb_fs_send_packet(dev, pkt, (uint32_t)len, 80) < 0) failed++;
        else sent++;
        usleep(5000);
    }
    if (verbose) {
        log_line("usb_hotplug: slot %d GIP init sequence sent (%d ok, %d failed)", dev->slot, sent, failed);
    }
}

typedef struct {
    int fd;
    int slot;
} hid_fetch_ctx_t;

/* GET_DESCRIPTOR(HID report) for one interface, logged so a new pad's report
 * layout can be read straight from anypad.log. */
static int fetch_hid_report_desc(void *user, const usb_iface_info_t *it, uint8_t *buf, size_t cap)
{
    hid_fetch_ctx_t *c = (hid_fetch_ctx_t *)user;
    uint16_t want = it->hid_report_len;
    if (want > cap) want = (uint16_t)cap;

    struct usb_ctl_request req;
    memset(&req, 0, sizeof(req));
    req.ucr_request.bmRequestType = 0x81; /* device to host, standard, interface */
    req.ucr_request.bRequest = 0x06;      /* GET_DESCRIPTOR */
    USETW(req.ucr_request.wValue, 0x2200); /* HID report descriptor, index 0 */
    USETW(req.ucr_request.wIndex, it->number);
    USETW(req.ucr_request.wLength, want);
    req.ucr_data = buf;
    req.ucr_flags = USB_SHORT_XFER_OK;
    if (ioctl(c->fd, USB_DO_REQUEST, &req) != 0) {
        log_line("usb_hotplug: slot %d HID report descriptor of interface %u unreadable (%s)",
                 c->slot, (unsigned)it->number, strerror(errno));
        return -1;
    }
    if (req.ucr_actlen <= 0) {
        log_line("usb_hotplug: slot %d HID report descriptor of interface %u is empty",
                 c->slot, (unsigned)it->number);
        return -1;
    }
    int n = req.ucr_actlen < (int)want ? (int)req.ucr_actlen : (int)want;
    char what[24];
    snprintf(what, sizeof(what), "hid-report if%u", (unsigned)it->number);
    log_hex_block(what, c->slot, buf, n);
    return n;
}


/* Asks a GameSir HID-mode pad for its full report format (A1 C8 ...), as SDL does.
 * Simple 9-byte reports are still decoded if the pad ignores the request. */
static void gamesir_send_mode_switch(usb_slot_device_t *dev)
{
    uint8_t cmd[64];
    size_t len = usb_gamesir_build_mode_switch(cmd, sizeof(cmd));
    int r = -1;
    const char *how = "interrupt OUT";

    if (len == 0) return;
    if (dev->has_out) {
        r = usb_fs_send_packet(dev, cmd, (uint32_t)len, 80);
    }
    if (r < 0) {
        /* HID SET_REPORT(Output, report id 0xA2) on the default control pipe */
        struct usb_ctl_request req;
        memset(&req, 0, sizeof(req));
        req.ucr_request.bmRequestType = 0x21;
        req.ucr_request.bRequest = 0x09;
        USETW(req.ucr_request.wValue, 0x0200 | cmd[0]);
        USETW(req.ucr_request.wIndex, dev->route.iface);
        USETW(req.ucr_request.wLength, (uint16_t)len);
        req.ucr_data = cmd;
        r = (ioctl(dev->fd, USB_DO_REQUEST, &req) == 0) ? (int)len : -1;
        how = "SET_REPORT";
    }
    log_line("usb_hotplug: slot %d GameSir mode switch via %s -> %s",
             dev->slot, how, r < 0 ? "failed (simple reports will be used)" : "ok");
}

#endif /* __PROSPERO__ */

/* Sends the ACK a GIP message asked for; unacknowledged reliable messages make
 * the pad fall back to the Arrival state (Xbox One S guide packets repeat forever). */
static void gip_ack_cb(void *user, const uint8_t *ack, size_t len)
{
#ifdef __PROSPERO__
    usb_slot_device_t *dev = (usb_slot_device_t *)user;
    if (dev->use_fs && dev->has_out) usb_fs_send_packet(dev, ack, (uint32_t)len, 40);
#else
    (void)user; (void)ack; (void)len;
#endif
}

typedef enum {
    PKT_STATE = 0,   /* decoded a full pad state */
    PKT_IGNORED,     /* valid protocol traffic without input meaning */
    PKT_RELEASE,     /* the pad is powering off or left its receiver: drop held input */
    PKT_UNPARSED     /* unknown bytes */
} pkt_kind_t;

/* Decodes one transfer with the protocol of the claimed interface. Routed
 * protocols never fall back to the generic HID guesser: a status, heartbeat or
 * vendor packet misread as a gamepad report is exactly what turned stick motion
 * into phantom button presses. */
static pkt_kind_t decode_packet(usb_slot_device_t *dev, const uint8_t *report, int n, pad_state_t *st)
{
    switch (dev->route.kind) {
    case USB_ROUTE_GIP: {
        int ev = gip_process(&dev->gip, report, (size_t)n, st, gip_ack_cb, dev);
        /* Hello: the pad waits in the Arrival state for Set Device State START */
        if (ev & GIP_EVENT_ANNOUNCE) dev->gip_reinit_pending = 1;
        if (ev & GIP_EVENT_POWER_OFF) {
            log_line("usb_hotplug: slot %d GIP status: pad powering off - input released", dev->slot);
            return PKT_RELEASE;
        }
        if (ev & GIP_EVENT_STATE) return PKT_STATE;
        return ev ? PKT_IGNORED : PKT_UNPARSED;
    }
    case USB_ROUTE_X360:
        if (dev->route.proto == 0x81) {
            /* Wireless receiver channel: own framing, plus "pad gone" packets */
            x360w_pkt_t w = usb_parse_x360w_report(report, (size_t)n, st);
            if (w == X360W_PKT_LINK_LOST) {
                log_line("usb_hotplug: slot %d wireless pad left the receiver - input released", dev->slot);
                return PKT_RELEASE;
            }
            if (w == X360W_PKT_STATE) return PKT_STATE;
            return w == X360W_PKT_STATUS ? PKT_IGNORED : PKT_UNPARSED;
        }
        return usb_parse_x360_report(report, (size_t)n, st) ? PKT_STATE : PKT_UNPARSED;
    default:
        break;
    }

    if (dev->type == CTRL_GAMESIR) {
        if (dev->route.kind == USB_ROUTE_HID) {
            /* Full reports identify themselves; on the 8K pads they come from a vendor
             * collection (report ID 0x43), not from the gamepad collection */
            if ((n >= 3 && report[0] == 0x43 && report[1] == 0xa1 && report[2] == 0xc8) ||
                (n >= 2 && report[0] == 0xa1 && report[1] == 0xc8)) {
                return usb_parse_gamesir_hid_report(report, (size_t)n, st) ? PKT_STATE : PKT_UNPARSED;
            }
            /* Keyboard, mouse and consumer reports the descriptor declares are not pad state */
            if (usb_route_hid_id_ignored(&dev->route, report[0])) {
                if (dev->n_hid_filtered++ < 3) log_hex_block("hid-filtered", dev->slot, report, n < 16 ? n : 16);
                return PKT_IGNORED;
            }
            /* The 9-byte simple layout may sit behind the gamepad report ID; the
             * Bluetooth-style 07/02 layouts carry their ID as part of the frame */
            uint8_t id = dev->route.hid_report_id;
            if (id != 0 && report[0] == id && n == 10 && id != 0x07 && id != 0x02) {
                return usb_parse_gamesir_hid_report(report + 1, 9, st) ? PKT_STATE : PKT_UNPARSED;
            }
            return usb_parse_gamesir_hid_report(report, (size_t)n, st) ? PKT_STATE : PKT_UNPARSED;
        }
        /* No usable descriptor: a GIP Hello (02 20 ss 1c, 28-byte payload) reveals
         * a GIP pad that only needs its init sequence */
        if (n >= 32 && report[0] == GIP_CMD_ANNOUNCE && report[1] == GIP_OPT_INTERNAL &&
            report[3] == 0x1c) {
            log_line("usb_hotplug: slot %d GIP Hello on a route-less GameSir pad - switching to GIP", dev->slot);
            dev->route.kind = USB_ROUTE_GIP;
            dev->gip_reinit_pending = 1;
            return PKT_IGNORED;
        }
        /* GameSir HID first, then strict wired XInput (the two never overlap) */
        return usb_parse_input_report(CTRL_GAMESIR, report, (size_t)n, st) ? PKT_STATE : PKT_UNPARSED;
    }

    /* Legacy VID/PID heuristics (Machenike HID mode, DS4, Switch Pro, DS3, generic HID) */
    if (usb_parse_input_report(dev->type, report, (size_t)n, st)) return PKT_STATE;
    if (usb_parse_input_report(CTRL_GENERIC_HID, report, (size_t)n, st)) return PKT_STATE;
    return PKT_UNPARSED;
}

/* No traffic at all for this long means the link is gone even though the device
 * (a 2.4 GHz receiver) is still enumerated. [MS-GIPUSB] has a started pad send
 * Status every 20 s; once a pad has shown that heartbeat (two Status messages
 * 15-25 s apart), two missed ones are enough. Report-on-change pads without it
 * only have to outlast a stick or trigger held perfectly still. */
#define GIP_SILENCE_RELEASE_MS   45000
#define EVENT_SILENCE_RELEASE_MS 60000
#define GIP_HEARTBEAT_MIN_MS     15000
#define GIP_HEARTBEAT_MAX_MS     25000

/* Pushes a neutral state (keeping the slot's identity) so a held input does not
 * outlive the link that reported it. */
static void release_held_input(usb_slot_device_t *dev, pad_state_t *last_state)
{
    pad_state_t n;
    pad_state_neutral(&n);
    n.conn_type = last_state->conn_type;
    n.vid = last_state->vid;
    n.pid = last_state->pid;
    memcpy(n.controller_name, last_state->controller_name, sizeof(n.controller_name));
    *last_state = n;
    if (!vpad_slot_is_free(dev->slot)) vpad_update(dev->slot, last_state);
}

static void *usb_reader_worker(void *arg)
{
    usb_slot_device_t *dev = (usb_slot_device_t *)arg;
    uint8_t report[256];
    pad_state_t st;
    pad_state_t last_state;
    int have_state = 0;
    long last_push_time = 0;

    long last_data_time = now_ms();
    long last_status_time = 0;
    unsigned long n_status = 0;
    int gip_heartbeat = 0;           /* the pad sends the periodic GIP Status */
#ifdef __PROSPERO__
    long last_led_time = 0;
    long last_init_time = 0;
    int init_attempts = 0;
    unsigned long n_announces = 0;
#endif
    int gip_len_logged = 0;
    long last_log_t = 0;
    long last_unparsed_log_t = 0;
    long last_idle_log_t = 0;
    int consecutive_errors = 0;
    int raw_logged = 0;
    unsigned long n_unparsed = 0;

    /* GIP, XInput and GameSir pads only send a report when something changes */
    int event_driven = (dev->route.kind == USB_ROUTE_GIP ||
                        dev->route.kind == USB_ROUTE_X360 ||
                        dev->type == CTRL_GAMESIR);

    pad_state_neutral(&last_state);
    log_line("usb_hotplug: Reader thread started for slot %d (%s on %s, route=%s)",
             dev->slot, dev->name, dev->dev_path, usb_route_name(dev->route.kind));

#ifdef __PROSPERO__
    int is_genuine_x360 = (dev->type == CTRL_XBOX_XINPUT && dev->vid == 0x045e && dev->pid == 0x028e);
    if (dev->use_fs && dev->route.kind == USB_ROUTE_GIP) {
        if (dev->has_out) {
            gip_send_init_sequence(dev, 1);
            init_attempts = 1;
            last_init_time = now_ms();
        } else {
            log_line("usb_hotplug: slot %d GIP device has no usable OUT endpoint - it cannot be powered on", dev->slot);
        }
    } else if (dev->use_fs && dev->type == CTRL_GAMESIR && dev->route.kind != USB_ROUTE_X360 &&
               (dev->pid == 0x10b8 || dev->pid == 0x103c)) {
        /* Only the 8K firmwares implement SDL's mode switch; the regular G7 Pro HID
         * identity (0x1022) streams its simple 9-byte report without it */
        gamesir_send_mode_switch(dev);
    }
#endif

    while (!worker_stop_requested(dev)) {
        long cur_time = now_ms();

#ifdef __PROSPERO__
        /* Send periodic Player 1 LED / keepalive ONLY for genuine Microsoft Xbox 360 controller */
        if (dev->use_fs && dev->has_out && is_genuine_x360 && (cur_time - last_led_time > 1500)) {
            last_led_time = cur_time;
            uint8_t x360_led[] = { 0x01, 0x03, 0x06 };
            usb_fs_send_packet(dev, x360_led, sizeof(x360_led), 40);
        }

        if (dev->use_fs && dev->has_out && dev->route.kind == USB_ROUTE_GIP) {
            if (dev->gip.started) init_attempts = 0;
            if (dev->gip_reinit_pending && (cur_time - last_init_time >= 400)) {
                /* Hello repeats every 500 ms until the pad is started */
                dev->gip_reinit_pending = 0;
                n_announces++;
                int verbose = (n_announces <= 5 || (n_announces % 20) == 0);
                if (verbose) {
                    log_line("usb_hotplug: slot %d GIP announce #%lu - sending init", dev->slot, n_announces);
                }
                last_init_time = cur_time;
                gip_send_init_sequence(dev, verbose);
            } else if (!dev->gip.started &&
                       (cur_time - last_init_time > (init_attempts < 5 ? 3000 : 15000))) {
                /* Neither input nor status: the first init may have been dropped while
                 * booting, or a receiver's pad is still off. Keep trying, slower later. */
                init_attempts++;
                last_init_time = cur_time;
                int verbose = (init_attempts <= 5 || (init_attempts % 20) == 0);
                if (verbose) {
                    log_line("usb_hotplug: slot %d GIP pad silent - retrying init (#%d)", dev->slot, init_attempts);
                }
                gip_send_init_sequence(dev, verbose);
            }
        }
#endif

        int n = 0;
#ifdef __PROSPERO__
        if (dev->use_fs) {
            n = usb_fs_recv_packet(dev, report, sizeof(report), 15);
        }
#else
        for (int i = 0; i < 6; i++) {
            if (dev->ep_fd[i] >= 0) {
                ssize_t rb = read(dev->ep_fd[i], report, sizeof(report));
                if (rb > 0) { n = (int)rb; break; }
            }
        }
        if (n <= 0 && dev->fd >= 0) {
            ssize_t rb = read(dev->fd, report, sizeof(report));
            if (rb > 0) n = (int)rb;
        }
#endif

        if (n > 0) {
            consecutive_errors = 0;
            if (raw_logged < 12) {
                /* First packets verbatim: enough to identify any new report format from the log */
                raw_logged++;
                log_hex_block("rx", dev->slot, report, n < 64 ? n : 64);
            }

            pkt_kind_t kind = decode_packet(dev, report, n, &st);
            if (dev->gip.n_status != n_status) {
                n_status = dev->gip.n_status;
                long gap = cur_time - last_status_time;
                if (!gip_heartbeat && last_status_time &&
                    gap >= GIP_HEARTBEAT_MIN_MS && gap <= GIP_HEARTBEAT_MAX_MS) {
                    gip_heartbeat = 1;
                    log_line("usb_hotplug: slot %d GIP Status heartbeat every ~%ld s", dev->slot, gap / 1000);
                }
                last_status_time = cur_time;
            }
            if (!gip_len_logged && dev->gip.have_input) {
                gip_len_logged = 1;
                log_line("usb_hotplug: slot %d GIP input report payload is %u bytes",
                         dev->slot, (unsigned)dev->gip.input_len);
            }
            if (kind == PKT_STATE) {
                st.conn_type = dev->conn_type;
                st.vid = dev->vid;
                st.pid = dev->pid;
                snprintf(st.controller_name, sizeof(st.controller_name), "%s", dev->name);
                /* If 2.4G dongle was in standby / slot freed, re-arm the virtual pad */
                if (dev->conn_type == CONN_USB_DONGLE_24G && vpad_slot_is_free(dev->slot)) {
                    log_line("usb_hotplug: slot %d 2.4G controller active — re-arming virtual pad", dev->slot);
                    vpad_add(dev->slot, dev->conn_type, dev->name);
                }
                vpad_update(dev->slot, &st);
                last_state = st;
                have_state = 1;
                last_push_time = cur_time;
                last_data_time = cur_time;

                if (st.buttons != 0 && (cur_time - last_log_t > 300)) {
                    last_log_t = cur_time;
                    log_line("usb_hotplug: slot %d button press 0x%08x (LX=%d LY=%d RX=%d RY=%d L2=%d R2=%d)",
                             dev->slot, (unsigned)st.buttons, st.lx, st.ly, st.rx, st.ry, st.l2, st.r2);
                }
            } else if (kind == PKT_IGNORED) {
                /* Heartbeats / status messages prove the link is alive */
                last_data_time = cur_time;
            } else if (kind == PKT_RELEASE) {
                /* Never re-arms a freed 2.4G slot: the pad is going, not coming */
                release_held_input(dev, &last_state);
                have_state = 0;
                last_data_time = cur_time;
            } else {
                n_unparsed++;
                if (cur_time - last_unparsed_log_t > 2000) {
                    last_unparsed_log_t = cur_time;
                    log_line("usb_hotplug: slot %d ignored %d-byte packet (%lu so far)",
                             dev->slot, n, n_unparsed);
                    log_hex_block("unparsed", dev->slot, report, n < 32 ? n : 32);
                }
            }
        } else if (n < 0) {
            consecutive_errors++;
            if (consecutive_errors >= 50) {
                log_line("usb_hotplug: slot %d disconnected (read error n=%d, errs=%d) — terminating reader",
                         dev->slot, n, consecutive_errors);
                request_worker_stop(dev);
                break;
            }
        } else if (cur_time - last_data_time > 3000) {
            if (cur_time - last_idle_log_t > 30000) {
                last_idle_log_t = cur_time;
                log_line("usb_hotplug: slot %d idle/standby (n=%d, use_fs=%d)",
                         dev->slot, n, dev->use_fs);
            }
            /* If 2.4G dongle receives no packets for > 60 seconds (1 minute), release vpad to free user profile */
            if (dev->conn_type == CONN_USB_DONGLE_24G && (cur_time - last_data_time > 60000)) {
                if (!vpad_slot_is_free(dev->slot)) {
                    log_line("usb_hotplug: slot %d 2.4G dongle idle for 60s — releasing virtual pad to standby", dev->slot);
                    vpad_remove(dev->slot);
                    have_state = 0;
                }
            }
        }

        /* A silent link (pad off, battery dead, out of range behind a receiver)
         * must not keep its last input held forever through the re-submit below */
        long silence_limit = gip_heartbeat ? GIP_SILENCE_RELEASE_MS : EVENT_SILENCE_RELEASE_MS;
        if (cur_time - last_data_time > silence_limit) {
            if (event_driven && have_state) {
                log_line("usb_hotplug: slot %d no traffic for %ld s - releasing held input",
                         dev->slot, (cur_time - last_data_time) / 1000);
                release_held_input(dev, &last_state);
                have_state = 0;
            }
            if (dev->route.kind == USB_ROUTE_GIP) {
                /* Forget guide/input so a pad that comes back without a Hello does not
                 * replay them; only a pad with a proven heartbeat is surely gone and
                 * needs START again (re-initialising an idle live pad is pointless) */
                gip_reset_input(&dev->gip);
                if (gip_heartbeat) dev->gip.started = 0;
            }
        }

        /* Re-submit the last state of report-on-change pads; otherwise vpad_poll's 500 ms
         * stale-state reset would release a stick or button that is simply being held. */
        if (event_driven && have_state && (cur_time - last_push_time >= 100) &&
            !vpad_slot_is_free(dev->slot)) {
            vpad_update(dev->slot, &last_state);
            last_push_time = cur_time;
        }

        /* After a packet, re-arm the IN transfer at once: the pad may report every
         * 1-2 ms. Idle reads already waited in select(), errors back off here. */
        if (n <= 0) usleep(4000);
    }
    log_line("usb_hotplug: Reader thread stopped for slot %d", dev->slot);
    return NULL;
}

static int usb_lifecycle_setup_io(void *context)
{
    usb_slot_device_t *sdev = (usb_slot_device_t *)context;
#ifdef __PROSPERO__
    return usb_fs_setup_device(sdev) ? 0 : EIO;
#else
    for (int ep = 1; ep <= 6; ep++) {
        char ep_path[40];
        snprintf(ep_path, sizeof(ep_path), "%s.%d", sdev->dev_path, ep);
        sdev->ep_fd[ep - 1] = open(ep_path, O_RDONLY | O_NONBLOCK);
    }
    return 0;
#endif
}

static int usb_lifecycle_reserve_slot(void *context)
{
    usb_slot_device_t *sdev = (usb_slot_device_t *)context;
    return vpad_add(sdev->slot, sdev->conn_type, sdev->name) ? 0 : ENOSPC;
}

static int usb_lifecycle_create_thread(void *context, pthread_t *thread,
                                       const pthread_attr_t *attributes,
                                       void *(*worker)(void *), void *argument)
{
    (void)context;
    return pthread_create(thread, attributes, worker, argument);
}

static int usb_lifecycle_join_thread(void *context, pthread_t thread,
                                     void **result)
{
    (void)context;
    return pthread_join(thread, result);
}

static void usb_lifecycle_release_slot(void *context)
{
    usb_slot_device_t *sdev = (usb_slot_device_t *)context;
    vpad_remove(sdev->slot);
}

static void usb_lifecycle_release_io(void *context)
{
    close_device_resources((usb_slot_device_t *)context);
}

static const usb_lifecycle_ops_t USB_LIFECYCLE_OPS = {
    usb_lifecycle_setup_io,
    usb_lifecycle_reserve_slot,
    usb_lifecycle_create_thread,
    usb_lifecycle_join_thread,
    usb_lifecycle_release_slot,
    usb_lifecycle_release_io
};

static int is_device_claimed(const char *path)
{
    for (int i = 0; i < MAX_SLOTS; i++) {
        if (g_usb_hotplug.devices[i].active &&
            strcmp(g_usb_hotplug.devices[i].dev_path, path) == 0) {
            return 1;
        }
    }
    return 0;
}

static int find_free_slot(void)
{
    for (int i = 0; i < MAX_SLOTS; i++) {
        if (!g_usb_hotplug.devices[i].active && vpad_slot_is_free(i)) {
            return i;
        }
    }
    return -1;
}

#ifdef __PROSPERO__
/* Which kernel driver (if any) holds each interface: a console HID driver owning
 * the gamepad interface would explain a pad that enumerates but stays silent. */
static void log_iface_drivers(int fd, int slot, const uint8_t *cfg, int cfg_len)
{
    if (cfg_len < 5 || cfg[1] != 0x02) return;
    int n_ifaces = cfg[4];
    for (int i = 0; i < n_ifaces && i < 8; i++) {
        char name[128];
        struct usb_gen_descriptor ugd;
        memset(name, 0, sizeof(name));
        memset(&ugd, 0, sizeof(ugd));
        ugd.ugd_data = name;
        ugd.ugd_maxlen = sizeof(name) - 1;
        ugd.ugd_iface_index = (uint8_t)i;
        if (ioctl(fd, USB_GET_IFACE_DRIVER, &ugd) == 0) {
            log_line("usb_hotplug: slot %d interface %d kernel driver: %s", slot, i, name[0] ? name : "none");
        } else {
            log_line("usb_hotplug: slot %d interface %d kernel driver: unknown (%s)", slot, i, strerror(errno));
        }
    }
}
#endif

/* Reads the active configuration descriptor and picks the gamepad interface.
 * Host builds have no descriptor access: the route stays NONE (legacy behavior). */
static usb_input_route_t usb_compute_route(int fd, uint16_t vid, uint16_t pid,
                                           uint8_t *raw, int cap, int *raw_len)
{
    usb_input_route_t route;
    memset(&route, 0, sizeof(route));
    route.aux_iface = 0xff;
    *raw_len = 0;
#ifdef __PROSPERO__
    int len = read_config_descriptor(fd, raw, (size_t)cap);
    if (len > 0) {
        /* ugd_actlen may be unset: trust wTotalLength when it is plausible */
        int total = (len >= 4 && raw[1] == 0x02) ? (raw[2] | (raw[3] << 8)) : len;
        if (total >= 9 && total < len) len = total;
        *raw_len = len;
        usb_config_info_t cfg;
        if (usb_desc_parse_config(raw, (size_t)len, &cfg) > 0) {
            usb_desc_select_input(&cfg, &route);
        }
    }
    /* Microsoft wireless adapters speak a radio protocol, never wired GIP */
    if (vid == 0x045e && (pid == 0x02e6 || pid == 0x02fe || pid == 0x02f9 || pid == 0x091e)) {
        memset(&route, 0, sizeof(route));
    } else if (route.kind == USB_ROUTE_NONE) {
        /* Descriptor unreadable: protocol implied by well-known IDs; endpoints are
         * then searched the legacy way (ep_in stays 0) */
        if ((vid == 0x3537 && (pid == 0x10a8 || pid == 0x1010)) ||
            (vid == 0x045e && (pid == 0x02d1 || pid == 0x02dd || pid == 0x02e3 || pid == 0x02ea ||
                               pid == 0x0b00 || pid == 0x0b12))) {
            route.kind = USB_ROUTE_GIP;
        } else if (vid == 0x3537 && (pid == 0x10a9 || pid == 0x1004 || pid == 0x100f)) {
            /* 1004/100f/1010 as listed in Linux xpad */
            route.kind = USB_ROUTE_X360;
        }
    }
    if (route.kind != USB_ROUTE_GIP) route.aux_iface = 0xff;
#else
    (void)fd; (void)vid; (void)pid; (void)raw; (void)cap;
#endif
    return route;
}

static void probe_usb_devices(void)
{
    char path[32];

    for (int bus = 0; bus <= 5; bus++) {
        for (int dev = 1; dev <= 20; dev++) {
            snprintf(path, sizeof(path), "/dev/ugen%d.%d", bus, dev);

            if (is_device_claimed(path)) continue;

            int fd = open(path, O_RDWR | O_NONBLOCK);
            if (fd < 0) continue;

            uint16_t vid = 0, pid = 0;
#ifdef __PROSPERO__
            struct usb_device_descriptor dd;
            if (ioctl(fd, USB_GET_DEVICE_DESC, &dd) == 0) {
                vid = UGETW(dd.idVendor);
                pid = UGETW(dd.idProduct);

                /* Explicitly ignore non-gamepad classes:
                 * 0x08 = Mass Storage, 0x09 = Hub, 0x02 = CDC, 0x01 = Audio, 0x0e = Video, 0xE0 = Bluetooth */
                if (dd.bDeviceClass == 0x08 || dd.bDeviceClass == 0x09 ||
                    dd.bDeviceClass == 0x02 || dd.bDeviceClass == 0x01 ||
                    dd.bDeviceClass == 0x0e || dd.bDeviceClass == 0xE0) {
                    close(fd);
                    continue;
                }
            }
#endif
            if (vid == 0 && pid == 0) {
                close(fd);
                continue;
            }

            const char *ctrl_name = NULL;
            usb_controller_type_t ctype = usb_identify_controller(vid, pid, &ctrl_name);

            uint8_t cfg_raw[512];
            int cfg_len = 0;
            usb_input_route_t route = usb_compute_route(fd, vid, pid, cfg_raw, (int)sizeof(cfg_raw), &cfg_len);

            /* If not found by VID/PID, accept any device exposing an Xbox GIP or XInput interface */
            if (ctype == CTRL_UNKNOWN) {
                if (route.kind == USB_ROUTE_GIP) {
                    ctype = CTRL_XBOX_XINPUT;
                    ctrl_name = "Xbox One/Series Compatible (GIP)";
                } else if (route.kind == USB_ROUTE_X360) {
                    ctype = CTRL_XBOX_XINPUT;
                    ctrl_name = "XInput Compatible Controller";
                }
            }

            if (vid != 0 || pid != 0) {
                uint32_t packed = ((uint32_t)vid << 16) | pid;
                if (s_last_seen[bus][dev] != packed) {
                    s_last_seen[bus][dev] = packed;
                    log_line("usb_hotplug: Device discovered on %s (VID 0x%04x, PID 0x%04x, type %d: %s)",
                             path, vid, pid, (int)ctype, ctrl_name ? ctrl_name : "unrecognized");
                }
            }

            /* Skip non-gamepad / unrecognized devices (flash drives, keyboards, SSDs) */
            if (ctype == CTRL_UNKNOWN) {
                close(fd);
                continue;
            }

            int slot = find_free_slot();
            if (slot < 0) {
                log_line("usb_hotplug: No free vpad slot for %s (%04x:%04x)", ctrl_name, vid, pid);
                close(fd);
                return;
            }

            log_line("usb_hotplug: Claiming %s on %s (slot %d, VID 0x%04x PID 0x%04x, route=%s iface=%u IN=0x%02x OUT=0x%02x)",
                     ctrl_name, path, slot, vid, pid, usb_route_name(route.kind),
                     (unsigned)route.iface, route.ep_in, route.ep_out);
            if (cfg_len > 0) log_hex_block("config", slot, cfg_raw, cfg_len);
#ifdef __PROSPERO__
            log_iface_drivers(fd, slot, cfg_raw, cfg_len);
            if (route.kind == USB_ROUTE_HID && cfg_len > 0) {
                /* Only now, once per claim: the report descriptors say which HID
                 * interface is the gamepad and how long its reports are */
                usb_config_info_t cfg;
                if (usb_desc_parse_config(cfg_raw, (size_t)cfg_len, &cfg) > 0) {
                    hid_fetch_ctx_t hctx = { fd, slot };
                    int found = usb_desc_refine_hid_route(&cfg, &route, fetch_hid_report_desc, &hctx);
                    log_line("usb_hotplug: slot %d HID gamepad collection %s (iface=%u IN=0x%02x report id=%u len=%u max=%u)",
                             slot, found ? "found" : "not found", (unsigned)route.iface, route.ep_in,
                             (unsigned)route.hid_report_id, (unsigned)route.hid_input_len,
                             (unsigned)route.hid_max_input_len);
                }
            }
#endif

            /* The XInput wake-up follows the interface, whatever the VID suggests (third-party
             * 360 pads listed as generic HID need it too); GIP pads and GameSir HID pads are
             * initialised by the reader thread. */
            if (route.kind == USB_ROUTE_X360) {
                usb_init_controller_handshake(fd, CTRL_XBOX_XINPUT);
            } else if (route.kind != USB_ROUTE_GIP && ctype != CTRL_GAMESIR) {
                usb_init_controller_handshake(fd, ctype);
            }

            usb_slot_device_t *sdev = &g_usb_hotplug.devices[slot];
            reset_device(sdev);
            sdev->active = 1;
            sdev->slot = slot;
            sdev->fd = fd;
            snprintf(sdev->dev_path, sizeof(sdev->dev_path), "%s", path);
            sdev->vid = vid;
            sdev->pid = pid;
            sdev->type = ctype;
            sdev->name = ctrl_name;
            sdev->stop_thread = 0;
            sdev->route = route;
            gip_init_ctx(&sdev->gip, vid, pid);

            int is_dongle = (vid == 0x045e && pid == 0x0719) ||
                            (vid == 0x045e && (pid == 0x02e6 || pid == 0x02fe)) ||
                            (vid == 0x1a34) ||
                            (vid == 0x2345 && pid == 0xe02f) ||
                            (vid == 0x2dc8) ||
                            (ctrl_name && (strstr(ctrl_name, "Dongle") != NULL ||
                                           strstr(ctrl_name, "Adapter") != NULL ||
                                           strstr(ctrl_name, "Receiver") != NULL));
            sdev->conn_type = is_dongle ? CONN_USB_DONGLE_24G : CONN_USB_WIRED;

            int thread_error = usb_lifecycle_start(
                &sdev->lifecycle, &USB_LIFECYCLE_OPS, sdev,
                usb_reader_worker, sdev);
            if (thread_error != 0) {
                log_line("usb_hotplug: Device lifecycle startup failed for %s (error=%d)",
                         path, thread_error);
                reset_device(sdev);
                continue;
            }
        }
    }
}

int usb_hotplug_init(void)
{
    pthread_mutex_lock(&g_usb_operation_lock);
    pthread_mutex_lock(&g_usb_hotplug.lock);
    if (g_usb_hotplug.initialized && g_usb_hotplug.shutting_down) {
        pthread_mutex_unlock(&g_usb_hotplug.lock);
        pthread_mutex_unlock(&g_usb_operation_lock);
        return 0;
    }
    if (!g_usb_hotplug.initialized) {
        for (int i = 0; i < MAX_SLOTS; i++) reset_device(&g_usb_hotplug.devices[i]);
        memset(s_last_seen, 0, sizeof(s_last_seen));
        g_usb_hotplug.last_scan_time = 0;
        g_usb_hotplug.initialized = 1;
    }
    g_usb_hotplug.shutting_down = 0;
    pthread_mutex_unlock(&g_usb_hotplug.lock);
    pthread_mutex_unlock(&g_usb_operation_lock);
    log_line("usb_hotplug: Initialized USB hotplug manager");
    return 1;
}

void usb_hotplug_poll(long now)
{
    int dead[MAX_SLOTS];
    int n_dead = 0;

    pthread_mutex_lock(&g_usb_operation_lock);
    pthread_mutex_lock(&g_usb_hotplug.lock);
    if (!g_usb_hotplug.initialized || g_usb_hotplug.shutting_down ||
        now - g_usb_hotplug.last_scan_time < 1500) {
        pthread_mutex_unlock(&g_usb_hotplug.lock);
        pthread_mutex_unlock(&g_usb_operation_lock);
        return;
    }
    g_usb_hotplug.last_scan_time = now;

    for (int i = 0; i < MAX_SLOTS; i++) {
        if (g_usb_hotplug.devices[i].active &&
            (access(g_usb_hotplug.devices[i].dev_path, F_OK) != 0 ||
             g_usb_hotplug.devices[i].stop_thread)) {
            log_line("usb_hotplug: Controller disconnected from %s (slot %d)",
                     g_usb_hotplug.devices[i].dev_path, i);
            g_usb_hotplug.devices[i].stop_thread = 1;
            dead[n_dead++] = i;
        }
    }
    pthread_mutex_unlock(&g_usb_hotplug.lock);

    /* Joins happen without the mutex the workers use to observe stop requests. */
    for (int d = 0; d < n_dead; d++) {
        int i = dead[d];
        usb_slot_device_t *sdev = &g_usb_hotplug.devices[i];
        int stop_error = usb_lifecycle_stop(&sdev->lifecycle,
                                            &USB_LIFECYCLE_OPS, sdev);
        if (stop_error != 0) {
            log_line("usb_hotplug: Device cleanup failed for slot %d (error=%d)",
                     i, stop_error);
            continue;
        }

        pthread_mutex_lock(&g_usb_hotplug.lock);
        int bus = 0, dev_idx = 0;
        if (sscanf(sdev->dev_path, "/dev/ugen%d.%d", &bus, &dev_idx) == 2 &&
            bus >= 0 && bus < 6 && dev_idx >= 0 && dev_idx < 21) {
            s_last_seen[bus][dev_idx] = 0;
        }
        reset_device(sdev);
        pthread_mutex_unlock(&g_usb_hotplug.lock);
    }

    pthread_mutex_lock(&g_usb_hotplug.lock);
    if (g_usb_hotplug.initialized && !g_usb_hotplug.shutting_down) {
        probe_usb_devices();
    }
    pthread_mutex_unlock(&g_usb_hotplug.lock);
    pthread_mutex_unlock(&g_usb_operation_lock);
}

void usb_hotplug_cleanup(void)
{
    pthread_mutex_lock(&g_usb_operation_lock);
    pthread_mutex_lock(&g_usb_hotplug.lock);
    if (!g_usb_hotplug.initialized) {
        pthread_mutex_unlock(&g_usb_hotplug.lock);
        pthread_mutex_unlock(&g_usb_operation_lock);
        return;
    }
    g_usb_hotplug.shutting_down = 1;
    for (int i = 0; i < MAX_SLOTS; i++) {
        if (g_usb_hotplug.devices[i].active) {
            g_usb_hotplug.devices[i].stop_thread = 1;
        }
    }
    pthread_mutex_unlock(&g_usb_hotplug.lock);

    for (int i = 0; i < MAX_SLOTS; i++) {
        usb_slot_device_t *sdev = &g_usb_hotplug.devices[i];
        if (!sdev->active) continue;
        int stop_error = usb_lifecycle_stop(&sdev->lifecycle,
                                            &USB_LIFECYCLE_OPS, sdev);
        if (stop_error != 0) {
            log_line("usb_hotplug: Cleanup failed for slot %d (error=%d)",
                     i, stop_error);
            continue;
        }
        pthread_mutex_lock(&g_usb_hotplug.lock);
        reset_device(sdev);
        pthread_mutex_unlock(&g_usb_hotplug.lock);
    }

    pthread_mutex_lock(&g_usb_hotplug.lock);
    int still_active = 0;
    for (int i = 0; i < MAX_SLOTS; i++) {
        if (g_usb_hotplug.devices[i].active) still_active = 1;
    }
    if (!still_active) {
        g_usb_hotplug.initialized = 0;
        g_usb_hotplug.shutting_down = 0;
        g_usb_hotplug.last_scan_time = 0;
    }
    pthread_mutex_unlock(&g_usb_hotplug.lock);
    pthread_mutex_unlock(&g_usb_operation_lock);
}
