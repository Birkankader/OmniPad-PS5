#include "gip.h"
#include "usb_controllers.h"

#include <string.h>

typedef struct {
    uint16_t vid;   /* 0 = any vendor */
    uint16_t pid;   /* 0 = any product */
    uint8_t  len;
    uint8_t  data[13];
} gip_init_packet_t;

/* Same packets and order as xboxone_init_packets[] in Linux xpad.c, plus the
 * final stop-rumble that SDL and xone send ("required for some exotic gamepads
 * to start input"). Byte 2 (sequence) is filled in when the packet is sent. */
static const gip_init_packet_t k_gip_init[] = {
    /* Hori pads need their identify request acknowledged first */
    { 0x0e6f, 0x0165, 13, { 0x01, 0x20, 0x00, 0x09, 0x00, 0x04, 0x20, 0x3a, 0x00, 0x00, 0x00, 0x80, 0x00 } },
    { 0x0f0d, 0x0067, 13, { 0x01, 0x20, 0x00, 0x09, 0x00, 0x04, 0x20, 0x3a, 0x00, 0x00, 0x00, 0x80, 0x00 } },
    /* Power on */
    { 0x0000, 0x0000, 5, { 0x05, 0x20, 0x00, 0x01, 0x00 } },
    /* Xbox One S / Elite 2 extra power packet */
    { 0x045e, 0x02ea, 5, { 0x05, 0x20, 0x00, 0x0f, 0x06 } },
    { 0x045e, 0x0b00, 5, { 0x05, 0x20, 0x00, 0x0f, 0x06 } },
    { 0x045e, 0x0b00, 6, { 0x4d, 0x10, 0x01, 0x02, 0x07, 0x00 } },
    /* Guide LED on */
    { 0x0000, 0x0000, 7, { 0x0a, 0x20, 0x00, 0x03, 0x00, 0x01, 0x14 } },
    /* "Authentication done": third-party pads wait for it before streaming input */
    { 0x0000, 0x0000, 6, { 0x06, 0x20, 0x00, 0x02, 0x01, 0x00 } },
    /* Some PowerA pads only start reporting after a rumble begin/end pair */
    { 0x24c6, 0x541a, 13, { 0x09, 0x00, 0x00, 0x09, 0x00, 0x0f, 0x00, 0x00, 0x1d, 0x1d, 0xff, 0x00, 0x00 } },
    { 0x24c6, 0x542a, 13, { 0x09, 0x00, 0x00, 0x09, 0x00, 0x0f, 0x00, 0x00, 0x1d, 0x1d, 0xff, 0x00, 0x00 } },
    { 0x24c6, 0x543a, 13, { 0x09, 0x00, 0x00, 0x09, 0x00, 0x0f, 0x00, 0x00, 0x1d, 0x1d, 0xff, 0x00, 0x00 } },
    { 0x24c6, 0x541a, 13, { 0x09, 0x00, 0x00, 0x09, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
    { 0x24c6, 0x542a, 13, { 0x09, 0x00, 0x00, 0x09, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
    { 0x24c6, 0x543a, 13, { 0x09, 0x00, 0x00, 0x09, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 } },
    /* Stop rumble */
    { 0x0000, 0x0000, 13, { 0x09, 0x00, 0x00, 0x09, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xeb } },
};

#define GIP_INIT_COUNT ((int)(sizeof(k_gip_init) / sizeof(k_gip_init[0])))

void gip_init_ctx(gip_ctx_t *ctx, uint16_t vid, uint16_t pid)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->vid = vid;
    ctx->pid = pid;
    ctx->out_seq = 1;
    /* Same quirk as xpad's MAP_SHARE_OFFSET and SDL's GIP_FEATURE_DYNAMIC_LATENCY_INPUT */
    ctx->dli = (vid == 0x045e && pid == 0x0b12);
    pad_state_neutral(&ctx->last);
}

void gip_restart_init(gip_ctx_t *ctx)
{
    if (ctx) ctx->init_step = 0;
}

void gip_reset_input(gip_ctx_t *ctx)
{
    if (!ctx) return;
    pad_state_neutral(&ctx->last);
    ctx->have_input = 0;
    ctx->guide_down = 0;
}

size_t gip_next_init_packet(gip_ctx_t *ctx, uint8_t *out, size_t cap)
{
    if (!ctx || !out || ctx->init_step < 0) return 0;
    while (ctx->init_step < GIP_INIT_COUNT) {
        const gip_init_packet_t *p = &k_gip_init[ctx->init_step++];
        if (p->vid != 0 && p->vid != ctx->vid) continue;
        if (p->pid != 0 && p->pid != ctx->pid) continue;
        if (cap < p->len) return 0;
        memcpy(out, p->data, p->len);
        /* Sequence 0x00 is reserved by [MS-GIPUSB]: count 1..255 like SDL and xone */
        if (ctx->out_seq == 0) ctx->out_seq = 1;
        out[2] = ctx->out_seq++;
        return p->len;
    }
    ctx->init_step = -1;
    return 0;
}

int gip_parse_header(const uint8_t *buf, size_t len, gip_header_t *hdr)
{
    if (!buf || !hdr || len < 4) return 0;
    memset(hdr, 0, sizeof(*hdr));
    hdr->command = buf[0];
    hdr->options = buf[1];
    hdr->sequence = buf[2];

    /* Payload length is a little-endian base-128 varint (at most 4 bytes) */
    uint32_t length = 0;
    size_t pos = 3;
    for (int shift = 0; ; shift += 7) {
        if (pos >= len || shift > 21) return 0;
        uint8_t b = buf[pos++];
        length |= (uint32_t)(b & 0x7f) << shift;
        if (!(b & 0x80)) break;
    }
    /* A fragment carries an extra varint (offset / total size) we do not need */
    if (hdr->options & GIP_OPT_CHUNK) {
        for (int i = 0; ; i++) {
            if (pos >= len || i > 3) return 0;
            if (!(buf[pos++] & 0x80)) break;
        }
    }
    hdr->length = length;
    hdr->header_len = pos;
    if (hdr->header_len + hdr->length > len) return 0;
    return 1;
}

size_t gip_build_ack(const uint8_t *msg, size_t len, uint8_t *out, size_t cap)
{
    gip_header_t h;
    if (!out || cap < 13 || !gip_parse_header(msg, len, &h)) return 0;
    if (!(h.options & GIP_OPT_ACK)) return 0;
    if (h.options & GIP_OPT_CHUNK) return 0; /* fragmented transfers are never requested */

    uint8_t client = (uint8_t)((h.options & GIP_OPT_CLIENT_MASK) | GIP_OPT_INTERNAL);
    memset(out, 0, 13);
    out[0] = GIP_CMD_ACK;
    out[1] = client;
    out[2] = h.sequence;
    out[3] = 0x09;
    out[4] = 0x00;
    out[5] = h.command;
    out[6] = client;
    out[7] = (uint8_t)(h.length & 0xff);       /* bytes received (le16) */
    out[8] = (uint8_t)((h.length >> 8) & 0xff);
    /* out[9..10] padding, out[11..12] bytes remaining = 0 */
    return 13;
}

int gip_parse_input(const uint8_t *buf, size_t len, int dli, pad_state_t *st)
{
    gip_header_t h;
    if (!st || !gip_parse_header(buf, len, &h)) return 0;
    if (h.command != GIP_CMD_INPUT || (h.options & GIP_OPT_CHUNK) || h.length < 14) return 0;

    const uint8_t *p = buf + h.header_len;
    pad_state_t out;
    pad_state_neutral(&out);

    uint8_t b0 = p[0], b1 = p[1];
    if (b0 & 0x04) out.buttons |= PAD_BTN_OPTIONS;  /* Menu */
    if (b0 & 0x08) out.buttons |= PAD_BTN_SHARE;    /* View -> Create */
    if (b0 & 0x10) out.buttons |= PAD_BTN_CROSS;    /* A */
    if (b0 & 0x20) out.buttons |= PAD_BTN_CIRCLE;   /* B */
    if (b0 & 0x40) out.buttons |= PAD_BTN_SQUARE;   /* X */
    if (b0 & 0x80) out.buttons |= PAD_BTN_TRIANGLE; /* Y */
    if (b1 & 0x01) out.buttons |= PAD_DPAD_UP;
    if (b1 & 0x02) out.buttons |= PAD_DPAD_DOWN;
    if (b1 & 0x04) out.buttons |= PAD_DPAD_LEFT;
    if (b1 & 0x08) out.buttons |= PAD_DPAD_RIGHT;
    if (b1 & 0x10) out.buttons |= PAD_BTN_L1;
    if (b1 & 0x20) out.buttons |= PAD_BTN_R1;
    if (b1 & 0x40) out.buttons |= PAD_BTN_L3;
    if (b1 & 0x80) out.buttons |= PAD_BTN_R3;

    /* 10-bit triggers (0..1023) */
    uint16_t lt = (uint16_t)(p[2] | ((uint16_t)p[3] << 8));
    uint16_t rt = (uint16_t)(p[4] | ((uint16_t)p[5] << 8));
    out.l2 = (uint8_t)(lt > 1023 ? 255 : (lt >> 2));
    out.r2 = (uint8_t)(rt > 1023 ? 255 : (rt >> 2));
    if (out.l2 >= PAD_TRIGGER_DIGITAL_THRESHOLD) out.buttons |= PAD_BTN_L2;
    if (out.r2 >= PAD_TRIGGER_DIGITAL_THRESHOLD) out.buttons |= PAD_BTN_R2;

    /* int16 LE sticks, Y up = positive */
    out.lx = axis16_to_u8((int16_t)(p[6] | ((uint16_t)p[7] << 8)), 0);
    out.ly = axis16_to_u8((int16_t)(p[8] | ((uint16_t)p[9] << 8)), 1);
    out.rx = axis16_to_u8((int16_t)(p[10] | ((uint16_t)p[11] << 8)), 0);
    out.ry = axis16_to_u8((int16_t)(p[12] | ((uint16_t)p[13] << 8)), 1);

    /* Share sits in the Console Function Map, the last 18 bytes of a report that
     * carries it (32-byte payload in the spec); SDL assumes it for 0x3537 pads too.
     * Series X|S payloads (SDL): firmware 5.0 32 bytes, Share at 14; 5.1 40 bytes,
     * Share at 14; 5.5 44 bytes, Share at 18 - 8 bytes of latency data follow the
     * map from 40 bytes on. */
    uint32_t share_off = 0;
    if (h.length >= 40 && dli) share_off = h.length - 26;
    else if (h.length >= 32) share_off = h.length - 18;
    if (share_off && (p[share_off] & 0x01)) out.buttons |= PAD_BTN_TOUCHPAD;

    *st = out;
    return 1;
}

/* Handles one complete message; returns GIP_EVENT_* bits. */
static int gip_handle_message(gip_ctx_t *ctx, const uint8_t *msg, size_t len, const gip_header_t *h)
{
    if (h->command == 0x00) return 0;                       /* zero padding */
    if (h->options & GIP_OPT_CHUNK) return GIP_EVENT_ALIVE;  /* metadata/security fragments */
    /* Attachments (headset, chatpad) use expansion indices 1..7. Their Hello repeats
     * every 500 ms until answered on their own index, so it must not reset the pad. */
    if (h->options & GIP_OPT_CLIENT_MASK) return GIP_EVENT_ALIVE;

    switch (h->command) {
    case GIP_CMD_INPUT: {
        pad_state_t in;
        if (!gip_parse_input(msg, len, ctx->dli, &in)) return GIP_EVENT_ALIVE;
        ctx->last = in;
        ctx->have_input = 1;
        ctx->started = 1;
        ctx->input_len = h->length;
        return GIP_EVENT_ALIVE | GIP_EVENT_STATE;
    }
    case GIP_CMD_VIRTUAL_KEY: {
        if (h->length < 1) return GIP_EVENT_ALIVE;
        int down = (msg[h->header_len] & 0x03) != 0;
        ctx->started = 1;
        if (down == ctx->guide_down) return GIP_EVENT_ALIVE; /* repeated report */
        ctx->guide_down = down;
        return GIP_EVENT_ALIVE | GIP_EVENT_STATE;
    }
    case GIP_CMD_STATUS: {
        if (!(h->options & GIP_OPT_INTERNAL)) return GIP_EVENT_ALIVE;
        ctx->started = 1;
        ctx->n_status++;
        /* Power level (bits 7:6) 00 = "powering off/resetting", which [MS-GIPUSB]
         * requires before the pad goes away; a 2.4 GHz receiver stays enumerated
         * and would otherwise keep the last input held. */
        if (h->length < 1 || (msg[h->header_len] & 0xc0) != 0) return GIP_EVENT_ALIVE | GIP_EVENT_STATUS;
        int events = GIP_EVENT_ALIVE | GIP_EVENT_STATUS | GIP_EVENT_POWER_OFF;
        if (ctx->have_input || ctx->guide_down) events |= GIP_EVENT_STATE;
        gip_reset_input(ctx);
        return events;
    }
    case GIP_CMD_ANNOUNCE: {
        if (!(h->options & GIP_OPT_INTERNAL)) return GIP_EVENT_ALIVE;
        /* Back in the Arrival state: whatever was held before is gone */
        int events = GIP_EVENT_ALIVE | GIP_EVENT_ANNOUNCE;
        ctx->started = 0;
        if (ctx->have_input || ctx->guide_down) events |= GIP_EVENT_STATE;
        gip_reset_input(ctx);
        return events;
    }
    default:
        return GIP_EVENT_ALIVE;
    }
}

int gip_process(gip_ctx_t *ctx, const uint8_t *buf, size_t len, pad_state_t *state,
                gip_ack_fn ack_fn, void *ack_user)
{
    if (!ctx || !buf || !state) return 0;

    int events = 0;
    size_t off = 0;
    /* One transfer may pack several messages ("coalescing" in [MS-GIPUSB]) */
    while (len - off >= 4) {
        const uint8_t *msg = buf + off;
        gip_header_t h;
        if (!gip_parse_header(msg, len - off, &h)) break;
        size_t msg_len = h.header_len + h.length;

        if (ack_fn) {
            uint8_t ack[16];
            size_t ack_len = gip_build_ack(msg, msg_len, ack, sizeof(ack));
            if (ack_len > 0) ack_fn(ack_user, ack, ack_len);
        }
        events |= gip_handle_message(ctx, msg, msg_len, &h);
        off += msg_len;
    }

    if (events & GIP_EVENT_STATE) {
        *state = ctx->last;
        if (ctx->guide_down) state->buttons |= PAD_BTN_PS;
    }
    return events;
}
