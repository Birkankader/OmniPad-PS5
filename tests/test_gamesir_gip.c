#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "../src/pad_types.h"
#include "../src/usb_controllers.h"
#include "../src/gip.h"

/* ---------- helpers ---------- */

typedef struct {
    int     count;
    uint8_t last[16];
    size_t  last_len;
} ack_log_t;

static void record_ack(void *user, const uint8_t *ack, size_t len)
{
    ack_log_t *log = (ack_log_t *)user;
    log->count++;
    assert(len <= sizeof(log->last));
    memcpy(log->last, ack, len);
    log->last_len = len;
}

static void put_le16(uint8_t *p, int v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
}

/* 0x20 input message with a 14-byte payload (header 4 bytes) */
static size_t gip_input(uint8_t *buf, uint8_t seq, uint8_t b0, uint8_t b1, int lt, int rt,
                        int lx, int ly, int rx, int ry)
{
    memset(buf, 0, 18);
    buf[0] = 0x20; buf[1] = 0x00; buf[2] = seq; buf[3] = 0x0e;
    buf[4] = b0; buf[5] = b1;
    put_le16(buf + 6, lt);
    put_le16(buf + 8, rt);
    put_le16(buf + 10, lx);
    put_le16(buf + 12, ly);
    put_le16(buf + 14, rx);
    put_le16(buf + 16, ry);
    return 18;
}

static size_t gip_hello(uint8_t *buf, uint8_t seq)
{
    memset(buf, 0, 32);
    buf[0] = 0x02; buf[1] = 0x20; buf[2] = seq; buf[3] = 0x1c;
    buf[12] = 0x37; buf[13] = 0x35; /* VID 0x3537 */
    buf[14] = 0xa8; buf[15] = 0x10; /* PID 0x10A8 */
    return 32;
}

/* ---------- GIP ---------- */

static void test_gip_init_sequence_gamesir(void)
{
    printf("Testing GIP init sequence for GameSir 3537:10A8... ");
    static const uint8_t power_on[] = { 0x05, 0x20, 0x01, 0x01, 0x00 };
    static const uint8_t led_on[]   = { 0x0a, 0x20, 0x02, 0x03, 0x00, 0x01, 0x14 };
    static const uint8_t auth[]     = { 0x06, 0x20, 0x03, 0x02, 0x01, 0x00 };
    static const uint8_t rumble[]   = { 0x09, 0x00, 0x04, 0x09, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xeb };
    const uint8_t *expect[] = { power_on, led_on, auth, rumble };
    const size_t expect_len[] = { sizeof(power_on), sizeof(led_on), sizeof(auth), sizeof(rumble) };

    gip_ctx_t ctx;
    gip_init_ctx(&ctx, 0x3537, 0x10a8);
    gip_restart_init(&ctx);
    uint8_t pkt[GIP_MAX_PACKET];
    for (int i = 0; i < 4; i++) {
        size_t n = gip_next_init_packet(&ctx, pkt, sizeof(pkt));
        assert(n == expect_len[i]);
        assert(memcmp(pkt, expect[i], n) == 0);
    }
    assert(gip_next_init_packet(&ctx, pkt, sizeof(pkt)) == 0);
    assert(gip_next_init_packet(&ctx, pkt, sizeof(pkt)) == 0);

    /* A re-announce restarts the table; sequence numbers keep counting */
    gip_restart_init(&ctx);
    assert(gip_next_init_packet(&ctx, pkt, sizeof(pkt)) == 5);
    assert(pkt[0] == 0x05 && pkt[2] == 0x05);
    printf("OK!\n");
}

static void test_gip_init_sequence_microsoft(void)
{
    printf("Testing GIP init sequence filtering for Xbox Elite 2 (045e:0b00)... ");
    gip_ctx_t ctx;
    gip_init_ctx(&ctx, 0x045e, 0x0b00);
    gip_restart_init(&ctx);
    uint8_t pkt[GIP_MAX_PACKET];
    uint8_t cmds[16];
    int count = 0;
    size_t n;
    while ((n = gip_next_init_packet(&ctx, pkt, sizeof(pkt))) > 0) {
        assert(count < 16);
        assert(pkt[2] == (uint8_t)(count + 1));
        cmds[count++] = pkt[0];
    }
    /* power_on, s_init, extra_input, led_on, auth_done, stop rumble */
    assert(count == 6);
    assert(cmds[0] == 0x05 && cmds[1] == 0x05 && cmds[2] == 0x4d);
    assert(cmds[3] == 0x0a && cmds[4] == 0x06 && cmds[5] == 0x09);
    printf("OK!\n");
}

static void test_gip_sequence_skips_zero(void)
{
    printf("Testing GIP sequence never uses reserved 0x00... ");
    gip_ctx_t ctx;
    gip_init_ctx(&ctx, 0x3537, 0x10a8);
    ctx.out_seq = 0xff;
    gip_restart_init(&ctx);
    uint8_t pkt[GIP_MAX_PACKET];
    assert(gip_next_init_packet(&ctx, pkt, sizeof(pkt)) > 0);
    assert(pkt[2] == 0xff);
    assert(gip_next_init_packet(&ctx, pkt, sizeof(pkt)) > 0);
    assert(pkt[2] == 0x01);
    printf("OK!\n");
}

static void test_gip_ack(void)
{
    printf("Testing GIP ACK construction... ");
    /* Xbox One S style guide press that requests an ACK (flags 0x30) */
    const uint8_t guide[] = { 0x07, 0x30, 0x05, 0x02, 0x01, 0x5b };
    const uint8_t expect[] = { 0x01, 0x20, 0x05, 0x09, 0x00, 0x07, 0x20, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t ack[16];
    assert(gip_build_ack(guide, sizeof(guide), ack, sizeof(ack)) == 13);
    assert(memcmp(ack, expect, 13) == 0);

    /* Client index comes from flag bits 2:0; reserved bit 3 is not copied */
    const uint8_t guide_client[] = { 0x07, 0x3a, 0x06, 0x02, 0x01, 0x5b };
    assert(gip_build_ack(guide_client, sizeof(guide_client), ack, sizeof(ack)) == 13);
    assert(ack[1] == 0x22 && ack[6] == 0x22);

    /* No ACK requested */
    uint8_t hello[32];
    gip_hello(hello, 1);
    assert(gip_build_ack(hello, sizeof(hello), ack, sizeof(ack)) == 0);
    uint8_t in[18];
    gip_input(in, 2, 0, 0, 0, 0, 0, 0, 0, 0);
    assert(gip_build_ack(in, sizeof(in), ack, sizeof(ack)) == 0);

    /* Too small output buffer / truncated message */
    assert(gip_build_ack(guide, sizeof(guide), ack, 12) == 0);
    assert(gip_build_ack(guide, 5, ack, sizeof(ack)) == 0);
    printf("OK!\n");
}

static void test_gip_input_mapping(void)
{
    printf("Testing GIP 0x20 input mapping... ");
    gip_ctx_t ctx;
    gip_init_ctx(&ctx, 0x3537, 0x10a8);
    uint8_t buf[18];
    pad_state_t st;
    /* Menu + A, D-pad up + LB, LT full, RT released, LX right, LY down */
    size_t n = gip_input(buf, 1, 0x04 | 0x10, 0x01 | 0x10, 1023, 0, 32767, -32768, 0, 0);
    int ev = gip_process(&ctx, buf, n, &st, NULL, NULL);
    assert(ev == (GIP_EVENT_STATE | GIP_EVENT_ALIVE));
    assert(st.buttons == (PAD_BTN_OPTIONS | PAD_BTN_CROSS | PAD_DPAD_UP | PAD_BTN_L1 | PAD_BTN_L2));
    assert(st.l2 == 255 && st.r2 == 0);
    assert(st.lx == 255 && st.ly == 255);
    assert(st.rx == 128 && st.ry == 128);
    assert(ctx.have_input && ctx.started && ctx.input_len == 14);

    /* Keep-alive (0x02) and reserved (0x01) bits of byte 0 are not buttons */
    n = gip_input(buf, 2, 0x03, 0x00, 0, 0, 0, 0, 0, 0);
    ev = gip_process(&ctx, buf, n, &st, NULL, NULL);
    assert(ev & GIP_EVENT_STATE);
    assert(st.buttons == 0);
    printf("OK!\n");
}

static void test_gip_right_stick_never_presses_buttons(void)
{
    printf("Testing GIP right stick sweep produces no buttons... ");
    gip_ctx_t ctx;
    gip_init_ctx(&ctx, 0x3537, 0x10a8);
    uint8_t buf[18];
    pad_state_t st;
    static const int vals[] = { -32768, -20000, -1, 0, 1, 12345, 32767 };
    for (size_t i = 0; i < sizeof(vals) / sizeof(vals[0]); i++) {
        for (size_t j = 0; j < sizeof(vals) / sizeof(vals[0]); j++) {
            size_t n = gip_input(buf, (uint8_t)(i * 8 + j + 1), 0, 0, 0, 0, 0, 0, vals[i], vals[j]);
            assert(gip_process(&ctx, buf, n, &st, NULL, NULL) & GIP_EVENT_STATE);
            assert(st.buttons == 0);
        }
    }
    assert(st.rx == 255 && st.ry == 0); /* last: right/up */
    printf("OK!\n");
}

static void test_gip_guide_merge(void)
{
    printf("Testing GIP guide button merge... ");
    gip_ctx_t ctx;
    gip_init_ctx(&ctx, 0x3537, 0x10a8);
    pad_state_t st;
    ack_log_t acks;
    memset(&acks, 0, sizeof(acks));

    uint8_t in[18];
    gip_input(in, 1, 0x10, 0, 0, 0, 0, 0, 0, 0); /* A held */
    assert(gip_process(&ctx, in, sizeof(in), &st, record_ack, &acks) & GIP_EVENT_STATE);

    const uint8_t press[]   = { 0x07, 0x20, 0x02, 0x02, 0x01, 0x5b };
    const uint8_t repeat[]  = { 0x07, 0x20, 0x03, 0x02, 0x01, 0x5b };
    const uint8_t release[] = { 0x07, 0x30, 0x04, 0x02, 0x00, 0x5b };

    int ev = gip_process(&ctx, press, sizeof(press), &st, record_ack, &acks);
    assert(ev & GIP_EVENT_STATE);
    assert(st.buttons == (PAD_BTN_CROSS | PAD_BTN_PS));

    ev = gip_process(&ctx, repeat, sizeof(repeat), &st, record_ack, &acks);
    assert(ev == GIP_EVENT_ALIVE);

    /* Release asked for an ACK; it must be sent and A must stay held */
    ev = gip_process(&ctx, release, sizeof(release), &st, record_ack, &acks);
    assert(ev & GIP_EVENT_STATE);
    assert(st.buttons == PAD_BTN_CROSS);
    assert(acks.count == 1 && acks.last_len == 13 && acks.last[2] == 0x04 && acks.last[5] == 0x07);

    /* Input while guide is held keeps PS set; ctx->last never stores it */
    gip_process(&ctx, press, sizeof(press), &st, NULL, NULL);
    gip_input(in, 5, 0x20, 0, 0, 0, 0, 0, 0, 0); /* B */
    assert(gip_process(&ctx, in, sizeof(in), &st, NULL, NULL) & GIP_EVENT_STATE);
    assert(st.buttons == (PAD_BTN_CIRCLE | PAD_BTN_PS));
    assert(!(ctx.last.buttons & PAD_BTN_PS));
    printf("OK!\n");
}

static void test_gip_announce_status_and_noise(void)
{
    printf("Testing GIP hello/status/ack messages are consumed silently... ");
    gip_ctx_t ctx;
    gip_init_ctx(&ctx, 0x3537, 0x10a8);
    pad_state_t st;
    pad_state_neutral(&st);
    st.buttons = 0xdeadu; /* must stay untouched without GIP_EVENT_STATE */

    uint8_t hello[32];
    gip_hello(hello, 1);
    int ev = gip_process(&ctx, hello, sizeof(hello), &st, NULL, NULL);
    assert(ev == (GIP_EVENT_ANNOUNCE | GIP_EVENT_ALIVE));
    assert(st.buttons == 0xdeadu);
    assert(!ctx.started);

    const uint8_t status[] = { 0x03, 0x20, 0x05, 0x04, 0xc0, 0x00, 0x00, 0x00 };
    ev = gip_process(&ctx, status, sizeof(status), &st, NULL, NULL);
    assert(ev == (GIP_EVENT_ALIVE | GIP_EVENT_STATUS));
    assert(ctx.started);

    const uint8_t ack_in[] = { 0x01, 0x20, 0x01, 0x09, 0x00, 0x05, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
    assert(gip_process(&ctx, ack_in, sizeof(ack_in), &st, NULL, NULL) == GIP_EVENT_ALIVE);
    assert(st.buttons == 0xdeadu);

    /* Truncated / garbage transfers are rejected outright */
    const uint8_t truncated[] = { 0x20, 0x00, 0x06, 0x0e, 0x00, 0x00 };
    assert(gip_process(&ctx, truncated, sizeof(truncated), &st, NULL, NULL) == 0);
    const uint8_t short_pkt[] = { 0x20, 0x00, 0x06 };
    assert(gip_process(&ctx, short_pkt, sizeof(short_pkt), &st, NULL, NULL) == 0);
    const uint8_t bad_varint[] = { 0x20, 0x00, 0x06, 0xff, 0xff, 0xff, 0xff, 0xff };
    assert(gip_process(&ctx, bad_varint, sizeof(bad_varint), &st, NULL, NULL) == 0);
    assert(st.buttons == 0xdeadu);

    /* Input shorter than 14 bytes is not a gamepad report */
    const uint8_t tiny_input[] = { 0x20, 0x00, 0x07, 0x02, 0xff, 0xff };
    assert(gip_process(&ctx, tiny_input, sizeof(tiny_input), &st, NULL, NULL) == GIP_EVENT_ALIVE);
    assert(!ctx.have_input);

    /* Fragments (metadata replies) are never decoded */
    const uint8_t chunk[] = { 0x04, 0xf0, 0x02, 0x3a, 0x00, 0x00, 0x00 };
    uint8_t chunk_buf[4 + 1 + 0x3a];
    memset(chunk_buf, 0, sizeof(chunk_buf));
    memcpy(chunk_buf, chunk, 5);
    ack_log_t acks;
    memset(&acks, 0, sizeof(acks));
    assert(gip_process(&ctx, chunk_buf, sizeof(chunk_buf), &st, record_ack, &acks) == GIP_EVENT_ALIVE);
    assert(acks.count == 0);
    printf("OK!\n");
}

static void test_gip_reannounce_releases_held_input(void)
{
    printf("Testing GIP re-announce releases held input... ");
    gip_ctx_t ctx;
    gip_init_ctx(&ctx, 0x3537, 0x10a8);
    pad_state_t st;
    uint8_t in[18];
    gip_input(in, 1, 0x10, 0x10, 0, 0, 20000, 0, 0, 0);
    gip_process(&ctx, in, sizeof(in), &st, NULL, NULL);
    assert(st.buttons != 0);

    uint8_t hello[32];
    gip_hello(hello, 2);
    int ev = gip_process(&ctx, hello, sizeof(hello), &st, NULL, NULL);
    assert(ev == (GIP_EVENT_ANNOUNCE | GIP_EVENT_ALIVE | GIP_EVENT_STATE));
    assert(st.buttons == 0 && st.lx == 128);
    assert(!ctx.started);
    printf("OK!\n");
}

static void test_gip_coalesced_messages(void)
{
    printf("Testing GIP coalesced messages in one transfer... ");
    gip_ctx_t ctx;
    gip_init_ctx(&ctx, 0x3537, 0x10a8);
    pad_state_t st;
    ack_log_t acks;
    memset(&acks, 0, sizeof(acks));

    uint8_t buf[64];
    size_t off = 0;
    const uint8_t status[] = { 0x03, 0x20, 0x01, 0x04, 0x80, 0x00, 0x00, 0x00 };
    memcpy(buf + off, status, sizeof(status)); off += sizeof(status);
    const uint8_t press[] = { 0x07, 0x30, 0x02, 0x02, 0x01, 0x5b };
    memcpy(buf + off, press, sizeof(press)); off += sizeof(press);
    off += gip_input(buf + off, 3, 0x80, 0x00, 0, 0, 0, 0, 0, 0); /* Y */

    int ev = gip_process(&ctx, buf, off, &st, record_ack, &acks);
    assert(ev == (GIP_EVENT_STATE | GIP_EVENT_ALIVE | GIP_EVENT_STATUS));
    assert(st.buttons == (PAD_BTN_TRIANGLE | PAD_BTN_PS));
    assert(acks.count == 1 && acks.last[5] == 0x07 && acks.last[2] == 0x02);

    /* Zero padding after a message is skipped */
    uint8_t padded[64];
    memset(padded, 0, sizeof(padded));
    gip_input(padded, 4, 0x40, 0, 0, 0, 0, 0, 0, 0); /* X */
    ev = gip_process(&ctx, padded, sizeof(padded), &st, NULL, NULL);
    assert(ev & GIP_EVENT_STATE);
    assert(st.buttons == (PAD_BTN_SQUARE | PAD_BTN_PS));
    printf("OK!\n");
}

static void test_gip_varint_and_share(void)
{
    printf("Testing GIP varint length and Share button... ");
    gip_ctx_t ctx;
    gip_init_ctx(&ctx, 0x3537, 0x10a8);
    pad_state_t st;

    /* Same 14-byte payload with a two-byte length encoding (0x8e 0x00) */
    uint8_t buf[19];
    memset(buf, 0, sizeof(buf));
    buf[0] = 0x20; buf[1] = 0x00; buf[2] = 0x01; buf[3] = 0x8e; buf[4] = 0x00;
    buf[5] = 0x10; /* A */
    assert(gip_process(&ctx, buf, sizeof(buf), &st, NULL, NULL) & GIP_EVENT_STATE);
    assert(st.buttons == PAD_BTN_CROSS);

    /* 32-byte payload: Console Function Map at payload[14], Share = bit 0 */
    uint8_t big[4 + 32];
    memset(big, 0, sizeof(big));
    big[0] = 0x20; big[1] = 0x00; big[2] = 0x02; big[3] = 32;
    big[4 + 14] = 0x01;
    assert(gip_process(&ctx, big, sizeof(big), &st, NULL, NULL) & GIP_EVENT_STATE);
    assert(st.buttons == PAD_BTN_TOUCHPAD);
    big[4 + 14] = 0x00;
    assert(gip_process(&ctx, big, sizeof(big), &st, NULL, NULL) & GIP_EVENT_STATE);
    assert(st.buttons == 0);
    printf("OK!\n");
}

/* Builds a 0x20 message with an arbitrary payload length and Share at payload[share_at] */
static size_t gip_input_len(uint8_t *buf, uint8_t seq, int plen, int share_at)
{
    memset(buf, 0, 4 + (size_t)plen);
    buf[0] = 0x20; buf[1] = 0x00; buf[2] = seq; buf[3] = (uint8_t)plen;
    if (share_at >= 0) buf[4 + share_at] = 0x01;
    return 4 + (size_t)plen;
}

static void test_gip_share_dynamic_latency_input(void)
{
    printf("Testing GIP Share offset with Dynamic Latency Input (Series X|S)... ");
    gip_ctx_t ctx;
    gip_init_ctx(&ctx, 0x045e, 0x0b12);
    assert(ctx.dli == 1);
    pad_state_t st;
    uint8_t buf[4 + 48];

    /* Firmware 5.0: 32-byte payload, Share at byte 14 (len - 18) */
    gip_input_len(buf, 1, 32, 14);
    assert(gip_process(&ctx, buf, 4 + 32, &st, NULL, NULL) & GIP_EVENT_STATE);
    assert(st.buttons == PAD_BTN_TOUCHPAD);
    /* Firmware 5.1: 40-byte payload, Share at byte 14 (len - 26); len - 18 is not Share */
    gip_input_len(buf, 2, 40, 14);
    assert(gip_process(&ctx, buf, 4 + 40, &st, NULL, NULL) & GIP_EVENT_STATE);
    assert(st.buttons == PAD_BTN_TOUCHPAD);
    gip_input_len(buf, 3, 40, 22);
    assert(gip_process(&ctx, buf, 4 + 40, &st, NULL, NULL) & GIP_EVENT_STATE);
    assert(st.buttons == 0);
    /* Firmware 5.5: 44-byte payload, Share at byte 18 (len - 26) */
    gip_input_len(buf, 4, 44, 18);
    assert(gip_process(&ctx, buf, 4 + 44, &st, NULL, NULL) & GIP_EVENT_STATE);
    assert(st.buttons == PAD_BTN_TOUCHPAD);
    /* Shorter payloads have no function map: right-stick bytes are never Share */
    for (int plen = 14; plen < 32; plen++) {
        for (int b = 10; b < 14; b++) {
            gip_input_len(buf, (uint8_t)(5 + plen), plen, b);
            assert(gip_process(&ctx, buf, 4 + (size_t)plen, &st, NULL, NULL) & GIP_EVENT_STATE);
            assert(!(st.buttons & PAD_BTN_TOUCHPAD));
        }
    }

    /* Other pads keep the plain function map offset */
    gip_init_ctx(&ctx, 0x3537, 0x10a8);
    assert(ctx.dli == 0);
    gip_input_len(buf, 5, 40, 22);
    assert(gip_process(&ctx, buf, 4 + 40, &st, NULL, NULL) & GIP_EVENT_STATE);
    assert(st.buttons == PAD_BTN_TOUCHPAD);
    gip_input_len(buf, 6, 28, 10); /* no function map below 32 bytes */
    assert(gip_process(&ctx, buf, 4 + 28, &st, NULL, NULL) & GIP_EVENT_STATE);
    assert(st.buttons == 0);
    printf("OK!\n");
}

static void test_gip_attachment_messages_ignored(void)
{
    printf("Testing GIP attachment (expansion index 1..7) traffic leaves the pad alone... ");
    gip_ctx_t ctx;
    gip_init_ctx(&ctx, 0x3537, 0x10a8);
    pad_state_t st;
    uint8_t in[18];
    gip_input(in, 1, 0x10, 0x00, 0, 1023, 0, 32767, 0, 0); /* A + RT + LY up */
    assert(gip_process(&ctx, in, sizeof(in), &st, NULL, NULL) & GIP_EVENT_STATE);
    pad_state_t held = st;

    /* Headset Hello on index 1, repeated every 500 ms until answered */
    uint8_t hello[32];
    gip_hello(hello, 2);
    hello[1] = 0x21;
    st.buttons = 0xdeadu;
    for (int i = 0; i < 4; i++) {
        assert(gip_process(&ctx, hello, sizeof(hello), &st, NULL, NULL) == GIP_EVENT_ALIVE);
    }
    assert(st.buttons == 0xdeadu);
    assert(ctx.started && ctx.have_input);
    assert(ctx.last.buttons == held.buttons && ctx.last.r2 == held.r2 && ctx.last.ly == held.ly);

    /* Input and guide from an attachment are not the pad's */
    gip_input(in, 3, 0x20, 0x00, 0, 0, 0, 0, 0, 0);
    in[1] = 0x02;
    assert(gip_process(&ctx, in, sizeof(in), &st, NULL, NULL) == GIP_EVENT_ALIVE);
    const uint8_t guide2[] = { 0x07, 0x22, 0x04, 0x02, 0x01, 0x5b };
    assert(gip_process(&ctx, guide2, sizeof(guide2), &st, NULL, NULL) == GIP_EVENT_ALIVE);
    assert(!ctx.guide_down);

    /* An attachment message asking for an ACK still gets one, on its own index */
    const uint8_t ack_me[] = { 0x07, 0x33, 0x05, 0x02, 0x01, 0x5b };
    ack_log_t acks;
    memset(&acks, 0, sizeof(acks));
    assert(gip_process(&ctx, ack_me, sizeof(ack_me), &st, record_ack, &acks) == GIP_EVENT_ALIVE);
    assert(acks.count == 1 && acks.last[1] == 0x23 && acks.last[6] == 0x23);

    /* Hello and status without the system flag are not system messages */
    gip_hello(hello, 6);
    hello[1] = 0x00;
    assert(gip_process(&ctx, hello, sizeof(hello), &st, NULL, NULL) == GIP_EVENT_ALIVE);
    assert(ctx.started);

    /* The primary pad's own Hello still restarts it and forgets the held input */
    gip_hello(hello, 7);
    assert(gip_process(&ctx, hello, sizeof(hello), &st, NULL, NULL) ==
           (GIP_EVENT_ALIVE | GIP_EVENT_ANNOUNCE | GIP_EVENT_STATE));
    assert(!ctx.started && !ctx.have_input && !ctx.guide_down);
    assert(st.buttons == 0 && st.r2 == 0 && st.ly == 128);
    printf("OK!\n");
}

static void test_gip_power_off_status_releases_input(void)
{
    printf("Testing GIP 'powering off' status releases held input... ");
    gip_ctx_t ctx;
    gip_init_ctx(&ctx, 0x3537, 0x10a8);
    pad_state_t st;
    uint8_t in[18];
    gip_input(in, 1, 0x00, 0x00, 0, 0, 0, 32767, 0, 0); /* left stick held up */
    gip_process(&ctx, in, sizeof(in), &st, NULL, NULL);
    const uint8_t press[] = { 0x07, 0x20, 0x02, 0x02, 0x01, 0x5b }; /* Home held to switch off */
    gip_process(&ctx, press, sizeof(press), &st, NULL, NULL);
    assert(st.ly == 0 && (st.buttons & PAD_BTN_PS));

    /* Full power, charging, rechargeable, full: just a heartbeat */
    const uint8_t alive[] = { 0x03, 0x20, 0x03, 0x04, 0x9b, 0x00, 0x00, 0x00 };
    assert(gip_process(&ctx, alive, sizeof(alive), &st, NULL, NULL) == (GIP_EVENT_ALIVE | GIP_EVENT_STATUS));

    /* Power level 00 = powering off, battery bits still set */
    const uint8_t off[] = { 0x03, 0x20, 0x04, 0x04, 0x0b, 0x00, 0x00, 0x00 };
    int ev = gip_process(&ctx, off, sizeof(off), &st, NULL, NULL);
    assert(ev == (GIP_EVENT_STATE | GIP_EVENT_ALIVE | GIP_EVENT_STATUS | GIP_EVENT_POWER_OFF));
    assert(st.buttons == 0 && st.ly == 128 && st.lx == 128);
    assert(!ctx.guide_down && !ctx.have_input);

    /* Repeated: nothing left to release */
    assert(gip_process(&ctx, off, sizeof(off), &st, NULL, NULL) ==
           (GIP_EVENT_ALIVE | GIP_EVENT_STATUS | GIP_EVENT_POWER_OFF));

    /* Legacy 1-byte status and an attachment's status */
    gip_process(&ctx, in, sizeof(in), &st, NULL, NULL);
    const uint8_t legacy_off[] = { 0x03, 0x20, 0x06, 0x01, 0x00 };
    assert(gip_process(&ctx, legacy_off, sizeof(legacy_off), &st, NULL, NULL) & GIP_EVENT_STATE);
    gip_process(&ctx, in, sizeof(in), &st, NULL, NULL);
    const uint8_t attach_off[] = { 0x03, 0x21, 0x07, 0x04, 0x00, 0x00, 0x00, 0x00 };
    assert(gip_process(&ctx, attach_off, sizeof(attach_off), &st, NULL, NULL) == GIP_EVENT_ALIVE);
    assert(ctx.have_input);

    /* A link that went silent: the reader forgets guide and input, so a pad that
     * comes back without a Hello does not replay them */
    gip_process(&ctx, press, sizeof(press), &st, NULL, NULL);
    assert(ctx.guide_down && ctx.have_input);
    gip_reset_input(&ctx);
    assert(!ctx.guide_down && !ctx.have_input && ctx.last.ly == 128);
    gip_input(in, 9, 0x00, 0x00, 0, 0, 1000, 0, 0, 0); /* left stick nudged */
    assert(gip_process(&ctx, in, sizeof(in), &st, NULL, NULL) & GIP_EVENT_STATE);
    assert(!(st.buttons & PAD_BTN_PS));
    printf("OK!\n");
}

/* ---------- GameSir HID ---------- */

static void test_gamesir_simple_report(void)
{
    printf("Testing GameSir 9-byte HID report... ");
    pad_state_t st;
    /* BTN1 BTN2 HAT LX LY RX RY L2 R2 : A + Start, hat neutral, sticks centred */
    uint8_t r[9] = { 0x01, 0x08, 0x08, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 };
    assert(usb_parse_gamesir_hid_report(r, sizeof(r), &st) == 1);
    assert(st.buttons == (PAD_BTN_CROSS | PAD_BTN_OPTIONS));
    assert(st.lx == 0x80 && st.ry == 0x80);

    /* Hat: 0 = up, 2 = right, 4 = down, 6 = left */
    r[0] = 0; r[1] = 0;
    r[2] = 0; assert(usb_parse_gamesir_hid_report(r, sizeof(r), &st) && st.buttons == PAD_DPAD_UP);
    r[2] = 3; assert(usb_parse_gamesir_hid_report(r, sizeof(r), &st) && st.buttons == (PAD_DPAD_DOWN | PAD_DPAD_RIGHT));
    r[2] = 6; assert(usb_parse_gamesir_hid_report(r, sizeof(r), &st) && st.buttons == PAD_DPAD_LEFT);
    r[2] = 0x0f; assert(usb_parse_gamesir_hid_report(r, sizeof(r), &st) && st.buttons == 0);

    /* Digital trigger bit with no analog value (hair-trigger lock) gives a full press */
    r[1] = 0x01 | 0x02; r[7] = 0x00; r[8] = 0x90;
    assert(usb_parse_gamesir_hid_report(r, sizeof(r), &st));
    assert(st.l2 == 255 && st.r2 == 0x90);
    assert(st.buttons == (PAD_BTN_L2 | PAD_BTN_R2));

    /* ...but a light pull keeps its analog value even when the bit is already set */
    r[7] = 8; r[8] = 29;
    assert(usb_parse_gamesir_hid_report(r, sizeof(r), &st));
    assert(st.l2 == 8 && st.r2 == 29);
    assert(st.buttons == 0);

    /* The same 9 bytes behind report ID 0x01 */
    uint8_t rid[10] = { 0x01, 0x01, 0x08, 0x08, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 };
    assert(usb_parse_gamesir_hid_report(rid, sizeof(rid), &st) == 1);
    assert(st.buttons == (PAD_BTN_CROSS | PAD_BTN_OPTIONS));
    printf("OK!\n");
}

static void test_gamesir_right_stick_regression(void)
{
    printf("Testing GameSir right stick never maps to buttons (reported bug)... ");
    pad_state_t st;
    for (int rx = 0; rx <= 255; rx += 5) {
        for (int ry = 0; ry <= 255; ry += 5) {
            uint8_t r[9] = { 0x00, 0x00, 0x08, 0x80, 0x80, (uint8_t)rx, (uint8_t)ry, 0x00, 0x00 };
            assert(usb_parse_gamesir_hid_report(r, sizeof(r), &st) == 1);
            assert(st.buttons == 0);
            assert(st.rx == rx && st.ry == ry);
            assert(st.lx == 0x80 && st.ly == 0x80);
            /* Same through the type dispatcher used by route-less callers */
            assert(usb_parse_input_report(CTRL_GAMESIR, r, sizeof(r), &st) == 1);
            assert(st.buttons == 0);
        }
    }
    printf("OK!\n");
}

static void test_gamesir_full_report(void)
{
    printf("Testing GameSir A1 C8 full report... ");
    pad_state_t st;
    uint8_t r[3 + 14 + 8];
    memset(r, 0, sizeof(r));
    r[0] = 0x43; r[1] = 0xa1; r[2] = 0xc8;
    uint8_t *p = r + 3;
    p[0] = 0x02 | 0x10;        /* B + Y */
    p[1] = 0x10 | 0x80;        /* Home + Capture */
    p[2] = 0x03;               /* hat: 1 = up, 3 = right */
    p[4] = 0x7f; p[5] = 0xff;  /* LX +32767 (BE) */
    p[6] = 0x7f; p[7] = 0xff;  /* LY +32767 = up */
    p[8] = 0x80; p[9] = 0x00;  /* RX -32768 */
    p[10] = 0x00; p[11] = 0x00;/* RY 0 */
    p[12] = 0x40; p[13] = 0xff;
    assert(usb_parse_gamesir_hid_report(r, sizeof(r), &st) == 1);
    assert(st.buttons == (PAD_BTN_CIRCLE | PAD_BTN_TRIANGLE | PAD_BTN_PS | PAD_BTN_TOUCHPAD |
                          PAD_DPAD_RIGHT | PAD_BTN_L2 | PAD_BTN_R2));
    assert(st.lx == 255 && st.ly == 0);
    assert(st.rx == 0 && st.ry == 128);
    assert(st.l2 == 0x40 && st.r2 == 0xff);

    /* Without the report ID */
    assert(usb_parse_gamesir_hid_report(r + 1, sizeof(r) - 1, &st) == 1);
    assert(st.buttons & PAD_BTN_PS);

    /* Header with a truncated payload */
    assert(usb_parse_gamesir_hid_report(r, 3 + 13, &st) == 0);
    printf("OK!\n");
}

static void test_gamesir_bt_simple_and_rejects(void)
{
    printf("Testing GameSir BT simple report and rejected packets... ");
    pad_state_t st;
    /* 07 LX LY RX RY HAT BTN1 BTN2 R2 L2 */
    uint8_t bt[10] = { 0x07, 0x00, 0xff, 0x80, 0x80, 0x08, 0x08, 0x04, 0x18, 0x10 };
    assert(usb_parse_gamesir_hid_report(bt, sizeof(bt), &st) == 1);
    assert(st.buttons == (PAD_BTN_SQUARE | PAD_BTN_SHARE));
    assert(st.lx == 0x00 && st.ly == 0xff);
    assert(st.r2 == 0x18 && st.l2 == 0x10);

    /* An XInput report and odd lengths are not GameSir HID reports */
    uint8_t x360[20];
    memset(x360, 0, sizeof(x360));
    x360[1] = 0x14;
    assert(usb_parse_gamesir_hid_report(x360, sizeof(x360), &st) == 0);
    uint8_t eight[8] = { 0 };
    assert(usb_parse_gamesir_hid_report(eight, sizeof(eight), &st) == 0);
    assert(usb_parse_gamesir_hid_report(NULL, 9, &st) == 0);

    /* A GIP Hello (02 20 ss 1c + 28 bytes) is not a Bluetooth-style 0x02 report:
     * decoded, its MAC bytes became PS/L2/R2 presses every 500 ms */
    uint8_t hello[32];
    gip_hello(hello, 1);
    for (int i = 0; i < 6; i++) hello[4 + i] = (uint8_t)(0xa5 + 17 * i); /* MAC */
    assert(usb_parse_gamesir_hid_report(hello, sizeof(hello), &st) == 0);
    assert(usb_parse_input_report(CTRL_GAMESIR, hello, sizeof(hello), &st) == 0);

    uint8_t cmd[64];
    assert(usb_gamesir_build_mode_switch(cmd, sizeof(cmd)) == 64);
    assert(cmd[0] == 0xa2 && cmd[1] == 0x01 && cmd[2] == 0x00 && cmd[63] == 0x00);
    assert(usb_gamesir_build_mode_switch(cmd, 63) == 0);
    printf("OK!\n");
}

/* G7 Pro HID identity 3537:1022. The 9-byte USB vectors follow the documented layout
 * (SDL HandleSimpleStatePacketUSB, macOS reports); the 11-byte ones are the Bluetooth
 * captures of the gamesir-g7-pro-controller-hub fixtures. No wired USB capture exists yet. */
static void test_gamesir_g7pro_hid_vectors(void)
{
    printf("Testing G7 Pro 3537:1022 captured report vectors... ");
    pad_state_t st;
    const uint8_t idle[9]    = { 0x00, 0x00, 0x08, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 };
    const uint8_t idle_f[9]  = { 0x00, 0x00, 0x0f, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 };
    const uint8_t press_a[9] = { 0x01, 0x00, 0x08, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 };
    assert(usb_parse_gamesir_hid_report(idle, 9, &st) && st.buttons == 0);
    assert(st.lx == 0x80 && st.ly == 0x80 && st.rx == 0x80 && st.ry == 0x80);
    assert(usb_parse_gamesir_hid_report(idle_f, 9, &st) && st.buttons == 0);
    assert(usb_parse_gamesir_hid_report(press_a, 9, &st) && st.buttons == PAD_BTN_CROSS);

    /* Left stick up-left: 8-bit sticks pass through (0 = up already matches DualSense) */
    const uint8_t ls[9] = { 0x00, 0x00, 0x08, 0x00, 0x00, 0x80, 0x80, 0x00, 0x00 };
    assert(usb_parse_gamesir_hid_report(ls, 9, &st) && st.lx == 0 && st.ly == 0 && st.buttons == 0);

    const uint8_t bt_idle[11]  = { 0x07, 0x80, 0x80, 0x80, 0x80, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t bt_south[11] = { 0x07, 0x80, 0x80, 0x80, 0x80, 0x0f, 0x01, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t bt_l2[11]    = { 0x07, 0x80, 0x80, 0x80, 0x80, 0x0f, 0x00, 0x01, 0x00, 0xff, 0x00 };
    const uint8_t bt_r2[11]    = { 0x07, 0x80, 0x80, 0x80, 0x80, 0x0f, 0x00, 0x02, 0xff, 0x00, 0x00 };
    assert(usb_parse_gamesir_hid_report(bt_idle, 11, &st) && st.buttons == 0);
    assert(usb_parse_gamesir_hid_report(bt_south, 11, &st) && st.buttons == PAD_BTN_CROSS);
    assert(usb_parse_gamesir_hid_report(bt_l2, 11, &st) && st.l2 == 0xff && st.r2 == 0 && st.buttons == PAD_BTN_L2);
    assert(usb_parse_gamesir_hid_report(bt_r2, 11, &st) && st.r2 == 0xff && st.l2 == 0 && st.buttons == PAD_BTN_R2);

    const char *name = NULL;
    assert(usb_identify_controller(0x3537, 0x1022, &name) == CTRL_GAMESIR);
    assert(name && strstr(name, "G7 Pro") != NULL);
    printf("OK!\n");
}

/* ---------- XInput ---------- */

static void test_x360_strict(void)
{
    printf("Testing strict XInput report parsing... ");
    pad_state_t st;
    uint8_t r[20];
    memset(r, 0, sizeof(r));
    r[0] = 0x00; r[1] = 0x14;
    r[2] = 0x10 | 0x01;        /* Start + D-pad up */
    r[3] = 0x10 | 0x01 | 0x04; /* A + LB + Guide */
    r[4] = 0xff;               /* LT */
    put_le16(r + 6, 32767);    /* LX */
    put_le16(r + 12, 32767);   /* RY up */
    assert(usb_parse_x360_report(r, sizeof(r), &st) == 1);
    assert(st.buttons == (PAD_BTN_OPTIONS | PAD_DPAD_UP | PAD_BTN_CROSS | PAD_BTN_L1 | PAD_BTN_PS | PAD_BTN_L2));
    assert(st.lx == 255 && st.ry == 0 && st.rx == 128);
    assert(st.l2 == 255 && st.r2 == 0);

    /* Right stick alone never sets buttons */
    memset(r + 2, 0, 18);
    put_le16(r + 10, -32768);
    put_le16(r + 12, -32768);
    assert(usb_parse_x360_report(r, sizeof(r), &st) == 1);
    assert(st.buttons == 0 && st.rx == 0 && st.ry == 255);

    /* LED / rumble / link status messages are not input */
    const uint8_t led[] = { 0x01, 0x03, 0x0e };
    const uint8_t rumble[] = { 0x03, 0x03, 0x03 };
    const uint8_t link[] = { 0x08, 0x80, 0x00 };
    const uint8_t unk[] = { 0x02, 0x03, 0x00 };
    assert(usb_parse_x360_report(led, sizeof(led), &st) == 0);
    assert(usb_parse_x360_report(rumble, sizeof(rumble), &st) == 0);
    assert(usb_parse_x360_report(link, sizeof(link), &st) == 0);
    assert(usb_parse_x360_report(unk, sizeof(unk), &st) == 0);
    assert(usb_parse_x360_report(r, 13, &st) == 0);

    /* Wireless receiver announce (00 0F ...) is not pad data */
    uint8_t ann[29];
    memset(ann, 0, sizeof(ann));
    ann[1] = 0x0f; ann[3] = 0xf0; ann[4] = 0xf0; ann[5] = 0xcc;
    assert(usb_parse_x360_report(ann, sizeof(ann), &st) == 0);

    /* Wireless receiver framing belongs to usb_parse_x360w_report() only */
    uint8_t w[29];
    memset(w, 0, sizeof(w));
    w[0] = 0x00; w[1] = 0x01; w[2] = 0x00; w[3] = 0xf0;
    w[4] = 0x00; w[5] = 0x13; w[7] = 0x20; /* B */
    assert(usb_parse_x360_report(w, sizeof(w), &st) == 0);
    printf("OK!\n");
}

static void test_x360_wireless_receiver(void)
{
    printf("Testing Xbox 360 wireless receiver framing... ");
    pad_state_t st;
    uint8_t w[29];
    memset(w, 0, sizeof(w));
    w[0] = 0x00; w[1] = 0x01; w[2] = 0x00; w[3] = 0xf0;
    w[4] = 0x00; w[5] = 0x13; w[7] = 0x20; /* B */
    put_le16(w + 4 + 10, 32767);          /* RX */
    assert(usb_parse_x360w_report(w, sizeof(w), &st) == X360W_PKT_STATE);
    assert(st.buttons == PAD_BTN_CIRCLE && st.rx == 255);

    /* Battery (00 00 00 13 lvl) and the trailer after presses (00 00 00 f0) decoded
     * as wired frames gave Cross|L1|R1|L2 and all four face buttons */
    pad_state_t untouched;
    pad_state_neutral(&untouched);
    untouched.buttons = 0xdeadu;
    uint8_t battery[29], trailer[29], announce[29];
    memset(battery, 0, sizeof(battery));
    battery[3] = 0x13; battery[4] = 0xa2;
    memset(trailer, 0, sizeof(trailer));
    trailer[3] = 0xf0;
    memset(announce, 0, sizeof(announce));
    announce[1] = 0x0f; announce[3] = 0xf0; announce[4] = 0xf0; announce[5] = 0xcc;
    st = untouched;
    assert(usb_parse_x360w_report(battery, sizeof(battery), &st) == X360W_PKT_STATUS);
    assert(usb_parse_x360w_report(trailer, sizeof(trailer), &st) == X360W_PKT_STATUS);
    assert(usb_parse_x360w_report(announce, sizeof(announce), &st) == X360W_PKT_STATUS);
    assert(st.buttons == 0xdeadu);
    assert(usb_parse_x360_report(battery, sizeof(battery), &st) == 1); /* wired framing would accept it */

    /* Presence: 08 80 connected, 08 00 pad gone (state released) */
    const uint8_t connected[] = { 0x08, 0x80 };
    const uint8_t gone[] = { 0x08, 0x00 };
    st = untouched;
    assert(usb_parse_x360w_report(connected, sizeof(connected), &st) == X360W_PKT_STATUS);
    assert(st.buttons == 0xdeadu);
    assert(usb_parse_x360w_report(gone, sizeof(gone), &st) == X360W_PKT_LINK_LOST);
    assert(st.buttons == 0 && st.lx == 128 && st.rx == 128 && st.l2 == 0);

    assert(usb_parse_x360w_report(w, 17, &st) == X360W_PKT_STATUS);
    assert(usb_parse_x360w_report(w, 1, &st) == X360W_PKT_UNKNOWN);
    assert(usb_parse_x360w_report(NULL, 29, &st) == X360W_PKT_UNKNOWN);
    printf("OK!\n");
}

/* ---------- identification ---------- */

static void test_gamesir_identification(void)
{
    printf("Testing GameSir identification... ");
    const char *name = NULL;
    assert(usb_identify_controller(0x3537, 0x10a8, &name) == CTRL_GAMESIR);
    assert(name && strcmp(name, "GameSir G7 Pro (Xbox/GIP)") == 0);
    assert(usb_identify_controller(0x3537, 0x10a9, &name) == CTRL_GAMESIR);
    assert(name && strstr(name, "XInput") != NULL);
    assert(usb_identify_controller(0x3537, 0x0575, &name) == CTRL_GAMESIR);
    assert(usb_identify_controller(0x3537, 0xbeef, &name) == CTRL_GAMESIR);
    assert(name && strcmp(name, "GameSir Controller") == 0);

    /* The type dispatcher still decodes an XInput report for a route-less GameSir pad */
    uint8_t r[20];
    memset(r, 0, sizeof(r));
    r[1] = 0x14;
    r[3] = 0x80; /* Y */
    pad_state_t st;
    assert(usb_parse_input_report(CTRL_GAMESIR, r, sizeof(r), &st) == 1);
    assert(st.buttons == PAD_BTN_TRIANGLE);
    /* ...but rejects GIP traffic instead of guessing */
    const uint8_t status[] = { 0x03, 0x20, 0x05, 0x04, 0xc0, 0x00, 0x00, 0x00 };
    assert(usb_parse_input_report(CTRL_GAMESIR, status, sizeof(status), &st) == 0);
    printf("OK!\n");
}

int main(void)
{
    test_gip_init_sequence_gamesir();
    test_gip_init_sequence_microsoft();
    test_gip_sequence_skips_zero();
    test_gip_ack();
    test_gip_input_mapping();
    test_gip_right_stick_never_presses_buttons();
    test_gip_guide_merge();
    test_gip_announce_status_and_noise();
    test_gip_reannounce_releases_held_input();
    test_gip_coalesced_messages();
    test_gip_varint_and_share();
    test_gip_share_dynamic_latency_input();
    test_gip_attachment_messages_ignored();
    test_gip_power_off_status_releases_input();

    test_gamesir_simple_report();
    test_gamesir_right_stick_regression();
    test_gamesir_full_report();
    test_gamesir_bt_simple_and_rejects();
    test_gamesir_g7pro_hid_vectors();

    test_x360_strict();
    test_x360_wireless_receiver();
    test_gamesir_identification();

    printf("All GameSir/GIP tests passed.\n");
    return 0;
}
