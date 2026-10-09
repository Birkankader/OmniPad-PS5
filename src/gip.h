#ifndef GIP_H
#define GIP_H

#include "pad_types.h"

#include <stdint.h>
#include <stddef.h>

/* Xbox One / Series "Gaming Input Protocol" over USB (interface FF/47/D0).
 * Pure packet logic modelled on Linux xpad.c and [MS-GIPUSB] (no I/O, host-testable). */

#define GIP_CMD_ACK          0x01
#define GIP_CMD_ANNOUNCE     0x02
#define GIP_CMD_STATUS       0x03
#define GIP_CMD_IDENTIFY     0x04
#define GIP_CMD_POWER        0x05
#define GIP_CMD_AUTHENTICATE 0x06
#define GIP_CMD_VIRTUAL_KEY  0x07
#define GIP_CMD_RUMBLE       0x09
#define GIP_CMD_LED          0x0a
#define GIP_CMD_INPUT        0x20

#define GIP_OPT_ACK          0x10
#define GIP_OPT_INTERNAL     0x20
#define GIP_OPT_CHUNK_START  0x40
#define GIP_OPT_CHUNK        0x80
#define GIP_OPT_CLIENT_MASK  0x07

#define GIP_MAX_PACKET       64

/* Bits returned by gip_process() */
#define GIP_EVENT_STATE      0x01   /* *state was updated (input or guide change) */
#define GIP_EVENT_ANNOUNCE   0x02   /* device (re)announced itself: init must be (re)sent */
#define GIP_EVENT_ALIVE      0x04   /* well-formed traffic was consumed */
#define GIP_EVENT_POWER_OFF  0x08   /* status "powering off/resetting": held input was released */
#define GIP_EVENT_STATUS     0x10   /* a Status message (the pad's periodic heartbeat) arrived */

typedef struct {
    uint8_t  command;
    uint8_t  options;
    uint8_t  sequence;
    uint32_t length;       /* payload length */
    size_t   header_len;   /* 4 for single-byte lengths, more for varint lengths */
} gip_header_t;

typedef struct {
    uint16_t   vid, pid;
    uint8_t    out_seq;       /* next outgoing sequence number, never 0 (reserved by the spec) */
    int        init_step;     /* index into the init table, -1 when finished */
    int        guide_down;
    int        have_input;    /* a 0x20 input report was decoded */
    int        started;       /* input or status seen: the pad left the Arrival state */
    uint32_t   input_len;     /* payload length of the last input report (diagnostics) */
    int        dli;           /* Dynamic Latency Input follows the function map (Series X|S) */
    unsigned long n_status;   /* Status messages from the pad (heartbeat detection) */
    pad_state_t last;         /* last state decoded from a 0x20 packet (PS bit excluded) */
} gip_ctx_t;

/* Called once per message that requested an acknowledgement. */
typedef void (*gip_ack_fn)(void *user, const uint8_t *ack, size_t len);

void gip_init_ctx(gip_ctx_t *ctx, uint16_t vid, uint16_t pid);

/* Restart the init sequence (after an announce). */
void gip_restart_init(gip_ctx_t *ctx);

/* Forgets held input (sticks, buttons, guide): the pad is gone or starting over. */
void gip_reset_input(gip_ctx_t *ctx);

/* Writes the next init packet for this device into out and returns its length,
 * or 0 when the sequence is complete. */
size_t gip_next_init_packet(gip_ctx_t *ctx, uint8_t *out, size_t cap);

int gip_parse_header(const uint8_t *buf, size_t len, gip_header_t *hdr);

/* Builds the acknowledgement for a message that carries GIP_OPT_ACK.
 * Returns the ACK length (13) or 0 when no ACK is needed/possible. */
size_t gip_build_ack(const uint8_t *msg, size_t len, uint8_t *out, size_t cap);

/* Decodes one USB transfer, which may pack several GIP messages. Each message
 * asking for an ACK is passed to ack_fn (may be NULL). Returns GIP_EVENT_* bits;
 * with GIP_EVENT_STATE, *state holds the full pad state (last input merged with
 * the guide button). Returns 0 when the transfer holds no well-formed message.
 * Only the primary device (expansion index 0) drives the state: messages from
 * attachments (headset, chatpad) are acknowledged but otherwise ignored. */
int gip_process(gip_ctx_t *ctx, const uint8_t *buf, size_t len, pad_state_t *state,
                gip_ack_fn ack_fn, void *ack_user);

/* Stateless decode of one 0x20 input message into *st (guide not included).
 * dli: the report may end with 8 bytes of Dynamic Latency Input data (Series X|S
 * firmware 5.1+, 40+ byte payloads), which moves Share 8 bytes from the end. */
int gip_parse_input(const uint8_t *buf, size_t len, int dli, pad_state_t *st);

#endif /* GIP_H */
