#ifndef UAC_UDP_TRANSPORT_H
#define UAC_UDP_TRANSPORT_H

#include <stdint.h>
#include <string.h>

/* Wire integers are little endian, matching the existing audio protocol.
 * Maximum UDP payload is 1016 bytes, below the IPv4 Ethernet MTU. */
#define UAC_UDP_MAGIC 0x31554441U /* ADU1 */
#define UAC_UDP_CHUNK 1000U
#define UAC_UDP_MAX_FRAME 1941U
#define UAC_UDP_REASSEMBLY_MS 30U
#define UAC_UDP_PEER_TIMEOUT_MS 3000U
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint32_t seq;
    uint16_t total;
    uint16_t offset;
    uint8_t client_id;
    uint8_t direction;
    uint16_t reserved;
} uac_udp_header_t;
#pragma pack(pop)
typedef char uac_udp_header_size[(sizeof(uac_udp_header_t) == 16) ? 1 : -1];

typedef struct {
    uint32_t seq;
    uint32_t started_ms;
    uint16_t total;
    uint8_t mask;
    uint8_t valid;
} uac_udp_assembly_t;

/* One bounded in-flight audio frame per source. Reordering of its two
 * fragments is supported; older frames and duplicates are discarded. */
static int uac_udp_assemble(uac_udp_assembly_t *a, uint8_t *dst,
                            const uac_udp_header_t *h, const uint8_t *payload,
                            uint32_t len, uint32_t now)
{
    uint32_t expected;
    uint8_t bit, complete;
    if (h->magic != UAC_UDP_MAGIC || h->reserved || h->total < 21U ||
        h->total > UAC_UDP_MAX_FRAME || h->offset >= h->total ||
        h->offset % UAC_UDP_CHUNK) return -1;
    expected = h->total - h->offset;
    if (expected > UAC_UDP_CHUNK) expected = UAC_UDP_CHUNK;
    if (len != expected) return -1;
    if (a->valid && h->seq != a->seq &&
        (int32_t)(h->seq - a->seq) <= 0) return -1;
    if (!a->valid || h->seq != a->seq) {
        a->seq = h->seq;
        a->started_ms = now;
        a->total = h->total;
        a->mask = 0;
        a->valid = 1;
    }
    if (a->total != h->total || now - a->started_ms > UAC_UDP_REASSEMBLY_MS)
        return -1;
    bit = (uint8_t)(1U << (h->offset / UAC_UDP_CHUNK));
    if (a->mask & bit) return -1;
    memcpy(dst + h->offset, payload, len);
    a->mask |= bit;
    complete = (h->total > UAC_UDP_CHUNK) ? 3U : 1U;
    return a->mask == complete ? (int)a->total : 0;
}
#endif
