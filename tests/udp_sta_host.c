/* Deterministic IO/time seams; protocol and codec functions come from source. */
static uint32_t now = 100;
static uint8_t incoming[1100], sent[1100], output_pcm[1920];
static int incoming_len = -1, sent_len, send_error, auto_ack, hello_count;
static uint32_t pcm_count;
static uint32_t tri_now_ms(void) { return now; }
static uint64_t tri_time_us(void) { return (uint64_t)now * 1000U; }
static int wlan_get_connect_status(void) { return 1; }
static void rtos_task_suspend(uint32_t ms) { now += ms; }
static int send(int fd, const void *buf, unsigned len, int flags)
{
    uint32_t magic;
    (void)fd; (void)flags;
    if (send_error) { errno = send_error; return -1; }
    assert(len <= sizeof(sent));
    memcpy(sent, buf, len); sent_len = len;
    memcpy(&magic, buf, 4);
    if (magic == APP_AUDIO_LINK_PACKET_MAGIC) {
        hello_count++;
        if (auto_ack && hello_count >= auto_ack) {
            memcpy(incoming, buf, len); incoming_len = len;
        }
    }
    return len;
}
static int recv(int fd, void *buf, unsigned len, int flags)
{
    int n = incoming_len;
    (void)fd; (void)flags;
    if (n < 0) { errno = EAGAIN; return -1; }
    if ((unsigned)n > len) n = len;
    memcpy(buf, incoming, n); incoming_len = -1;
    return n;
}
static void tri_rx_queue_push(const int16_t *pcm)
{
    memcpy(output_pcm, pcm, sizeof(output_pcm)); pcm_count++;
}
static void tri_play_rx_arrival_mark(const audio_header_t *h, uint32_t q)
{ (void)h; (void)q; }
static void tri_record_decimator_reset(void) {}
static uint32_t tri_tx_queue_trim_to_latest(uint8_t keep) { (void)keep; return 0; }
static int16_t queued[2][960];
static unsigned queued_count, queued_read;
static int tri_tx_queue_pop_10ms(int16_t *pcm)
{
    if (queued_read == queued_count) return -1;
    memcpy(pcm, queued[queued_read++], 1920);
    s_tx_count = queued_count - queued_read;
    return 0;
}
static void tri_tx_queue_push_front_10ms(const int16_t *pcm)
{ (void)pcm; queued_read--; s_tx_count++; }

/* PRODUCTION FUNCTIONS */

static void reset_session(void)
{
    memset(s_udp_session, 0, sizeof(s_udp_session));
    s_udp_session[0].token = 12345;
    s_udp_session[1].token = 67890;
    s_udp_session[0].acked = s_udp_session[1].acked = 1;
    s_udp_session[0].ack_ms = s_udp_session[1].ack_ms = now;
    s_udp_session[0].hello_ms = s_udp_session[1].hello_ms = now;
    memset(&s_rx_assembly, 0, sizeof(s_rx_assembly));
    incoming_len = -1; send_error = 0; auto_ack = 0; hello_count = 0;
}
static void fragment(const uint8_t *frame, uint32_t seq, uint16_t total,
                     uint16_t offset, uint8_t id, uint8_t direction)
{
    uac_udp_header_t h = {UAC_UDP_MAGIC, seq, total, offset, id, direction, 0};
    unsigned bytes = total - offset;
    if (bytes > UAC_UDP_CHUNK) bytes = UAC_UDP_CHUNK;
    memcpy(incoming, &h, sizeof(h));
    memcpy(incoming + sizeof(h), frame + offset, bytes);
    incoming_len = sizeof(h) + bytes;
}
static void test_registration(void)
{
    audio_header_t h;
    uint8_t hello[25];
    reset_session();
    assert(tri_udp_send_hello(1, 1) == 0 && sent_len == 25);
    memcpy(hello, sent, sizeof(hello));
    memcpy(&h, sent, sizeof(h));
    assert(h.timestamp == 12345 && h.seq_num == 0 && h.direction == 1);
    now += 1000;
    assert(tri_udp_send_hello(1, 1) == 0);
    assert(!memcmp(hello, sent, 25)); /* fixed token across keepalives */
    assert(tri_udp_accept_ack(sent, 25, 1) == 1);
    assert(tri_udp_accept_ack(sent, 25, 2) == 0);
    sent[8] ^= 1; /* mismatched session token */
    assert(tri_udp_accept_ack(sent, 25, 1) == 0);
    reset_session();
    s_udp_session[1].acked = 0;
    auto_ack = 3; /* first two HELLOs/ACKs lost */
    assert(tri_send_hello(2, 2) == 0);
    assert(hello_count == 3 && s_udp_session[1].acked);
    auto_ack = 0;
    now += 3000;
    memcpy(incoming, sent, 25); incoming_len = 25; /* queued but late ACK */
    assert(tri_service_udp_rx(2, 2) == -1);
    reset_session();
    s_udp_session[1].acked = 0;
    assert(tri_send_hello(2, 2) == -1); /* never ready without ACK */
}
static void test_playback(void)
{
    uint8_t frame[1941];
    audio_header_t h = {APP_AUDIO_LINK_PACKET_MAGIC, 10, 1000,
        APP_AUDIO_LINK_PACKET_TYPE_UAC_PCM, 1, TRIANGLE_DEVICE_ID, 1920};
    audio_ctrl_payload_t ctrl = {APP_AUDIO_LINK_CTRL_HEARTBEAT, 1, 0};
    unsigned i;
    reset_session();
    memcpy(frame, &h, sizeof(h));
    for (i = 21; i < sizeof(frame); i++) frame[i] = i;
    fragment(frame, 10, sizeof(frame), 1000, TRIANGLE_DEVICE_ID, 1);
    assert(tri_service_udp_rx(1, 1) == 0 && pcm_count == 0);
    fragment(frame, 10, sizeof(frame), 0, TRIANGLE_DEVICE_ID, 1);
    assert(tri_service_udp_rx(1, 1) == 0 && pcm_count == 1);
    assert(!memcmp(output_pcm, frame + 21, 1920));
    fragment(frame, 10, sizeof(frame), 0, TRIANGLE_DEVICE_ID, 1);
    tri_service_udp_rx(1, 1);
    assert(pcm_count == 1); /* duplicate */
    fragment(frame, 11, sizeof(frame), 0, TRIANGLE_DEVICE_ID, 1);
    tri_service_udp_rx(1, 1);
    now += 31;
    fragment(frame, 11, sizeof(frame), 1000, TRIANGLE_DEVICE_ID, 1);
    tri_service_udp_rx(1, 1);
    assert(pcm_count == 1); /* incomplete frame expired */
    fragment(frame, 12, sizeof(frame), 0, 5, 1);
    tri_service_udp_rx(1, 1);
    assert(s_rx_assembly.seq == 11); /* wrong device */
    fragment(frame, 12, sizeof(frame), 0, TRIANGLE_DEVICE_ID, 2);
    tri_service_udp_rx(1, 1);
    assert(s_rx_assembly.seq == 11); /* wrong direction */
    fragment(frame, 12, sizeof(frame), 0, TRIANGLE_DEVICE_ID, 1);
    incoming_len++; /* malformed datagram length */
    tri_service_udp_rx(1, 1);
    assert(s_rx_assembly.seq == 11);
    h.packet_type = APP_AUDIO_LINK_PACKET_TYPE_CTRL;
    h.data_len = sizeof(ctrl);
    memcpy(incoming, &h, sizeof(h));
    memcpy(incoming + sizeof(h), &ctrl, sizeof(ctrl));
    incoming_len = 25;
    tri_service_udp_rx(1, 1);
    assert(s_mic_streaming == 1); /* heartbeat repairs lost MIC state */
    ctrl.value = 0;
    memcpy(incoming + sizeof(h), &ctrl, sizeof(ctrl));
    incoming_len = 25;
    tri_service_udp_rx(1, 1);
    assert(!s_mic_streaming && s_record_reset_pending);
    incoming_len = 0; /* UDP zero-length packet is not EOF */
    assert(tri_service_udp_rx(1, 1) == 0);
}
static void test_record(void)
{
    uint8_t reassembled[1941], original[1941];
    uac_udp_assembly_t assembly = {0};
    uac_udp_header_t uh;
    audio_header_t h;
    uint32_t seed = 1234, i;
    reset_session();
    s_tx_seq = 1; s_tx_len = s_tx_off = s_tx_packet_loaded = 0;
    queued_read = 0; queued_count = 1; s_tx_count = 1;
    for (i = 0; i < 960; i++) {
        seed = seed * 1664525U + 1013904223U;
        queued[0][i] = (int16_t)(seed >> 16);
    }
    tri_prepare_record_frame(now);
    memcpy(&h, s_tx_wire, sizeof(h));
    assert(h.packet_type == APP_AUDIO_LINK_PACKET_TYPE_UAC_PCM && h.data_len == 1920);
    memcpy(original, s_tx_wire, s_tx_len);
    assert(tri_service_record_tx(2, now) == 0 && sent_len == 1016);
    memcpy(&uh, sent, sizeof(uh));
    assert(uh.direction == 2 && uh.client_id == TRIANGLE_DEVICE_ID && uh.offset == 0);
    assert(uac_udp_assemble(&assembly, reassembled, &uh, sent + 16, sent_len - 16, now) == 0);
    assert(tri_service_record_tx(2, now) == 0 && sent_len == 957);
    memcpy(&uh, sent, sizeof(uh));
    assert(uh.offset == 1000);
    assert(uac_udp_assemble(&assembly, reassembled, &uh, sent + 16, sent_len - 16, now) == 1941);
    assert(!memcmp(original, reassembled, 1941) && s_tx_len == 0);
    /* A congested second fragment is discarded after 20 ms, not reconnected. */
    queued_read = 0; s_tx_count = 1;
    tri_prepare_record_frame(now);
    assert(tri_service_record_tx(2, now) == 0 && s_tx_off == 1000);
    send_error = EAGAIN;
    assert(tri_service_record_tx(2, now) == 0);
    now += 20;
    assert(tri_service_record_tx(2, now) == 0 && s_tx_len == 0);
    assert(s_record_stale_drop == 1);
    /* Highly compressible REC20 remains within the AP's 1941-byte limit. */
    send_error = 0; queued_count = s_tx_count = 2; queued_read = 0;
    memset(queued, 0, sizeof(queued));
    tri_prepare_record_frame(now);
    memcpy(&h, s_tx_wire, sizeof(h));
    assert(h.packet_type == TRI_PACKET_TYPE_UAC_PCM_LOSSLESS20);
    assert(s_tx_len <= UAC_UDP_MAX_FRAME && h.data_len <= 1920);
    {
        tri_rec20_header_t group;
        int16_t pcm[960];
        uint8_t *payload = s_tx_wire + sizeof(h);
        memcpy(&group, payload, sizeof(group));
        assert(tri_lossless_decode_payload(payload + sizeof(group), group.len0, 1920, 480, pcm) == 0);
        assert(!memcmp(pcm, queued[0], 1920));
        assert(tri_lossless_decode_payload(payload + sizeof(group) + group.len0, group.len1, 1920, 480, pcm) == 0);
        assert(!memcmp(pcm, queued[1], 1920));
    }
}
int main(void)
{
    test_registration();
    test_playback();
    test_record();
    printf("PASS: STA%u UDP registration/retry/timeout, RX validation/reassembly, control, TX fragmentation/stale drop and REC20 codec\n", TRIANGLE_DEVICE_ID);
    return 0;
}
