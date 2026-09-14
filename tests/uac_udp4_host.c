/* Included after the production codec/mixer definitions by run_uac_udp4.py. */
#include <assert.h>
#include <stdio.h>

static void test_transport(void)
{
    uint8_t raw[UAC_UDP_MAX_FRAME], guarded[UAC_UDP_MAX_FRAME + 2];
    uac_udp_assembly_t a = {0};
    uac_udp_header_t h = {UAC_UDP_MAGIC, UINT32_MAX, UAC_UDP_MAX_FRAME, 1000, 4, 2, 0};
    unsigned i;
    for (i = 0; i < sizeof(raw); ++i) raw[i] = (uint8_t)i;
    memset(guarded, 0xA7, sizeof(guarded));
    assert(uac_udp_assemble(&a, guarded + 1, &h, raw + 1000, 941, 100) == 0);
    assert(uac_udp_assemble(&a, guarded + 1, &h, raw + 1000, 941, 101) < 0);
    h.offset = 0;
    assert(uac_udp_assemble(&a, guarded + 1, &h, raw, 1000, 102) == 1941);
    assert(!memcmp(raw, guarded + 1, sizeof(raw)));
    assert(guarded[0] == 0xA7 && guarded[sizeof(guarded)-1] == 0xA7);
    assert(uac_udp_assemble(&a, guarded + 1, &h, raw, 1000, 103) < 0);
    h.seq = 0; /* serial number wrap */
    assert(uac_udp_assemble(&a, guarded + 1, &h, raw, 1000, 110) == 0);
    h.offset = 1000;
    assert(uac_udp_assemble(&a, guarded + 1, &h, raw+1000, 941, 141) < 0);
    h.seq = 1;
    assert(uac_udp_assemble(&a, guarded + 1, &h, raw+1000, 941, 142) == 0);
    h.seq = 0;
    assert(uac_udp_assemble(&a, guarded + 1, &h, raw+1000, 941, 143) < 0);
    h.seq = 2; h.offset = 1;
    assert(uac_udp_assemble(&a, guarded + 1, &h, raw, 1000, 144) < 0);
    h.offset = 0; h.total = 1942;
    assert(uac_udp_assemble(&a, guarded + 1, &h, raw, 1000, 144) < 0);
    h.total = 1941;
    assert(uac_udp_assemble(&a, guarded + 1, &h, raw, 999, 144) < 0);
    h.total = 100; h.seq = 3;
    assert(uac_udp_assemble(&a, guarded + 1, &h, raw, 100, 145) == 100);
    h.reserved = 1;
    assert(uac_udp_assemble(&a, guarded + 1, &h, raw, 100, 145) < 0);
}

static void test_codec(void)
{
    int16_t pcm[960], decoded[960];
    uint8_t wire[1920];
    uint16_t len;
    unsigned i;
    for (i = 0; i < 960; ++i) pcm[i] = (int16_t)((i / 2) * (i % 2 ? -4 : 3));
    assert(!uacm_lossless_encode_payload(pcm, 480, sizeof(pcm), wire, sizeof(wire), &len));
    assert(len < 1920);
    assert(!uacm_lossless_decode_payload(wire, len, 1920, 480, decoded));
    assert(!memcmp(pcm, decoded, sizeof(pcm)));
    wire[8] ^= 1; /* corrupt CRC, not frame geometry */
    assert(uacm_lossless_decode_payload(wire, len, 1920, 480, decoded) == -2);
    wire[8] ^= 1;
    assert(uacm_lossless_decode_payload(wire, len-1, 1920, 480, decoded) != 0);
    for (i = 0; i < 960; ++i) pcm[i] = (i & 2) ? INT16_MAX : INT16_MIN;
    /* Incompressible PCM must signal raw fallback, never overflow the buffer. */
    assert(uacm_lossless_encode_payload(pcm, 480, 1920, wire, sizeof(wire), &len) != 0);
}

static void test_sequence(void)
{
    uacm_session_t a = {0};
    assert(uacm_track_record_seq(&a, UINT32_MAX-1) == 0);
    assert(uacm_track_record_seq(&a, UINT32_MAX) == 0);
    assert(uacm_track_record_seq(&a, 0) == 0);
    assert(uacm_track_record_seq(&a, 0) == UINT32_MAX);
    assert(uacm_track_record_seq(&a, 3) == 2);
    assert(uacm_track_record_seq(&a, 2) == UINT32_MAX);
    assert(a.diag_recv == 4 && a.seq_gap == 2 && a.seq_old == 2);
}

static void test_mixer(void)
{
    static uacm_session_t sessions[UACM_SESSION_COUNT];
    int16_t pcm[UACM_SESSION_COUNT][UACM_SAMPLES_5MS], out[UACM_SAMPLES_5MS];
    uint8_t valid[UACM_SESSION_COUNT];
    unsigned i, j, k, selected;
    s_session = sessions;
    /* Every channel, including 3 and 4, can be the only source. */
    for (selected = 0; selected < 4; ++selected) {
        memset(sessions, 0, sizeof(sessions)); memset(valid, 0, sizeof(valid));
        valid[selected] = 1;
        for (i = 0; i < 4; ++i) for (j = 0; j < UACM_SAMPLES_5MS; ++j)
            pcm[i][j] = (j & 1) ? -1234 : 2345;
        for (k = 0; k < 100; ++k) uacm_mix_pcm(pcm, valid, out);
        assert(out[0] == 2345 && out[1] == -1234);
    }
    /* Four full-scale, coherent inputs must stay bounded through gain changes. */
    memset(sessions, 0, sizeof(sessions)); memset(valid, 1, sizeof(valid));
    for (i = 0; i < 4; ++i) for (j = 0; j < UACM_SAMPLES_5MS; ++j)
        pcm[i][j] = (j & 1) ? INT16_MIN : INT16_MAX;
    for (k = 0; k < 300; ++k) {
        uint32_t sum = 0;
        if (k == 150) valid[0] = valid[1] = valid[2] = 0;
        uacm_mix_pcm(pcm, valid, out);
        for (i = 0; i < 4; ++i) sum += sessions[i].gain_q15;
        assert(sum <= 32768 && out[0] > 0 && out[1] < 0);
    }
    /* Quiet valid sources retain the all-inactive fallback. */
    memset(sessions, 0, sizeof(sessions)); memset(valid, 1, sizeof(valid));
    for (i = 0; i < 4; ++i) for (j = 0; j < UACM_SAMPLES_5MS; ++j) pcm[i][j] = 100;
    for (k = 0; k < 100; ++k) uacm_mix_pcm(pcm, valid, out);
    assert(out[0] == 100);
    memset(valid, 0, sizeof(valid));
    uacm_mix_pcm(pcm, valid, out);
    for (j = 0; j < UACM_SAMPLES_5MS; ++j) assert(out[j] == 0);
}

static void hello(unsigned id, unsigned direction, uint64_t token)
{
    audio_header_t h = {AUDIO_PACKET_MAGIC, 0, token, AUDIO_PACKET_TYPE_CTRL, direction, id, 4};
    audio_ctrl_payload_t ctrl = {AUDIO_CTRL_SESSION_HELLO, id, 0};
    memcpy(mock_wire, &h, sizeof(h)); memcpy(mock_wire + sizeof(h), &ctrl, sizeof(ctrl));
    mock_len = sizeof(h) + sizeof(ctrl); mock_pending = 1;
    assert(!uacm_poll_udp(7, direction));
}

static void test_sessions(void)
{
    static uacm_session_t a[4];
    unsigned i;
    s_session = a; mock_now = 100;
    memset(a, 0, sizeof(a));
    for (i = 0; i < 4; ++i) {
        a[i].client_id = i+1; a[i].record_fd = a[i].playback_fd = -1;
    }
    mock_peer.sin_family = AF_INET; mock_peer.sin_port = 4000;
    for (i = 1; i <= 4; ++i) {
        mock_peer.sin_addr.s_addr = i;
        hello(i, 2, 123); hello(i, 1, 123);
        assert(a[i-1].record_fd == 7 && a[i-1].playback_fd == 7);
    }
    assert(mock_acks == 8);
    hello(5, 2, 123); assert(mock_acks == 8); /* fifth client rejected */
    a[3].rx_ring.write_pos = 100;
    hello(4, 2, 123); /* duplicate HELLO does not flush jitter or seq state */
    assert(a[3].rx_ring.write_pos == 100 && mock_acks == 9);
    hello(4, 2, 456); assert(mock_acks == 9); /* live token pinned */
    mock_peer.sin_port++;
    hello(4, 2, 123); assert(mock_acks == 9); /* live endpoint pinned */
    mock_peer.sin_port--;
    {
        uint8_t packet[sizeof(audio_header_t) + 100];
        audio_header_t h = {AUDIO_PACKET_MAGIC, 1, 0, 4, 2, 4, 100};
        uac_udp_header_t uh = {UAC_UDP_MAGIC, 1, sizeof(packet), 0, 4, 2, 0};
        memset(packet, 0, sizeof(packet)); memcpy(packet, &h, sizeof(h));
        memcpy(mock_wire, &uh, sizeof(uh)); memcpy(mock_wire + sizeof(uh), packet, sizeof(packet));
        mock_len = sizeof(uh) + sizeof(packet); mock_pending = 1;
        assert(!uacm_poll_udp(7, 2)); assert(mock_audio == 1);
        mock_pending = 1; assert(!uacm_poll_udp(7, 2)); assert(mock_audio == 1);
        assert(a[3].diag_rx_pkt == 2); /* duplicate is still a received datagram */
        uh.seq = h.seq_num = 2;
        memcpy(mock_wire, &uh, sizeof(uh)); memcpy(mock_wire + sizeof(uh), &h, sizeof(h));
        mock_peer.sin_port++; mock_pending = 1;
        assert(!uacm_poll_udp(7, 2)); assert(mock_audio == 1); /* wrong endpoint */
        mock_peer.sin_port--;
    }
    {
        /* A 9-byte final audio fragment has the same datagram size as HELLO. */
        uint8_t packet[1009] = {0};
        audio_header_t h = {AUDIO_PACKET_MAGIC, 3, 0, 4, 2, 4, 988};
        uac_udp_header_t uh = {UAC_UDP_MAGIC, 3, sizeof(packet), 0, 4, 2, 0};
        memcpy(packet, &h, sizeof(h));
        memcpy(mock_wire, &uh, sizeof(uh)); memcpy(mock_wire + sizeof(uh), packet, 1000);
        mock_len = sizeof(uh) + 1000; mock_pending = 1;
        assert(!uacm_poll_udp(7, 2)); assert(mock_audio == 1);
        uh.offset = 1000;
        memcpy(mock_wire, &uh, sizeof(uh)); memcpy(mock_wire + sizeof(uh), packet + 1000, 9);
        mock_len = sizeof(uh) + 9; mock_pending = 1;
        assert(!uacm_poll_udp(7, 2)); assert(mock_audio == 2);
    }
    mock_now += UAC_UDP_PEER_TIMEOUT_MS;
    assert(!uacm_poll_udp(7, 2));
    for (i = 0; i < 4; ++i) assert(a[i].record_fd < 0 && a[i].playback_fd == 7);
    hello(4, 2, 456); assert(a[3].record_fd == 7 && a[3].rx_ring.write_pos == 0);
    assert(!uacm_poll_udp(7, 1));
    for (i = 0; i < 4; ++i) assert(a[i].playback_fd < 0);
}

static void test_diag_timeout(void)
{
    uacm_session_t a[UACM_SESSION_COUNT] = {0};
    unsigned i;
    s_session = a;
    for (i = 0; i < UACM_SESSION_COUNT; ++i)
        a[i].record_fd = a[i].playback_fd = -1;
    a[0].assembly.valid = 1;
    a[0].assembly.total = 1941;
    a[0].assembly.mask = 1;
    a[0].assembly.started_ms = 100;
    mock_now = 131; mock_pending = 0;
    assert(!uacm_poll_udp(7, 2));
    assert(a[0].diag_asm_timeout == 1);
    mock_now++;
    assert(!uacm_poll_udp(7, 2));
    assert(a[0].diag_asm_timeout == 1);
    a[0].assembly.mask = 3;
    a[0].diag_asm_expired = 0;
    assert(!uacm_poll_udp(7, 2));
    assert(a[0].diag_asm_timeout == 1); /* completed assembly never times out */
}

static void test_send(void)
{
    uacm_session_t a = {0};
    audio_header_t h = {AUDIO_PACKET_MAGIC, 99, 0, 3, 1, 4, 1920};
    uac_udp_header_t uh;
    uac_udp_assembly_t assembly = {0};
    uint8_t rebuilt[1941];
    a.client_id = 4; a.playback_fd = 7; a.tx_kind = UACM_TX_KIND_PCM;
    a.tx_len = 1941; a.tx_payload_len = 1920; a.tx_progress_ms = 99;
    memcpy(a.tx_wire, &h, sizeof(h));
    mock_now = 100; mock_send_errno = EAGAIN;
    assert(!uacm_service_tx(&a, 0)); /* stale caller timestamp cannot underflow */
    assert(a.tx_len == 1941 && a.tx_off == 0);
    mock_now = 119;
    assert(!uacm_service_tx(&a, 118));
    assert(!a.tx_len && a.playback_drop == 1 && a.playback_fd == 7);
    assert(a.diag_drop_to == 1 && a.diag_tx_busy == 2);
    mock_send_errno = 0; a.tx_kind = UACM_TX_KIND_PCM;
    a.tx_len = 1941; a.tx_progress_ms = mock_now;
    assert(!uacm_service_tx(&a, 119));
    assert(a.tx_off == 1000 && mock_sent_len == 1016);
    memcpy(&uh, mock_sent, sizeof(uh));
    assert(uh.seq == 99 && uh.client_id == 4 && uh.direction == 1 && !uh.offset);
    assert(uac_udp_assemble(&assembly, rebuilt, &uh, mock_sent+sizeof(uh), 1000, 119) == 0);
    mock_now++;
    assert(!uacm_service_tx(&a, 119));
    assert(!a.tx_len && !a.tx_off && a.playback_packets == 1 && mock_sent_len == 957);
    assert(a.diag_tx_pkt == 2);
    memcpy(&uh, mock_sent, sizeof(uh));
    assert(uh.offset == 1000);
    assert(uac_udp_assemble(&assembly, rebuilt, &uh, mock_sent+sizeof(uh), 941, 120) == 1941);
    assert(!memcmp(rebuilt, a.tx_wire, sizeof(rebuilt)));
    a.tx_ring.write_pos = 1920;
    a.tx_len = 1941;
    a.tx_kind = UACM_TX_KIND_PCM;
    uacm_close_playback(&a);
    assert(a.diag_drop_err == 2);
    uacm_close_playback(&a);
    assert(a.diag_drop_err == 2); /* cleanup cannot count the same frame twice */
}

int main(void)
{
    char rate[16];
    uacm_diag_rate(rate, 4, 1000);
    assert(!strcmp(rate, "0.40%"));
    uacm_diag_rate(rate, 0, 0);
    assert(!strcmp(rate, "N/A"));
    uacm_diag_rate(rate, 1000, 1000);
    assert(!strcmp(rate, "100.00%"));
    test_transport(); test_codec(); test_sequence(); test_mixer(); test_sessions();
    test_diag_timeout(); test_send();
    {
        uacm_session_t sessions[UACM_SESSION_COUNT] = {0};
        s_session = sessions;
        sessions[0].client_id = 1;
        sessions[0].playback_packets = 996;
        sessions[0].diag_recv = 498;
        uacm_log_audio_diag();
        assert(strstr(mock_log, "PLAY sent=996/1000"));
        assert(strstr(mock_log, "REC recv=498/500"));
        assert(strstr(mock_log, "local_loss=0.40%"));
        assert(strstr(mock_log, "gap_rate=0.40%"));
        assert(strstr(mock_log, "\n==========\nAP T"));
        mock_log[0] = 0;
        sessions[0].playback_packets += 900;
        sessions[0].diag_recv += 600;
        uacm_log_audio_diag();
        assert(strstr(mock_log, "PLAY sent=900/1000"));
        assert(strstr(mock_log, "local_loss=10.00%"));
        assert(strstr(mock_log, "REC recv=600/500"));
        assert(strstr(mock_log, "gap_rate=0.00%"));
    }
    puts("PASS: UDP fragments/reorder/duplicates/timeout/bounds/wrap; codec/CRC; four-source mix; registration/pinning/expiry/reconnect; TX fragments/stale drop");
    return 0;
}
