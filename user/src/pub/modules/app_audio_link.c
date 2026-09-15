/*
 * Triangle microphone STA transport for the four-STA UDP Receiver AP.
 * Build with a unique TRIANGLE_DEVICE_ID in 1..4.
 *
 * UDP/8888: periodic HELLO/ACK registration, then ADU1 playback fragments.
 * UDP/8890: independent HELLO/ACK registration and ADU1 recording fragments.
 * Native 48 kHz stereo BPK2/REC20 and raw fallback match the AP codec.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>

#include "dbg.h"
#include "rtos.h"
#include "rtos_al.h"
#include "plf.h"
#include "fhost.h"
#include "rwnx_msg_tx.h"
#include "wlan_user.h"
#include "wlan_if.h"
#include "sleep_api.h"
#include "us_ticker_api.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "uac_udp_transport.h"

#if PLF_WIFI_STACK
#include "ipc_host.h"
#endif

#include "app_audio_link.h"

#define TRI_VERSION                         "v8.0-UDP4-BPK2-REC48-REC20-BUF20"
#define TRI_TASK_STACK                      4096U
#define TRI_RECORD_TASK_STACK               4096U
#define TRI_TASK_PRIO                       2U
#define TRI_RECORD_TASK_PRIO                3U
#define TRI_CONNECT_TIMEOUT_MS              5000U
#define TRI_RETRY_MS                        1000U
#define TRI_RECORD_RETRY_MS                 200U
#define TRI_LOOP_MS                         1U
#define TRI_SOCKET_BUF_BYTES                (4U * 1024U)
#define TRI_RECORD_SOCKET_BUF_BYTES         (4U * 1024U)
#define TRI_TX_QUEUE_BLOCKS                 16U
#define TRI_TX_HIGH_WATER_BLOCKS            6U  /* 60 ms, mature-project high water */
#define TRI_TX_KEEP_BLOCKS                  4U  /* keep newest 40 ms */
#define TRI_PLAYBACK_PACKET_MS              10U
#define TRI_PLAYBACK_WIRE_BYTES             APP_AUDIO_LINK_UAC_BYTES_PER_PKT
#define TRI_PLAYBACK_FRAMES_PER_CHANNEL      \
    (TRI_PLAYBACK_WIRE_BYTES / (APP_AUDIO_LINK_UAC_CHANNELS * sizeof(int16_t)))
#define TRI_RX_QUEUE_PACKETS               12U  /* BUF20: 120 ms total playback jitter ring */
#define TRI_RX_PREBUFFER_PACKETS             9U  /* BUF20: 90 ms startup/rebuffer target */
#define TRI_RX_TIMELOCK_LOW_PACKETS          8U  /* BUF20: normal reserve floor, 80 ms */
#define TRI_RX_TIMELOCK_HIGH_PACKETS         9U  /* BUF20: normal reserve ceiling, 90 ms */
#define TRI_RX_TIMELOCK_ADJUST_FRAMES        4U  /* +/-4 of 480 frames = +/-0.83% bounded correction */
#define TRI_RX_TIMELOCK_LOW_BYTES            (TRI_RX_TIMELOCK_LOW_PACKETS * TRI_PLAYBACK_WIRE_BYTES)
#define TRI_RX_TIMELOCK_HIGH_BYTES           (TRI_RX_TIMELOCK_HIGH_PACKETS * TRI_PLAYBACK_WIRE_BYTES)
#define TRI_PLAYBACK_FADE_FRAMES            144U /* PLAYFIX1: 3 ms at 48 kHz; underflow/rebuffer boundary only */
#define TRI_PLAYBACK_RX_BUDGET              8U
#define TRI_RX_STREAM_BYTES                 UAC_UDP_MAX_FRAME
#define TRI_HEARTBEAT_MS                    1000U
#define TRI_UNSENT_STALE_DROP_MS             20U
#define TRI_PARTIAL_STALL_MS                500U
#define TRI_TX_STALE_KEEP_BLOCKS             TRI_TX_KEEP_BLOCKS
#define TRI_RECORD_FRAME_PACE_MS            20U
#define TRI_RECORD_GROUP_SUBFRAMES           2U
#define TRI_RECORD_GROUP_RAW_BYTES           (APP_AUDIO_LINK_RECORD_WIRE_BYTES * TRI_RECORD_GROUP_SUBFRAMES)
#define TRI_AUDIO_LOG_MS                    5000U
#define TRI_PLAY_EXPECTED_PPS               100U
#define TRI_RECORD_EXPECTED_PPS             50U
#define TRI_PLAY_STREAM_IDLE_MS             1500U
/* REC48CHK1: direct 48 kHz record wire; no 48->16 kHz FIR decimation. */
#define TRI_RECORD_WIRE_SAMPLE_RATE_HZ       48000U
#if (TRI_RECORD_WIRE_SAMPLE_RATE_HZ != 16000U) && (TRI_RECORD_WIRE_SAMPLE_RATE_HZ != 48000U)
#error "Triangle record wire rate must be 16000 or 48000 Hz"
#endif
#define TRI_RECORD_DECIM_TAPS               21U
#define TRI_RECORD_DECIM_HISTORY_FRAMES     (TRI_RECORD_DECIM_TAPS - 1U)

/* LL3-BPK2-48PREP1-CPUDIAG1: bounded-time bidirectional lossless transport.  Each channel is
 * split into 64-frame blocks; a block uses DELTA1 + fixed-width packing when
 * smaller, otherwise only that local block stays RAW.  There is no Rice unary
 * loop and no long CPU-triggered RAW guard. */
#define TRI_PACKET_TYPE_UAC_PCM_LOSSLESS     0x04U
#define TRI_PACKET_TYPE_UAC_PCM_LOSSLESS20   0x05U
#define TRI_REC20_MAGIC                      0x5232U
#define TRI_REC20_VERSION                    1U
#define TRI_LOSSLESS_MAGIC                   0x4C52U
#define TRI_LOSSLESS_VERSION                 4U
#define TRI_LOSSLESS_BLOCK_FRAMES            64U
#define TRI_LOSSLESS_MAX_WIDTH               17U
#define TRI_LOSSLESS_DESC_RAW                 0x80U
#define TRI_LOSSLESS_DESC_WIDTH_MASK          0x1FU
#define TRI_LOSSLESS_SELFTEST_START_FRAMES   0U
#define TRI_LOSSLESS_SELFTEST_EVERY          0U /* REC20PPS1: disable local decode+memcmp in realtime path; AP CRC remains per subframe */
#define TRI_LOSSLESS_LOG_MS                 30000U /* long lossless logs deferred until streams stop */
#define TRI_LOSSLESS_RECORD_WARN_US           1000U
/* PBUF3: duplicate codec timing probes are disabled.  The normal encode/decode
 * pass, per-frame CRC verification and sparse local self-test remain enabled. */
#define TRI_CODEC_PROBE_EVERY                    0U
#define TRI_LLCHK_LOG_MS                    61000U /* short playback integrity/buffer line */
#define TRI_PLAYBACK_BYTES_PER_MS           (TRI_PLAYBACK_WIRE_BYTES / TRI_PLAYBACK_PACKET_MS)
#define TRI_RXHOLE_WARN_US                  25000U /* diagnostic threshold only; no behavior change */

#pragma pack(push, 1)
typedef struct {
    uint32_t magic;
    uint32_t seq_num;
    uint64_t timestamp;
    uint8_t  packet_type;
    uint8_t  direction;
    uint8_t  client_id;
    uint16_t data_len;
} audio_header_t;

typedef struct {
    uint8_t  ctrl_type;
    uint8_t  value;
    uint16_t reserved;
} audio_ctrl_payload_t;

/* Compressed payload header shared by both directions.  v4 uses fixed 64-frame
 * channel blocks with local RAW/DELTA1-bitpack selection.  raw_len identifies
 * the decoded PCM size and raw_crc32 covers the original interleaved PCM, so
 * the receiving endpoint verifies every compressed frame after decompression. */
typedef struct {
    uint16_t magic;
    uint8_t  version;
    uint8_t  block_frames;
    uint16_t raw_len;
    uint8_t  block_count;
    uint8_t  reserved;
    uint32_t raw_crc32;
} tri_lossless_header_t;

/* REC20PPS1 container.  Each subframe is a complete 10 ms BPK2 v4 payload
 * with its own source-PCM CRC32.  The container is only used when both
 * compressed subframes together fit in the existing 1920-byte payload
 * budget; otherwise the source block(s) safely fall back to the legacy
 * 10 ms packet path. */
typedef struct {
    uint16_t magic;
    uint8_t  version;
    uint8_t  subframes;
    uint16_t len0;
    uint16_t len1;
} tri_rec20_header_t;
#pragma pack(pop)

typedef char tri_header_size_check[(sizeof(audio_header_t) == 21U) ? 1 : -1];
typedef char tri_playback_10ms_size_check[(TRI_PLAYBACK_WIRE_BYTES == 1920U) ? 1 : -1];
typedef char tri_playback_frame_count_check[(TRI_PLAYBACK_FRAMES_PER_CHANNEL == 480U) ? 1 : -1];
typedef char tri_lossless_header_size_check[(sizeof(tri_lossless_header_t) == 12U) ? 1 : -1];
typedef char tri_rec20_header_size_check[(sizeof(tri_rec20_header_t) == 8U) ? 1 : -1];
#if TRI_RECORD_WIRE_SAMPLE_RATE_HZ == 16000U
typedef char tri_record_rate_frame_check[(APP_AUDIO_LINK_RECORD_WIRE_FRAMES == 160U) ? 1 : -1];
typedef char tri_record_rate_byte_check[(APP_AUDIO_LINK_RECORD_WIRE_BYTES == 640U) ? 1 : -1];
#else
typedef char tri_record_rate_frame_check[(APP_AUDIO_LINK_RECORD_WIRE_FRAMES == 480U) ? 1 : -1];
typedef char tri_record_rate_byte_check[(APP_AUDIO_LINK_RECORD_WIRE_BYTES == 1920U) ? 1 : -1];
#endif
typedef char tri_record_source_frame_check[(APP_AUDIO_LINK_RECORD_SOURCE_FRAMES == 480U) ? 1 : -1];
typedef char tri_record_source_sample_check[(APP_AUDIO_LINK_RECORD_SOURCE_SAMPLES == 960U) ? 1 : -1];
typedef char tri_udp_frame_size_check[
    (sizeof(audio_header_t) + APP_AUDIO_LINK_RECORD_WIRE_BYTES <= UAC_UDP_MAX_FRAME) ? 1 : -1];

typedef struct {
    int16_t pcm[APP_AUDIO_LINK_UAC_SAMPLES_PER_PKT];
    uint8_t used;
} tri_tx_block_t;

static volatile uint8_t s_inited;
static volatile uint8_t s_running;
static volatile uint8_t s_stop;
static volatile uint8_t s_wifi_connected;
static volatile uint8_t s_play_connected;
static volatile uint8_t s_record_connected;
static volatile uint8_t s_mic_streaming;
static volatile uint8_t s_record_reset_pending;
static volatile uint8_t s_tx_agg_update_pending = 1U;
static uint8_t s_tx_agg_disabled;
static rtos_task_handle s_task;
static rtos_task_handle s_record_task;
static volatile uint8_t s_record_running;
static rtos_mutex s_tx_mutex;
static rtos_mutex s_rx_mutex;

static tri_tx_block_t s_tx_queue[TRI_TX_QUEUE_BLOCKS];
static uint8_t s_tx_rd;
static uint8_t s_tx_wr;
static uint8_t s_tx_count;
static uint8_t s_rx_pcm_ring[TRI_PLAYBACK_WIRE_BYTES * TRI_RX_QUEUE_PACKETS]
    __attribute__((aligned(4)));
static int16_t s_play_lossless_decode[APP_AUDIO_LINK_UAC_SAMPLES_PER_PKT]
    __attribute__((aligned(4)));
static uint32_t s_rx_pcm_rd;
static uint32_t s_rx_pcm_wr;
static uint32_t s_rx_pcm_count;
static uint8_t s_rx_started;

static uint8_t s_rx_stream[TRI_RX_STREAM_BYTES] __attribute__((aligned(4)));
static uac_udp_assembly_t s_rx_assembly;
typedef struct {
    uint64_t token;
    uint32_t hello_ms;
    uint32_t ack_ms;
    uint8_t acked;
} tri_udp_session_t;
/* Each direction and its socket are owned by exactly one task. */
static tri_udp_session_t s_udp_session[2];
static uint8_t s_rx_seq_valid;
static uint32_t s_rx_last_seq;

static uint8_t s_tx_wire[sizeof(audio_header_t) + APP_AUDIO_LINK_RECORD_WIRE_BYTES]
    __attribute__((aligned(4)));
static int16_t s_record_source_48k[APP_AUDIO_LINK_RECORD_SOURCE_SAMPLES]
    __attribute__((aligned(4)));
#if TRI_RECORD_WIRE_SAMPLE_RATE_HZ == 16000U
static int16_t s_record_wire_16k[APP_AUDIO_LINK_RECORD_WIRE_SAMPLES]
    __attribute__((aligned(4)));
#endif
/* 21-tap Hamming-window low-pass FIR, 48 kHz -> 16 kHz, Q15.
 * Sum is exactly 32768.  State is per STA and spans 10 ms packet boundaries. */
static const int16_t s_record_decim_q15[TRI_RECORD_DECIM_TAPS] = {
    22, 110, 189, 52, -488, -1114, -888, 1076, 4585, 8072, 9536,
    8072, 4585, 1076, -888, -1114, -488, 52, 189, 110, 22
};
static int16_t s_record_decim_history[TRI_RECORD_DECIM_HISTORY_FRAMES *
                                      APP_AUDIO_LINK_UAC_CHANNELS];
static uint8_t s_record_decim_history_valid;
static volatile uint8_t s_record_decim_reset_pending = 1U;
static uint16_t s_tx_len;
static uint16_t s_tx_off;
static uint8_t s_tx_packet_loaded;
static uint8_t s_tx_current_pcm;
static uint8_t s_tx_current_lossless;
static uint8_t s_tx_current_group_ms;
static uint16_t s_tx_current_payload_len;
static uint32_t s_tx_seq;
static uint32_t s_tx_blocked_since_ms;
static uint8_t s_tx_blocked;
static uint32_t s_last_tx_ms;
static uint32_t s_next_audio_tx_ms;

static uint32_t s_play_rx_packets;
static uint32_t s_play_rx_drop;
/* NETDIAG1/2: AP clock/sequence marker from the last successfully accepted
 * AP->STA PCM packet.  h->timestamp is generated by the AP and therefore
 * provides an AP-clock anchor in the STA log without synchronizing clocks. */
static volatile uint32_t s_play_last_ap_seq;
static volatile uint32_t s_play_last_ap_timestamp_ms;
static uint32_t s_last_play_pcm_ms;
static uint32_t s_play_seq_gap;
static uint32_t s_record_tx_packets;
static uint32_t s_record_tx_20ms_packets;
static uint32_t s_record_tx_10ms_fallback_packets;
static uint32_t s_record_tx_raw_equiv_bytes;
static uint32_t s_record_tx_drop;
static uint32_t s_record_stale_drop;
static uint32_t s_record_trim_drop;
static uint32_t s_record_partial_reset;
static uint32_t s_record_eagain;
static uint32_t s_record_enomem;
static uint32_t s_record_enobufs;
static uint32_t s_record_last_fault_log_ms;
static uint32_t s_reconnects;

/* AP->STA playback lossless receive diagnostics.  CRC is checked on every
 * compressed frame before PCM is admitted to the existing playback ring. */
static uint32_t s_play_lossless_comp_packets;
static uint32_t s_play_lossless_raw_packets;
static uint32_t s_play_lossless_payload_bytes;
static uint32_t s_play_lossless_crc_fail;
static uint32_t s_play_lossless_decode_fail;
static uint32_t s_play_lossless_len_fail;
static uint32_t s_play_lossless_verify_ok;
static uint32_t s_play_lossless_decode_max_us;
static uint32_t s_play_lossless_probe_est_max_us;
static uint32_t s_play_lossless_probe_count;
static uint32_t s_play_lossless_probe_fail;

/* UARTSAFE2/PBUF3 playback-buffer diagnostics.  No extra PCM storage is added.
 * q_min is the cumulative minimum; q_min_window is reset after each 5 s AUDIO
 * report so burst starvation is visible without the old startup min=0 sticking
 * forever.  uf/rb/ovf remain cumulative and the AUDIO line reports window deltas. */
static uint32_t s_play_q_min_bytes = 0xFFFFFFFFU;
static uint32_t s_play_q_min_window_bytes = 0xFFFFFFFFU;
static uint32_t s_play_underflow_events;
static uint32_t s_play_rebuffer_events;
static uint32_t s_play_overflow_events;
static uint8_t s_play_started_once;
static uint8_t s_play_waiting_rebuffer;
static uint8_t s_play_fade_in_pending;
static uint8_t s_play_last_sample_valid;
static int16_t s_play_last_sample[APP_AUDIO_LINK_UAC_CHANNELS];
static uint32_t s_play_fade_out_events;
static uint32_t s_play_fade_in_events;

/* BUF20: retain JCTRL3 byte-correct time-lock and +/-4-frame correction,
 * while shifting the normal playback reserve up by 20 ms.  Startup/rebuffer is
 * 90 ms, LOW/HIGH are 80/90 ms, and the ring is 120 ms so 30 ms of upper
 * headroom remains for burst arrivals.  pop_gap measures downstream reader
 * cadence; rx_gap measures accepted AP->STA 10 ms PCM packet arrival cadence.
 * ovf counts ring-full oldest-packet discards only. */
static volatile uint32_t s_play_tl_slow_blocks;
static volatile uint32_t s_play_tl_normal_blocks;
static volatile uint32_t s_play_tl_fast_blocks;
static volatile uint32_t s_play_tl_last_source_frames = APP_AUDIO_LINK_UAC_FRAMES_PER_PKT;
static volatile uint8_t s_play_pop_timing_active;
static volatile uint32_t s_play_pop_last_us;
static volatile uint32_t s_play_pop_gap_max_window_us;
static volatile uint32_t s_play_pop_gap_max_total_us;
static volatile uint8_t s_play_rx_timing_active;
static volatile uint32_t s_play_rx_last_us;
static volatile uint32_t s_play_rx_gap_max_window_us;
static volatile uint32_t s_play_rx_gap_max_total_us;
/* NETDIAG2: companion data for the exact packet pair that produced the
 * 5-second window's maximum rxgap.  apgap is the AP header-generation gap,
 * seqgap is the AP header sequence delta (not necessarily packet loss),
 * extra = rxgap - apgap, and hole_q is the approximate playback queue depth
 * immediately before the later packet is pushed into the ring. */
static volatile uint64_t s_play_rx_prev_ap_timestamp_us;
static volatile uint32_t s_play_rx_prev_ap_seq;
static volatile uint32_t s_play_rx_hole_apgap_us_window;
static volatile uint32_t s_play_rx_hole_seqgap_window;
static volatile int32_t s_play_rx_hole_extra_us_window;
static volatile uint32_t s_play_rx_hole_q_ms_window;

/* STA->AP record lossless transmit diagnostics. */
static uint32_t s_lossless_comp_packets;
static uint32_t s_lossless_raw_packets;
static uint32_t s_lossless_payload_bytes;
static uint16_t s_lossless_payload_min = 0xFFFFU;
static uint16_t s_lossless_payload_max;
static uint32_t s_lossless_selftest_pass;
static uint32_t s_lossless_selftest_fail;
static uint32_t s_lossless_encode_frames;
static uint32_t s_lossless_encode_max_us;
static uint32_t s_lossless_slow_hits;
static uint32_t s_lossless_probe_est_max_us;
static uint32_t s_lossless_probe_count;
static uint32_t s_lossless_probe_fail;

static uint64_t tri_time_us(void)
{
    static uint32_t last;
    static uint64_t wrap;
    uint32_t protect = rtos_protect();
    uint32_t now = us_ticker_read();
    uint64_t result;

    /* Keep the useful R12 fix while rolling back its QoS/reorder changes. */
    if ((now < last) && ((last - now) > 0x80000000U)) {
        wrap += 0x100000000ULL;
    }
    last = now;
    result = wrap + now;
    rtos_unprotect(protect);
    return result;
}

static uint32_t tri_now_ms(void)
{
    return (uint32_t)(tri_time_us() / 1000ULL);
}

#define TRI_FLOW_OFF                         0U
#define TRI_FLOW_DOWN                        1U
#define TRI_FLOW_ACTIVE                      2U

static uint32_t tri_rate_per_second(uint32_t delta, uint32_t elapsed_ms)
{
    if (elapsed_ms == 0U) {
        return 0U;
    }
    return (uint32_t)((((uint64_t)delta * 1000ULL) +
                       ((uint64_t)elapsed_ms / 2ULL)) /
                      (uint64_t)elapsed_ms);
}

static uint32_t tri_effective_packets(uint32_t completed,
                                      uint32_t lost,
                                      uint32_t expected)
{
    uint32_t maximum;
    if (lost >= expected) {
        return 0U;
    }
    maximum = expected - lost;
    if (completed > maximum) {
        completed = maximum;
    }
    return completed;
}

static void tri_log_audio_rate(uint8_t play_state, uint32_t play_actual,
                               uint8_t record_state, uint32_t record_actual,
                               uint32_t play_q_ms, uint32_t play_min_win_ms,
                               uint32_t uf_delta, uint32_t rb_delta,
                               uint32_t ovf_delta,
                               uint32_t tl_slow_delta, uint32_t tl_normal_delta,
                               uint32_t tl_fast_delta, uint32_t pop_gap_max_us,
                               uint32_t rx_gap_max_us, uint32_t rx_hole_apgap_us,
                               uint32_t rx_hole_seqgap, int32_t rx_hole_extra_us,
                               uint32_t rx_hole_q_ms)
{
    const unsigned id = (unsigned)TRIANGLE_DEVICE_ID;
    const unsigned play_expected = (unsigned)TRI_PLAY_EXPECTED_PPS;
    const unsigned record_expected = (unsigned)TRI_RECORD_EXPECTED_PPS;
    const unsigned now_ms = (unsigned)tri_now_ms();
    const unsigned ap_ms = (unsigned)s_play_last_ap_timestamp_ms;
    const unsigned ap_seq = (unsigned)s_play_last_ap_seq;

    if (play_state == TRI_FLOW_OFF) {
        if (record_state == TRI_FLOW_OFF) {
            dbg("TRI%u AUDIO play=OFF rec=OFF\n", id);
        } else if (record_state == TRI_FLOW_DOWN) {
            dbg("TRI%u AUDIO play=OFF rec=DOWN\n", id);
        } else {
            dbg("TRI%u AUDIO play=OFF rec=%u/%u\n",
                id, (unsigned)record_actual, record_expected);
        }
    } else if (play_state == TRI_FLOW_DOWN) {
        if (record_state == TRI_FLOW_OFF) {
            dbg("TRI%u AUDIO play=DOWN rec=OFF\n", id);
        } else if (record_state == TRI_FLOW_DOWN) {
            dbg("TRI%u AUDIO play=DOWN rec=DOWN\n", id);
        } else {
            dbg("TRI%u AUDIO play=DOWN rec=%u/%u\n",
                id, (unsigned)record_actual, record_expected);
        }
    } else {
        if (record_state == TRI_FLOW_OFF) {
            dbg("TRI%u AUDIO play=%u/%u rec=OFF q=%ums min_win=%ums uf+=%u rb+=%u ovf+=%u tl+=%u/%u/%u cons=%uf popgap=%uus rxgap=%uus uf=%u rb=%u ovf=%u t=%u ap=%u seq=%u hole=%u apgap=%uus seqgap=%u extra=%dus hq=%ums\n",
                id, (unsigned)play_actual, play_expected,
                (unsigned)play_q_ms, (unsigned)play_min_win_ms,
                (unsigned)uf_delta, (unsigned)rb_delta,
                (unsigned)ovf_delta,
                (unsigned)tl_slow_delta, (unsigned)tl_normal_delta,
                (unsigned)tl_fast_delta,
                (unsigned)s_play_tl_last_source_frames,
                (unsigned)pop_gap_max_us,
                (unsigned)rx_gap_max_us,
                (unsigned)s_play_underflow_events,
                (unsigned)s_play_rebuffer_events,
                (unsigned)s_play_overflow_events,
                now_ms, ap_ms, ap_seq,
                (unsigned)(rx_gap_max_us >= TRI_RXHOLE_WARN_US),
                (unsigned)rx_hole_apgap_us,
                (unsigned)rx_hole_seqgap,
                (int)rx_hole_extra_us,
                (unsigned)rx_hole_q_ms);
        } else if (record_state == TRI_FLOW_DOWN) {
            dbg("TRI%u AUDIO play=%u/%u rec=DOWN q=%ums min_win=%ums uf+=%u rb+=%u ovf+=%u tl+=%u/%u/%u cons=%uf popgap=%uus rxgap=%uus uf=%u rb=%u ovf=%u t=%u ap=%u seq=%u hole=%u apgap=%uus seqgap=%u extra=%dus hq=%ums\n",
                id, (unsigned)play_actual, play_expected,
                (unsigned)play_q_ms, (unsigned)play_min_win_ms,
                (unsigned)uf_delta, (unsigned)rb_delta,
                (unsigned)ovf_delta,
                (unsigned)tl_slow_delta, (unsigned)tl_normal_delta,
                (unsigned)tl_fast_delta,
                (unsigned)s_play_tl_last_source_frames,
                (unsigned)pop_gap_max_us,
                (unsigned)rx_gap_max_us,
                (unsigned)s_play_underflow_events,
                (unsigned)s_play_rebuffer_events,
                (unsigned)s_play_overflow_events,
                now_ms, ap_ms, ap_seq,
                (unsigned)(rx_gap_max_us >= TRI_RXHOLE_WARN_US),
                (unsigned)rx_hole_apgap_us,
                (unsigned)rx_hole_seqgap,
                (int)rx_hole_extra_us,
                (unsigned)rx_hole_q_ms);
        } else {
            dbg("TRI%u AUDIO play=%u/%u rec=%u/%u q=%ums min_win=%ums uf+=%u rb+=%u ovf+=%u tl+=%u/%u/%u cons=%uf popgap=%uus rxgap=%uus uf=%u rb=%u ovf=%u t=%u ap=%u seq=%u hole=%u apgap=%uus seqgap=%u extra=%dus hq=%ums\n",
                id, (unsigned)play_actual, play_expected,
                (unsigned)record_actual, record_expected,
                (unsigned)play_q_ms, (unsigned)play_min_win_ms,
                (unsigned)uf_delta, (unsigned)rb_delta,
                (unsigned)ovf_delta,
                (unsigned)tl_slow_delta, (unsigned)tl_normal_delta,
                (unsigned)tl_fast_delta,
                (unsigned)s_play_tl_last_source_frames,
                (unsigned)pop_gap_max_us,
                (unsigned)rx_gap_max_us,
                (unsigned)s_play_underflow_events,
                (unsigned)s_play_rebuffer_events,
                (unsigned)s_play_overflow_events,
                now_ms, ap_ms, ap_seq,
                (unsigned)(rx_gap_max_us >= TRI_RXHOLE_WARN_US),
                (unsigned)rx_hole_apgap_us,
                (unsigned)rx_hole_seqgap,
                (int)rx_hole_extra_us,
                (unsigned)rx_hole_q_ms);
        }
    }
}

static void tri_close(int *fd)
{
    if ((fd != NULL) && (*fd >= 0)) {
        (void)shutdown(*fd, SHUT_RDWR);
        close(*fd);
        *fd = -1;
    }
}

static int tri_udp_temporary_error(void)
{
    return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR ||
           errno == ENOMEM
#ifdef ENOBUFS
           || errno == ENOBUFS
#endif
           ;
}

/* UDP connect only selects/filters the AP endpoint; HELLO ACK establishes
 * application readiness. No TCP handshake or stream socket options. */
static int tri_connect(uint16_t port, const char *name)
{
    struct sockaddr_in addr;
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    uint8_t direction = port == APP_AUDIO_LINK_UAC_SERVER_PORT ?
        APP_AUDIO_LINK_DIRECTION_AP_TO_STA : APP_AUDIO_LINK_DIRECTION_STA_TO_AP;
    tri_udp_session_t *session = &s_udp_session[direction - 1U];
#if LWIP_SO_RCVBUF
    int sockbuf = direction == APP_AUDIO_LINK_DIRECTION_AP_TO_STA ?
        TRI_SOCKET_BUF_BYTES : TRI_RECORD_SOCKET_BUF_BYTES;
#endif
    (void)name;
    if (fd < 0) return -1;
#if LWIP_SO_RCVBUF
    if (setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &sockbuf, sizeof(sockbuf)) < 0) {
        close(fd);
        return -1;
    }
#endif
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(APP_AUDIO_LINK_SERVER_IP);
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }
    {
        uint64_t token = tri_time_us();
        if (token <= session->token) token = session->token + 1U;
        memset(session, 0, sizeof(*session));
        session->token = token;
    }
    if (direction == APP_AUDIO_LINK_DIRECTION_AP_TO_STA)
        memset(&s_rx_assembly, 0, sizeof(s_rx_assembly));
    return fd;
}

static int tri_udp_send_hello(int fd, uint8_t direction)
{
    uint8_t buf[sizeof(audio_header_t) + sizeof(audio_ctrl_payload_t)];
    audio_header_t h;
    audio_ctrl_payload_t ctrl;
    int n;
    tri_udp_session_t *session = &s_udp_session[direction - 1U];
    h.magic = APP_AUDIO_LINK_PACKET_MAGIC;
    h.seq_num = 0U; /* HELLO never consumes an audio sequence number. */
    h.timestamp = session->token;
    h.packet_type = APP_AUDIO_LINK_PACKET_TYPE_CTRL;
    h.direction = direction;
    h.client_id = TRIANGLE_DEVICE_ID;
    h.data_len = sizeof(ctrl);
    ctrl.ctrl_type = APP_AUDIO_LINK_CTRL_SESSION_HELLO;
    ctrl.value = TRIANGLE_DEVICE_ID;
    ctrl.reserved = 0U;
    memcpy(buf, &h, sizeof(h));
    memcpy(buf + sizeof(h), &ctrl, sizeof(ctrl));
    session->hello_ms = tri_now_ms();
    n = send(fd, buf, sizeof(buf), MSG_DONTWAIT);
    if (n == sizeof(buf)) return 0;
    return n < 0 && tri_udp_temporary_error() ? 0 : -1;
}

static int tri_udp_accept_ack(const uint8_t *wire, int n, uint8_t direction)
{
    audio_header_t h;
    audio_ctrl_payload_t ctrl;
    tri_udp_session_t *session = &s_udp_session[direction - 1U];
    if (n != sizeof(h) + sizeof(ctrl)) return 0;
    memcpy(&h, wire, sizeof(h));
    memcpy(&ctrl, wire + sizeof(h), sizeof(ctrl));
    if (h.magic != APP_AUDIO_LINK_PACKET_MAGIC ||
        h.packet_type != APP_AUDIO_LINK_PACKET_TYPE_CTRL ||
        h.direction != direction || h.client_id != TRIANGLE_DEVICE_ID ||
        h.data_len != sizeof(ctrl) || h.seq_num != 0U ||
        h.timestamp != session->token ||
        ctrl.ctrl_type != APP_AUDIO_LINK_CTRL_SESSION_HELLO ||
        ctrl.value != TRIANGLE_DEVICE_ID || ctrl.reserved) return 0;
    session->ack_ms = tri_now_ms();
    session->acked = 1U;
    return 1;
}

/* Initial registration retries the same token/socket. Allows the AP's old
 * 3-second ID lease to expire after a reboot before declaring failure. */
static int tri_send_hello(int fd, uint8_t direction)
{
    uint32_t start = tri_now_ms();
    tri_udp_session_t *session = &s_udp_session[direction - 1U];
    uint8_t wire[sizeof(audio_header_t) + sizeof(audio_ctrl_payload_t) + 1U];
    if (tri_udp_send_hello(fd, direction) < 0) return -1;
    while (!s_stop && wlan_get_connect_status() &&
           tri_now_ms() - start < TRI_CONNECT_TIMEOUT_MS) {
        int n = recv(fd, wire, sizeof(wire), MSG_DONTWAIT);
        if (n >= 0 && tri_udp_accept_ack(wire, n, direction)) return 0;
        if (n < 0 && !tri_udp_temporary_error()) return -1;
        if (tri_now_ms() - session->hello_ms >= TRI_HEARTBEAT_MS &&
            tri_udp_send_hello(fd, direction) < 0) return -1;
        rtos_task_suspend(TRI_LOOP_MS);
    }
    return -1;
}

static void tri_record_decimator_reset(void)
{
    /* May be requested by the playback/control task.  Apply it inside the
     * record task instead of racing a history memset against the FIR loop. */
    s_record_decim_reset_pending = 1U;
}

static int16_t tri_clip_s16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return -32768;
    }
    return (int16_t)value;
}

static void tri_record_downsample_48k_to_16k(const int16_t *src, int16_t *dst)
{
    uint32_t out_frame;
    uint32_t ch;

    if (s_record_decim_reset_pending || !s_record_decim_history_valid) {
        uint32_t i;
        for (i = 0U; i < TRI_RECORD_DECIM_HISTORY_FRAMES; i++) {
            for (ch = 0U; ch < APP_AUDIO_LINK_UAC_CHANNELS; ch++) {
                s_record_decim_history[i * APP_AUDIO_LINK_UAC_CHANNELS + ch] =
                    src[ch];
            }
        }
        s_record_decim_history_valid = 1U;
        s_record_decim_reset_pending = 0U;
    }

    for (out_frame = 0U; out_frame < APP_AUDIO_LINK_RECORD_WIRE_FRAMES;
         out_frame++) {
        int32_t endpoint = (int32_t)(out_frame * 3U);
        for (ch = 0U; ch < APP_AUDIO_LINK_UAC_CHANNELS; ch++) {
            int32_t acc = 0;
            uint32_t tap;
            for (tap = 0U; tap < TRI_RECORD_DECIM_TAPS; tap++) {
                int32_t src_index = endpoint -
                    (int32_t)(TRI_RECORD_DECIM_TAPS - 1U - tap);
                int16_t sample;
                if (src_index < 0) {
                    sample = s_record_decim_history[
                        (uint32_t)(src_index +
                        (int32_t)TRI_RECORD_DECIM_HISTORY_FRAMES) *
                        APP_AUDIO_LINK_UAC_CHANNELS + ch];
                } else {
                    sample = src[(uint32_t)src_index *
                                 APP_AUDIO_LINK_UAC_CHANNELS + ch];
                }
                acc += (int32_t)sample * (int32_t)s_record_decim_q15[tap];
            }
            if (acc >= 0) {
                acc += 16384;
            } else {
                acc -= 16384;
            }
            dst[out_frame * APP_AUDIO_LINK_UAC_CHANNELS + ch] =
                tri_clip_s16(acc >> 15);
        }
    }

    memcpy(s_record_decim_history,
           src + ((APP_AUDIO_LINK_RECORD_SOURCE_FRAMES -
                   TRI_RECORD_DECIM_HISTORY_FRAMES) *
                  APP_AUDIO_LINK_UAC_CHANNELS),
           sizeof(s_record_decim_history));
}

static void tri_tx_queue_reset(void)
{
    if (s_tx_mutex != NULL) {
        rtos_mutex_lock(s_tx_mutex, -1);
    }
    memset(s_tx_queue, 0, sizeof(s_tx_queue));
    s_tx_rd = s_tx_wr = s_tx_count = 0U;
    if (s_tx_mutex != NULL) {
        rtos_mutex_unlock(s_tx_mutex);
    }
    s_tx_len = s_tx_off = 0U;
    s_tx_packet_loaded = 0U;
    s_tx_current_pcm = 0U;
    s_tx_current_lossless = 0U;
    s_tx_current_group_ms = 0U;
    s_tx_current_payload_len = 0U;
    s_next_audio_tx_ms = 0U;
    s_tx_blocked_since_ms = 0U;
    s_tx_blocked = 0U;
    tri_record_decimator_reset();
}

static int tri_tx_queue_push(const int16_t *pcm)
{
    uint8_t trimmed = 0U;

    if (s_tx_mutex != NULL) {
        rtos_mutex_lock(s_tx_mutex, -1);
    }
    if (s_tx_count >= TRI_TX_HIGH_WATER_BLOCKS) {
        while (s_tx_count > TRI_TX_KEEP_BLOCKS) {
            s_tx_queue[s_tx_rd].used = 0U;
            s_tx_rd = (uint8_t)((s_tx_rd + 1U) % TRI_TX_QUEUE_BLOCKS);
            s_tx_count--;
            /* One queued 10 ms source block is one 10 ms wire packet. */
            s_record_tx_drop++;
            s_record_trim_drop++;
            trimmed = 1U;
        }
    }
    memcpy(s_tx_queue[s_tx_wr].pcm, pcm, APP_AUDIO_LINK_UAC_BYTES_PER_PKT);
    s_tx_queue[s_tx_wr].used = 1U;
    s_tx_wr = (uint8_t)((s_tx_wr + 1U) % TRI_TX_QUEUE_BLOCKS);
    s_tx_count++;
    if (s_tx_mutex != NULL) {
        rtos_mutex_unlock(s_tx_mutex);
    }
    if (trimmed) {
        tri_record_decimator_reset();
    }
    return 0;
}

static int tri_tx_queue_pop_10ms(int16_t *pcm)
{
    int ret = -1;
    uint32_t block;

    if (s_tx_mutex != NULL) {
        rtos_mutex_lock(s_tx_mutex, -1);
    }
    if (s_tx_count >= APP_AUDIO_LINK_RECORD_SOURCE_BLOCKS) {
        for (block = 0U; block < APP_AUDIO_LINK_RECORD_SOURCE_BLOCKS; block++) {
            memcpy(pcm + block * APP_AUDIO_LINK_UAC_SAMPLES_PER_PKT,
                   s_tx_queue[s_tx_rd].pcm,
                   APP_AUDIO_LINK_UAC_BYTES_PER_PKT);
            s_tx_queue[s_tx_rd].used = 0U;
            s_tx_rd = (uint8_t)((s_tx_rd + 1U) % TRI_TX_QUEUE_BLOCKS);
            s_tx_count--;
        }
        ret = 0;
    }
    if (s_tx_mutex != NULL) {
        rtos_mutex_unlock(s_tx_mutex);
    }
    return ret;
}

/* Put one 10 ms source block back at the head.  REC20 uses this only when
 * the second subframe does not fit the bounded 20 ms compressed container.
 * We just popped that block, so queue capacity is guaranteed. */
static void tri_tx_queue_push_front_10ms(const int16_t *pcm)
{
    if (s_tx_mutex != NULL) rtos_mutex_lock(s_tx_mutex, -1);
    s_tx_rd = (uint8_t)((s_tx_rd + TRI_TX_QUEUE_BLOCKS - 1U) % TRI_TX_QUEUE_BLOCKS);
    memcpy(s_tx_queue[s_tx_rd].pcm, pcm, APP_AUDIO_LINK_UAC_BYTES_PER_PKT);
    s_tx_queue[s_tx_rd].used = 1U;
    s_tx_count++;
    if (s_tx_mutex != NULL) rtos_mutex_unlock(s_tx_mutex);
}

/* Keep only recent queued 10 ms source packets after a stale unsent packet
 * is discarded.  Each queue entry is already one complete wire packet. */
static uint32_t tri_tx_queue_trim_to_latest(uint8_t keep_blocks)
{
    uint32_t dropped_packets = 0U;

    if (s_tx_mutex != NULL) {
        rtos_mutex_lock(s_tx_mutex, -1);
    }
    while (s_tx_count > keep_blocks) {
        s_tx_queue[s_tx_rd].used = 0U;
        s_tx_rd = (uint8_t)((s_tx_rd + 1U) % TRI_TX_QUEUE_BLOCKS);
        s_tx_count--;
        dropped_packets++;
    }
    if (s_tx_mutex != NULL) {
        rtos_mutex_unlock(s_tx_mutex);
    }

    s_record_tx_drop += dropped_packets;
    s_record_trim_drop += dropped_packets;
    return dropped_packets;
}

static void tri_rx_queue_reset(void)
{
    if (s_rx_mutex != NULL) {
        rtos_mutex_lock(s_rx_mutex, -1);
    }
    s_rx_pcm_rd = 0U;
    s_rx_pcm_wr = 0U;
    s_rx_pcm_count = 0U;
    s_rx_started = 0U;
    s_play_fade_in_pending = 0U;
    s_play_last_sample_valid = 0U;
    s_play_q_min_window_bytes = 0xFFFFFFFFU;
    s_play_tl_last_source_frames = APP_AUDIO_LINK_UAC_FRAMES_PER_PKT;
    s_play_pop_last_us = 0U;
    s_play_pop_gap_max_window_us = 0U;
    s_play_rx_last_us = 0U;
    s_play_rx_gap_max_window_us = 0U;
    s_play_rx_prev_ap_timestamp_us = 0ULL;
    s_play_rx_prev_ap_seq = 0U;
    s_play_rx_hole_apgap_us_window = 0U;
    s_play_rx_hole_seqgap_window = 0U;
    s_play_rx_hole_extra_us_window = 0;
    s_play_rx_hole_q_ms_window = 0U;
    if (s_rx_mutex != NULL) {
        rtos_mutex_unlock(s_rx_mutex);
    }
    memset(&s_rx_assembly, 0, sizeof(s_rx_assembly));
    s_rx_seq_valid = 0U;
    s_play_last_ap_seq = 0U;
    s_play_last_ap_timestamp_ms = 0U;
}

static void tri_rx_ring_discard_locked(uint32_t bytes)
{
    if (bytes > s_rx_pcm_count) {
        bytes = s_rx_pcm_count;
    }
    s_rx_pcm_rd = (s_rx_pcm_rd + bytes) % sizeof(s_rx_pcm_ring);
    s_rx_pcm_count -= bytes;
}

static void tri_rx_ring_write_locked(const uint8_t *src, uint32_t bytes)
{
    uint32_t first = sizeof(s_rx_pcm_ring) - s_rx_pcm_wr;
    if (first > bytes) {
        first = bytes;
    }
    memcpy(s_rx_pcm_ring + s_rx_pcm_wr, src, first);
    if (bytes > first) {
        memcpy(s_rx_pcm_ring, src + first, bytes - first);
    }
    s_rx_pcm_wr = (s_rx_pcm_wr + bytes) % sizeof(s_rx_pcm_ring);
    s_rx_pcm_count += bytes;
}

static void tri_rx_ring_read_locked(uint8_t *dst, uint32_t bytes)
{
    uint32_t first = sizeof(s_rx_pcm_ring) - s_rx_pcm_rd;
    if (first > bytes) {
        first = bytes;
    }
    if (dst != NULL) {
        memcpy(dst, s_rx_pcm_ring + s_rx_pcm_rd, first);
        if (bytes > first) {
            memcpy(dst + first, s_rx_pcm_ring, bytes - first);
        }
    }
    tri_rx_ring_discard_locked(bytes);
}

static int16_t tri_rx_ring_peek_s16_locked(uint32_t sample_index)
{
    uint32_t byte_offset = sample_index * sizeof(int16_t);
    uint32_t pos = (s_rx_pcm_rd + byte_offset) % sizeof(s_rx_pcm_ring);
    uint16_t raw;
    if ((pos + 1U) < sizeof(s_rx_pcm_ring)) {
        raw = (uint16_t)s_rx_pcm_ring[pos] |
              ((uint16_t)s_rx_pcm_ring[pos + 1U] << 8);
    } else {
        raw = (uint16_t)s_rx_pcm_ring[pos] |
              ((uint16_t)s_rx_pcm_ring[0] << 8);
    }
    return (int16_t)raw;
}

static void tri_rx_queue_push(const int16_t *pcm)
{
    if (s_rx_mutex != NULL) {
        rtos_mutex_lock(s_rx_mutex, -1);
    }
    while ((sizeof(s_rx_pcm_ring) - s_rx_pcm_count) < TRI_PLAYBACK_WIRE_BYTES) {
        tri_rx_ring_discard_locked(TRI_PLAYBACK_WIRE_BYTES);
        s_play_rx_drop++;
        s_play_overflow_events++;
    }
    tri_rx_ring_write_locked((const uint8_t *)pcm, TRI_PLAYBACK_WIRE_BYTES);
    if (s_rx_mutex != NULL) {
        rtos_mutex_unlock(s_rx_mutex);
    }
}

static void tri_playback_apply_fade_in(int16_t *pcm)
{
    uint32_t frame;
    uint32_t ch;
    uint32_t fade_frames = TRI_PLAYBACK_FADE_FRAMES;

    if (fade_frames > APP_AUDIO_LINK_UAC_FRAMES_PER_PKT) {
        fade_frames = APP_AUDIO_LINK_UAC_FRAMES_PER_PKT;
    }
    for (frame = 0U; frame < fade_frames; frame++) {
        int32_t gain_num = (int32_t)(frame + 1U);
        for (ch = 0U; ch < APP_AUDIO_LINK_UAC_CHANNELS; ch++) {
            int32_t v = pcm[frame * APP_AUDIO_LINK_UAC_CHANNELS + ch];
            pcm[frame * APP_AUDIO_LINK_UAC_CHANNELS + ch] =
                (int16_t)((v * gain_num) / (int32_t)fade_frames);
        }
    }
}

static void tri_playback_make_fade_out(int16_t *pcm)
{
    uint32_t frame;
    uint32_t ch;
    uint32_t fade_frames = TRI_PLAYBACK_FADE_FRAMES;

    memset(pcm, 0, APP_AUDIO_LINK_UAC_BYTES_PER_PKT);
    if (!s_play_last_sample_valid) {
        return;
    }
    if (fade_frames > APP_AUDIO_LINK_UAC_FRAMES_PER_PKT) {
        fade_frames = APP_AUDIO_LINK_UAC_FRAMES_PER_PKT;
    }
    for (frame = 0U; frame < fade_frames; frame++) {
        int32_t gain_num = (int32_t)(fade_frames - frame);
        for (ch = 0U; ch < APP_AUDIO_LINK_UAC_CHANNELS; ch++) {
            int32_t v = s_play_last_sample[ch];
            pcm[frame * APP_AUDIO_LINK_UAC_CHANNELS + ch] =
                (int16_t)((v * gain_num) / (int32_t)fade_frames);
        }
    }
}

static int tri_rx_queue_pop_10ms(int16_t *pcm)
{
    int ret = -1;
    uint32_t source_frames = APP_AUDIO_LINK_UAC_FRAMES_PER_PKT;
    uint32_t source_bytes;

    if (s_rx_mutex != NULL) {
        rtos_mutex_lock(s_rx_mutex, -1);
    }
    if (!s_rx_started &&
        (s_rx_pcm_count >= (TRI_RX_PREBUFFER_PACKETS * TRI_PLAYBACK_WIRE_BYTES))) {
        s_rx_started = 1U;
        s_play_fade_in_pending = 1U;
        if (s_play_started_once) {
            if (s_play_waiting_rebuffer) {
                s_play_rebuffer_events++;
                s_play_waiting_rebuffer = 0U;
            }
        } else {
            s_play_started_once = 1U;
        }
    }

    if (s_rx_started) {
        /* JCTRL3: keep JCTRL2 byte-correct thresholds, with LOW/HIGH now
         * 6/7 x current 10 ms packet bytes = 60/70 ms.  Do not use the legacy
         * APP_AUDIO_LINK_UAC_WIRE_FRAMES unit here. */
        if (s_rx_pcm_count >= TRI_RX_TIMELOCK_HIGH_BYTES) {
            source_frames = APP_AUDIO_LINK_UAC_FRAMES_PER_PKT +
                            TRI_RX_TIMELOCK_ADJUST_FRAMES;
        } else if (s_rx_pcm_count <= TRI_RX_TIMELOCK_LOW_BYTES) {
            source_frames = APP_AUDIO_LINK_UAC_FRAMES_PER_PKT -
                            TRI_RX_TIMELOCK_ADJUST_FRAMES;
        }
        source_bytes = source_frames * APP_AUDIO_LINK_UAC_CHANNELS * sizeof(int16_t);

        if (s_rx_pcm_count >= source_bytes) {
            if (source_frames == APP_AUDIO_LINK_UAC_FRAMES_PER_PKT) {
                tri_rx_ring_read_locked((uint8_t *)pcm,
                                        APP_AUDIO_LINK_UAC_BYTES_PER_PKT);
            } else {
                uint32_t out_frame;
                for (out_frame = 0U;
                     out_frame < APP_AUDIO_LINK_UAC_FRAMES_PER_PKT;
                     out_frame++) {
                    uint32_t numerator = out_frame * (source_frames - 1U);
                    uint32_t idx = numerator /
                                   (APP_AUDIO_LINK_UAC_FRAMES_PER_PKT - 1U);
                    uint32_t rem = numerator %
                                   (APP_AUDIO_LINK_UAC_FRAMES_PER_PKT - 1U);
                    uint32_t frac_q15 = (rem << 15) /
                                        (APP_AUDIO_LINK_UAC_FRAMES_PER_PKT - 1U);
                    uint32_t idx1 = (idx + 1U < source_frames) ? (idx + 1U) : idx;
                    uint32_t ch;
                    for (ch = 0U; ch < APP_AUDIO_LINK_UAC_CHANNELS; ch++) {
                        int32_t a = tri_rx_ring_peek_s16_locked(
                            idx * APP_AUDIO_LINK_UAC_CHANNELS + ch);
                        int32_t b = tri_rx_ring_peek_s16_locked(
                            idx1 * APP_AUDIO_LINK_UAC_CHANNELS + ch);
                        pcm[out_frame * APP_AUDIO_LINK_UAC_CHANNELS + ch] =
                            (int16_t)(a + (((b - a) * (int32_t)frac_q15) >> 15));
                    }
                }
                tri_rx_ring_discard_locked(source_bytes);
            }

            if (source_frames < APP_AUDIO_LINK_UAC_FRAMES_PER_PKT) {
                s_play_tl_slow_blocks++;
            } else if (source_frames > APP_AUDIO_LINK_UAC_FRAMES_PER_PKT) {
                s_play_tl_fast_blocks++;
            } else {
                s_play_tl_normal_blocks++;
            }
            s_play_tl_last_source_frames = source_frames;

            if (s_play_fade_in_pending) {
                tri_playback_apply_fade_in(pcm);
                s_play_fade_in_pending = 0U;
                s_play_fade_in_events++;
            }
            {
                uint32_t last_frame = APP_AUDIO_LINK_UAC_FRAMES_PER_PKT - 1U;
                uint32_t ch;
                for (ch = 0U; ch < APP_AUDIO_LINK_UAC_CHANNELS; ch++) {
                    s_play_last_sample[ch] =
                        pcm[last_frame * APP_AUDIO_LINK_UAC_CHANNELS + ch];
                }
                s_play_last_sample_valid = 1U;
            }

            if (s_rx_pcm_count < s_play_q_min_bytes) {
                s_play_q_min_bytes = s_rx_pcm_count;
            }
            if (s_rx_pcm_count < s_play_q_min_window_bytes) {
                s_play_q_min_window_bytes = s_rx_pcm_count;
            }
            ret = 0;
        } else {
            s_rx_started = 0U;
            if (s_play_started_once) {
                s_play_underflow_events++;
                s_play_waiting_rebuffer = 1U;
                s_play_q_min_bytes = 0U;
                s_play_q_min_window_bytes = 0U;

                /* Emit one 10 ms block with a 1 ms ramp from the last sample
                 * to zero.  Then keep the existing no-data behavior until the
                 * 70 ms rebuffer target has accumulated again. */
                tri_playback_make_fade_out(pcm);
                s_play_fade_out_events++;
                s_play_last_sample_valid = 0U;
                ret = 0;
            }
        }
    }
    if (s_rx_mutex != NULL) {
        rtos_mutex_unlock(s_rx_mutex);
    }
    return ret;
}

static void tri_playbuf_diag_snapshot(uint32_t *q_ms, uint32_t *min_ms)
{
    uint32_t q_bytes;
    uint32_t min_bytes;

    if (s_rx_mutex != NULL) {
        rtos_mutex_lock(s_rx_mutex, -1);
    }
    q_bytes = s_rx_pcm_count;
    min_bytes = s_play_q_min_bytes;
    if (s_rx_mutex != NULL) {
        rtos_mutex_unlock(s_rx_mutex);
    }
    if (min_bytes == 0xFFFFFFFFU) {
        min_bytes = q_bytes;
    }
    *q_ms = q_bytes / TRI_PLAYBACK_BYTES_PER_MS;
    *min_ms = min_bytes / TRI_PLAYBACK_BYTES_PER_MS;
}

static void tri_playbuf_window_snapshot_reset(uint32_t *q_ms, uint32_t *min_win_ms)
{
    uint32_t q_bytes;
    uint32_t min_bytes;

    if (s_rx_mutex != NULL) {
        rtos_mutex_lock(s_rx_mutex, -1);
    }
    q_bytes = s_rx_pcm_count;
    min_bytes = s_play_q_min_window_bytes;
    s_play_q_min_window_bytes = q_bytes;
    if (s_rx_mutex != NULL) {
        rtos_mutex_unlock(s_rx_mutex);
    }
    if (min_bytes == 0xFFFFFFFFU) {
        min_bytes = q_bytes;
    }
    *q_ms = q_bytes / TRI_PLAYBACK_BYTES_PER_MS;
    *min_win_ms = min_bytes / TRI_PLAYBACK_BYTES_PER_MS;
}

static void tri_playbuf_window_reset(void)
{
    if (s_rx_mutex != NULL) {
        rtos_mutex_lock(s_rx_mutex, -1);
    }
    s_play_q_min_window_bytes = s_rx_pcm_count;
    if (s_rx_mutex != NULL) {
        rtos_mutex_unlock(s_rx_mutex);
    }
}

static uint32_t tri_play_pop_gap_window_snapshot_reset(void)
{
    /* Diagnostic-only cross-task counters are 32-bit/volatile.  A boundary
     * race can move one sample to the adjacent 5 s window, but deliberately
     * avoid masking interrupts or taking a mutex in the real-time reader. */
    uint32_t value = s_play_pop_gap_max_window_us;
    s_play_pop_gap_max_window_us = 0U;
    return value;
}

static void tri_play_pop_gap_window_reset(void)
{
    s_play_pop_last_us = 0U;
    s_play_pop_gap_max_window_us = 0U;
}

static uint32_t tri_play_rx_gap_window_snapshot_reset(uint32_t *apgap_us,
                                                       uint32_t *seqgap,
                                                       int32_t *extra_us,
                                                       uint32_t *hole_q_ms)
{
    uint32_t value = s_play_rx_gap_max_window_us;
    *apgap_us = s_play_rx_hole_apgap_us_window;
    *seqgap = s_play_rx_hole_seqgap_window;
    *extra_us = s_play_rx_hole_extra_us_window;
    *hole_q_ms = s_play_rx_hole_q_ms_window;
    s_play_rx_gap_max_window_us = 0U;
    s_play_rx_hole_apgap_us_window = 0U;
    s_play_rx_hole_seqgap_window = 0U;
    s_play_rx_hole_extra_us_window = 0;
    s_play_rx_hole_q_ms_window = 0U;
    return value;
}

static void tri_play_rx_gap_window_reset(void)
{
    s_play_rx_last_us = 0U;
    s_play_rx_gap_max_window_us = 0U;
    s_play_rx_prev_ap_timestamp_us = 0ULL;
    s_play_rx_prev_ap_seq = 0U;
    s_play_rx_hole_apgap_us_window = 0U;
    s_play_rx_hole_seqgap_window = 0U;
    s_play_rx_hole_extra_us_window = 0;
    s_play_rx_hole_q_ms_window = 0U;
}

static void tri_log_short_play_check(void)
{
    uint32_t q_ms;
    uint32_t min_ms;
    tri_playbuf_diag_snapshot(&q_ms, &min_ms);

    /* raw/dec + len_fail=0 confirms every accepted compressed frame decoded
     * to the original 1920-byte PCM size.  match + crc_fail=0 confirms the
     * decoded PCM CRC32 equals the AP's pre-compression PCM CRC32. */
    dbg("TRI%u LLCHK P raw/dec=%u/%u ok=%u fail=%u/%u/%u q=%ums min=%ums uf=%u rb=%u ovf=%u tl=%u/%u/%u cons=%uf popgap=%uus rxgap=%uus t=%u ap=%u seq=%u\n",
        (unsigned)TRIANGLE_DEVICE_ID,
        (unsigned)TRI_PLAYBACK_WIRE_BYTES,
        (unsigned)TRI_PLAYBACK_WIRE_BYTES,
        (unsigned)s_play_lossless_verify_ok,
        (unsigned)s_play_lossless_crc_fail,
        (unsigned)s_play_lossless_decode_fail,
        (unsigned)s_play_lossless_len_fail,
        (unsigned)q_ms,
        (unsigned)min_ms,
        (unsigned)s_play_underflow_events,
        (unsigned)s_play_rebuffer_events,
        (unsigned)s_play_overflow_events,
        (unsigned)s_play_tl_slow_blocks,
        (unsigned)s_play_tl_normal_blocks,
        (unsigned)s_play_tl_fast_blocks,
        (unsigned)s_play_tl_last_source_frames,
        (unsigned)s_play_pop_gap_max_total_us,
        (unsigned)s_play_rx_gap_max_total_us,
        (unsigned)tri_now_ms(),
        (unsigned)s_play_last_ap_timestamp_ms,
        (unsigned)s_play_last_ap_seq);
}

static void tri_track_rx_seq(uint32_t seq)
{
    if (!s_rx_seq_valid) {
        s_rx_seq_valid = 1U;
        s_rx_last_seq = seq;
        return;
    }
    if (seq > (s_rx_last_seq + 1U)) {
        s_play_seq_gap += seq - (s_rx_last_seq + 1U);
    }
    s_rx_last_seq = seq;
}

static int tri_lossless_decode_payload(const uint8_t *payload, uint16_t payload_len,
                                       uint16_t expected_raw_len, uint16_t frames,
                                       int16_t *pcm_out);

static void tri_play_rx_arrival_mark(const audio_header_t *h, uint32_t hole_q_ms)
{
    /* NETDIAG2: mark only successfully decoded/validated AP->STA PCM packets.
     * Capture companion AP-generation timing for the exact pair that sets the
     * window maximum rxgap.  No printf, mutex or control action is performed
     * here so this remains a low-perturbation read-only diagnostic. */
    if (s_play_rx_timing_active) {
        uint32_t now_us = us_ticker_read();
        if ((s_play_rx_last_us != 0U) &&
            (s_play_rx_prev_ap_timestamp_us != 0ULL)) {
            uint32_t gap_us = now_us - s_play_rx_last_us;
            uint64_t apgap64 = (h->timestamp >= s_play_rx_prev_ap_timestamp_us) ?
                (h->timestamp - s_play_rx_prev_ap_timestamp_us) : 0ULL;
            uint32_t apgap_us = (apgap64 > 0xFFFFFFFFULL) ?
                0xFFFFFFFFU : (uint32_t)apgap64;
            uint32_t seqgap = (h->seq_num >= s_play_rx_prev_ap_seq) ?
                (h->seq_num - s_play_rx_prev_ap_seq) : 0U;
            int64_t extra64 = (int64_t)(uint64_t)gap_us - (int64_t)apgap_us;
            int32_t extra_us;

            if (extra64 > 2147483647LL) extra_us = 2147483647;
            else if (extra64 < (-2147483647LL - 1LL)) extra_us = (-2147483647 - 1);
            else extra_us = (int32_t)extra64;

            if (gap_us > s_play_rx_gap_max_window_us) {
                s_play_rx_gap_max_window_us = gap_us;
                s_play_rx_hole_apgap_us_window = apgap_us;
                s_play_rx_hole_seqgap_window = seqgap;
                s_play_rx_hole_extra_us_window = extra_us;
                s_play_rx_hole_q_ms_window = hole_q_ms;
            }
            if (gap_us > s_play_rx_gap_max_total_us) {
                s_play_rx_gap_max_total_us = gap_us;
            }
        }
        s_play_rx_last_us = now_us;
        s_play_rx_prev_ap_timestamp_us = h->timestamp;
        s_play_rx_prev_ap_seq = h->seq_num;
    } else {
        s_play_rx_last_us = 0U;
        s_play_rx_prev_ap_timestamp_us = 0ULL;
        s_play_rx_prev_ap_seq = 0U;
    }
}

static void tri_handle_rx(const audio_header_t *h, const uint8_t *payload)
{
    if ((h->client_id != TRIANGLE_DEVICE_ID) ||
        (h->direction != APP_AUDIO_LINK_DIRECTION_AP_TO_STA)) {
        return;
    }
    tri_track_rx_seq(h->seq_num);
    if ((h->packet_type == APP_AUDIO_LINK_PACKET_TYPE_CTRL) &&
        (h->data_len >= sizeof(audio_ctrl_payload_t))) {
        audio_ctrl_payload_t ctrl;
        memcpy(&ctrl, payload, sizeof(ctrl));
        if (ctrl.ctrl_type == APP_AUDIO_LINK_CTRL_UAC_MIC_STREAMING ||
            ctrl.ctrl_type == APP_AUDIO_LINK_CTRL_HEARTBEAT) {
            uint8_t new_streaming = ctrl.value ? 1U : 0U;
            if (new_streaming != s_mic_streaming) {
                s_mic_streaming = new_streaming;
                if (!s_mic_streaming) {
                    s_record_reset_pending = 1U;
                }
                dbg("TRI%u USB record request %s\n",
                    (unsigned)TRIANGLE_DEVICE_ID,
                    s_mic_streaming ? "ON" : "OFF");
                s_tx_agg_update_pending = 1U;
            }
        }
    } else if (h->packet_type == TRI_PACKET_TYPE_UAC_PCM_LOSSLESS) {
        uint64_t dec_start;
        uint64_t dec_us;
        uint32_t pre_q_ms;
        int dec;

        dec_start = tri_time_us();
        dec = tri_lossless_decode_payload(payload, h->data_len,
                                          TRI_PLAYBACK_WIRE_BYTES,
                                          TRI_PLAYBACK_FRAMES_PER_CHANNEL,
                                          s_play_lossless_decode);
        dec_us = tri_time_us() - dec_start;
        if (dec_us > s_play_lossless_decode_max_us)
            s_play_lossless_decode_max_us = (uint32_t)dec_us;
        if (dec != 0) {
            if (dec == -2) s_play_lossless_crc_fail++;
            else if (dec == -3) s_play_lossless_len_fail++;
            else s_play_lossless_decode_fail++;
            s_play_rx_drop++;
            return;
        }
        s_play_lossless_verify_ok++;
        pre_q_ms = s_rx_pcm_count / TRI_PLAYBACK_BYTES_PER_MS;
        tri_rx_queue_push(s_play_lossless_decode);
        tri_play_rx_arrival_mark(h, pre_q_ms);
        s_play_last_ap_seq = h->seq_num;
        s_play_last_ap_timestamp_ms = (uint32_t)(h->timestamp / 1000ULL);
        s_play_rx_packets++;
        s_play_lossless_comp_packets++;
        s_play_lossless_payload_bytes += h->data_len;
        s_last_play_pcm_ms = tri_now_ms();
    } else if ((h->packet_type == APP_AUDIO_LINK_PACKET_TYPE_UAC_PCM) &&
               (h->data_len == TRI_PLAYBACK_WIRE_BYTES)) {
        uint32_t pre_q_ms = s_rx_pcm_count / TRI_PLAYBACK_BYTES_PER_MS;
        tri_rx_queue_push((const int16_t *)payload);
        tri_play_rx_arrival_mark(h, pre_q_ms);
        s_play_last_ap_seq = h->seq_num;
        s_play_last_ap_timestamp_ms = (uint32_t)(h->timestamp / 1000ULL);
        s_play_rx_packets++;
        s_play_lossless_raw_packets++;
        s_play_lossless_payload_bytes += h->data_len;
        s_last_play_pcm_ms = tri_now_ms();
    }
}

/* Receive whole datagrams. The connected UDP socket filters source IP/port;
 * validate identity, direction, session ACK and both audio headers here. */
static int tri_service_udp_rx(int fd, uint8_t direction)
{
    uint8_t wire[sizeof(uac_udp_header_t) + UAC_UDP_CHUNK + 1U];
    uint32_t budget = TRI_PLAYBACK_RX_BUDGET;
    tri_udp_session_t *session = &s_udp_session[direction - 1U];
    /* A late ACK must not revive an expired lease with stale audio state. */
    if (!session->acked ||
        tri_now_ms() - session->ack_ms >= UAC_UDP_PEER_TIMEOUT_MS) return -1;
    if (tri_now_ms() - session->hello_ms >= TRI_HEARTBEAT_MS &&
        tri_udp_send_hello(fd, direction) < 0) return -1;
    while (budget-- > 0U) {
        uint32_t magic;
        audio_header_t h;
        int n = recv(fd, wire, sizeof(wire), MSG_DONTWAIT);
        if (n < 0) {
            if (tri_udp_temporary_error()) break;
            return -1;
        }
        if (tri_udp_accept_ack(wire, n, direction)) continue;
        if (direction != APP_AUDIO_LINK_DIRECTION_AP_TO_STA ||
            n < sizeof(magic)) continue;
        memcpy(&magic, wire, sizeof(magic));
        if (magic == APP_AUDIO_LINK_PACKET_MAGIC &&
            n == sizeof(h) + sizeof(audio_ctrl_payload_t)) {
            audio_ctrl_payload_t ctrl;
            memcpy(&h, wire, sizeof(h));
            memcpy(&ctrl, wire + sizeof(h), sizeof(ctrl));
            if (h.packet_type != APP_AUDIO_LINK_PACKET_TYPE_CTRL ||
                h.direction != direction || h.client_id != TRIANGLE_DEVICE_ID ||
                h.data_len != sizeof(ctrl) || ctrl.reserved ||
                (ctrl.ctrl_type != APP_AUDIO_LINK_CTRL_UAC_MIC_STREAMING &&
                 ctrl.ctrl_type != APP_AUDIO_LINK_CTRL_HEARTBEAT)) continue;
            tri_handle_rx(&h, wire + sizeof(h));
        } else if (magic == UAC_UDP_MAGIC &&
                   n > sizeof(uac_udp_header_t) && n < sizeof(wire)) {
            uac_udp_header_t uh;
            int complete;
            memcpy(&uh, wire, sizeof(uh));
            if (uh.client_id != TRIANGLE_DEVICE_ID || uh.direction != direction)
                continue;
            complete = uac_udp_assemble(&s_rx_assembly, s_rx_stream, &uh,
                wire + sizeof(uh), n - sizeof(uh), tri_now_ms());
            if (complete <= 0) continue;
            memcpy(&h, s_rx_stream, sizeof(h));
            if (h.magic != APP_AUDIO_LINK_PACKET_MAGIC ||
                h.seq_num != uh.seq || h.client_id != uh.client_id ||
                h.direction != direction || sizeof(h) + h.data_len != complete ||
                (h.packet_type != APP_AUDIO_LINK_PACKET_TYPE_UAC_PCM &&
                 h.packet_type != TRI_PACKET_TYPE_UAC_PCM_LOSSLESS)) continue;
            tri_handle_rx(&h, s_rx_stream + sizeof(h));
        }
    }
    /* Audio/control traffic alone does not renew the registration lease. */
    return !session->acked ||
        tri_now_ms() - session->ack_ms >= UAC_UDP_PEER_TIMEOUT_MS ? -1 : 0;
}

static int tri_service_playback_rx(int fd)
{
    return tri_service_udp_rx(fd, APP_AUDIO_LINK_DIRECTION_AP_TO_STA);
}

/* ================= LL3-BPK2-48PREP1-CPUDIAG1 bounded block lossless codec =================
 * Real-time oriented replacement for FAST1 whole-frame Rice coding.
 *
 * Wire v4 is frame-independent and bit-exact, but divides each channel into
 * 64-frame blocks.  Each channel block uses either:
 *   - DELTA1 + fixed bit-width packing, or
 *   - local RAW PCM when packing would not save bytes.
 *
 * There is no unary/Rice loop and no 200-frame RAW guard.  Runtime per block is
 * bounded by a small fixed number of sample operations.  A difficult/noisy
 * block falls back locally instead of forcing the whole 10 ms stream RAW.
 * The complete frame still falls back to legacy RAW if v3 would not be smaller
 * than the original PCM.  CRC32 is over the original interleaved PCM and is
 * verified after every compressed-frame decode.
 */
typedef struct {
    uint8_t *buf;
    uint32_t cap;
    uint32_t pos;
    uint32_t acc;
    uint8_t bits;
    uint8_t overflow;
} tri_bpk_bw_t;

typedef struct {
    const uint8_t *buf;
    uint32_t size;
    uint32_t pos;
    uint32_t acc;
    uint8_t bits;
} tri_bpk_br_t;

static const uint32_t tri_crc32_nib[16] = {
    0x00000000U, 0x1DB71064U, 0x3B6E20C8U, 0x26D930ACU,
    0x76DC4190U, 0x6B6B51F4U, 0x4DB26158U, 0x5005713CU,
    0xEDB88320U, 0xF00F9344U, 0xD6D6A3E8U, 0xCB61B38CU,
    0x9B64C2B0U, 0x86D3D2D4U, 0xA00AE278U, 0xBDBDF21CU
};

static inline uint32_t tri_crc32_byte(uint32_t crc, uint8_t v)
{
    crc ^= v;
    crc = (crc >> 4) ^ tri_crc32_nib[crc & 0x0FU];
    crc = (crc >> 4) ^ tri_crc32_nib[crc & 0x0FU];
    return crc;
}

static uint32_t tri_lossless_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    while (len >= 4U) {
        crc = tri_crc32_byte(crc, data[0]);
        crc = tri_crc32_byte(crc, data[1]);
        crc = tri_crc32_byte(crc, data[2]);
        crc = tri_crc32_byte(crc, data[3]);
        data += 4U;
        len -= 4U;
    }
    while (len-- != 0U) crc = tri_crc32_byte(crc, *data++);
    return ~crc;
}

static uint32_t tri_lossless_zigzag(int32_t x)
{
    return (x >= 0) ? ((uint32_t)x << 1) : (((uint32_t)(-x) << 1) - 1U);
}

static int32_t tri_lossless_unzigzag(uint32_t u)
{
    return (u & 1U) ? -(int32_t)((u + 1U) >> 1) : (int32_t)(u >> 1);
}

static uint8_t tri_bpk_width(uint32_t v)
{
    return (v == 0U) ? 0U : (uint8_t)(32U - (uint32_t)__builtin_clz(v));
}

static void tri_bpk_put_le16(uint8_t *p, int16_t v)
{
    uint16_t u = (uint16_t)v;
    p[0] = (uint8_t)(u & 0xFFU);
    p[1] = (uint8_t)(u >> 8);
}

static int16_t tri_bpk_get_le16(const uint8_t *p)
{
    return (int16_t)(uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void tri_bpk_bw_init(tri_bpk_bw_t *bw, uint8_t *buf, uint32_t cap)
{
    bw->buf = buf;
    bw->cap = cap;
    bw->pos = 0U;
    bw->acc = 0U;
    bw->bits = 0U;
    bw->overflow = 0U;
}

static inline void tri_bpk_bw_put(tri_bpk_bw_t *bw, uint32_t v, uint8_t width)
{
    if ((width == 0U) || bw->overflow) return;
    bw->acc |= v << bw->bits;
    bw->bits = (uint8_t)(bw->bits + width);
    while (bw->bits >= 8U) {
        if (bw->pos >= bw->cap) {
            bw->overflow = 1U;
            return;
        }
        bw->buf[bw->pos++] = (uint8_t)(bw->acc & 0xFFU);
        bw->acc >>= 8;
        bw->bits = (uint8_t)(bw->bits - 8U);
    }
}

static int tri_bpk_bw_finish(tri_bpk_bw_t *bw, uint32_t expected)
{
    if (bw->overflow) return -1;
    if (bw->bits != 0U) {
        if (bw->pos >= bw->cap) return -1;
        bw->buf[bw->pos++] = (uint8_t)(bw->acc & 0xFFU);
        bw->acc = 0U;
        bw->bits = 0U;
    }
    return (bw->pos == expected) ? 0 : -1;
}

static void tri_bpk_br_init(tri_bpk_br_t *br, const uint8_t *buf, uint32_t size)
{
    br->buf = buf;
    br->size = size;
    br->pos = 0U;
    br->acc = 0U;
    br->bits = 0U;
}

static inline int tri_bpk_br_get(tri_bpk_br_t *br, uint8_t width,
                                  uint32_t mask, uint32_t *v)
{
    if (width == 0U) {
        *v = 0U;
        return 0;
    }
    while (br->bits < width) {
        if (br->pos >= br->size) return -1;
        br->acc |= (uint32_t)br->buf[br->pos++] << br->bits;
        br->bits = (uint8_t)(br->bits + 8U);
    }
    *v = br->acc & mask;
    br->acc >>= width;
    br->bits = (uint8_t)(br->bits - width);
    return 0;
}

static int tri_bpk_encode_subblock(const int16_t *pcm, uint16_t start,
                                    uint16_t count, uint32_t ch,
                                    uint8_t *out, uint32_t cap,
                                    uint16_t *used, uint8_t *raw_used)
{
    uint32_t max_u = 0U;
    uint32_t i;
    uint8_t width;
    uint32_t packed_bytes;
    uint32_t comp_bytes;
    uint32_t raw_bytes;

    if ((count == 0U) || (cap == 0U)) return -1;
    for (i = 1U; i < count; i++) {
        int32_t cur = pcm[((uint32_t)start + i) * APP_AUDIO_LINK_UAC_CHANNELS + ch];
        int32_t prev = pcm[((uint32_t)start + i - 1U) * APP_AUDIO_LINK_UAC_CHANNELS + ch];
        uint32_t u = tri_lossless_zigzag(cur - prev);
        if (u > max_u) max_u = u;
    }
    width = tri_bpk_width(max_u);
    packed_bytes = (((uint32_t)count - 1U) * width + 7U) >> 3;
    comp_bytes = 1U + 2U + packed_bytes; /* descriptor + warm sample + bits */
    raw_bytes = 1U + (uint32_t)count * 2U;

    if ((width > TRI_LOSSLESS_MAX_WIDTH) || (comp_bytes >= raw_bytes)) {
        if (cap < raw_bytes) return -1;
        out[0] = TRI_LOSSLESS_DESC_RAW;
        for (i = 0U; i < count; i++) {
            tri_bpk_put_le16(out + 1U + i * 2U,
                pcm[((uint32_t)start + i) * APP_AUDIO_LINK_UAC_CHANNELS + ch]);
        }
        *used = (uint16_t)raw_bytes;
        *raw_used = 1U;
        return 0;
    } else {
        tri_bpk_bw_t bw;
        int16_t prev;
        if (cap < comp_bytes) return -1;
        out[0] = width;
        prev = pcm[(uint32_t)start * APP_AUDIO_LINK_UAC_CHANNELS + ch];
        tri_bpk_put_le16(out + 1U, prev);
        tri_bpk_bw_init(&bw, out + 3U, packed_bytes);
        for (i = 1U; i < count; i++) {
            int16_t cur = pcm[((uint32_t)start + i) * APP_AUDIO_LINK_UAC_CHANNELS + ch];
            uint32_t u = tri_lossless_zigzag((int32_t)cur - (int32_t)prev);
            tri_bpk_bw_put(&bw, u, width);
            prev = cur;
        }
        if (tri_bpk_bw_finish(&bw, packed_bytes) != 0) return -1;
        *used = (uint16_t)comp_bytes;
        *raw_used = 0U;
        return 0;
    }
}

static int tri_bpk_decode_subblock(const uint8_t *in, uint32_t avail,
                                    uint16_t start, uint16_t count,
                                    uint32_t ch, int16_t *pcm,
                                    uint16_t *used)
{
    uint8_t desc;
    uint32_t i;
    if ((count == 0U) || (avail == 0U)) return -1;
    desc = in[0];
    if (desc == TRI_LOSSLESS_DESC_RAW) {
        uint32_t bytes = 1U + (uint32_t)count * 2U;
        if (avail < bytes) return -1;
        for (i = 0U; i < count; i++) {
            pcm[((uint32_t)start + i) * APP_AUDIO_LINK_UAC_CHANNELS + ch] =
                tri_bpk_get_le16(in + 1U + i * 2U);
        }
        *used = (uint16_t)bytes;
        return 0;
    } else {
        uint8_t width = desc & TRI_LOSSLESS_DESC_WIDTH_MASK;
        uint32_t packed_bytes;
        uint32_t bytes;
        uint32_t mask;
        int16_t prev;
        tri_bpk_br_t br;
        if ((desc & 0x60U) != 0U || width > TRI_LOSSLESS_MAX_WIDTH) return -1;
        mask = (width == 0U) ? 0U : ((1U << width) - 1U);
        packed_bytes = (((uint32_t)count - 1U) * width + 7U) >> 3;
        bytes = 3U + packed_bytes;
        if (avail < bytes) return -1;
        prev = tri_bpk_get_le16(in + 1U);
        pcm[(uint32_t)start * APP_AUDIO_LINK_UAC_CHANNELS + ch] = prev;
        tri_bpk_br_init(&br, in + 3U, packed_bytes);
        for (i = 1U; i < count; i++) {
            uint32_t u;
            int32_t sample;
            if (tri_bpk_br_get(&br, width, mask, &u) != 0) return -1;
            sample = (int32_t)prev + tri_lossless_unzigzag(u);
            if ((sample < -32768) || (sample > 32767)) return -1;
            prev = (int16_t)sample;
            pcm[((uint32_t)start + i) * APP_AUDIO_LINK_UAC_CHANNELS + ch] = prev;
        }
        *used = (uint16_t)bytes;
        return 0;
    }
}

static int tri_lossless_encode_payload(const int16_t *pcm, uint16_t frames,
                                        uint16_t raw_len, uint8_t *out,
                                        uint16_t out_cap, uint16_t *out_len)
{
    tri_lossless_header_t lh;
    uint32_t pos = sizeof(lh);
    uint16_t start = 0U;
    uint8_t block_count;
    uint8_t b;

    if ((pcm == NULL) || (out == NULL) || (out_len == NULL) ||
        ((uint32_t)frames * APP_AUDIO_LINK_UAC_CHANNELS * sizeof(int16_t) != raw_len) ||
        (out_cap <= sizeof(lh))) return 1;
    block_count = (uint8_t)(((uint32_t)frames + TRI_LOSSLESS_BLOCK_FRAMES - 1U) /
                            TRI_LOSSLESS_BLOCK_FRAMES);
    lh.magic = TRI_LOSSLESS_MAGIC;
    lh.version = TRI_LOSSLESS_VERSION;
    lh.block_frames = TRI_LOSSLESS_BLOCK_FRAMES;
    lh.raw_len = raw_len;
    lh.block_count = block_count;
    lh.reserved = 0U;
    lh.raw_crc32 = 0U;

    for (b = 0U; b < block_count; b++) {
        uint16_t count = (uint16_t)(frames - start);
        uint32_t ch;
        if (count > TRI_LOSSLESS_BLOCK_FRAMES) count = TRI_LOSSLESS_BLOCK_FRAMES;
        for (ch = 0U; ch < APP_AUDIO_LINK_UAC_CHANNELS; ch++) {
            uint16_t used = 0U;
            uint8_t local_raw = 0U;
            if (tri_bpk_encode_subblock(pcm, start, count, ch, out + pos,
                                         (uint32_t)out_cap - pos,
                                         &used, &local_raw) != 0) return 1;
            (void)local_raw;
            pos += used;
            /* Abort early: whole-frame RAW is both smaller and cheaper. */
            if (pos >= raw_len) return 1;
        }
        start = (uint16_t)(start + count);
    }
    if ((start != frames) || (pos >= raw_len) || (pos > out_cap)) return 1;
    lh.raw_crc32 = tri_lossless_crc32((const uint8_t *)pcm, raw_len);
    memcpy(out, &lh, sizeof(lh));
    *out_len = (uint16_t)pos;
    return 0;
}

/* return 0=bit-exact success, -1=format/decode failure, -2=CRC mismatch,
 *       -3=decoded-length/header mismatch */
static int tri_lossless_decode_payload(const uint8_t *payload,
                                        uint16_t payload_len,
                                        uint16_t expected_raw_len,
                                        uint16_t frames, int16_t *pcm_out)
{
    tri_lossless_header_t lh;
    uint32_t pos = sizeof(lh);
    uint16_t start = 0U;
    uint8_t expected_blocks;
    uint8_t b;
    uint32_t crc;
    if ((payload == NULL) || (pcm_out == NULL) ||
        (payload_len < sizeof(lh)) ||
        ((uint32_t)frames * APP_AUDIO_LINK_UAC_CHANNELS * sizeof(int16_t) != expected_raw_len)) return -3;
    memcpy(&lh, payload, sizeof(lh));
    expected_blocks = (uint8_t)(((uint32_t)frames + TRI_LOSSLESS_BLOCK_FRAMES - 1U) /
                                TRI_LOSSLESS_BLOCK_FRAMES);
    if ((lh.magic != TRI_LOSSLESS_MAGIC) ||
        (lh.version != TRI_LOSSLESS_VERSION) ||
        (lh.block_frames != TRI_LOSSLESS_BLOCK_FRAMES) ||
        (lh.raw_len != expected_raw_len) ||
        (lh.block_count != expected_blocks)) return -3;
    if (lh.reserved != 0U) return -1;

    for (b = 0U; b < lh.block_count; b++) {
        uint16_t count = (uint16_t)(frames - start);
        uint32_t ch;
        if (count > TRI_LOSSLESS_BLOCK_FRAMES) count = TRI_LOSSLESS_BLOCK_FRAMES;
        for (ch = 0U; ch < APP_AUDIO_LINK_UAC_CHANNELS; ch++) {
            uint16_t used = 0U;
            if (pos >= payload_len) return -1;
            if (tri_bpk_decode_subblock(payload + pos, (uint32_t)payload_len - pos,
                                         start, count, ch, pcm_out, &used) != 0) return -1;
            pos += used;
        }
        start = (uint16_t)(start + count);
    }
    if ((start != frames) || (pos != payload_len)) return -1;
    crc = tri_lossless_crc32((const uint8_t *)pcm_out, expected_raw_len);
    return (crc == lh.raw_crc32) ? 0 : -2;
}
/* =========================== end LL3-BPK2-48PREP1-CPUDIAG1 codec =========================== */

static uint8_t tri_deadline_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return ((int32_t)(now_ms - deadline_ms) >= 0) ? 1U : 0U;
}

static void tri_prepare_record_frame(uint32_t now_ms)
{
    audio_header_t h;
    tri_rec20_header_t gh;
    uint8_t *payload = s_tx_wire + sizeof(h);
    uint16_t len0 = 0U;
    uint16_t len1 = 0U;
    uint16_t payload_len = 0U;
    uint8_t group20 = 0U;
    uint8_t first_lossless = 0U;
    uint64_t enc_start;
    uint64_t enc_us;
    int comp_ret;

    if (!s_tx_packet_loaded) {
        if (tri_tx_queue_pop_10ms(s_record_source_48k) != 0) return;
        s_tx_packet_loaded = 1U;
    }

#if TRI_RECORD_WIRE_SAMPLE_RATE_HZ == 16000U
#error "REC20PPS1 is intended for native 48 kHz record wire"
#endif

    /* Encode first 10 ms source block after the small REC20 container header.
     * The original BPK2 codec stays unchanged: each 10 ms subframe remains
     * independently decodable and independently CRC protected. */
    enc_start = tri_time_us();
    comp_ret = tri_lossless_encode_payload(s_record_source_48k,
                                           APP_AUDIO_LINK_RECORD_WIRE_FRAMES,
                                           APP_AUDIO_LINK_RECORD_WIRE_BYTES,
                                           payload + sizeof(gh),
                                           (uint16_t)(APP_AUDIO_LINK_RECORD_WIRE_BYTES - sizeof(gh)),
                                           &len0);
    enc_us = tri_time_us() - enc_start;
    if (enc_us > s_lossless_encode_max_us) s_lossless_encode_max_us = (uint32_t)enc_us;
    if (enc_us > TRI_LOSSLESS_RECORD_WARN_US) s_lossless_slow_hits++;
    s_lossless_encode_frames++;
    first_lossless = (comp_ret == 0) ? 1U : 0U;

    if (first_lossless &&
        (s_tx_count >= APP_AUDIO_LINK_RECORD_SOURCE_BLOCKS) &&
        (tri_tx_queue_pop_10ms(s_record_source_48k) == 0)) {
        uint16_t cap1 = (uint16_t)(APP_AUDIO_LINK_RECORD_WIRE_BYTES - sizeof(gh) - len0);
        enc_start = tri_time_us();
        comp_ret = tri_lossless_encode_payload(s_record_source_48k,
                                               APP_AUDIO_LINK_RECORD_WIRE_FRAMES,
                                               APP_AUDIO_LINK_RECORD_WIRE_BYTES,
                                               payload + sizeof(gh) + len0,
                                               cap1,
                                               &len1);
        enc_us = tri_time_us() - enc_start;
        if (enc_us > s_lossless_encode_max_us) s_lossless_encode_max_us = (uint32_t)enc_us;
        if (enc_us > TRI_LOSSLESS_RECORD_WARN_US) s_lossless_slow_hits++;
        s_lossless_encode_frames++;

        if ((comp_ret == 0) &&
            ((uint32_t)sizeof(gh) + len0 + len1 <= APP_AUDIO_LINK_RECORD_WIRE_BYTES)) {
            gh.magic = TRI_REC20_MAGIC;
            gh.version = TRI_REC20_VERSION;
            gh.subframes = TRI_RECORD_GROUP_SUBFRAMES;
            gh.len0 = len0;
            gh.len1 = len1;
            memcpy(payload, &gh, sizeof(gh));
            payload_len = (uint16_t)(sizeof(gh) + len0 + len1);
            group20 = 1U;
        } else {
            /* Preserve the second 10 ms block and send the already encoded
             * first block using the mature 10 ms packet format.  This keeps
             * transport lossless even for incompressible/noisy material. */
            tri_tx_queue_push_front_10ms(s_record_source_48k);
        }
    }

    if (group20) {
        h.packet_type = TRI_PACKET_TYPE_UAC_PCM_LOSSLESS20;
        s_tx_current_lossless = 1U;
        s_tx_current_group_ms = 20U;
    } else if (first_lossless) {
        /* First BPK2 payload currently starts after the REC20 header; compact
         * it to the normal payload position for legacy 10 ms fallback. */
        memmove(payload, payload + sizeof(gh), len0);
        payload_len = len0;
        h.packet_type = TRI_PACKET_TYPE_UAC_PCM_LOSSLESS;
        s_tx_current_lossless = 1U;
        s_tx_current_group_ms = 10U;
    } else {
        payload_len = APP_AUDIO_LINK_RECORD_WIRE_BYTES;
        memcpy(payload, s_record_source_48k, payload_len);
        h.packet_type = APP_AUDIO_LINK_PACKET_TYPE_UAC_PCM;
        s_tx_current_lossless = 0U;
        s_tx_current_group_ms = 10U;
    }

    h.magic = APP_AUDIO_LINK_PACKET_MAGIC;
    h.seq_num = s_tx_seq++;
    h.timestamp = tri_time_us();
    h.direction = APP_AUDIO_LINK_DIRECTION_STA_TO_AP;
    h.client_id = TRIANGLE_DEVICE_ID;
    h.data_len = payload_len;
    memcpy(s_tx_wire, &h, sizeof(h));
    s_tx_len = (uint16_t)(sizeof(h) + payload_len);
    s_tx_off = 0U;
    s_tx_current_pcm = 1U;
    s_tx_current_payload_len = payload_len;
    /* Pace from frame start: sending fragment two on the next task tick must
     * not add 1 ms to every 10/20 ms audio interval. */
    s_next_audio_tx_ms = now_ms + s_tx_current_group_ms;
    s_tx_blocked_since_ms = now_ms;
    s_tx_blocked = 0U;
}

static int tri_service_record_tx(int fd, uint32_t now_ms)
{
    int n;
    if (s_tx_len == 0U) {
        if (s_mic_streaming &&
            (s_tx_packet_loaded ||
             (s_tx_count >= (APP_AUDIO_LINK_RECORD_SOURCE_BLOCKS * TRI_RECORD_GROUP_SUBFRAMES)))) {
            if ((s_next_audio_tx_ms != 0U) &&
                !tri_deadline_reached(now_ms, s_next_audio_tx_ms)) {
                return 0;
            }
            tri_prepare_record_frame(now_ms);
        } else {
            return 0;
        }
    }
    {
        uint8_t wire[sizeof(uac_udp_header_t) + UAC_UDP_CHUNK];
        uac_udp_header_t uh;
        audio_header_t h;
        uint32_t bytes = s_tx_len - s_tx_off;
        if (!s_tx_len || s_tx_len > UAC_UDP_MAX_FRAME ||
            s_tx_off >= s_tx_len || s_tx_off % UAC_UDP_CHUNK) return -1;
        if (bytes > UAC_UDP_CHUNK) bytes = UAC_UDP_CHUNK;
        memcpy(&h, s_tx_wire, sizeof(h));
        uh.magic = UAC_UDP_MAGIC;
        uh.seq = h.seq_num;
        uh.total = s_tx_len;
        uh.offset = s_tx_off;
        uh.client_id = TRIANGLE_DEVICE_ID;
        uh.direction = APP_AUDIO_LINK_DIRECTION_STA_TO_AP;
        uh.reserved = 0U;
        memcpy(wire, &uh, sizeof(uh));
        memcpy(wire + sizeof(uh), s_tx_wire + s_tx_off, bytes);
        n = send(fd, wire, sizeof(uh) + bytes, MSG_DONTWAIT);
        if (n >= 0) {
            if ((uint32_t)n != sizeof(uh) + bytes) return -1;
            n = (int)bytes;
        }
    }
    now_ms = tri_now_ms();
    if (n > 0) {
        s_tx_off = (uint16_t)(s_tx_off + (uint16_t)n);
        s_tx_blocked_since_ms = now_ms;
        s_tx_blocked = 0U;
        if (s_tx_off >= s_tx_len) {
            if (s_tx_current_pcm) {
                s_record_tx_packets++;
                if (s_tx_current_lossless) s_lossless_comp_packets++;
                else s_lossless_raw_packets++;
                if (s_tx_current_group_ms == 20U) s_record_tx_20ms_packets++;
                else s_record_tx_10ms_fallback_packets++;
                s_record_tx_raw_equiv_bytes += (s_tx_current_group_ms == 20U) ?
                    TRI_RECORD_GROUP_RAW_BYTES : APP_AUDIO_LINK_RECORD_WIRE_BYTES;
                s_lossless_payload_bytes += s_tx_current_payload_len;
                if (s_tx_current_payload_len < s_lossless_payload_min)
                    s_lossless_payload_min = s_tx_current_payload_len;
                if (s_tx_current_payload_len > s_lossless_payload_max)
                    s_lossless_payload_max = s_tx_current_payload_len;
                s_tx_packet_loaded = 0U;
            }
            s_tx_current_pcm = 0U;
            s_tx_current_lossless = 0U;
            s_tx_current_group_ms = 0U;
            s_tx_current_payload_len = 0U;
            s_tx_len = s_tx_off = 0U;
            s_last_tx_ms = now_ms;
        }
        return 0;
    }
    if ((n < 0) && ((errno == EAGAIN) || (errno == EWOULDBLOCK) ||
                      (errno == ENOMEM)
#ifdef ENOBUFS
                      || (errno == ENOBUFS)
#endif
                      )) {
        int32_t blocked_delta;
        int temp_errno = errno;
        uint32_t stall_ms;
        uint8_t pcm_frame = s_tx_current_pcm;

        if ((temp_errno == EAGAIN) || (temp_errno == EWOULDBLOCK)) {
            s_record_eagain++;
        } else if (temp_errno == ENOMEM) {
            s_record_enomem++;
#ifdef ENOBUFS
        } else if (temp_errno == ENOBUFS) {
            s_record_enobufs++;
#endif
        }

        /* R10: start the stale timer at the first temporary send failure.
         * R9 compared an outer now_ms with a newer timestamp taken while the
         * frame was prepared.  Crossing a 1 ms boundary produced unsigned
         * underflow (0xffffffff ms) and immediately discarded fresh frames. */
        if (!s_tx_blocked) {
            s_tx_blocked = 1U;
            s_tx_blocked_since_ms = now_ms;
            return 0;
        }
        blocked_delta = (int32_t)(now_ms - s_tx_blocked_since_ms);
        stall_ms = (blocked_delta > 0) ? (uint32_t)blocked_delta : 0U;

        /* Do not reset the whole record socket merely because no complete
         * frame finished for 200 ms.  The 20 ms zero-byte stale-drop below
         * already bounds local latency and safely advances to fresh audio.
         * Reconnecting here caused the observed STA2 close/reconnect storm. */

        if (stall_ms >= TRI_UNSENT_STALE_DROP_MS) {
            uint32_t trimmed = 0U;

            /* Drop the entire stale UDP frame, including an unsent second
             * fragment. The AP expires partial assembly and sees a seq gap. */
            if (pcm_frame) {
                s_record_tx_drop++;
                s_record_stale_drop++;
                s_tx_packet_loaded = 0U;
                trimmed = tri_tx_queue_trim_to_latest(TRI_TX_STALE_KEEP_BLOCKS);
                if (trimmed != 0U) {
                    tri_record_decimator_reset();
                }
            }

            s_tx_len = 0U;
            s_tx_off = 0U;
            s_tx_current_pcm = 0U;
            s_tx_current_lossless = 0U;
            s_tx_current_group_ms = 0U;
            s_tx_current_payload_len = 0U;
            s_tx_blocked_since_ms = now_ms;
            s_tx_blocked = 0U;
            s_next_audio_tx_ms = now_ms + TRI_RECORD_FRAME_PACE_MS;
            if (!pcm_frame) {
                /* Avoid rebuilding the same blocked heartbeat every tick. */
                s_last_tx_ms = now_ms;
            }

            if ((s_record_last_fault_log_ms == 0U) ||
                ((now_ms - s_record_last_fault_log_ms) >= 1000U)) {
                dbg("TRI%u TXSTALE drop=%u trim=%u q=%u keep=%u stall=%u ms partial_reset=%u temp(eagain/enomem/enobufs)=%u/%u/%u\n",
                    (unsigned)TRIANGLE_DEVICE_ID,
                    (unsigned)s_record_stale_drop,
                    (unsigned)s_record_trim_drop,
                    (unsigned)s_tx_count,
                    (unsigned)TRI_TX_STALE_KEEP_BLOCKS,
                    (unsigned)stall_ms,
                    (unsigned)s_record_partial_reset,
                    (unsigned)s_record_eagain,
                    (unsigned)s_record_enomem,
                    (unsigned)s_record_enobufs);
                s_record_last_fault_log_ms = now_ms;
            }
            (void)trimmed;
            return 0;
        }

        return 0;
    }
    s_tx_blocked_since_ms = 0U;
    s_tx_blocked = 0U;
    return -1;
}

static uint8_t tri_apply_record_tx_agg_mode(uint8_t disable)
{
#if PLF_WIFI_STACK && (PLF_AIC8800MC || PLF_AIC8800M40 || defined(CFG_WIFI_RAM_VER))
    uint8_t sta_idx = fhost_vif_get_staid(0, NULL);
    if (sta_idx == 0xFFU) {
        return 0U;
    }
    /* R11: always keep record TX AMPDU enabled.  R10 forced 200 TCP audio
     * frames/s per STA through the non-AMPDU path.  With two STAs this caused
     * periodic descriptor/ACK pressure, true 20 ms stalls and roughly forty
     * stale drops per second during each blocked interval. */
    (void)disable;
    (void)rwnx_set_disable_agg_req(0U, 0U, sta_idx);
#else
    (void)disable;
#endif
    return 1U;
}

static int tri_wait_wifi_ready(void)
{
#if PLF_WIFI_STACK
#if (PLF_AIC8800)
    const int ready = 0;
#elif (PLF_AIC8800MC) || (PLF_AIC8800M40)
    const int ready = IPC_EMB_START;
#else
    const int ready = 0;
#endif
    while (wifi_get_init_status() != ready) {
        rtos_task_suspend(10U);
    }
#endif
    return 0;
}

static void tri_log_record48_check(void)
{
    uint32_t total = s_lossless_comp_packets + s_lossless_raw_packets;
    uint32_t avg = total ? (s_lossless_payload_bytes / total) : 0U;
    uint32_t ratio_x10 = s_record_tx_raw_equiv_bytes ?
        (uint32_t)(((uint64_t)s_lossless_payload_bytes * 1000ULL) /
                   (uint64_t)s_record_tx_raw_equiv_bytes) : 0U;

    /* REC20PPS1 performance log only.  Expensive STA local encode->decode
     * memcmp is removed from the realtime path.  Every BPK2 subframe still
     * carries the original PCM CRC32 and the AP verifies it on decode. */
    dbg("TRI%u REC20 raw20=%uB wire_packets=%u group20=%u fallback10=%u comp/raw=%u/%u avg=%uB ratio=%u.%u%% AP_CRC_VERIFY=ON local_memcmp=OFF encmax=%uus\n",
        (unsigned)TRIANGLE_DEVICE_ID,
        (unsigned)TRI_RECORD_GROUP_RAW_BYTES,
        (unsigned)s_record_tx_packets,
        (unsigned)s_record_tx_20ms_packets,
        (unsigned)s_record_tx_10ms_fallback_packets,
        (unsigned)s_lossless_comp_packets,
        (unsigned)s_lossless_raw_packets,
        (unsigned)avg,
        (unsigned)(ratio_x10 / 10U),
        (unsigned)(ratio_x10 % 10U),
        (unsigned)s_lossless_encode_max_us);
}

static void tri_log_lossless_snapshot(void)
{
    uint32_t play_q_ms;
    uint32_t play_min_ms;
    uint32_t play_total = s_play_lossless_comp_packets + s_play_lossless_raw_packets;
    uint32_t play_avg = play_total ? (s_play_lossless_payload_bytes / play_total) : 0U;
    uint32_t play_ratio_x10 = play_total ?
        (uint32_t)(((uint64_t)s_play_lossless_payload_bytes * 1000ULL) /
                   ((uint64_t)play_total * TRI_PLAYBACK_WIRE_BYTES)) : 0U;
    uint32_t rec_total = s_lossless_comp_packets + s_lossless_raw_packets;
    uint32_t rec_avg = rec_total ? (s_lossless_payload_bytes / rec_total) : 0U;
    uint32_t rec_ratio_x10 = rec_total ?
        (uint32_t)(((uint64_t)s_lossless_payload_bytes * 1000ULL) /
                   ((uint64_t)rec_total * APP_AUDIO_LINK_RECORD_WIRE_BYTES)) : 0U;

    tri_playbuf_diag_snapshot(&play_q_ms, &play_min_ms);

    dbg("TRI%u PLAYLOSS-SAFE recv(comp/raw)=%u/%u avg=%uB ratio=%u.%u%% raw/dec=1920/1920 match=%u crc/dec/len_fail=%u/%u/%u dec_wall_max=%uus cpu_est_max=%uus probe=%u pfail=%u q=%ums min=%ums uf=%u rb=%u ovf=%u tl=%u/%u/%u cons=%uf popgap=%uus rxgap=%uus fade_out/in=%u/%u\n",
        (unsigned)TRIANGLE_DEVICE_ID,
        (unsigned)s_play_lossless_comp_packets,
        (unsigned)s_play_lossless_raw_packets,
        (unsigned)play_avg,
        (unsigned)(play_ratio_x10 / 10U),
        (unsigned)(play_ratio_x10 % 10U),
        (unsigned)s_play_lossless_verify_ok,
        (unsigned)s_play_lossless_crc_fail,
        (unsigned)s_play_lossless_decode_fail,
        (unsigned)s_play_lossless_len_fail,
        (unsigned)s_play_lossless_decode_max_us,
        (unsigned)s_play_lossless_probe_est_max_us,
        (unsigned)s_play_lossless_probe_count,
        (unsigned)s_play_lossless_probe_fail,
        (unsigned)play_q_ms,
        (unsigned)play_min_ms,
        (unsigned)s_play_underflow_events,
        (unsigned)s_play_rebuffer_events,
        (unsigned)s_play_overflow_events,
        (unsigned)s_play_tl_slow_blocks,
        (unsigned)s_play_tl_normal_blocks,
        (unsigned)s_play_tl_fast_blocks,
        (unsigned)s_play_tl_last_source_frames,
        (unsigned)s_play_pop_gap_max_total_us,
        (unsigned)s_play_rx_gap_max_total_us,
        (unsigned)s_play_fade_out_events,
        (unsigned)s_play_fade_in_events);

    dbg("TRI%u RECLOSS-SAFE sent(comp/raw)=%u/%u avg=%uB ratio=%u.%u%% min=%u max=%u selftest(pass/fail)=%u/%u enc_wall_max=%uus cpu_est_max=%uus probe=%u pfail=%u slow_wall(>%uus)=%u\n",
        (unsigned)TRIANGLE_DEVICE_ID,
        (unsigned)s_lossless_comp_packets,
        (unsigned)s_lossless_raw_packets,
        (unsigned)rec_avg,
        (unsigned)(rec_ratio_x10 / 10U),
        (unsigned)(rec_ratio_x10 % 10U),
        (unsigned)((s_lossless_payload_min == 0xFFFFU) ? 0U : s_lossless_payload_min),
        (unsigned)s_lossless_payload_max,
        (unsigned)s_lossless_selftest_pass,
        (unsigned)s_lossless_selftest_fail,
        (unsigned)s_lossless_encode_max_us,
        (unsigned)s_lossless_probe_est_max_us,
        (unsigned)s_lossless_probe_count,
        (unsigned)s_lossless_probe_fail,
        (unsigned)TRI_LOSSLESS_RECORD_WARN_US,
        (unsigned)s_lossless_slow_hits);
}

static void tri_record_task_fn(void *param)
{
    int record_fd = -1;
    uint8_t record_seen = 0U;
    uint8_t record_down = 0U;

    (void)param;
    s_record_running = 1U;
    dbg("TRI%u RECORD mature-pipeline task prio=%u sndbuf=%u pace=%ums high=%ums keep=%ums stale=%ums partial=%ums\n",
        (unsigned)TRIANGLE_DEVICE_ID,
        (unsigned)TRI_RECORD_TASK_PRIO,
        (unsigned)TRI_RECORD_SOCKET_BUF_BYTES,
        (unsigned)TRI_RECORD_FRAME_PACE_MS,
        (unsigned)(TRI_TX_HIGH_WATER_BLOCKS * 10U),
        (unsigned)(TRI_TX_KEEP_BLOCKS * 10U),
        (unsigned)TRI_UNSENT_STALE_DROP_MS,
        (unsigned)TRI_PARTIAL_STALL_MS);

    while (!s_stop) {
        uint32_t now_ms;

        if (!s_wifi_connected || !wlan_get_connect_status()) {
            if (record_fd >= 0) {
                tri_close(&record_fd);
                s_record_connected = 0U;
                record_down = record_seen;
                tri_tx_queue_reset();
            }
            rtos_task_suspend(10U);
            continue;
        }

        if (s_tx_agg_update_pending) {
            if (tri_apply_record_tx_agg_mode(0U)) {
                s_tx_agg_update_pending = 0U;
                s_tx_agg_disabled = 0U;
                dbg("TRI%u WIFI record TX AMPDU enabled\n",
                    (unsigned)TRIANGLE_DEVICE_ID);
            }
        }

        if (record_fd < 0) {
            record_fd = tri_connect(APP_AUDIO_LINK_UAC_RETURN_SERVER_PORT,
                                    "record-tx");
            if ((record_fd >= 0) &&
                (tri_send_hello(record_fd,
                                APP_AUDIO_LINK_DIRECTION_STA_TO_AP) == 0)) {
                s_record_connected = 1U;
                tri_tx_queue_reset();
                s_tx_seq = 1U;
                s_last_tx_ms = tri_now_ms();
                if (!record_seen) {
                    dbg("TRI%u READY record connected\n",
                        (unsigned)TRIANGLE_DEVICE_ID);
                } else if (record_down) {
                    dbg("TRI%u RECOVER record reconnected\n",
                        (unsigned)TRIANGLE_DEVICE_ID);
                }
                record_seen = 1U;
                record_down = 0U;
            } else {
                tri_close(&record_fd);
                s_record_connected = 0U;
                rtos_task_suspend(TRI_RECORD_RETRY_MS);
                continue;
            }
        }

        now_ms = tri_now_ms();
        if (s_record_reset_pending) {
            s_record_reset_pending = 0U;
            tri_tx_queue_reset();
        }
        if ((record_fd >= 0) &&
            ((tri_service_udp_rx(record_fd, APP_AUDIO_LINK_DIRECTION_STA_TO_AP) != 0) ||
             (tri_service_record_tx(record_fd, now_ms) != 0))) {
            tri_close(&record_fd);
            s_record_connected = 0U;
            tri_tx_queue_reset();
            s_reconnects++;
            if (!record_down) {
                dbg("TRI%u WARN record disconnected\n",
                    (unsigned)TRIANGLE_DEVICE_ID);
                record_down = 1U;
            }
            rtos_task_suspend(TRI_RECORD_RETRY_MS);
            continue;
        }

        rtos_task_suspend(1U);
    }

    tri_close(&record_fd);
    s_record_connected = 0U;
    s_record_running = 0U;
    s_record_task = NULL;
    rtos_task_delete(NULL);
}

static void tri_network_task(void *param)
{
    int play_fd = -1;
    uint32_t last_audio_ms = 0U;
    uint32_t prev_play = 0U;
    uint32_t prev_play_drop = 0U;
    uint32_t prev_play_gap = 0U;
    uint32_t prev_record = 0U;
    uint32_t prev_record_drop = 0U;
    uint32_t prev_play_uf = 0U;
    uint32_t prev_play_rb = 0U;
    uint32_t prev_play_ovf = 0U;
    uint32_t prev_tl_slow = 0U;
    uint32_t prev_tl_normal = 0U;
    uint32_t prev_tl_fast = 0U;
    uint8_t prev_play_state = 0xFFU;
    uint8_t prev_record_state = 0xFFU;
    uint8_t diag_run_seen = 0U;
    uint32_t last_llchk_ms = 0U;
    uint8_t wifi_seen = 0U;
    uint8_t play_seen = 0U;
    uint8_t play_down = 0U;

    (void)param;
    s_running = 1U;
    (void)tri_wait_wifi_ready();
    us_ticker_init();
    dbg("TRI%u STA %s ready: playback=48k stereo 10ms/%uB/%upps, record wire=%uk stereo REC20 20ms-group\n",
        (unsigned)TRIANGLE_DEVICE_ID, TRI_VERSION,
        (unsigned)TRI_PLAYBACK_WIRE_BYTES, (unsigned)TRI_PLAY_EXPECTED_PPS,
        (unsigned)(TRI_RECORD_WIRE_SAMPLE_RATE_HZ / 1000U));
    dbg("TRI%u PLAYBACK P10 packet=%ums pcm=%uB pps=%u rxring=%uB prebuffer=%ums timelock=%u-%ums threshold=packet_bytes adjust=%uf fade=%ums\n",
        (unsigned)TRIANGLE_DEVICE_ID,
        (unsigned)TRI_PLAYBACK_PACKET_MS,
        (unsigned)TRI_PLAYBACK_WIRE_BYTES,
        (unsigned)TRI_PLAY_EXPECTED_PPS,
        (unsigned)sizeof(s_rx_pcm_ring),
        (unsigned)(TRI_RX_PREBUFFER_PACKETS * TRI_PLAYBACK_PACKET_MS),
        (unsigned)(TRI_RX_TIMELOCK_LOW_PACKETS * TRI_PLAYBACK_PACKET_MS),
        (unsigned)(TRI_RX_TIMELOCK_HIGH_PACKETS * TRI_PLAYBACK_PACKET_MS),
        (unsigned)TRI_RX_TIMELOCK_ADJUST_FRAMES,
        (unsigned)(TRI_PLAYBACK_FADE_FRAMES * 1000U / 48000U));
    dbg("TRI%u RXBYP1 record_tx_ampdu=ON full_duplex_disable=0 AP_reord_pressure_bypass=1\n",
        (unsigned)TRIANGLE_DEVICE_ID);
    dbg("TRI%u LL3-BPK2-48PREP1-CPUDIAG3-UARTSAFE2-PBUF3 bidir_lossless=delta1+64frame-bitpack-fast packet_type=0x%02x play_decode=%uB record_encode=%uB raw_fallback=1 crc32_verify_each_rx=1\n",
        (unsigned)TRIANGLE_DEVICE_ID,
        (unsigned)TRI_PACKET_TYPE_UAC_PCM_LOSSLESS,
        (unsigned)TRI_PLAYBACK_WIRE_BYTES,
        (unsigned)APP_AUDIO_LINK_RECORD_WIRE_BYTES);
    dbg("TRI%u LL3-BPK2-48PREP1-CPUDIAG3-UARTSAFE2-PBUF3 record_selftest local_memcmp first=%u then_every=%u; playback decoded_len=%uB crc_fail must stay 0\n",
        (unsigned)TRIANGLE_DEVICE_ID,
        (unsigned)TRI_LOSSLESS_SELFTEST_START_FRAMES,
        (unsigned)TRI_LOSSLESS_SELFTEST_EVERY,
        (unsigned)TRI_PLAYBACK_WIRE_BYTES);
    dbg("TRI%u REC20PPS1 current_record=%uHz/%uB direct48k=1 codec_block=%u wire_v=%u\n",
        (unsigned)TRIANGLE_DEVICE_ID,
        (unsigned)TRI_RECORD_WIRE_SAMPLE_RATE_HZ,
        (unsigned)APP_AUDIO_LINK_RECORD_WIRE_BYTES,
        (unsigned)TRI_LOSSLESS_BLOCK_FRAMES,
        (unsigned)TRI_LOSSLESS_VERSION);
    dbg("TRI%u CPUDIAG3-UARTSAFE2-PBUF3 wall=authoritative_task_latency codec_probe=OFF long_uart=dump_after_stream_off irq_mask=0 priority_change=0\n",
        (unsigned)TRIANGLE_DEVICE_ID);
    dbg("TRI%u HOSTBUF2-JCTRL3-NETDIAG2-PLAYFIX2-REC20PPS1-BUF20 playback_logic=PBUF3 timelock_adjust=%uf threshold_bytes=%u/%u prebuffer=%ums ring=%ums ovf_diag=ON pop_gap_diag=ON rx_gap_diag=ON fw_debug_mask=0x0000 AP_record_rcvbuf=2048B AP_record_prio=4\n",
        (unsigned)TRIANGLE_DEVICE_ID,
        (unsigned)TRI_RX_TIMELOCK_ADJUST_FRAMES,
        (unsigned)TRI_RX_TIMELOCK_LOW_BYTES,
        (unsigned)TRI_RX_TIMELOCK_HIGH_BYTES,
        (unsigned)(TRI_RX_PREBUFFER_PACKETS * TRI_PLAYBACK_PACKET_MS),
        (unsigned)(TRI_RX_QUEUE_PACKETS * TRI_PLAYBACK_PACKET_MS));
    dbg("TRI%u NETDIAG2-PLAYFIX2-REC20PPS1-BUF20 audio_fields=t_sta/ap_apclk/seq_ap_pcm + rxgap_pair(apgap/seqgap/extra/hq); hole_threshold=%uus buffer=120ms prebuffer=90ms timelock=80-90ms underflow_fade=3ms\n",
        (unsigned)TRIANGLE_DEVICE_ID, (unsigned)TRI_RXHOLE_WARN_US);
    dbg("TRI%u UARTSAFE2 audio_log=%ums llchk=%ums pbuf_diag=q/min_win/uf/rb/ovf+tl_slow_normal_fast/cons/pop_gap/rx_gap long_lossless=after_stream_off\n",
        (unsigned)TRIANGLE_DEVICE_ID,
        (unsigned)TRI_AUDIO_LOG_MS,
        (unsigned)TRI_LLCHK_LOG_MS);
    dbg("TRI%u R18 rollback_qos=1 reorder=20ms time_lock=1 record_wire=48k_stereo_16bit_REC20 raw_group=3840B subframe=%uB target_pps=%u decim_fir=OFF legacy_taps=%u split_tasks=1 app_prio(play/record)=%u/%u wifi_tcpip=3 wifi_ipc_tx=4 record_pace=%ums high=%ums keep=%ums stale_drop=%ums partial_reset=%ums record_tx_ampdu=ON record_sndbuf=%u first_eagain_timer=1 underflow_guard=1 loaded_packet_fix=1 stale_retry_paced=1 playback_budget=%u\n",
        (unsigned)TRIANGLE_DEVICE_ID,
        (unsigned)APP_AUDIO_LINK_RECORD_WIRE_BYTES,
        (unsigned)TRI_RECORD_EXPECTED_PPS,
        (unsigned)TRI_RECORD_DECIM_TAPS,
        (unsigned)TRI_TASK_PRIO,
        (unsigned)TRI_RECORD_TASK_PRIO,
        (unsigned)TRI_RECORD_FRAME_PACE_MS,
        (unsigned)(TRI_TX_HIGH_WATER_BLOCKS * 10U),
        (unsigned)(TRI_TX_KEEP_BLOCKS * 10U),
        (unsigned)TRI_UNSENT_STALE_DROP_MS,
        (unsigned)TRI_PARTIAL_STALL_MS,
        (unsigned)TRI_RECORD_SOCKET_BUF_BYTES,
        (unsigned)TRI_PLAYBACK_RX_BUDGET);

    while (!s_stop) {
        uint32_t now_ms;

        if (!wlan_get_connect_status()) {
            if (s_wifi_connected) {
                dbg("TRI%u WARN WiFi disconnected\n",
                    (unsigned)TRIANGLE_DEVICE_ID);
                tri_close(&play_fd);
                s_play_connected = 0U;
                s_mic_streaming = 0U;
                s_record_reset_pending = 1U;
                tri_rx_queue_reset();
                s_wifi_connected = 0U;
                play_down = play_seen;
            }

            if (wlan_start_sta((uint8_t *)APP_AUDIO_LINK_SSID,
                               (uint8_t *)APP_AUDIO_LINK_PASSWORD,
                               TRI_CONNECT_TIMEOUT_MS) != 0) {
                rtos_task_suspend(TRI_RETRY_MS);
                continue;
            }
            s_wifi_connected = 1U;
            user_sleep_allow(0);
            s_tx_agg_update_pending = 1U;
            dbg(wifi_seen ? "TRI%u RECOVER WiFi reconnected\n" :
                            "TRI%u READY WiFi connected\n",
                (unsigned)TRIANGLE_DEVICE_ID);
            wifi_seen = 1U;
        }

        /* Wi-Fi may already have been joined by the SDK startup path. */
        if (!s_wifi_connected && wlan_get_connect_status()) {
            s_wifi_connected = 1U;
            user_sleep_allow(0);
            s_tx_agg_update_pending = 1U;
        }

        if (play_fd < 0) {
            play_fd = tri_connect(APP_AUDIO_LINK_UAC_SERVER_PORT, "playback-rx");
            if ((play_fd >= 0) &&
                (tri_send_hello(play_fd,
                                APP_AUDIO_LINK_DIRECTION_AP_TO_STA) == 0)) {
                s_play_connected = 1U;
                tri_rx_queue_reset();
                if (!play_seen) {
                    dbg("TRI%u READY playback connected\n",
                        (unsigned)TRIANGLE_DEVICE_ID);
                } else if (play_down) {
                    dbg("TRI%u RECOVER playback reconnected\n",
                        (unsigned)TRIANGLE_DEVICE_ID);
                }
                play_seen = 1U;
                play_down = 0U;
            } else {
                tri_close(&play_fd);
                s_play_connected = 0U;
            }
        }

        if ((play_fd >= 0) && (tri_service_playback_rx(play_fd) != 0)) {
            tri_close(&play_fd);
            s_play_connected = 0U;
            s_mic_streaming = 0U;
            s_record_reset_pending = 1U;
            tri_rx_queue_reset();
            s_reconnects++;
            if (!play_down) {
                dbg("TRI%u WARN playback disconnected\n",
                    (unsigned)TRIANGLE_DEVICE_ID);
                play_down = 1U;
            }
        }

        now_ms = tri_now_ms();
        {
            uint8_t play_stream_on =
                (s_play_connected && (s_last_play_pcm_ms != 0U) &&
                 ((now_ms - s_last_play_pcm_ms) < TRI_PLAY_STREAM_IDLE_MS)) ?
                1U : 0U;
            uint8_t play_state = !s_play_connected ? TRI_FLOW_DOWN :
                (play_stream_on ? TRI_FLOW_ACTIVE : TRI_FLOW_OFF);
            uint8_t record_state = !s_mic_streaming ? TRI_FLOW_OFF :
                (s_record_connected ? TRI_FLOW_ACTIVE : TRI_FLOW_DOWN);
            uint8_t state_changed =
                ((play_state != prev_play_state) ||
                 (record_state != prev_record_state)) ? 1U : 0U;
            uint8_t any_stream_on = (play_stream_on || s_mic_streaming) ? 1U : 0U;

            /* UARTSAFE2: keep the real-time tasks free of long printf while
             * audio is active.  Dump the accumulated codec snapshot only
             * after both playback and record streaming have gone idle. */
            if (any_stream_on) {
                diag_run_seen = 1U;
            } else if (diag_run_seen) {
                tri_log_lossless_snapshot();
                diag_run_seen = 0U;
            }

            /* One short line every 61 s exposes both bit-exact playback
             * integrity and actual playback-buffer health.  It replaces the
             * old long periodic PLAYLOSS dump and is deliberately de-phased
             * from the 5 s AUDIO status and 997-frame codec probe. */
            if (play_stream_on) {
                if (last_llchk_ms == 0U) {
                    last_llchk_ms = now_ms;
                } else if ((now_ms - last_llchk_ms) >= TRI_LLCHK_LOG_MS) {
                    tri_log_short_play_check();
                    last_llchk_ms = now_ms;
                }
            } else {
                last_llchk_ms = 0U;
            }

            if ((last_audio_ms == 0U) || state_changed) {
                prev_play = s_play_rx_packets;
                prev_play_drop = s_play_rx_drop;
                prev_play_gap = s_play_seq_gap;
                prev_record = s_record_tx_packets;
                prev_record_drop = s_record_tx_drop;
                prev_play_uf = s_play_underflow_events;
                prev_play_rb = s_play_rebuffer_events;
                prev_play_ovf = s_play_overflow_events;
                prev_tl_slow = s_play_tl_slow_blocks;
                prev_tl_normal = s_play_tl_normal_blocks;
                prev_tl_fast = s_play_tl_fast_blocks;
                if (play_state == TRI_FLOW_ACTIVE) {
                    tri_playbuf_window_reset();
                    tri_play_pop_gap_window_reset();
                    tri_play_rx_gap_window_reset();
                    s_play_pop_timing_active = 1U;
                    s_play_rx_timing_active = 1U;
                } else {
                    s_play_pop_timing_active = 0U;
                    s_play_rx_timing_active = 0U;
                    tri_play_pop_gap_window_reset();
                    tri_play_rx_gap_window_reset();
                }
                prev_play_state = play_state;
                prev_record_state = record_state;
                last_audio_ms = now_ms;
            } else if (any_stream_on &&
                       ((now_ms - last_audio_ms) >= TRI_AUDIO_LOG_MS)) {
                uint32_t elapsed_ms = now_ms - last_audio_ms;
                uint32_t play_done = tri_rate_per_second(
                    s_play_rx_packets - prev_play, elapsed_ms);
                uint32_t play_loss = tri_rate_per_second(
                    (s_play_rx_drop - prev_play_drop) +
                    (s_play_seq_gap - prev_play_gap), elapsed_ms);
                uint32_t record_done = tri_rate_per_second(
                    s_record_tx_packets - prev_record, elapsed_ms);
                uint32_t record_loss = tri_rate_per_second(
                    s_record_tx_drop - prev_record_drop, elapsed_ms);
                uint32_t play_actual = tri_effective_packets(
                    play_done, play_loss, TRI_PLAY_EXPECTED_PPS);
                uint32_t record_actual = tri_effective_packets(
                    record_done, record_loss, TRI_RECORD_EXPECTED_PPS);

                uint32_t play_q_ms = 0U;
                uint32_t play_min_win_ms = 0U;
                uint32_t uf_delta = s_play_underflow_events - prev_play_uf;
                uint32_t rb_delta = s_play_rebuffer_events - prev_play_rb;
                uint32_t ovf_delta = s_play_overflow_events - prev_play_ovf;
                uint32_t tl_slow_delta = s_play_tl_slow_blocks - prev_tl_slow;
                uint32_t tl_normal_delta = s_play_tl_normal_blocks - prev_tl_normal;
                uint32_t tl_fast_delta = s_play_tl_fast_blocks - prev_tl_fast;
                uint32_t pop_gap_max_us = 0U;
                uint32_t rx_gap_max_us = 0U;
                uint32_t rx_hole_apgap_us = 0U;
                uint32_t rx_hole_seqgap = 0U;
                int32_t rx_hole_extra_us = 0;
                uint32_t rx_hole_q_ms = 0U;
                if (play_state == TRI_FLOW_ACTIVE) {
                    tri_playbuf_window_snapshot_reset(&play_q_ms,
                                                      &play_min_win_ms);
                    pop_gap_max_us = tri_play_pop_gap_window_snapshot_reset();
                    rx_gap_max_us = tri_play_rx_gap_window_snapshot_reset(
                        &rx_hole_apgap_us, &rx_hole_seqgap,
                        &rx_hole_extra_us, &rx_hole_q_ms);
                }

                tri_log_audio_rate(play_state, play_actual,
                                   record_state, record_actual,
                                   play_q_ms, play_min_win_ms,
                                   uf_delta, rb_delta, ovf_delta,
                                   tl_slow_delta, tl_normal_delta,
                                   tl_fast_delta, pop_gap_max_us,
                                   rx_gap_max_us, rx_hole_apgap_us,
                                   rx_hole_seqgap, rx_hole_extra_us,
                                   rx_hole_q_ms);
                if (record_state == TRI_FLOW_ACTIVE) {
                    tri_log_record48_check();
                }
                prev_play = s_play_rx_packets;
                prev_play_drop = s_play_rx_drop;
                prev_play_gap = s_play_seq_gap;
                prev_record = s_record_tx_packets;
                prev_record_drop = s_record_tx_drop;
                prev_play_uf = s_play_underflow_events;
                prev_play_rb = s_play_rebuffer_events;
                prev_play_ovf = s_play_overflow_events;
                prev_tl_slow = s_play_tl_slow_blocks;
                prev_tl_normal = s_play_tl_normal_blocks;
                prev_tl_fast = s_play_tl_fast_blocks;
                last_audio_ms = now_ms;
            }
        }
        rtos_task_suspend(TRI_LOOP_MS);
    }

    tri_close(&play_fd);
    s_play_connected = 0U;
    s_wifi_connected = 0U;
    s_running = 0U;
    s_task = NULL;
    rtos_task_delete(NULL);
}

int app_audio_link_init(void)
{
    if (s_inited) {
        return 0;
    }
    s_inited = 1U;
    s_stop = 0U;
    if (rtos_mutex_create(&s_tx_mutex) != 0) {
        s_tx_mutex = NULL;
        return -1;
    }
    if (rtos_mutex_create(&s_rx_mutex) != 0) {
        s_rx_mutex = NULL;
        return -1;
    }
    tri_tx_queue_reset();
    tri_rx_queue_reset();
    return 0;
}

int app_audio_link_start(void)
{
    if (!s_inited && (app_audio_link_init() != 0)) {
        return -1;
    }
    if ((s_task != NULL) || s_running) {
        return 0;
    }
    s_stop = 0U;
    if (rtos_task_create(tri_network_task, "TRI_PLAY", APPLICATION_TASK,
                         TRI_TASK_STACK, NULL,
                         RTOS_TASK_PRIORITY(TRI_TASK_PRIO), &s_task) != 0) {
        s_task = NULL;
        return -1;
    }
    if (rtos_task_create(tri_record_task_fn, "TRI_REC", APPLICATION_TASK,
                         TRI_RECORD_TASK_STACK, NULL,
                         RTOS_TASK_PRIORITY(TRI_RECORD_TASK_PRIO),
                         &s_record_task) != 0) {
        s_record_task = NULL;
        s_stop = 1U;
        return -2;
    }
    return 0;
}

void app_audio_link_stop(void)
{
    s_stop = 1U;
}

void app_audio_link_notify_mute_state(uint8_t muted)
{
    (void)muted;
}

void app_audio_link_set_remote_mute_callback(app_audio_link_remote_mute_cb_t cb)
{
    (void)cb;
}

int app_audio_link_ap_set_sta_mute(uint8_t client_id, uint8_t muted)
{
    (void)client_id;
    (void)muted;
    return -1;
}

int app_audio_link_ap_send_uac_pcm(const int16_t *pcm_stereo, uint16_t frames)
{
    if ((pcm_stereo == NULL) || (frames != APP_AUDIO_LINK_UAC_FRAMES_PER_PKT)) {
        return -1;
    }
    if (!s_mic_streaming) {
        return 0;
    }
    return tri_tx_queue_push(pcm_stereo);
}

int app_audio_link_ap_read_uac_rx_pcm(int16_t *pcm_stereo, uint16_t frames)
{
    if ((pcm_stereo == NULL) || (frames != APP_AUDIO_LINK_UAC_FRAMES_PER_PKT)) {
        return -1;
    }

    /* JCTRL3 diagnostic retained from JCTRL2: this API is called by the downstream 10 ms playback worker.
     * Measuring call-to-call spacing here requires no bridge-source changes
     * and distinguishes renderer scheduling stalls from Wi-Fi ring starvation. */
    if (s_play_pop_timing_active) {
        uint32_t now_us = us_ticker_read();
        if (s_play_pop_last_us != 0U) {
            uint32_t gap_us = now_us - s_play_pop_last_us;
            if (gap_us > s_play_pop_gap_max_window_us) {
                s_play_pop_gap_max_window_us = gap_us;
            }
            if (gap_us > s_play_pop_gap_max_total_us) {
                s_play_pop_gap_max_total_us = gap_us;
            }
        }
        s_play_pop_last_us = now_us;
    } else {
        s_play_pop_last_us = 0U;
    }

    return tri_rx_queue_pop_10ms(pcm_stereo);
}

uint8_t app_audio_link_ap_is_uac_connected(void)
{
    return (s_play_connected && s_record_connected) ? 1U : 0U;
}

uint8_t app_audio_link_is_sta_connected(void)
{
    return s_wifi_connected;
}

uint8_t app_audio_link_is_tcp_connected(void)
{
    return app_audio_link_ap_is_uac_connected();
}

uint8_t app_audio_link_is_connected(void)
{
    return app_audio_link_ap_is_uac_connected();
}

uint8_t app_audio_link_get_role(void)
{
    return APP_AUDIO_LINK_ROLE_STA;
}

int app_audio_link_wait_sta_connected(uint32_t timeout_ms)
{
    uint32_t start = rtos_now(false);
    while (!app_audio_link_ap_is_uac_connected()) {
        if ((rtos_now(false) - start) >= timeout_ms) {
            return -1;
        }
        rtos_task_suspend(10U);
    }
    return 0;
}
