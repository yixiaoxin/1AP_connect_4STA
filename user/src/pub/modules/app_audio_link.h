#ifndef __APP_AUDIO_LINK_H__
#define __APP_AUDIO_LINK_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APP_AUDIO_LINK_ROLE_STA                  0U
#define APP_AUDIO_LINK_ROLE_AP                   1U
#define APP_AUDIO_LINK_ROLE                      APP_AUDIO_LINK_ROLE_STA
#define APP_AUDIO_LINK_AP_MAX_STA                4U

#ifndef TRIANGLE_DEVICE_ID
#define TRIANGLE_DEVICE_ID                       1U
#endif

#if (TRIANGLE_DEVICE_ID < 1U) || (TRIANGLE_DEVICE_ID > 4U)
#error "TRIANGLE_DEVICE_ID must be in 1..4."
#endif

#define APP_AUDIO_LINK_SSID                      "aic8800m40"
#define APP_AUDIO_LINK_PASSWORD                  "12345678"
#define APP_AUDIO_LINK_SERVER_IP                 "192.168.88.1"
#define APP_AUDIO_LINK_UAC_SERVER_PORT           8888U
#define APP_AUDIO_LINK_UAC_RETURN_SERVER_PORT    8890U

#define APP_AUDIO_LINK_UAC_SAMPLE_RATE           48000U
#define APP_AUDIO_LINK_UAC_CHANNELS              2U
#define APP_AUDIO_LINK_UAC_BITS_PER_SAMPLE       16U
#define APP_AUDIO_LINK_UAC_PACKET_MS             10U
#define APP_AUDIO_LINK_UAC_WIRE_PACKET_MS        5U
#define APP_AUDIO_LINK_UAC_UPLINK_PACKET_MS      APP_AUDIO_LINK_UAC_WIRE_PACKET_MS
#define APP_AUDIO_LINK_UAC_FRAMES_PER_PKT        480U
#define APP_AUDIO_LINK_UAC_SAMPLES_PER_PKT       960U
#define APP_AUDIO_LINK_UAC_BYTES_PER_PKT         1920U
#define APP_AUDIO_LINK_UAC_WIRE_FRAMES           240U
#define APP_AUDIO_LINK_UAC_UPLINK_FRAMES_PER_PKT APP_AUDIO_LINK_UAC_WIRE_FRAMES
#define APP_AUDIO_LINK_UAC_WIRE_SAMPLES          480U
#define APP_AUDIO_LINK_UAC_WIRE_BYTES            960U

/* Native 48 kHz stereo recording matches the four-STA UDP AP. */
#define APP_AUDIO_LINK_RECORD_WIRE_SAMPLE_RATE   48000U
#define APP_AUDIO_LINK_RECORD_WIRE_PACKET_MS     10U
#define APP_AUDIO_LINK_RECORD_SOURCE_BLOCKS      1U
#define APP_AUDIO_LINK_RECORD_SOURCE_FRAMES      (APP_AUDIO_LINK_UAC_FRAMES_PER_PKT * APP_AUDIO_LINK_RECORD_SOURCE_BLOCKS)
#define APP_AUDIO_LINK_RECORD_SOURCE_SAMPLES     (APP_AUDIO_LINK_RECORD_SOURCE_FRAMES * APP_AUDIO_LINK_UAC_CHANNELS)
#define APP_AUDIO_LINK_RECORD_WIRE_FRAMES        480U
#define APP_AUDIO_LINK_RECORD_WIRE_SAMPLES       960U
#define APP_AUDIO_LINK_RECORD_WIRE_BYTES         1920U

#define APP_AUDIO_LINK_PACKET_MAGIC              0xA55A5AA5U
#define APP_AUDIO_LINK_DIRECTION_AP_TO_STA       0x01U
#define APP_AUDIO_LINK_DIRECTION_STA_TO_AP       0x02U
#define APP_AUDIO_LINK_PACKET_TYPE_CTRL          0x02U
#define APP_AUDIO_LINK_PACKET_TYPE_UAC_PCM       0x03U
#define APP_AUDIO_LINK_CTRL_UAC_MIC_STREAMING    0x04U
#define APP_AUDIO_LINK_CTRL_HEARTBEAT             0x05U
#define APP_AUDIO_LINK_CTRL_SESSION_HELLO         0x10U

typedef void (*app_audio_link_remote_mute_cb_t)(uint8_t muted);

int app_audio_link_init(void);
int app_audio_link_start(void);
void app_audio_link_stop(void);
void app_audio_link_notify_mute_state(uint8_t muted);
void app_audio_link_set_remote_mute_callback(app_audio_link_remote_mute_cb_t cb);
int app_audio_link_ap_set_sta_mute(uint8_t client_id, uint8_t muted);

/* Compatibility names retained so the proven SSD212 I2S bridge need not be
 * structurally rewritten. On this image these APIs operate as Triangle STA. */
int app_audio_link_ap_send_uac_pcm(const int16_t *pcm_stereo, uint16_t frames);
int app_audio_link_ap_read_uac_rx_pcm(int16_t *pcm_stereo, uint16_t frames);
uint8_t app_audio_link_ap_is_uac_connected(void);
uint8_t app_audio_link_is_sta_connected(void);
/* Legacy API name: reports both UDP directions registered with HELLO ACK. */
uint8_t app_audio_link_is_tcp_connected(void);
uint8_t app_audio_link_is_connected(void);
uint8_t app_audio_link_get_role(void);
int app_audio_link_wait_sta_connected(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __APP_AUDIO_LINK_H__ */
