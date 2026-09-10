#ifndef UAC_AUDIO_BRIDGE_H
#define UAC_AUDIO_BRIDGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UAC_BRIDGE_SAMPLE_RATE_HZ       48000U
#define UAC_BRIDGE_CHANNELS             2U
#define UAC_BRIDGE_BITS_PER_SAMPLE      16U
#define UAC_BRIDGE_USB_PACKET_MS        1U
#define UAC_BRIDGE_BLOCK_MS        5U
#define UAC_BRIDGE_BYTES_PER_SAMPLE     (UAC_BRIDGE_BITS_PER_SAMPLE / 8U)
#define UAC_BRIDGE_FRAME_BYTES          (UAC_BRIDGE_CHANNELS * UAC_BRIDGE_BYTES_PER_SAMPLE)
#define UAC_BRIDGE_USB_PACKET_BYTES     ((UAC_BRIDGE_SAMPLE_RATE_HZ * UAC_BRIDGE_USB_PACKET_MS / 1000U) * UAC_BRIDGE_FRAME_BYTES)
#define UAC_BRIDGE_BLOCK_PCM_BYTES        ((UAC_BRIDGE_SAMPLE_RATE_HZ * UAC_BRIDGE_BLOCK_MS / 1000U) * UAC_BRIDGE_FRAME_BYTES)

/* Called from the TinyUSB Speaker OUT completion path.  The function never
 * blocks and never touches the network socket.  It accepts one 1 ms stereo packet. */
int uac_bridge_usb_speaker_push_isr(const uint8_t *pcm, uint32_t len);

/* Called from the TinyUSB Microphone IN completion path.  It always renders
 * one 1 ms stereo packet.  Return 1 for AP PCM, 0 for generated silence. */
int uac_bridge_usb_mic_render_isr(uint8_t *dst, uint32_t len);

/* Called from TinyUSB SET_INTERFACE callbacks. */
void uac_bridge_usb_stream_state(uint8_t mic_on, uint8_t speaker_on);

/* Stores the latest host playback control state.  PCM framing is unchanged. */
void uac_bridge_usb_playback_ctrl(int16_t volume_raw, uint8_t mute);

#ifdef __cplusplus
}
#endif

#endif /* UAC_AUDIO_BRIDGE_H */
