/* Deterministic socket/time seam for the production UDP dispatcher. */
#include <errno.h>
typedef unsigned socklen_t;
struct sockaddr { uint8_t bytes[16]; };
#define AF_INET 2
#define MSG_DONTWAIT 0
static uint32_t mock_now;
static int mock_pending, mock_len, mock_acks, mock_audio;
static int mock_send_errno;
static uint8_t mock_sent[1016];
static unsigned mock_sent_len;
static uint8_t mock_wire[1017];
static struct sockaddr_in mock_peer;
static volatile uint8_t s_usb_mic_on;
static char mock_log[4096];
static uint32_t rtos_protect(void) { return 0; }
static void rtos_unprotect(uint32_t level) { (void)level; }
static void dbg(const char *format, ...)
{
    va_list args;
    size_t used = strlen(mock_log);
    va_start(args, format);
    vsnprintf(mock_log + used, sizeof(mock_log) - used, format, args);
    va_end(args);
}
static uint32_t uacm_now_ms(void) { return mock_now; }
static uint64_t uacm_time_us(void) { return (uint64_t)mock_now * 1000; }
/* TX tests seed an already-encoded frame; codec itself is tested separately. */
static int uacm_prepare_pcm(uacm_session_t *s) { (void)s; return -1; }
static void rtos_mutex_lock(rtos_mutex m, int t) { (void)m; (void)t; }
static void rtos_mutex_unlock(rtos_mutex m) { (void)m; }
static int recvfrom(int fd, void *dst, unsigned size, int flags,
                    struct sockaddr *peer, socklen_t *peer_len)
{
    (void)fd; (void)flags;
    if (!mock_pending) { errno = EAGAIN; return -1; }
    mock_pending = 0;
    memcpy(peer, &mock_peer, sizeof(mock_peer)); *peer_len = sizeof(mock_peer);
    if ((unsigned)mock_len > size) mock_len = size;
    memcpy(dst, mock_wire, mock_len);
    return mock_len;
}
static int sendto(int fd, const void *p, unsigned len, int flags,
                  const struct sockaddr *peer, unsigned peer_len)
{
    (void)fd; (void)p; (void)flags; (void)peer; (void)peer_len;
    if (mock_send_errno) { errno = mock_send_errno; return -1; }
    if (len <= sizeof(mock_sent)) memcpy(mock_sent, p, len);
    mock_sent_len = len;
    mock_acks++;
    return len;
}
static void uacm_handle_record_packet(uacm_session_t *s, const audio_header_t *h,
                                       const uint8_t *payload)
{
    (void)s; (void)h; (void)payload;
    mock_audio++;
}
