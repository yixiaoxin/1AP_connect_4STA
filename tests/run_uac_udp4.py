"""Run production pure-C logic on the host, without emulating the hardware.
Usage: python tests/run_uac_udp4.py [--cc gcc]
"""
import argparse
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument('--cc', default='gcc')
args = parser.parse_args()
src = (root / 'user/src/pub/demo_src.c').read_text(encoding='utf-8')
def function(name):
    match = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', src, re.M)
    if not match:
        raise RuntimeError('Missing production function: ' + name)
    pos, depth = match.end(), 1
    while depth:
        depth += (src[pos] == '{') - (src[pos] == '}')
        pos += 1
    return src[match.start():pos] + '\n'

prefix = '''#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#define NX_REMOTE_STA_MAX 8
#include "uac_audio_bridge.h"
#include "uac_udp_transport.h"
typedef void *rtos_mutex;
struct sockaddr_in {
    uint16_t sin_family, sin_port;
    struct { uint32_t s_addr; } sin_addr;
    uint8_t padding[8];
};
'''
types = src[src.index('#define UACM_VERSION'):src.index('static uint8_t s_usb_spk_storage')]
codec = src[src.index('/* ================= LL3-BPK2/48PREP1 bounded'):src.index('static void uacm_enqueue_record_5ms')]
globals_ = '''static uacm_session_t *s_session;
static uint32_t s_mix_zero_blocks, s_mix_fallback_blocks, s_mix_dual_blocks;
'''
functions = ''.join(function(n) for n in [
    'uacm_diag_rate', 'uacm_track_record_seq', 'uacm_block_power', 'uacm_ema_u64',
    'uacm_update_vad', 'uacm_smooth_gain', 'uacm_sat_s16', 'uacm_mix_pcm'])
network = (root / 'tests/uac_udp4_net_mocks.h').read_text() + ''.join(function(n) for n in [
    'uacm_barrier', 'uacm_ring_reset', 'uacm_rx_lock', 'uacm_rx_unlock',
    'uacm_close_record', 'uacm_close_playback', 'uacm_same_peer', 'uacm_poll_udp',
    'uacm_build_header', 'uacm_prepare_ctrl', 'uacm_service_tx'])
out = root / 'build/uac_udp4_tests'
out.mkdir(parents=True, exist_ok=True)
cfile = out / 'core.c'
cfile.write_text(prefix + types + codec + globals_ + functions + network +
                 (root / 'tests/uac_udp4_host.c').read_text(), encoding='utf-8')
exe = out / 'core.exe'
subprocess.run([args.cc, '-std=c99', '-O2', '-Wall', '-Wextra',
                '-Wno-unused-const-variable', '-I' + str(root / 'user/src/pub'),
                str(cfile), '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
