"""Exercise production UDP registration, RX/TX and codec with mock IO/time."""
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parents[1]
src = (root / 'user/src/pub/modules/app_audio_link.c').read_text()

def function(name):
    match = re.search(r'^static [^;{}]*\b' + name + r'\([^;{}]*\)\s*\{', src, re.M)
    if not match:
        raise RuntimeError(name)
    pos, depth = match.end(), 1
    while depth:
        depth += (src[pos] == '{') - (src[pos] == '}')
        pos += 1
    return src[match.start():pos] + '\n'

prefix = """#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include "app_audio_link.h"
#include "uac_udp_transport.h"
typedef void *rtos_mutex;
typedef void *rtos_task_handle;
#define dbg(...) ((void)0)
#define MSG_DONTWAIT 0
"""
definitions = src[src.index('#define TRI_VERSION'):src.index('static uint64_t tri_time_us')]
codec = src[src.index('/* ================= LL3-BPK2'):src.index('static uint8_t tri_deadline_reached')]
functions = ''.join(function(n) for n in [
    'tri_udp_temporary_error', 'tri_udp_send_hello', 'tri_udp_accept_ack',
    'tri_send_hello', 'tri_track_rx_seq', 'tri_handle_rx',
    'tri_service_udp_rx', 'tri_deadline_reached', 'tri_prepare_record_frame',
    'tri_service_record_tx'])
out = root / 'build/udp_sta_tests'
out.mkdir(parents=True, exist_ok=True)
test = (root / 'tests/udp_sta_host.c').read_text()
mocks, main = test.split('/* PRODUCTION FUNCTIONS */')
cfile = out / 'udp_sta.c'
cfile.write_text(prefix + definitions + mocks + codec + functions + main)
for device in [1, 4]:
    exe = out / ('udp_sta' + str(device))
    subprocess.run(['gcc', '-std=c99', '-O1', '-g', '-fsanitize=undefined',
                    '-Werror=implicit-function-declaration',
                    '-DTRIANGLE_DEVICE_ID=' + str(device),
                    '-I' + str(root / 'user/src/pub/modules'), str(cfile),
                    '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
