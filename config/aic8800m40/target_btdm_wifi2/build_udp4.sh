#!/bin/sh
# Four-STA UAC profile. Run with the SDK Python 2 / ARM GCC 9.2.1 environment.
set -eu
cd "$(dirname "$0")"
exec sh ./build_btdm_wifi_8800m40.sh "$@" SOFTAP=on DPD=off HEAP_SIZE=0x40000 USB_DEVICE=on
