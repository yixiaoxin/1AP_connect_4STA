"""SDK-header syntax check on a host GCC; this is NOT an ARM firmware build.
The Win32 ABI flags avoid host pointer/size_t mismatches in the embedded SDK.
"""
import argparse
from pathlib import Path
import subprocess

p = argparse.ArgumentParser()
p.add_argument('--cc', default='gcc')
args = p.parse_args()
root = Path(__file__).resolve().parents[1]
dirs = ['user/src/pub', 'plf/aic8800m40/src/arch',
        'config/aic8800m40/target_btdm_wifi/tgt_cfg',
        'config/aic8800m40/target_btdm_wifi/tgt_cfg_hw',
        'freertos/rtos_port/cortex/armgcc_4_8']
for base in ['plf/aic8800m40', 'modules', 'wifi', 'freertos', 'lwip/net_al']:
    dirs += sorted({str(h.parent.relative_to(root)).replace('\\', '/')
                    for h in (root / base).rglob('*.h')
                    if 'LinuxDriver' not in str(h) and 'tinyusb_bak' not in str(h)})
lwip = 'lwip/lwip-STABLE-2_0_2_RELEASE_VER'
dirs += [lwip + '/src/include', lwip + '/ports/rtos/include']
flags = ['-fsyntax-only', '-std=gnu99', '-m32', '-Wall', '-Wextra',
         '-Werror=implicit-function-declaration',
         '-DCFG_AIC8800M40', '-DCFG_WIFI_RAM_VER', '-DCFG_HOSTAPD',
         '-DCFG_WIFI_STACK', '-DCFG_DBG', '-DCFG_RTOS', '-DCFG_USB_DEVICE',
         '-DCFG_HW_PLATFORM=2', '-DCFG_RF_MODE=3', '-DCFG_BT_MODE=0',
         '-DCFG_DACL_MIXER_MODE=0', '-DCFG_DACR_MIXER_MODE=0',
         '-DCFG_AIC1000_MIC_MATRIX=0', '-DCONFIG_RWNX_LWIP',
         '-D__packed=__attribute__((packed))', '-DLWIP_NO_STDINT_H=1']
# The SDK port supplies its own integer typedefs. Windows errno macro
# redefinition warnings are expected and retained in the log.
cmd = [args.cc] + flags + ['-I' + d for d in dict.fromkeys(dirs)] + [
    '-idirafter', lwip + '/src/include/lwip', 'user/src/pub/demo_src.c']
result = subprocess.run(cmd, cwd=root, capture_output=True, text=True)
out = root / 'build/uac_udp4_tests'
out.mkdir(parents=True, exist_ok=True)
(out / 'syntax.log').write_text(result.stdout + result.stderr, encoding='utf-8')
print('PASS: SDK-header host syntax check (not ARM link)' if not result.returncode else result.stderr)
raise SystemExit(result.returncode)
