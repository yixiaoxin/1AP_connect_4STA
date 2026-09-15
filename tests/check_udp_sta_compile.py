"""Compile the changed application with ARM GCC and SDK headers (no link)."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
dirs = ['user/src/pub', 'plf/aic8800m40/src/arch',
        'config/aic8800m40/target_btdm_wifi/tgt_cfg',
        'config/aic8800m40/target_btdm_wifi/tgt_cfg_hw',
        'freertos/rtos_port/cortex/armgcc_4_8']
for base in ['plf/aic8800m40', 'modules', 'wifi', 'freertos', 'lwip/net_al']:
    dirs += sorted({str(h.parent.relative_to(root)) for h in (root / base).rglob('*.h')
                    if 'LinuxDriver' not in str(h) and 'tinyusb_bak' not in str(h)})
lwip = 'lwip/lwip-STABLE-2_0_2_RELEASE_VER'
dirs += [lwip + '/src/include', lwip + '/ports/rtos/include']
flags = ['-c', '-std=gnu99', '-mcpu=cortex-m4', '-mthumb', '-O2', '-Wall',
         '-Werror=implicit-function-declaration',
         '-DCFG_AIC8800M40', '-DCFG_WIFI_RAM_VER', '-DCFG_HOSTAPD',
         '-DCFG_WIFI_STACK', '-DCFG_DBG', '-DCFG_RTOS',
         '-DCFG_HW_PLATFORM=2', '-DCFG_RF_MODE=3', '-DCFG_BT_MODE=0',
         '-DCFG_DACL_MIXER_MODE=0', '-DCFG_DACR_MIXER_MODE=0',
         '-DCFG_AIC1000_MIC_MATRIX=0', '-DCONFIG_RWNX_LWIP',
         '-D__packed=__attribute__((packed))', '-DLWIP_NO_STDINT_H=1',
         '-DLWIP_TIMEVAL_PRIVATE=0']
out = root / 'build/udp_sta_tests'
out.mkdir(parents=True, exist_ok=True)
for device in [1, 4]:
    for source in ['user/src/pub/modules/app_audio_link.c',
                   'user/src/pub/app_i2s_pcm_lower_wifi.c']:
        cmd = ['arm-none-eabi-gcc'] + flags + ['-DTRIANGLE_DEVICE_ID=' + str(device)]
        cmd += ['-I' + d for d in dict.fromkeys(dirs)]
        cmd += ['-idirafter', lwip + '/src/include/lwip', source,
                '-o', str(out / (Path(source).stem + str(device) + '.o'))]
        result = subprocess.run(cmd, cwd=root, capture_output=True, text=True)
        (out / (Path(source).stem + str(device) + '.log')).write_text(
            result.stdout + result.stderr)
        if result.returncode:
            print(result.stderr)
            raise SystemExit(result.returncode)
print('PASS: ARM application objects, device IDs 1 and 4 (not firmware link)')
