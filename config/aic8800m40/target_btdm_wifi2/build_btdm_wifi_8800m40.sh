#!/bin/sh

# check toolchain version & libary path
ver=`arm-none-eabi-gcc --version | grep release | cut -d ')' -f2 | awk '{print $1}'`
if [ "$ver" != "9.2.1" ]; then
  echo "WRONG toolchain: $ver"
  exit 1
fi
ver=`echo $GNUARM_4_8_LIB | awk -F"/" '{print $NF}'`
if [ "$ver" != "9.2.1" ]; then
  echo "WRONG libary path: $GNUARM_4_8_LIB"
  exit 1
fi

# build rom files
opt=""
if [ -f ./build_tgt.sh ]; then
  arg=$@" -libuse"
else
  arg=$@
fi
for var in $arg
do
  if [ $var = "-libuse" ]; then
    opt=$opt" USE_LIB_DRV=on USE_LIB_BT=on USE_LIB_AUDIO=on USE_LIB_WIFI=on USE_LIB_WPA=on USE_LIB_USBD=on"
  elif [ $var = "-libgen" ]; then
    opt=$opt" GEN_LIB_DRV=on GEN_LIB_BT=on GEN_LIB_AUDIO=on GEN_LIB_WIFI=on GEN_LIB_WPA=on GEN_LIB_USBD=on"
  else
    opt=$opt" "$var
  fi
done
# BLE_DFT_STATE       : adv/scan/init/none
# BLE_APP_AUDTRANSMIT : client/server/none
# BLE_APP_SMARTCONFIG : off/on
# BLE_APP_HID         : off/on
# BLE_APP_UDF         : client/server/all_role/none
python ../../../tools/scons.py -u . PRODUCT=host-wb PLF=aic8800m40 BT=armgcc_4_8 BUILD_CMD="$0 $*" AON=off\
         TGTNAME=btdm_wifi_8800m40 \
        APPS=on BTDM=on BT_BLE=on\
        BLE_DFT_STATE=none BLE_APP_AUDTRANSMIT=none BLE_APP_SMARTCONFIG=off BLE_APP_HID=off  \
        A2DP=off A2DP_SOURCE=off HFP_AG=off LINK_ONE=off AUD_USED=off \
		BAND5G=on SOFTAP=on IPERF=on PING=on CHKSUM=off DPD=off TGT_MODE=btdm_wifi FHOST_APP=console\
    WIFI_RAM_VER=on HEAP_SIZE=0x40000 USB_DEVICE=on  $opt
