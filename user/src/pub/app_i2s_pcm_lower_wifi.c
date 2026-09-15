#include <stdint.h>
#include "dbg.h"
#include "rtos.h"
#include "rtos_al.h"
#include "rtos_def.h"
#include "fhost_config.h"
#include "flash_api_wifi.h"
#include "modules/app_audio_pcm.h"
#include "modules/app_audio_link.h"
#include "modules/app_i2s_ssd212_bridge.h"

#ifndef TRIANGLE_STA_ENTRY_DELAY_MS
#define TRIANGLE_STA_ENTRY_DELAY_MS 100U
#endif

static void triangle_sta_user_task(void *param)
{
    int ret;
    (void)param;
    rtos_task_suspend(TRIANGLE_STA_ENTRY_DELAY_MS);

    /* Fixed STA MAC address, persisted to flash.  Tail byte is derived from
     * TRIANGLE_DEVICE_ID (1 -> 88:00:33:AA:BB:C1 through 4 -> ...:C4) so all
     * STAs built from this source do not collide on the same network. */
    {
        uint8_t fixed_mac[6] = {0x88, 0x00, 0x33, 0xAA, 0xBB,
                                0xC0U | (uint8_t)TRIANGLE_DEVICE_ID};
        set_mac_address(fixed_mac);
        flash_wifi_sta_macaddr_write(fixed_mac);
    }

    dbg("TRI%u boot UDP4: capture/playback/record 48k stereo, UDP 8888/8890\r\n",
        (unsigned)TRIANGLE_DEVICE_ID);

    app_audio_pcm_init();
    ret = app_audio_link_init();
    if (ret != 0) {
        dbg("SYSTEM: TRI%u audio link init failed ret=%d\r\n",
            (unsigned)TRIANGLE_DEVICE_ID, ret);
        rtos_task_delete(NULL);
        return;
    }
    ret = app_audio_link_start();
    if (ret != 0) {
        dbg("SYSTEM: TRI%u audio link start failed ret=%d\r\n",
            (unsigned)TRIANGLE_DEVICE_ID, ret);
        rtos_task_delete(NULL);
        return;
    }
    ret = app_i2s_ssd212_bridge_start();
    if (ret != 0) {
        dbg("SYSTEM: TRI%u I2S start failed ret=%d\r\n",
            (unsigned)TRIANGLE_DEVICE_ID, ret);
        app_audio_link_stop();
        rtos_task_delete(NULL);
        return;
    }
    dbg("TRI%u READY audio bridge started\r\n",
        (unsigned)TRIANGLE_DEVICE_ID);
    rtos_task_delete(NULL);
}

void user_code_entry(void)
{
    if (rtos_task_create(triangle_sta_user_task,
                         "TRI_ENTRY",
                         USER_CODE_TASK,
                         2048U,
                         NULL,
                         TASK_PRIORITY_USER_CODE,
                         NULL)) {
        dbg("SYSTEM: Triangle STA%u entry task create failed\r\n",
            (unsigned)TRIANGLE_DEVICE_ID);
    }
}
