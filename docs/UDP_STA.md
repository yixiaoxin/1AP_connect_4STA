# STA1 与四 STA UDP AP 对接

当前编译入口由 `user/config/sourcelist.txt` 指定：
`app_i2s_pcm_lower_wifi.c` 调用 `modules/app_audio_link.c`。
`demo_src.c` 是未参与当前构建的旧示例，切换其中的 TCPSelect 不会修改实际音频链路。

- Wi-Fi：`aic8800m40` / `12345678`，AP 地址 `192.168.88.1`。
- 播放：UDP 8888；录音：UDP 8890。各方向独立持有一个 socket。
- 默认设备 ID 为 1；`app_audio_link.h` 支持 `TRIANGLE_DEVICE_ID=1..4`。
  四台设备必须分别使用不同 ID，对应 MAC 尾字节 C1～C4。
- UDP `connect()` 只绑定目标 AP 地址并过滤来源，应用就绪以 HELLO ACK 为准。
  首次注册最多等待 5 秒，每秒重发同一 socket/token 的 HELLO；
  成功后继续每秒发送，3 秒未收到有效 ACK 则重建该方向。
- HELLO 是原协议的完整 25 字节数据报，序号固定为 0，不占用录音音频序号。
  普通下行麦克风控制和心跳均可更新录音开关；上电默认等待 AP 开关状态。
- 音频使用与 AP 相同的 ADU1 16 字节分片头，每片最多携带 1000 字节，
  完整帧最大 1941 字节。播放按数据报重组，支持同帧乱序、重复丢弃和 30 ms 超时。
  网络缓冲暂时不足时，发送端最多等待 20 ms 后丢弃整帧，包括已发送首片的残帧。
- 录音沿用 48 kHz、16 bit、立体声 BPK2/REC20，REC20 含组头不超过 1920 字节；
  装不下时回退为独立 10 ms BPK2 或原始 PCM 帧。I2S 接口和播放缓冲策略沿用原工程。

验证命令（STA1 根目录）：

```sh
python3 tests/run_udp_sta.py
python3 tests/check_udp_sta_compile.py
```

第一项提取实际生产函数，使用模拟 socket/时钟验证注册重试、ACK 超时、
错误 ID/方向、分片重组、麦克风状态、原始帧分片、拥塞丢弃与 REC20 编解码；
分别运行 ID 1 和 4，并启用 UBSan。第二项使用 ARM GCC 9.2.1 和 SDK 头文件
编译修改的两个应用源文件，输出到 `build/udp_sta_tests`；它不是完整固件链接，
其中 `LWIP_TIMEVAL_PRIVATE=0` 用于消除独立检查时 SDK 与 newlib 的 timeval 重定义。

完整固件已使用工程原有 Wi-Fi 配置编译、链接成功，命令如下：

```sh
cd config/aic8800m40/target_wifi
sh build_wifi_case_8800m40.sh HCLK_MCLK=on USER_CODE=src -j4
```

输出为 `build/host-wifi-aic8800m40/host_wb_aic8800m40.bin`，默认设备 ID 1。
已核对 ELF 中的 UDP 会话/收包函数和 BIN 中的 UDP4 版本标识。
构建仍有 SDK 原有告警；未进行烧录和四设备联调。

上板需确认两条 `READY ... connected` 日志，再测试 USB 播放/录音开关、
设备断线重连及四台并发。主机测试不代表无线链路、I2S 时序或端到端延迟已经实测。
