# 单 STA 断电诊断日志

第一阶段增加诊断并修正录音关闭时的告警；第二阶段加入以下保活重试和疑似离线停发机制。AP 与 STA 需分别重新编译烧录；STA 编号保持各板唯一。

## AP 日志

每约 10 秒输出一次 `AP Tn DIAG totals`。以下计数自启动累计，客户端重连不清零；判断一次故障应比较前后差值。

| 字段 | 含义 |
|---|---|
| tx_again | 播放发送返回 EAGAIN/EWOULDBLOCK 的次数（两者可能同值，合并统计） |
| tx_nomem / tx_nobuf | 播放发送返回 ENOMEM / ENOBUFS 的次数；不能仅据此确定具体驱动缓冲池 |
| tx_other | 其他播放发送错误次数 |
| last_errno | 最近一次播放发送失败的错误码；发送恢复后仍保留，不代表当前还在失败 |
| send_max_ms | 自启动以来单次播放发送路径的最大耗时，毫秒分辨率，包含线程被抢占时间；0 不代表零耗时 |
| ack_fail_play / ack_fail_rec | 对应方向 HELLO ACK 回送失败次数，包括短发送 |
| ack_errno_play / ack_errno_rec | 对应方向最近 ACK 发送错误码；短发送记为 0 |
| hello_age_play_ms / hello_age_rec_ms | 距对应方向最近有效 HELLO 的时间；仅在线会话具有直接诊断意义，未连接时可能只是距启动的时间 |
| t | 本板启动后的毫秒时刻；不同板的 t 不能直接对齐 |

`UACM DIAG play_loop_max_ms` 是本统计窗口内 AP 播放网络任务相邻两轮开始的最大间隔，包含主动休眠、抢占、打印和处理时间。它不是纯 CPU 执行耗时。

现有 `tx_busy` 是统计窗口内 PCM 发送临时失败次数，可对同一帧重复计数；不是丢包数或无线重传数。`sent` 是本地提交成功的音频帧数，不保证对端收到。

异常事件即时输出：

- `playback_close/record_close reason=hello_timeout`：该客户端对应方向 HELLO 过期，`age_ms` 是过期年龄。
- `playback_close reason=send_error`：非临时播放发送错误，查看 errno 和 result。
- `playback_close reason=short_send`：发送长度不符合 UDP 报文预期。
- `socket_reset dir=... errno=... scope=all_sessions_in_direction`：共享接收 socket 报错，现有逻辑将重建 socket 并清理该方向所有会话。dir=1 播放，dir=2 录音。

`REC OFF session=up/down` 表示录音流关闭，但保活会话可能在线；不计算录音丢包告警。录音开启时仍使用原固定 10 秒目标，因此启停边界窗口的百分比不能直接视为网络丢包率。

## STA 日志

- `hello_send_fail dir=... errno=... temporary=... count=... ack_age_ms=... t=...`：HELLO 未成功提交；临时错误最多每方向每秒打印一次。count 自当前 UDP 会话创建以来累计，重建时清零。首次 ACK 前 ack_age_ms 不代表有效租约年龄。
- `session_close reason=ack_timeout`：收包前发现 ACK 租约过期（或尚未确认）；acked 表示是否曾收到 ACK。
- `session_close reason=ack_timeout_after_rx`：本轮收包后仍未得到有效续期 ACK。
- `session_close reason=recv_error`：非临时 UDP 接收错误。
- 原有 `WARN WiFi disconnected` 表示 Wi-Fi 状态断开；`WARN playback disconnected` 只表示播放会话断开。

## 断电复测

1. 同时保存 AP 和一块不掉电 STA 的串口日志，先播放约 20 秒取得正常基线。
2. 只关闭另一个 STA，继续记录至少 30 秒，记录实际断电时刻。
3. 对比故障前后 tx_* 和 ack_fail_* 差值，以及 loop/send 最大耗时。
4. 若多路 tx_again/nomem/nobuf 同涨，支持公共发送资源受限；若 ACK 失败增长并伴随 STA ack_timeout，支持保活受拥塞影响；若出现 socket_reset，则确认共享 socket 重建路径被触发。
5. 底层 `_st` 状态值仍需 SDK 定义，不能由本日志直接解码。

日志采用周期汇总及异常事件输出，仍可能影响实时性；需关注输出密度。无实际硬件测试，不能据此认定根因已修复。

## 第二阶段：保活和停发

- AP：HELLO ACK 遇 EAGAIN/EWOULDBLOCK、ENOMEM、ENOBUFS 或 EINTR，每 50ms 最多补试 5 次；最新有效 HELLO 替换旧 ACK，方向独立，关闭会话时取消。短发送和永久错误不重试。重试在音频发送前的 UDP 轮询执行，不阻塞等待。
- STA：HELLO 临时发送失败后每 50ms 最多补试 5 次；成功后恢复 1 秒周期，预算用完也回到 1 秒周期。只改变本地提交失败的重试，不将普通音频作为续期依据。
- AP：播放 HELLO 年龄达到 1500ms 时，输出 `playback_pause reason=hello_stale`，清空本客户端待发旧音频、暂停入队和发送。其他客户端独立处理。有效 HELLO 到达后输出 `playback_resume reason=hello`，恢复新音频。
- 3 秒会话超时保持不变。1500ms 是初始工程参数，比正常 1 秒心跳多留 500ms 余量；无线抖动也可能触发暂停，需上板验证。应用停发不能撤回已交给驱动的报文。

## LWIP 分配诊断（AP 默认 v2_0_x）

新增 `LWIP ALLOC_FAIL site=... count=... bytes=... t=...`。每个埋点独立累计、首次立即打印，此后有失败时最多每秒一次；不是所有埋点合并限频。count 为启动以来该位置失败次数，bytes 为请求载荷长度（不是包含内部结构的总分配量）。多个埋点可能来自同一次失败，不应相加作为丢包数。

| site | 失败位置 |
|---|---|
| pbuf_pool_head / pbuf_pool_tail | PBUF_POOL 首节点／后续节点分配失败 |
| pbuf_ram | PBUF_RAM 内存分配失败 |
| pbuf_descriptor | PBUF_REF/ROM 描述符池分配失败 |
| sendto_netbuf_alloc / sendto_netbuf_ref | UDP 在提交协议栈前建立报文失败 |
| sendto_netconn_send | 已进入发送协议栈后返回 ERR_MEM，应结合同时间的 pbuf 诊断定位；也可能涉及未细分的 ARP 等路径 |

配置默认使用 `lwip-STABLE-2_0_2_RELEASE_VER`，本次未改备用 v2_1_x。若构建切换 NETS_VER，需同步埋点。失败日志不额外分配内存，计数受保护、打印在保护区外，但打印仍会有运行开销。

验证命令：

```
python3 tests/run_uac_udp4.py
python3 tests/check_uac_syntax.py --arm --cc arm-none-eabi-gcc
```

STA 工程：`python3 tests/run_udp_sta.py` 与 `python3 tests/check_udp_sta_compile.py`。
新增测试覆盖 ACK 重试节拍、预算耗尽、恢复成功、过期取消、单客户端暂停/恢复与隔离，以及 STA HELLO 有界重试。

## 第三阶段：描述符占用与 SDK 释放入口

AP 每约 10 秒增加两行：

```
LWIP PBUF_POOL capacity=... heap_mode=0 used=... free=... peak=... window_peak=... alloc=... freed=... fail=... free_underflow=... t=...
LWIP TX_REF submit_ok=... submit_fail=... ref_rollback=... release_calls=... freed_nodes=... t=...
```

`PBUF_POOL` 这一日志标签指 **MEMP_PBUF 描述符池**，不是用于负载存储的 `PBUF_POOL` 分配类型。

- capacity：实际编译配置的描述符总数量；heap_mode=1 时无固定容量，capacity/free 为 0，不可按池余量解释。
- used/free：采样瞬间的占用／剩余；在真正的池申请和释放保护区内计数，包含所有使用 MEMP_PBUF 的网络业务。
- peak：启动以来最高占用；window_peak：两次日志之间最高占用。即使打印时已恢复，window_peak 达到 capacity 仍说明窗口中曾满池。
- alloc/freed：成功申请／归还描述符累计次数；正常情况下 alloc-freed=used（32 位计数回绕按无符号运算）。它们不是报文成功发送次数。
- fail：池申请失败累计次数，不仅限于 UDP 的 netbuf_ref。
- free_underflow：归还时统计占用已经为零的异常次数；不应增长。此字段不替代内存完整性检测。
- submit_ok/submit_fail：net_if_output 向 Wi-Fi 直接提交或向配置的 TX 任务提交的成功／失败次数。成功不等于空口发送成功。
- ref_rollback：提交失败后撤销额外 pbuf 引用的次数，应与 submit_fail 同步增长。
- release_calls：SDK 调用 net_buf_tx_free 的次数；freed_nodes：这些调用中 pbuf_free 实际释放的链节点总数，节点可能属于不同类型。

**不要使用 submit_ok-release_calls 推导在途报文数量**：释放入口还可能收到 L2/EAPOL 等未经过 net_if_output 的报文；一条 pbuf 链也可能有多个节点，且释放调用不一定马上释放所有引用。

判断方法：在同样播放负载下，比较断电前、断电故障期间和恢复后的 used/window_peak/fail。若 window_peak=capacity 且 fail 增加，确认池被耗尽；若释放持续增长且 used 回到正常基线，更支持短时占用／释放延迟；若反复断电后 used 基线持续抬升，需继续追查滞留或引用泄漏，不能仅凭一次采样定性。

### SDK 接口检查结论

- `wifi/fhost/fhost_tx.h` 的 fhost_tx_start 接收发送完成回调及私有参数，返回 0 表示提交成功。
- `lwip/net_al/net_al.c` 的 net_l2_send 已使用该回调，但现有可见源码不含 fhost_tx_start 实现；无法仅凭头文件确认提交失败、离线清理时回调是否必达，以及回调与 pbuf 释放的先后顺序。
- `net_buf_tx_free` 是可见 SDK 释放入口，本阶段只计数，不改变释放所有权和回调行为。
- `FHOST_CFG_TX_LFT` 是报文寿命配置。fhost_config.c 在 PLF_WIFI_AUDIO 条件下设置 40ms，其他分支为 1000ms；头文件描述长度为 4，而 fw_config 项实际长度为 2，正式修改前应按 SDK 实现确认单位、长度和适用范围，不能盲目写入。
- 本阶段未扩池、未实施每 STA 在途配额。完成这些限制前，需证明失败／离线清理路径也会可靠归还额度；否则可能引入永久停发。

验证：`python3 tests/check_pbuf_pool_diag.py` 使用实际生产分配／归还代码验证满池失败、释放恢复、累计计数和窗口峰值；`python3 tests/check_uac_syntax.py --arm --cc arm-none-eabi-gcc` 还检查 memp.c 与 net_al.c。

## 第四阶段：AP 描述符池 11 → 32 受控验证

`lwip/net_al/lwipopts.h` 在 CFG_SOFTAP 或 CFG_HOSTAPD 下使用独立的 `AP_UDP_PBUF_COUNT=32`，不再由 TCP_SND_BUF 推导 MEMP_NUM_PBUF。其他分支保留原配置，STA1 工程未修改。可通过编译宏覆盖 AP_UDP_PBUF_COUNT；32 是实验起点，不是每 STA 配额或最终容量保证。

使用当前 ARM 检查配置编译 memp.c，对象符号 memp_memory_PBUF_base 从 0xB0（176 字节）增至 0x200（512 字节），直接描述符存储增加 336 字节。此数不是完整固件链接后的总内存变化；最终以板端构建 map 为准。应用停发、HELLO 重试和 3 秒租约未变。

Clean/Rebuild AP（确保 lwIP 被重新编译），烧录后首先确认 `LWIP PBUF_POOL capacity=32`。在相同播放负载下反复关闭／开启任意 STA，并测试四 STA：

- 比较故障前后 fail 是否增长，window_peak 是否达到 32。
- 检查未断电 STA 是否暂停／重连，以及 drop_q/drop_to/drop_err 和 STA 欠载。
- 检查恢复后 used 是否回到基线，alloc/freed 是否保持平衡。
- 若仍满池，应继续做在途资源隔离和报文释放排查，不应无上限扩池。

USB drop 与固定目标 999/1000 告警不属于本次扩池修改范围。

### 扩池宏条件修正

AP 构建脚本 target_btdm_wifi2/build_btdm_wifi_8800m40.sh 使用 SOFTAP=on。
config/SConscript 将其转换为 CFG_SOFTAP，和 HOSTAPD/CFG_HOSTAPD 独立。
原扩池分支仅检查 CFG_HOSTAPD，导致实际 SoftAP 固件仍为 11。
现同时识别两种宏；ARM 检查也改为 CFG_SOFTAP，匹配 SoftAP 分支。
重新编译 lwIP 并烧录后，确认 capacity=32 再做断电测试。

### 32 描述符实测结果与发送寿命试验

断电 T3 的上板日志显示 `capacity=32 window_peak=32 fail=5128`；T1/T2 同时出现 `tx_nomem`、ACK 发送失败、HELLO 超时。断电流量停止后 `used=0` 且 `alloc=freed`，说明本次是短时占满，不能据此认定永久泄漏。T3 的 `playback_pause` 出现在首个 `ALLOC_FAIL` 约 1 秒后，现有 HELLO 暂停无法及时挡住积压。

该 AP 构建未启用 `WIFI_AUDIO`，因此 `fhost_config.c` 原来选择 1000 ms Wi-Fi TX lifetime。现对 `CFG_SOFTAP && CFG_USB_DEVICE` 使用 SDK 音频模式已有的 40 ms 值，并打印 `UACM AP tx_lft_cfg_ms=40`。这是发送帧寿命的配置值，是否让固件更早释放断电 STA 的积压报文，仍需以板端 `window_peak/fail` 和其他 STA 的连续播放检验。更短寿命可能使弱信号下本来还能重传成功的帧较早丢弃。
