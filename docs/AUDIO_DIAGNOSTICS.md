# AP audio diagnostics

Every approximately 10 seconds the network task prints three lines per STA.
Actual counters are interval deltas, not lifetime totals. Expected values are
fixed: playback 1000 frames (10 ms/frame), recording 2000 PCM blocks (5 ms/block).
They are not calculated from actual counts or sequence gaps. No logging occurs in
the USB ISR; only a short counter snapshot is protected, never UART output.

```text
erro:AP T1 PLAY sent=996/1000 drop_q=2 drop_to=1 drop_err=1 local_loss=0.40%
erro:AP T1 REC recv=1992/2000 missing=2 late=0 decode_fail=0 gap_rate=0.40%
AP T1 UDP tx_pkt=1500 rx_pkt=800 tx_busy=3 asm_timeout=1 rx_gap_max_ms=42
==========
AP T2 PLAY sent=down
AP T2 REC recv=down
AP T2 UDP tx_pkt=0 rx_pkt=0 tx_busy=0 asm_timeout=0 rx_gap_max_ms=0
==========
```

Each direction is up only while its UDP registration is present and its HELLO
lease has not expired. Disconnected directions print only `sent=down` or
`recv=down`, without a rate or error color. This is registration state, not USB
stream activity. Snapshots still advance while down, avoiding stale deltas on
reconnect. UDP interval counters and the separator remain visible.

An accepted HELLO that establishes or re-establishes a session prints:

```text
T1 play_back 192.168.88.1->192.168.88.24 (hello,connected)
T1 record 192.168.88.1->192.168.88.24 (hello,connected)
```

The left address comes from `get_ap_ip_addr()` (the SoftAP address used as the
STA gateway); the right address is the HELLO sender's IPv4 address. Periodic
keepalive HELLOs and rejected HELLOs do not print connected messages. This
reports AP-side registration acceptance, not receipt of the ACK by the STA.

Session teardown also prints `T1 play_back disconnect` or `T1 record disconnect`
immediately in the corresponding AP worker task, once per up-to-down transition.
Initial registration and repeated cleanup of an already-down session are silent.
This covers registration expiry (3 seconds without HELLO), socket errors and
replacement of an existing session; UDP does not report a remote disconnect
immediately. These transition logs are independent of the 10-second summaries.

Connected PLAY/REC lines with a positive shortfall have the exact prefix
`erro:AP T...` and ANSI red (`ESC[31m`), followed by a reset (`ESC[0m`) before
the newline. Zero-shortfall lines are not colored. A terminal with ANSI color
support is required; otherwise escape bytes may appear literally. UDP lines
and separators are not colored by this feature.

- PLAY: each frame represents 10 ms PCM. `sent` means all fragments were
  accepted by the local socket, not acknowledged by the STA. `drop_q` counts
  queue discards, `drop_to` stale-send discards, and `drop_err` pending frames
  discarded during error/disconnect/session replacement cleanup. A frame is
  counted only once. Expected = 1000. The explicit drop counters exclude USB
  ingress drops, but throughput shortfall can reflect any upstream shortage.
- REC: `recv` is the interval delta of `record_packets`: real 5 ms PCM blocks
  processed by `uacm_enqueue_record_5ms` after successful decoding (or accepted
  raw PCM). Expected = 2000. Each successfully decoded REC20 contributes four
  blocks, each REC10 two; partial REC20 decode contributes only its successful
  half. PLC-generated blocks are excluded. This is received/decoded audio volume,
  not a guarantee of later USB delivery: jitter-ring discards can still occur.
  `missing` remains a network application-frame sequence count, NOT 5 ms blocks.
  `late` also includes duplicate
  sequences reaching the tracker (duplicates rejected by reassembly do not
  reach it). `decode_fail` counts at most once per received application frame,
  including REC20 subframe decode/CRC/length errors. Queue drops after decoding
  are excluded. A recording frame may represent 10 or 20 ms.
- UDP: `tx_pkt` counts successfully submitted audio datagrams; `rx_pkt` counts
  audio datagrams passing source/identity/basic fragment checks, including
  duplicates and old fragments. HELLO, ACK and controls are excluded.
  `tx_busy` counts temporary failed audio send attempts, not lost frames.
  `asm_timeout` counts each incomplete assembly whose age exceeds 30 ms once,
  even without another datagram arriving. An assembly superseded by a newer
  frame before that deadline is not a timeout; subsequent sequence gaps can
  still expose its loss. `rx_gap_max_ms` is the maximum interval between
  accepted complete recording frames, not network latency. It excludes
  inactive recording periods and resets its time baseline on session/stream
  changes; it cannot report an ongoing final silence until another frame arrives.

Both rates now measure nominal throughput shortfall, NOT the previous local
drop/sequence-gap ratios; the existing field names are retained. They are
max(expected - actual, 0) / expected, rounded to two decimals. No traffic prints
0/1000 or 0/2000 and 100.00% for connected but inactive STAs; disconnected
directions instead print `down`. Only compare
full, continuously active windows. Startup/stop windows can show shortfall
without packet loss. Mixed REC20/REC10 grouping no longer changes the PCM-unit
target. Window-boundary bursts may exceed 2000 and are clamped to 0.00% shortfall.
Scheduling delays can also
make actual window duration slightly exceed 10 seconds. Sequence gaps are
immediate real-time gaps, not a
measurement of pure RF loss. First-frame, final-tail and pre-sequence losses
cannot be inferred. Missing and assembly timeout may describe the same loss;
do not add them. Receive sequence baselines reset on observed microphone stream
changes and session replacement. Cumulative diagnostic counters survive reconnects.

UART output runs in the playback network task and can affect scheduling,
especially with slow baud rates. If measurements perturb playback, move the
snapshot output to a low-priority diagnostic task or lengthen the interval.
