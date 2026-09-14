# AP audio diagnostics

Every approximately 10 seconds the network task prints three lines per STA.
Actual counters are interval deltas, not lifetime totals. Expected values are
fixed: playback 1000 frames (10 ms/frame), recording 500 frames (20 ms/frame).
They are not calculated from actual counts or sequence gaps. No logging occurs in
the USB ISR; only a short counter snapshot is protected, never UART output.

```text
AP T1 PLAY sent=996/1000 drop_q=2 drop_to=1 drop_err=1 local_loss=0.40%
AP T1 REC recv=498/500 missing=2 late=0 decode_fail=0 gap_rate=0.40%
AP T1 UDP tx_pkt=1500 rx_pkt=800 tx_busy=3 asm_timeout=1 rx_gap_max_ms=42
==========
```

- PLAY: each frame represents 10 ms PCM. `sent` means all fragments were
  accepted by the local socket, not acknowledged by the STA. `drop_q` counts
  queue discards, `drop_to` stale-send discards, and `drop_err` pending frames
  discarded during error/disconnect/session replacement cleanup. A frame is
  counted only once. Expected = 1000. The explicit drop counters exclude USB
  ingress drops, but throughput shortfall can reflect any upstream shortage.
- REC: `recv` counts complete, header-valid, accepted new audio frames before
  decoding. Expected = 500. `late` also includes duplicate
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
0/1000 or 0/500 and 100.00%, including disconnected/inactive STAs. Only compare
full, continuously active windows. Startup/stop windows can show shortfall
without packet loss. REC10 fallback can exceed 500 frames and prints 0.00%;
its ratio is not a reliable measure of audio loss. Scheduling delays can also
make actual window duration slightly exceed 10 seconds. Sequence gaps are
immediate real-time gaps, not a
measurement of pure RF loss. First-frame, final-tail and pre-sequence losses
cannot be inferred. Missing and assembly timeout may describe the same loss;
do not add them. Receive sequence baselines reset on observed microphone stream
changes and session replacement. Cumulative diagnostic counters survive reconnects.

UART output runs in the playback network task and can affect scheduling,
especially with slow baud rates. If measurements perturb playback, move the
snapshot output to a low-priority diagnostic task or lengthen the interval.
