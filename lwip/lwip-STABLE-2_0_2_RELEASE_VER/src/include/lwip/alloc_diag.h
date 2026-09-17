/* Allocation failure diagnostics for the AP SDK. No allocation on failure.
 * Each call site reports first failure and at most once per second thereafter.
 * Counters include suppressed events; printing is outside SYS_ARCH_PROTECT. */
#ifndef LWIP_ALLOC_DIAG_H
#define LWIP_ALLOC_DIAG_H
#include "lwip/sys.h"
#include "dbg.h"
#define LWIP_ALLOC_FAIL(site, bytes) do { \
  static u32_t diag_count, diag_last_ms; \
  u32_t diag_now = sys_now(), diag_snapshot; \
  int diag_report; \
  SYS_ARCH_DECL_PROTECT(diag_level); \
  SYS_ARCH_PROTECT(diag_level); \
  diag_snapshot = ++diag_count; \
  diag_report = diag_snapshot == 1U || (u32_t)(diag_now - diag_last_ms) >= 1000U; \
  if (diag_report) diag_last_ms = diag_now; \
  SYS_ARCH_UNPROTECT(diag_level); \
  if (diag_report) dbg("LWIP ALLOC_FAIL site=%s count=%u bytes=%u t=%u\n", \
      site, (unsigned)diag_snapshot, (unsigned)(bytes), (unsigned)diag_now); \
} while (0)
#endif
