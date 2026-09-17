#ifndef LWIP_PBUF_DIAG_H
#define LWIP_PBUF_DIAG_H
#include "lwip/arch.h"
/* Lifetime counters. Snapshot is coherent under SYS_ARCH_PROTECT. */
struct memp_pbuf_diag {
  u32_t capacity, used, peak, alloc_ok, freed, fail, free_underflow;
  u32_t window_peak;
  u8_t heap_mode;
};
void memp_pbuf_diag_get(struct memp_pbuf_diag *out);
void memp_pbuf_diag_log(void);
#endif
