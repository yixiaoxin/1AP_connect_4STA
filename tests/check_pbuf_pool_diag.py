"""Exercise production pool mutation/snapshot code with a tiny fixed host pool."""
from pathlib import Path
import subprocess
root = Path(__file__).resolve().parents[1]
src = (root / 'lwip/lwip-STABLE-2_0_2_RELEASE_VER/src/core/memp.c').read_text()
header = (root / 'lwip/lwip-STABLE-2_0_2_RELEASE_VER/src/include/lwip/pbuf_diag.h').read_text()
struct = header[header.index('struct memp_pbuf_diag {'):header.index('void memp_pbuf_diag_get')]
snapshot = src[src.index('static struct memp_pbuf_diag pbuf_diag;'):src.index('static void*\n#if !MEMP_OVERFLOW_CHECK')]
alloc = src[src.index('static void*\n#if !MEMP_OVERFLOW_CHECK'):src.index('/**\n * Get an element from a custom pool.')]
free = src[src.index('static void\ndo_memp_free_pool'):src.index('/**\n * Put a custom pool element back')]
prefix = r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <stdio.h>
typedef uint32_t u32_t;
typedef uint8_t u8_t;
typedef uintptr_t mem_ptr_t;
#define MEMP_MEM_MALLOC 0
#define MEMP_OVERFLOW_CHECK 0
#define MEMP_STATS 0
#define MEMP_SANITY_CHECK 0
#define MEMP_SIZE sizeof(struct memp)
#define MEM_ALIGNMENT sizeof(void *)
#define MEMP_PBUF 0
#define SYS_ARCH_DECL_PROTECT(x) unsigned x
#define SYS_ARCH_PROTECT(x) do { x = 0; } while (0)
#define SYS_ARCH_UNPROTECT(x) ((void)(x))
#define LWIP_ASSERT(msg, expr) assert(expr)
#define LWIP_DEBUGF(level, msg) ((void)0)
#define dbg(...) ((void)0)
struct memp { struct memp *next; };
struct memp_desc { struct memp **tab; unsigned num; };
static struct memp *head;
static struct memp_desc pool = {&head, 2};
static const struct memp_desc *memp_pools[] = {&pool};
'''
main = r'''
int main(void) {
    struct { struct memp h; uintptr_t payload; } storage[2];
    struct memp_pbuf_diag d;
    void *a, *b;
    head = &storage[0].h; storage[0].h.next = &storage[1].h;
    storage[1].h.next = NULL;
    a = do_memp_malloc_pool(&pool); b = do_memp_malloc_pool(&pool);
    assert(a && b && !do_memp_malloc_pool(&pool));
    memp_pbuf_diag_get(&d);
    assert(d.capacity == 2 && d.used == 2 && d.peak == 2);
    assert(d.alloc_ok == 2 && d.fail == 1 && !d.freed && !d.heap_mode);
    do_memp_free_pool(&pool, a);
    memp_pbuf_diag_get(&d);
    assert(d.used == 1 && d.freed == 1 && d.window_peak == 2);
    memp_pbuf_diag_log();
    memp_pbuf_diag_get(&d); assert(d.window_peak == 1 && d.peak == 2);
    a = do_memp_malloc_pool(&pool); assert(a);
    do_memp_free_pool(&pool, a); do_memp_free_pool(&pool, b);
    memp_pbuf_diag_get(&d);
    assert(!d.used && d.alloc_ok == d.freed && !d.free_underflow);
    assert(d.fail == 1 && d.peak == 2 && d.window_peak == 2);
    puts("PASS: production MEMP_PBUF exhaustion/recovery, counts, peaks and snapshot");
    return 0;
}
'''
out = root / 'build/uac_udp4_tests'
out.mkdir(parents=True, exist_ok=True)
c = out / 'pool_diag.c'; exe = out / 'pool_diag'
c.write_text(prefix + struct + snapshot + alloc + free + main)
subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                '-fsanitize=undefined', str(c), '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
