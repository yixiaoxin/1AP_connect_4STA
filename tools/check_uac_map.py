"""Check a fresh GNU ld map for DRAM, heap and fixed Wi-Fi receive regions."""
import argparse
from pathlib import Path
import re
import sys

p = argparse.ArgumentParser()
p.add_argument('map', type=Path)
args = p.parse_args()
s = args.map.read_text(encoding='utf-8', errors='replace')
def section(name):
    m = re.search(r'^' + re.escape(name) + r'\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)', s, re.M)
    if not m:
        raise ValueError('Missing section ' + name)
    return tuple(int(x, 16) for x in m.groups())
def symbol(name):
    m = re.search(r'(0x[0-9a-fA-F]+)\s+' + re.escape(name) + r'\s*=', s)
    if not m:
        raise ValueError('Missing symbol ' + name)
    return int(m.group(1), 16)
def region(name):
    m = re.search(r'^' + name + r'\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+[!a-z]+', s, re.M)
    return tuple(int(x, 16) for x in m.groups()) if m else None

try:
    dram = region('DRAM')
    if not dram:
        raise ValueError('Missing DRAM region')
    heap_start, heap_size = section('.heap')
    stack_limit = symbol('__StackLimit')
    static_end = max(sum(section(n)) for n in ('.data', '.bss', '.uninited'))
    heap_region = region('D_HEAP') or dram
    if heap_region == dram:
        static_end = max(static_end, heap_start + heap_size)
    margin = stack_limit - static_end
    heap_ok = heap_start >= heap_region[0] and heap_start + heap_size <= sum(heap_region)
    print('DRAM: 0x%x..0x%x (%d bytes)' % (dram[0], sum(dram), dram[1]))
    print('Data/heap end: 0x%x; main stack bottom: 0x%x; margin: %d bytes' %
          (static_end, stack_limit, margin))
    print('RTOS heap: 0x%x..0x%x (%d bytes); region bounds: %s' %
          (heap_start, heap_start + heap_size, heap_size, 'PASS' if heap_ok else 'FAIL'))
    ok = margin >= 0 and heap_ok and dram[0] <= stack_limit < sum(dram)
    stack_start, stack_size = section('.stack_dummy')
    stack_ok = (dram[0] <= stack_start and stack_start + stack_size <= sum(dram)
                and stack_limit + stack_size == sum(dram))
    print('Main stack reservation: %d bytes; bounds: %s' %
          (stack_size, 'PASS' if stack_ok else 'FAIL'))
    ok = ok and stack_ok
    for section_name, region_name in [('HOST_RXBUF', 'host_rxbuf_memory'),
                                       ('IPC_SHARED', 'ipc_shared_memory')]:
        bounds = region(region_name)
        if bounds is None:
            print('%s: region absent; not checked' % section_name)
            continue
        start, size = section(section_name)
        free_tail = sum(bounds) - (start + size)
        fits = start >= bounds[0] and free_tail >= 0
        print('%s: %d / %d bytes; tail margin: %d bytes; %s' %
              (section_name, size, bounds[1], free_tail, 'PASS' if fits else 'FAIL'))
        ok = ok and fits
    print('PASS (static layout only; measure runtime heap/stacks on hardware)' if ok else 'FAIL: memory overlap')
    sys.exit(0 if ok else 1)
except ValueError as e:
    p.error(str(e))
