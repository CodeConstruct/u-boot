#include <config.h>

#include <ppc_asm.tmpl>

#include <asm/cache.h>

_GLOBAL(invalidate_l1_instruction_cache)
	li	r3, 0
	isync
	icbi	0, r3
	blr

_GLOBAL(invalidate_l1_data_cache)
	li	r3, 0
	dcbf	0, r3
	blr

_GLOBAL(flush_dcache)
	li	r3, 0
	dcbf	0, r3
	blr

_GLOBAL(flush_icache_range)
	isync
	icbi	0, r3
	blr

_GLOBAL(clean_dcache_range)
	blr

_GLOBAL(icache_enable)
	blr

_GLOBAL(icache_disable)
	blr

_GLOBAL(icache_status)
	li	r3, 1
	blr

_GLOBAL(l1dcache_enable)
	blr

_GLOBAL(dcache_enable)
	blr

_GLOBAL(dcache_disable)
	blr

_GLOBAL(dcache_status)
	li	r3, 1
	blr

_GLOBAL(l2cache_invalidate)
	blr

_GLOBAL(l2cache_enable)
	blr

_GLOBAL(l2cache_disable)
	blr
