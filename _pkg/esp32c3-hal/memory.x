/* ESP32-C3 */
/* All these values assume the flash cache is on

   see:
*/
MEMORY
{
  /* ICache size is fixed to 16KB on ESP32-C3 */
  /* ICACHE : ORIGIN = 0x4037C000, len = 16k */

  IRAM     (RX) : ORIGIN = (0x4037C000 + 0x4000), LENGTH = 320k
  DRAM     (RW) : ORIGIN = (0x3FC7C000 + 0x4000), LENGTH = 320k

  /* IROM     (RX) : ORIGIN = 0x42000020, LENGTH = 0x8000000-0x20 */
  /* DROM     (R)  : ORIGIN = 0x3C000020, LENGTH = 0x8000000-0x20 */

  /* RTC_IRAM (RWX): ORIGIN = 0x50000000, LENGTH = 8k */
}

REGION_ALIAS("REGION_TEXT", IRAM);
REGION_ALIAS("REGION_RODATA", DRAM);
REGION_ALIAS("REGION_DATA", DRAM);
REGION_ALIAS("REGION_BSS", DRAM);
REGION_ALIAS("REGION_HEAP", DRAM);
REGION_ALIAS("REGION_STACK", DRAM);

// this is from the esp go sdk

/* For the garbage collector.
 */
_globals_start = _sdata;
_globals_end = _ebss;
_heap_start = _ebss;
_heap_end = ORIGIN(DRAM) + LENGTH(DRAM);

_stack_size = 4K;

/* From ESP-IDF:
 * components/esp_rom/esp32/ld/esp32.rom.newlib-funcs.ld
 * This is the subset that is sometimes used by LLVM during codegen, and thus
 * must always be present.
 */
memcpy  = 0x4000c2c8;
memmove = 0x4000c3c0;
memset  = 0x4000c44c;

/* From ESP-IDF:
 * components/esp_rom/esp32/ld/esp32.rom.libgcc.ld
 * These are called from LLVM during codegen. The original license is Apache
 * 2.0, but I believe that a list of function names and addresses can't really
 * be copyrighted.
 */
__absvdi2      = 0x4006387c;
__absvsi2      = 0x40063868;
__adddf3       = 0x40002590;
__addsf3       = 0x400020e8;
__addvdi3      = 0x40002cbc;
__addvsi3      = 0x40002c98;
__ashldi3      = 0x4000c818;
__ashrdi3      = 0x4000c830;
__bswapdi2     = 0x40064b08;
__bswapsi2     = 0x40064ae0;
__clrsbdi2     = 0x40064b7c;
__clrsbsi2     = 0x40064b64;
__clzdi2       = 0x4000ca50;
__clzsi2       = 0x4000c7e8;
__cmpdi2       = 0x40063820;
__ctzdi2       = 0x4000ca64;
__ctzsi2       = 0x4000c7f0;
__divdc3       = 0x400645a4;
__divdf3       = 0x40002954;
__divdi3       = 0x4000ca84;
__divsi3       = 0x4000c7b8;
__eqdf2        = 0x400636a8;
__eqsf2        = 0x40063374;
__extendsfdf2  = 0x40002c34;
__ffsdi2       = 0x4000ca2c;
__ffssi2       = 0x4000c804;
__fixdfdi      = 0x40002ac4;
__fixdfsi      = 0x40002a78;
__fixsfdi      = 0x4000244c;
__fixsfsi      = 0x4000240c;
__fixunsdfsi   = 0x40002b30;
__fixunssfdi   = 0x40002504;
__fixunssfsi   = 0x400024ac;
__floatdidf    = 0x4000c988;
__floatdisf    = 0x4000c8c0;
__floatsidf    = 0x4000c944;
__floatsisf    = 0x4000c870;
__floatundidf  = 0x4000c978;
__floatundisf  = 0x4000c8b0;
__floatunsidf  = 0x4000c938;
__floatunsisf  = 0x4000c864;
__gcc_bcmp     = 0x40064a70;
__gedf2        = 0x40063768;
__gesf2        = 0x4006340c;
__gtdf2        = 0x400636dc;
__gtsf2        = 0x400633a0;
__ledf2        = 0x40063704;
__lesf2        = 0x400633c0;
__lshrdi3      = 0x4000c84c;
__ltdf2        = 0x40063790;
__ltsf2        = 0x4006342c;
__moddi3       = 0x4000cd4c;
__modsi3       = 0x4000c7c0;
__muldc3       = 0x40063c90;
__muldf3       = 0x4006358c;
__muldi3       = 0x4000c9fc;
__mulsf3       = 0x400632c8;
__mulsi3       = 0x4000c7b0;
__mulvdi3      = 0x40002d78;
__mulvsi3      = 0x40002d60;
__nedf2        = 0x400636a8;
__negdf2       = 0x400634a0;
__negdi2       = 0x4000ca14;
__negsf2       = 0x400020c0;
__negvdi2      = 0x40002e98;
__negvsi2      = 0x40002e78;
__nesf2        = 0x40063374;
__nsau_data    = 0x3ff96544;
__paritysi2    = 0x40002f3c;
__popcount_tab = 0x3ff96544;
__popcountdi2  = 0x40002ef8;
__popcountsi2  = 0x40002ed0;
__powidf2      = 0x400638e4;
__subdf3       = 0x400026e4;
__subsf3       = 0x400021d0;
__subvdi3      = 0x40002d20;
__subvsi3      = 0x40002cf8;
__truncdfsf2   = 0x40002b90;
__ucmpdi2      = 0x40063840;
__udiv_w_sdiv  = 0x40064bec;
__udivdi3      = 0x4000cff8;
__udivmoddi4   = 0x40064bf4;
__udivsi3      = 0x4000c7c8;
__umoddi3      = 0x4000d280;
__umodsi3      = 0x4000c7d0;
__umulsidi3    = 0x4000c7d8;
__unorddf2     = 0x400637f4;
__unordsf2     = 0x40063478;
