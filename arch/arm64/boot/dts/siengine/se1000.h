#ifndef __SIENGINE_SE1000_DTS_H__
#define __SIENGINE_SE1000_DTS_H__

/* for AP/CP choice */
#define ARM64_CORE_CP 0
#define ARM64_CORE_AP 1

/* if ARM64_CORE_AP dts config as ap mode, other wise config as ARM64_CORE_CP cp mode */
#define ENABLE_AP_CP_SIENGINE	ARM64_CORE_AP
#define EMMC_ENABLE 1
// #define SD_ENABLE 1
#endif /* __SIENGINE_SE1000_DTS_H__ */
