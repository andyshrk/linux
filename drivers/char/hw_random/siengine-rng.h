/*
 * siengine-rng.h
 *
 *  Created on: Jan 17,2022
 *      Author: mingfei.wu
 */

#ifndef DRIVERS_CHAR_HW_RANDOM_SIENGINE_RNG_H_
#define DRIVERS_CHAR_HW_RANDOM_SIENGINE_RNG_H_

#define SIENGINE_TRNG_REG_CTRL                  0x00
#define SIENGINE_TRNG_REG_STAT                  0x04
#define SIENGINE_TRNG_REG_MODE                  0x08
#define SIENGINE_TRNG_REG_SMODE                 0x0c
#define SIENGINE_TRNG_REG_IE                    0x10
#define SIENGINE_TRNG_REG_ISTAT                 0x14
#define SIENGINE_TRNG_REG_COREKIT_REL   0x18
#define SIENGINE_TRNG_REG_FEATURES              0x1c
#define SIENGINE_TRNG_REG_RAND0                 0x20
#define SIENGINE_TRNG_REG_RAND1                 0x24
#define SIENGINE_TRNG_REG_RAND2                 0x28
#define SIENGINE_TRNG_REG_RAND3                 0x2c
#define SIENGINE_TRNG_REG_RAND4                 0x30
#define SIENGINE_TRNG_REG_RAND5                 0x34
#define SIENGINE_TRNG_REG_RAND6                 0x38
#define SIENGINE_TRNG_REG_RAND7                 0x3c
#define SIENGINE_TRNG_REG_SEED0                 0x40
#define SIENGINE_TRNG_REG_SEED1                 0x44
#define SIENGINE_TRNG_REG_SEED2                 0x48
#define SIENGINE_TRNG_REG_SEED3                 0x4c
#define SIENGINE_TRNG_REG_SEED4                 0x50
#define SIENGINE_TRNG_REG_SEED5                 0x54
#define SIENGINE_TRNG_REG_SEED6                 0x58
#define SIENGINE_TRNG_REG_SEED7                 0x5c
#define SIENGINE_TRNG_REG_AUTO_RQSTS    0x60
#define SIENGINE_TRNG_REG_AUTO_AGE              0x64
#define SIENGINE_TRNG_REG_TIME_TO_SEED  0x6c
#define SIENGINE_TRNG_REG_BUILD_CFG0    0xf0

#define SIENGINE_RNG_SEED_BASE                  SIENGINE_TRNG_REG_SEED0
#define SIENGINE_RNG_SEED(n)                    (SIENGINE_RNG_SEED_BASE + (n * 0x4))
#define SIENGINE_RNG_OUT_BASE                   SIENGINE_TRNG_REG_RAND0
#define SIENGINE_RNG_OUT(n)                             (SIENGINE_RNG_OUT_BASE + (n * 0x4))

/* num of seed need init */
#define SIENGINE_RNG_SEED_REG_NUM               8

/* num of rand register on 128bits mode */
#define SIENGINE_RNG_RAND_REG_4_NUM             4

/* num of rand register on 256bits mode */
#define SIENGINE_RNG_RAND_REG_8_NUM             8

/* CTRL */
#define TRNG_REG_CTRL_CMD_NOP                   0
#define TRNG_REG_CTRL_CMD_GEN_RAND              1
#define TRNG_REG_CTRL_CMD_RAND_RESEED   2
#define TRNG_REG_CTRL_CMD_NONCE_RESEED  3

/* MODE */

#define _TRNG_REG_MODE_R256                             3

#define TRNG_REG_MODE_R256                              BIT(_TRNG_REG_MODE_R256)

/* SMODE */

#define _TRNG_REG_SMODE_MISSION_MODE    8
#define _TRNG_REG_SMODE_NONCE_MODE              2

#define TRNG_REG_SMODE_MISSION_MODE             BIT(_TRNG_REG_SMODE_MISSION_MODE)
#define TRNG_REG_SMODE_NONCE_MODE               BIT(_TRNG_REG_SMODE_NONCE_MODE)

/* STAT */
#define _TRNG_REG_STAT_RAND_RESEEDING   31
#define _TRNG_REG_STAT_RAND_GENERATING  30
#define _TRNG_REG_STAT_SRVC_RQST                27
#define _TRNG_REG_STAT_LAST_RESEED_2    18
#define _TRNG_REG_STAT_LAST_RESEED_1    17
#define _TRNG_REG_STAT_LAST_RESEED_0    16
#define _TRNG_REG_STAT_SEEDED                   9
#define _TRNG_REG_STAT_MISSION_MODE             8
#define _TRNG_REG_STAT_R256                             3
#define _TRNG_REG_STAT_NONCE_MODE               2

#define TRNG_REG_STAT_RAND_RESEEDING    BIT(_TRNG_REG_STAT_RAND_RESEEDING)
#define TRNG_REG_STAT_RAND_GENERATING   BIT(_TRNG_REG_STAT_RAND_GENERATING)
#define TRNG_REG_STAT_SRVC_RQST                 BIT(_TRNG_REG_STAT_SRVC_RQST)
#define TRNG_REG_STAT_LAST_RESEED               (BIT(_TRNG_REG_STAT_LAST_RESEED_2)|BIT(_TRNG_REG_STAT_LAST_RESEED_1)|BIT(_TRNG_REG_STAT_LAST_RESEED_0))
#define TRNG_REG_STAT_SEEDED                    BIT(_TRNG_REG_STAT_SEEDED)
#define TRNG_REG_STAT_MISSION_MODE              BIT(_TRNG_REG_STAT_MISSION_MODE)
#define TRNG_REG_STAT_R256                              BIT(_TRNG_REG_STAT_R256)
#define TRNG_REG_STAT_NONCE_MODE                BIT(_TRNG_REG_STAT_NONCE_MODE)

/* IE */
#define _TRNG_REG_ISTAT_GLBL_EN                 31
#define _TRNG_REG_ISTAT_LFSR_LOOKUP_EN  4
#define _TRNG_REG_ISTAT_RQST_ALARMS_EN  3
#define _TRNG_REG_ISTAT_AGE_ALARM_EN    2
#define _TRNG_REG_ISTAT_SEED_DONE_EN    1
#define _TRNG_REG_ISTAT_RAND_RDY_EN             0
#define TRNG_REG_ISTAT_GLBL_EN                  BIT(_TRNG_REG_ISTAT_GLBL_EN)
#define TRNG_REG_ISTAT_LFSR_LOOKUP_EN   BIT(_TRNG_REG_ISTAT_LFSR_LOOKUP_EN)
#define TRNG_REG_ISTAT_RQST_ALARMS_EN   BIT(_TRNG_REG_ISTAT_RQST_ALARMS_EN)
#define TRNG_REG_ISTAT_AGE_ALARM_EN             BIT(_TRNG_REG_ISTAT_AGE_ALARM_EN)
#define TRNG_REG_ISTAT_SEED_DONE_EN             BIT(_TRNG_REG_ISTAT_SEED_DONE_EN)
#define TRNG_REG_ISTAT_RAND_RDY_EN              BIT(_TRNG_REG_ISTAT_RAND_RDY_EN)

/* ISTAT */
#define _TRNG_REG_ISTAT_LFSR_LOOKUP             4
#define _TRNG_REG_ISTAT_RQST_ALARMS             3
#define _TRNG_REG_ISTAT_AGE_ALARM               2
#define _TRNG_REG_ISTAT_SEED_DONE               1
#define _TRNG_REG_ISTAT_RAND_RDY                0

#define TRNG_REG_ISTAT_LFSR_LOOKUP              BIT(_TRNG_REG_ISTAT_LFSR_LOOKUP)
#define TRNG_REG_ISTAT_RQST_ALARMS              BIT(_TRNG_REG_ISTAT_RQST_ALARMS)
#define TRNG_REG_ISTAT_AGE_ALARM                BIT(_TRNG_REG_ISTAT_AGE_ALARM)
#define TRNG_REG_ISTAT_SEED_DONE                BIT(_TRNG_REG_ISTAT_SEED_DONE)
#define TRNG_REG_ISTAT_RAND_RDY                 BIT(_TRNG_REG_ISTAT_RAND_RDY)

/* BUILD_CFG0 */
#define TRNG_REG_BUILD_CFG0_AUTO_RESEED_LOOPBACK    BIT(5)
#define TRNG_REG_BUILD_CFG0_MAX_PRNG_LEN            BIT(2)

/* FEATURES */
#define TRNG_REG_FEATURES_MAX_PRNG_LEN(x)                       (((x)>>2)&1)
#define TRNG_REG_FEATURES_AUTO_RESEED_LOOPBACK(x)       (((x)>>5)&1)

#define TRNG_AGE_MAX                                0xffff


#endif /* DRIVERS_CHAR_HW_RANDOM_SIENGINE_RNG_H_ */
