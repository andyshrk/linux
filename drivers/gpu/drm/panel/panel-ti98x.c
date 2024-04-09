#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <asm/unaligned.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_panel.h>
#include <video/of_display_timing.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif
#ifdef CONFIG_IPO_MANAGER
#include <linux/ipo_manager.h>
#endif

//#define DEBUG

#define csd_printk_error   pr_err
#define csd_printk_warning pr_warn
#define csd_printk_info    pr_info
#define csd_printk_debug   pr_debug

#define DEVICE_NAME "geely-2k-touchscreen"
#define aaa135 0

#define UB98X_MODE_HPD_FORCE_TRIGE 0x01
#define UB98X_MODE_VP_RESET        0x02
#define I2C_7BIT_ADDR(x)   (x>>1)
#define UB98X_MODE_VP_CONFIG           0x02
#define UB98X_MODE_SER_TX_LINK_CONFIG  0x04
#define UB98X_MODE_DES_CONFIG          0x08
#define UB98X_MODE_DES_DP_CONFIG       0x10
#define UB98X_MODE_DES_DP_CONFIG_PART2 0x11
#define UB98X_MODE_SER_PATGEN_CONFIG   0x20
#define UB98X_MODE_DES_PATGEN_CONFIG   0x40

#define PAGE_09 9
#define PAGE_12 12

//page 0 reg
#define FPD3_ICR 0xc6
#define FPD3_ISR 0xc7


//page 12 reg
#define INTR_CTL_VP_VP0 0x33
#define INTR_CTL_VP_VP1 0x73
#define INTR_CTL_VP_VP2 0xb3
#define INTR_CTL_VP_VP3 0xf3

//page 12 reg
#define INTR_STS_VP_VP0 0x31
#define INTR_STS_VP_VP1 0x71
#define INTR_STS_VP_VP2 0xb1
#define INTR_STS_VP_VP3 0xf1

//page 9 reg
#define INTR_CTL_DP_RX_PORT 0x3e
#define INTR_STS_DP_RX_PORT 0x3f

//page 9 reg
#define INTR_STS_FPD4_PORT0 0x8d
#define INTR_STS_FPD4_PORT1 0x9d

//page 9 reg
#define INTR_CTL_FPD4_PORT0 0x8c
#define INTR_CTL_FPD4_PORT1 0x9c

//page 9 reg
#define DISPLAYPORT_INTERRUPT_MASK 0x180
#define DISPLAYPORT_INTERRUPT_CAUSE 0x188

#define SERADDR    (0x18)
#define DESADDR0   (0x58)
#define DESALIAS0  (0x58)

struct geely_data {
	struct i2c_client *client;
	int state;

	bool wake_irq_enabled;
	bool keep_power_in_suspend;

	int gpio_vdd1v8_en;
	int gpio_vdd1v1_en;
	int gpio_pdb;
	int gpio_bl_en1;
	int gpio_bl_en2;
	int gpio_bl_pwm1;
	int gpio_bl_pwm2;

	struct mutex rw_lock;
	char input_name[128];

	struct drm_panel base;
	struct videomode vm;
	u32 width_mm;
	u32 height_mm;
};

struct ub98x_reg_cmds{
	char i2c_addr;
	char addr;
	char val;
};

struct ub98x_cmds{
	char mode_type;
	const struct ub98x_reg_cmds *cmds;
};

unsigned int has_error = 0;
struct geely_data *g_ts = NULL;

const struct ub98x_reg_cmds reg_init_cmd0[] = {
	{SERADDR, 0x70, DESADDR0},
	{SERADDR, 0x78, DESALIAS0},
	{SERADDR, 0x88, 0x0},
	// Program SER to FPD-Link IV mode;
	{SERADDR, 0x5b, 0x23},//Disable FPD3 FIFO pass through;
	{SERADDR, 0x5,  0x2c},//Force FPD4_TX single port 0 mode;
	// Set up FPD IV PLL Settings - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed;
	{SERADDR, 0x40, 0x8},//Select PLL reg page;
	{SERADDR, 0x41, 0x1b},
	{SERADDR, 0x42, 0x8},//Disable PLL0
	{SERADDR, 0x41, 0x5b},
	{SERADDR, 0x42, 0x8},//Disable PLL1;
	{SERADDR, 0x2,  0xd1},//Enable mode overwrite;
	{SERADDR, 0x2d, 0x1},
	{SERADDR, 0x40, 0x8},//Select PLL page;
	{SERADDR, 0x41, 0x5},//Select Ncount Reg;
	{SERADDR, 0x42, 0x7d},//Set Ncount;
	{SERADDR, 0x41, 0x13},//Select post div reg;
	#if aaa135
	{SERADDR,0x42,0x80},//Set post div for 13.5 Gbps;
	#else
	{SERADDR,0x42,0x90},//Set post div for 6.75 Gbps;
	#endif
	{SERADDR,0x2d,0x1},//Select write reg to port 0;
	#if aaa135
	{SERADDR,0x6a,0x4a},//set BC sampling rate;
	#else
	{SERADDR,0x6a,0xa},//set BC sampling rate;
	#endif
	{SERADDR,0x6e,0x80},//set BC fractional sampling;
	{SERADDR,0x40,0x4},//Select FPD page and set BC settings for FPD IV port 0;
	{SERADDR,0x41,0x6},
	{SERADDR,0x42,0x0},
	{SERADDR,0x41,0xd},
	{SERADDR,0x42,0x34},
	{SERADDR,0x41,0xe},
	{SERADDR,0x42,0x53},
	{SERADDR,0x40,0x8},//Select PLL page;
	{SERADDR,0x41,0x45},//Select Ncount Reg;
	{SERADDR,0x42,0x7d},//Set Ncount;
	{SERADDR,0x41,0x53},//Select post div reg;
	#if aaa135
	{SERADDR,0x42,0x80},//Set post div for 13.5 Gbps;
	#else
	{SERADDR,0x42,0x90},//Set post div for 6.75 Gbps;
	#endif
	{SERADDR,0x2d,0x12},//Select write reg to port 1;
	#if aaa135
	{SERADDR,0x6a,0x4a},//set BC sampling rate;
	#else
	{SERADDR,0x6a,0xa},//set BC sampling rate;
	#endif
	{SERADDR,0x6e,0x80},//set BC fractional sampling;
	{SERADDR,0x40,0x4},//Select FPD page and set BC settings for FPD IV port 1;
	{SERADDR,0x41,0x26},
	{SERADDR,0x42,0x0},
	{SERADDR,0x41,0x2d},
	{SERADDR,0x42,0x34},
	{SERADDR,0x41,0x2e},
	{SERADDR,0x42,0x53},
	{SERADDR,0x2,0xd1},//Set HALFRATE_MODE;
	// Zero out PLL fractional - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed;
	{SERADDR,0x40,0x8},//Select PLL page;
	{SERADDR,0x41,0x4},
	{SERADDR,0x42,0x1},
	{SERADDR,0x41,0x1e},
	{SERADDR,0x42,0x0},
	{SERADDR,0x41,0x1f},
	{SERADDR,0x42,0x0},
	{SERADDR,0x41,0x20},
	{SERADDR,0x42,0x0},
	{SERADDR,0x41,0x44},
	{SERADDR,0x42,0x1},
	{SERADDR,0x41,0x5e},
	{SERADDR,0x42,0x0},
	{SERADDR,0x41,0x5f},
	{SERADDR,0x42,0x0},
	{SERADDR,0x41,0x60},
	{SERADDR,0x42,0x0},
	// Configure and Enable PLLs - This section can be commented out to improve bringup time if 983/981 MODE_SEL0 and MODE_SEL2 are strapped to the correct FPD IV speed;
	{SERADDR,0x41,0xe},//Select VCO reg;
	{SERADDR,0x42,0xc7},//Set VCO;
	{SERADDR,0x41,0x4e},//Select VCO reg;
	{SERADDR,0x42,0xc7},//Set VCO;
	{SERADDR,0x1,0x30},//soft reset PLL;
	{SERADDR,0x40,0x8},//Select PLL page;
	{SERADDR,0x41,0x1b},
	{SERADDR,0x42,0x0},//Enable PLL0;
	{SERADDR,0x1,0x1},//soft reset Ser;
};

//UB98X_MODE_HPD_FORCE_TRIGE
const struct ub98x_reg_cmds reg_init_cmd1[] = {
	{SERADDR,0x48,0x1},//Enable APB Interface;
	{SERADDR,0x49,0x0},//Force HPD low to configure 983 DP settings;
	{SERADDR,0x4a,0x0},
	{SERADDR,0x4b,0x0},
	{SERADDR,0x4c,0x0},
	{SERADDR,0x4d,0x0},
	{SERADDR,0x4e,0x0},
	{SERADDR,0x49,0x74},//Set max advertised link rate = 2.7Gbps;
	{SERADDR,0x4a,0x0},
	{SERADDR,0x4b,0xa},
	{SERADDR,0x4c,0x0},
	{SERADDR,0x4d,0x0},
	{SERADDR,0x4e,0x0},
	{SERADDR,0x49,0x70},//Set max advertised lane count = 4;
	{SERADDR,0x4a,0x0},
	{SERADDR,0x4b,0x4},
	{SERADDR,0x4c,0x0},
	{SERADDR,0x4d,0x0},
	{SERADDR,0x4e,0x0},
	{SERADDR,0x49,0x14},//Request min VOD swing of 0x02;
	{SERADDR,0x4a,0x2},
	{SERADDR,0x4b,0x2},
	{SERADDR,0x4c,0x0},
	{SERADDR,0x4d,0x0},
	{SERADDR,0x4e,0x0},
	{SERADDR,0x49,0x18},//Set SST/MST mode and DP/eDP Mode;
	{SERADDR,0x4a,0x0},
	{SERADDR,0x4b,0x14},
	{SERADDR,0x4c,0x0},
	{SERADDR,0x4d,0x0},
	{SERADDR,0x4e,0x0},
	{SERADDR,0x49,0x0},//Force HPD high to trigger link training;
	{SERADDR,0x4a,0x0},
	{SERADDR,0x4b,0x1},
	{SERADDR,0x4c,0x0},
	{SERADDR,0x4d,0x0},
	{SERADDR,0x4e,0x0},
};

const struct ub98x_reg_cmds reg_init_cmd2[] = {
	{SERADDR,0x49,0x54},//Video Input Reset (should be executed after DP video is available from the source},
	{SERADDR,0x4a,0x0},
	{SERADDR,0x4b,0x1},
	{SERADDR,0x4c,0x0},
	{SERADDR,0x4d,0x0},
	{SERADDR,0x4e,0x0},
	// Program VP Configs;
	{SERADDR,0x40,0x32},
	{SERADDR,0x41,0x1},
	{SERADDR,0x42,0xa8},//Set VP_SRC_SELECT to Stream 0 for SST Mode;
	{SERADDR,0x41,0x2},
	{SERADDR,0x42,0x0},//VID H Active;
	{SERADDR,0x42,0xa},//VID H Active;
	{SERADDR,0x41,0x10},
	{SERADDR,0x42,0x0},//Horizontal Active;
	{SERADDR,0x42,0xa},//Horizontal Active;
	{SERADDR,0x42,0x37},//Horizontal Back Porch;
	{SERADDR,0x42,0x0},//Horizontal Back Porch;
	{SERADDR,0x42,0x37},//Horizontal Sync;
	{SERADDR,0x42,0x0},//Horizontal Sync;
	{SERADDR,0x42,0xc4},//Horizontal Total;
	{SERADDR,0x42,0xa},//Horizontal Total;
	{SERADDR,0x42,0x40},//Vertical Active;
	{SERADDR,0x42,0x6},//Vertical Active;
	{SERADDR,0x42,0x8},//Vertical Back Porch;
	{SERADDR,0x42,0x0},//Vertical Back Porch;
	{SERADDR,0x42,0x2},//Vertical Sync;
	{SERADDR,0x42,0x0},//Vertical Sync;
	{SERADDR,0x42,0x20},//Vertical Front Porch;
	{SERADDR,0x42,0x0},//Vertical Front Porch;
	{SERADDR,0x41,0x27},
	{SERADDR,0x42,0x0},//HSYNC Polarity = +, VSYNC Polarity = +;
	{SERADDR,0x41,0x23},//M/N Register;
	#if aaa135
	{SERADDR,0x42,0xbe},//M value;
	{SERADDR,0x42,0x19},//M value;
	#else
	{SERADDR,0x42,0x7d},//M value;
	{SERADDR,0x42,0x33},//M value;
	#endif
	{SERADDR,0x42,0xf},//N value;
	// Enable VPs;
	{SERADDR,0x43,0x0},//Set number of VPs used = 1;
	{SERADDR,0x44,0x1},//Enable video processors;
};

//UB98X_MODE_SER_PATGEN_CONFIG
const struct ub98x_reg_cmds reg_init_cmd3[] = {
	{SERADDR,0x40,0x30},
	{SERADDR,0x41,0x29},
	{SERADDR,0x42,0x8},//Set PATGEN Color Depth to 24bpp for VP0;
	{SERADDR,0x41,0x28},
	{SERADDR,0x42,0x91},//Enable PATGEN on VP0 - Comment out this line to disable PATGEN and enable end to end video;
};

//UB98X_MODE_SER_TX_LINK_CONFIG
const struct ub98x_reg_cmds reg_init_cmd4[] = {
	{SERADDR,0x40,0x2e},//Link layer Reg page;
	{SERADDR,0x41,0x1},//Link layer 0 stream enable;
	{SERADDR,0x42,0x1},//Link layer 0 stream enable;
	{SERADDR,0x41,0x6},//Link layer 0 time slot 0;
	{SERADDR,0x42,0x41},//Link layer 0 time slot;
	{SERADDR,0x41,0x20},//Set Link layer vp bpp;
	{SERADDR,0x42,0x55},//Set Link layer vp bpp according to VP Bit per pixel;
	{SERADDR,0x41,0x0},//Link layer 0 enable;
	{SERADDR,0x42,0x3},//Link layer 0 enable;
};

//UB98X_MODE_DES_CONFIG
const struct ub98x_reg_cmds reg_init_cmd5[] = {
	{DESALIAS0,0xe,0x3},//Enable Write to P0 and P1;
	{DESALIAS0,0x61,0x0},
	{DESALIAS0,0x5a,0x74},
	{DESALIAS0,0x5f,0x4},
	{DESALIAS0,0x40,0x3c},
	{DESALIAS0,0x41,0xf5},
	{DESALIAS0,0x42,0x21},
	{DESALIAS0,0x40,0x54},
	{DESALIAS0,0x41,0x43},
	{DESALIAS0,0x42,0x3},
	{DESALIAS0,0x40,0x58},
	{DESALIAS0,0x41,0x43},
	{DESALIAS0,0x42,0x3},
	{DESALIAS0,0x40,0x54},
	{DESALIAS0,0x41,0x5},
	{DESALIAS0,0x42,0x0},
	{DESALIAS0,0x40,0x58},
	{DESALIAS0,0x41,0x5},
	{DESALIAS0,0x42,0x0},
	{DESALIAS0,0x40,0x54},
	{DESALIAS0,0x41,0x6},
	{DESALIAS0,0x42,0x1},
	{DESALIAS0,0x40,0x58},
	{DESALIAS0,0x41,0x6},
	{DESALIAS0,0x42,0x1},
	{DESALIAS0,0x40,0x54},
	{DESALIAS0,0x41,0x37},
	{DESALIAS0,0x42,0x32},
	{DESALIAS0,0x40,0x58},
	{DESALIAS0,0x41,0x37},
	{DESALIAS0,0x42,0x32},
	{DESALIAS0,0x40,0x54},
	{DESALIAS0,0x41,0x8d},
	{DESALIAS0,0x42,0xff},
	{DESALIAS0,0x40,0x58},
	{DESALIAS0,0x41,0x8d},
	{DESALIAS0,0x42,0xff},
	{DESALIAS0,0x40,0x5c},
	{DESALIAS0,0x41,0x20},
	{DESALIAS0,0x42,0x3c},
	{DESALIAS0,0x40,0x5c},
	{DESALIAS0,0x41,0xa0},
	{DESALIAS0,0x42,0x3c},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x24},
	{DESALIAS0,0x42,0x61},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x54},
	{DESALIAS0,0x42,0x61},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x2c},
	{DESALIAS0,0x42,0x19},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x5c},
	{DESALIAS0,0x42,0x19},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x2e},
	{DESALIAS0,0x42,0x0},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x5e},
	{DESALIAS0,0x42,0x0},
	{DESALIAS0,0x40,0x10},
	{DESALIAS0,0x41,0x18},
	{DESALIAS0,0x42,0x4b},
	{DESALIAS0,0x40,0x10},
	{DESALIAS0,0x41,0x38},
	{DESALIAS0,0x42,0x4b},
	{DESALIAS0,0x40,0x54},
	{DESALIAS0,0x41,0x15},
	{DESALIAS0,0x42,0x0},
	{DESALIAS0,0x40,0x58},
	{DESALIAS0,0x41,0x15},
	{DESALIAS0,0x42,0x0},
	{DESALIAS0,0x40,0x54},
	{DESALIAS0,0x41,0x4a},
	{DESALIAS0,0x42,0x1},
	{DESALIAS0,0x40,0x58},
	{DESALIAS0,0x41,0x4a},
	{DESALIAS0,0x42,0x1},
	{DESALIAS0,0x40,0x54},
	{DESALIAS0,0x41,0xaa},
	{DESALIAS0,0x42,0x2c},
	{DESALIAS0,0x40,0x58},
	{DESALIAS0,0x41,0xaa},
	{DESALIAS0,0x42,0x2c},
	{DESALIAS0,0x40,0x54},
	{DESALIAS0,0x41,0xab},
	{DESALIAS0,0x42,0x2c},
	{DESALIAS0,0x40,0x58},
	{DESALIAS0,0x41,0xab},
	{DESALIAS0,0x42,0x2c},
	{DESALIAS0,0x40,0x54},
	{DESALIAS0,0x41,0xac},
	{DESALIAS0,0x42,0x4c},
	{DESALIAS0,0x40,0x58},
	{DESALIAS0,0x41,0xac},
	{DESALIAS0,0x42,0x4c},
	{DESALIAS0,0x40,0x54},
	{DESALIAS0,0x41,0xad},
	{DESALIAS0,0x42,0x4c},
	{DESALIAS0,0x40,0x58},
	{DESALIAS0,0x41,0xad},
	{DESALIAS0,0x42,0x4c},
	{DESALIAS0,0x40,0x54},
	{DESALIAS0,0x41,0xae},
	{DESALIAS0,0x42,0xac},
	{DESALIAS0,0x40,0x58},
	{DESALIAS0,0x41,0xae},
	{DESALIAS0,0x42,0xac},
	{DESALIAS0,0x40,0x54},
	{DESALIAS0,0x41,0xaf},
	{DESALIAS0,0x42,0xac},
	{DESALIAS0,0x40,0x58},
	{DESALIAS0,0x41,0xaf},
	{DESALIAS0,0x42,0xac},
	{DESALIAS0,0x40,0x10},
	{DESALIAS0,0x41,0x5},
	{DESALIAS0,0x42,0xa},
	{DESALIAS0,0x40,0x10},
	{DESALIAS0,0x41,0x25},
	{DESALIAS0,0x42,0xa},
	{DESALIAS0,0x40,0x54},
	{DESALIAS0,0x41,0x89},
	{DESALIAS0,0x42,0x38},
	{DESALIAS0,0x40,0x58},
	{DESALIAS0,0x41,0x89},
	{DESALIAS0,0x42,0x38},
	{DESALIAS0,0x40,0x10},
	{DESALIAS0,0x41,0x1a},
	{DESALIAS0,0x42,0x8},
	{DESALIAS0,0x40,0x10},
	{DESALIAS0,0x41,0x3a},
	{DESALIAS0,0x42,0x8},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x6f},
	{DESALIAS0,0x42,0x54},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x70},
	{DESALIAS0,0x42,0x5},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x80},
	{DESALIAS0,0x42,0x55},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x81},
	{DESALIAS0,0x42,0x44},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x82},
	{DESALIAS0,0x42,0x3},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x86},
	{DESALIAS0,0x42,0x2c},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x87},
	{DESALIAS0,0x42,0x6},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x18},
	{DESALIAS0,0x42,0x32},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x48},
	{DESALIAS0,0x42,0x32},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x19},
	{DESALIAS0,0x42,0xe},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x49},
	{DESALIAS0,0x42,0xe},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x17},
	{DESALIAS0,0x42,0x72},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x47},
	{DESALIAS0,0x42,0x72},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x26},
	{DESALIAS0,0x42,0x87},
	{DESALIAS0,0x40,0x38},
	{DESALIAS0,0x41,0x56},
	{DESALIAS0,0x42,0x87},
	{DESALIAS0,0x40,0x2c},
	{DESALIAS0,0x41,0x3d},
	{DESALIAS0,0x42,0xd5},
	{DESALIAS0,0x40,0x2c},
	{DESALIAS0,0x41,0x3e},
	{DESALIAS0,0x42,0x15},
	{DESALIAS0,0x40,0x2c},
	{DESALIAS0,0x41,0x7d},
	{DESALIAS0,0x42,0xd5},
	{DESALIAS0,0x40,0x2c},
	{DESALIAS0,0x41,0x7e},
	{DESALIAS0,0x42,0x15},
	{DESALIAS0,0x40,0x2c},
	{DESALIAS0,0x41,0x82},
	{DESALIAS0,0x42,0x1},
	{DESALIAS0,0x40,0x2c},
	{DESALIAS0,0x41,0x29},
	{DESALIAS0,0x42,0x0},
	{DESALIAS0,0x40,0x10},
	{DESALIAS0,0x41,0x41},
	{DESALIAS0,0x42,0x0},
	{DESALIAS0,0x40,0x10},
	{DESALIAS0,0x41,0x42},
	{DESALIAS0,0x42,0x0},
	{DESALIAS0,0x40,0x24},
	{DESALIAS0,0x41,0x20},
	{DESALIAS0,0x42,0x0},
	{DESALIAS0,0x40,0x24},
	{DESALIAS0,0x41,0x21},
	{DESALIAS0,0x42,0x0},
	{DESALIAS0,0x40,0x24},
	{DESALIAS0,0x41,0x23},
	{DESALIAS0,0x42,0x30},
	{DESALIAS0,0x40,0x10},
	{DESALIAS0,0x41,0x14},
	{DESALIAS0,0x42,0x78},
	{DESALIAS0,0x40,0x10},
	{DESALIAS0,0x41,0x35},
	{DESALIAS0,0x42,0x7e},
	{DESALIAS0,0x40,0x6c},
	{DESALIAS0,0x41,0xd},
	{DESALIAS0,0x42,0x0},
	{DESALIAS0,0x40,0x1c},
	{DESALIAS0,0x41,0x8},
	{DESALIAS0,0x42,0x13},
	{DESALIAS0,0x40,0x1c},
	{DESALIAS0,0x41,0x28},
	{DESALIAS0,0x42,0x13},
	{DESALIAS0,0x40,0x14},
	{DESALIAS0,0x41,0x62},
	{DESALIAS0,0x42,0x31},
	{DESALIAS0,0x40,0x14},
	{DESALIAS0,0x41,0x72},
	{DESALIAS0,0x42,0x31},
	{DESALIAS0,0x40,0x14},
	{DESALIAS0,0x41,0x61},
	{DESALIAS0,0x42,0x26},
	{DESALIAS0,0x1,0x1},//Soft Reset DES;
};

const struct ub98x_reg_cmds reg_init_cmd6[] = {
	// Hold Des DTG in reset;
	{DESALIAS0,0x40,0x50},//Select DTG Page;
	{DESALIAS0,0x41,0x32},
	{DESALIAS0,0x42,0x6},//Hold Port 0 DTG in reset;
	{DESALIAS0,0x41,0x62},
	{DESALIAS0,0x42,0x6},//Hold Port 1 DTG in reset;
	// Disable Stream Mapping;
	{DESALIAS0,0xe,0x3},//Select both Output Ports;
	{DESALIAS0,0xd0,0x0},//Disable FPD4 video forward to Output Port;
	{DESALIAS0,0xd7,0x0},//Disable FPD3 video forward to Output Port;
	// Force DP Rate;
	{DESALIAS0,0x40,0x2c},//Select DP Page;
	{DESALIAS0,0x41,0x81},
	{DESALIAS0,0x42,0x60},//Set DP Rate to 2.7Gbps;
	{DESALIAS0,0x41,0x82},
	{DESALIAS0,0x42,0x3},//Enable force DP rate with calibration disabled;
	{DESALIAS0,0x40,0x2c},//Select DP Page;
	{DESALIAS0,0x41,0x91},
	{DESALIAS0,0x42,0xc},//Force 4 lanes;
	{DESALIAS0,0x40,0x30},//Disable DP SSCG;
	{DESALIAS0,0x41,0xf},
	{DESALIAS0,0x42,0x1},
	{DESALIAS0,0x1,0x40},
	// Setup DP ports;
	{DESALIAS0,0xe,0x12},//Select Port 1 registers;
	{DESALIAS0,0x46,0x0},//Disable DP Port 1;
	{DESALIAS0,0xe,0x1},//Select Port 0 registers;
	{DESALIAS0,0x1,0x40},//DP-TX-PLL RESET Applied;
	// Map video to display output;
	{DESALIAS0,0xe,0x3},//Select both Output Ports;
	{DESALIAS0,0xd0,0xc},//Enable FPD_RX video forward to Output Port;
	{DESALIAS0,0xd1,0xf},//Every stream forwarded on DC;
	{DESALIAS0,0xd6,0x8},//Send Stream 0 to Output Port 0 and Send Stream 1 to Output Port 1;
	{DESALIAS0,0xd7,0x0},//FPD3 mapping disabled;
	{DESALIAS0,0xe,0x1},//Select Port 0 ;
};

//UB98X_MODE_DES_PATGEN_CONFIG
const struct ub98x_reg_cmds reg_init_cmd7[] = {
	{DESALIAS0,0x40,0x50},//Set Patgen page
	{DESALIAS0,0x41,0x1},//Set patgen address
	{DESALIAS0,0x42,0xc},//Set bit per pixel
	{DESALIAS0,0x41,0x2},//Set patgen address
	{DESALIAS0,0x42,0x86},//Set patgen address auto increment
	{DESALIAS0,0x41,0x3},//Set patgen address
	{DESALIAS0,0x42,0xc4},//Set patgen THW
	{DESALIAS0,0x42,0xa},//Set patgen THW
	{DESALIAS0,0x42,0x6a},//Set patgen TVW
	{DESALIAS0,0x42,0x6},//Set patgen TVW
	{DESALIAS0,0x42,0x0},//Set patgen AHW
	{DESALIAS0,0x42,0xa},//Set patgen AHW
	{DESALIAS0,0x42,0x40},//Set patgen AVW
	{DESALIAS0,0x42,0x6},//Set patgen AVW
	{DESALIAS0,0x42,0x37},//Set patgen HSW
	{DESALIAS0,0x42,0x0},//Set patgen HSW
	{DESALIAS0,0x42,0x2},//Set patgen VSW
	{DESALIAS0,0x42,0x0},//Set patgen VSW
	{DESALIAS0,0x42,0x37},//Set patgen HBP
	{DESALIAS0,0x42,0x0},//Set patgen HBP
	{DESALIAS0,0x42,0x8},//Set patgen VBP
	{DESALIAS0,0x42,0x0},//Set patgen VBP
	{DESALIAS0,0x42,0x0},//HSYNC Polarity = +, VSYNC Polarity = +
	{DESALIAS0,0x41,0x0},//Set patgen address
	{DESALIAS0,0x42,0x95},//Enable Patgen color bar
};

const struct ub98x_reg_cmds reg_init_cmd8[] = {
	// Program quad pixel clock for DP port 0;
	{DESALIAS0,0xe,0x1},//Select Port0 registers;
	{DESALIAS0,0xb1,0x1},//Enable clock divider;
	{DESALIAS0,0xb2,0xa0},//Program M value lower byte;
	{DESALIAS0,0xb3,0x24},//Program M value middle byte;
	{DESALIAS0,0xb4,0x4},//Program M value upper byte;
	{DESALIAS0,0xb5,0xc0},//Program N value lower byte;
	{DESALIAS0,0xb6,0x7a},//Program N value middle byte;
	{DESALIAS0,0xb7,0x10},//Program N value upper byte;
	{DESALIAS0,0xe,0x1},//Select Port 0 registers;
	// Setup DTG for port 0;
	{DESALIAS0,0x40,0x50},//Select DTG Page;
	{DESALIAS0,0x41,0x20},
	{DESALIAS0,0x42,0x93},//Set up DTG BPP, Sync Polarities, and Measurement Type;
	{DESALIAS0,0x41,0x29},//Set Hstart;
	{DESALIAS0,0x42,0x80},//Hstart upper byte;
	{DESALIAS0,0x41,0x2a},
	{DESALIAS0,0x42,0x6e},//Hstart lower byte;
	{DESALIAS0,0x41,0x2f},//Set HSW;
	{DESALIAS0,0x42,0x40},//HSW upper byte;
	{DESALIAS0,0x41,0x30},
	{DESALIAS0,0x42,0x37},//HSW lower byte;
	// Program DPTX for DP port 0;
	{DESALIAS0,0x48,0x1},//Enable APB interface;
	{DESALIAS0,0x48,0x1},
	{DESALIAS0,0x49,0xa4},//Set bit per color;
	{DESALIAS0,0x4a,0x1},
	{DESALIAS0,0x4b,0x20},
	{DESALIAS0,0x4c,0x0},
	{DESALIAS0,0x4d,0x0},
	{DESALIAS0,0x4e,0x0},
	{DESALIAS0,0x48,0x1},
	{DESALIAS0,0x49,0xb8},//Set pixel width;
	{DESALIAS0,0x4a,0x1},
	{DESALIAS0,0x4b,0x4},
	{DESALIAS0,0x4c,0x0},
	{DESALIAS0,0x4d,0x0},
	{DESALIAS0,0x4e,0x0},
	{DESALIAS0,0x48,0x1},
	{DESALIAS0,0x49,0xac},//Set DP Mvid;
	{DESALIAS0,0x4a,0x1},
	{DESALIAS0,0x4b,0xb8},
	{DESALIAS0,0x4c,0x80},
	{DESALIAS0,0x4d,0x0},
	{DESALIAS0,0x4e,0x0},
	{DESALIAS0,0x48,0x1},
	{DESALIAS0,0x49,0xb4},//Set DP Nvid;
	{DESALIAS0,0x4a,0x1},
	{DESALIAS0,0x4b,0x0},
	{DESALIAS0,0x4c,0x80},
	{DESALIAS0,0x4d,0x0},
	{DESALIAS0,0x4e,0x0},
	{DESALIAS0,0x48,0x1},
	{DESALIAS0,0x49,0xc8},//Set TU Mode;
	{DESALIAS0,0x4a,0x1},
	{DESALIAS0,0x4b,0x0},
	{DESALIAS0,0x4c,0x0},
	{DESALIAS0,0x4d,0x0},
	{DESALIAS0,0x4e,0x0},
	{DESALIAS0,0x48,0x1},
	{DESALIAS0,0x49,0xb0},//Set TU Size;
	{DESALIAS0,0x4a,0x1},
	{DESALIAS0,0x4b,0x40},
	{DESALIAS0,0x4c,0x0},
	{DESALIAS0,0x4d,0x30},
	{DESALIAS0,0x4e,0x6},
	{DESALIAS0,0x48,0x1},
	{DESALIAS0,0x49,0xc8},//Set FIFO Size;
	{DESALIAS0,0x4a,0x0},
	{DESALIAS0,0x4b,0x5},
	{DESALIAS0,0x4c,0x40},
	{DESALIAS0,0x4d,0x0},
	{DESALIAS0,0x4e,0x0},
	{DESALIAS0,0x48,0x1},
	{DESALIAS0,0x49,0xbc},//Set data count;
	{DESALIAS0,0x4a,0x1},
	{DESALIAS0,0x4b,0x80},
	{DESALIAS0,0x4c,0x7},
	{DESALIAS0,0x4d,0x0},
	{DESALIAS0,0x4e,0x0},
	{DESALIAS0,0x48,0x1},
	{DESALIAS0,0x49,0xc0},//Disable STREAM INTERLACED;
	{DESALIAS0,0x4a,0x1},
	{DESALIAS0,0x4b,0x0},
	{DESALIAS0,0x4c,0x0},
	{DESALIAS0,0x4d,0x0},
	{DESALIAS0,0x4e,0x0},
	{DESALIAS0,0x48,0x1},
	{DESALIAS0,0x49,0xc4},//Set SYNC polarity;
	{DESALIAS0,0x4a,0x1},
	{DESALIAS0,0x4b,0xc},
	{DESALIAS0,0x4c,0x0},
	{DESALIAS0,0x4d,0x0},
	{DESALIAS0,0x4e,0x0},
	// Release Des DTG reset;
	{DESALIAS0,0x40,0x50},//Select DTG Page;
	{DESALIAS0,0x41,0x32},
	{DESALIAS0,0x42,0x4},//Release Port 0 DTG;
	{DESALIAS0,0x41,0x62},
	{DESALIAS0,0x42,0x4},//Release Port 1 DTG;
	{DESALIAS0,0x48,0x1},
	{DESALIAS0,0x49,0x80},//Set Htotal;
	{DESALIAS0,0x4a,0x1},
	{DESALIAS0,0x4b,0xc4},
	{DESALIAS0,0x4c,0xa},
	{DESALIAS0,0x4d,0x0},
	{DESALIAS0,0x4e,0x0},
	// Enable DP 0 output;
	{DESALIAS0,0x48,0x1},
	{DESALIAS0,0x49,0x84},//Enable DP output;
	{DESALIAS0,0x4a,0x0},
	{DESALIAS0,0x4b,0x1},
	{DESALIAS0,0x4c,0x0},
	{DESALIAS0,0x4d,0x0},
	{DESALIAS0,0x4e,0x0},
};

//初始化GPIO口状态
unsigned char ub98x_dev_init_reg [][6] = {
//   addr,  reg, value, rbdelay, retry, delay(defore)
	{0x0c, 0x2d,  0x01,    0x01,  0x05, 0x00 },
	{0x0c, 0x15,  0x10,    0x01,  0x05, 0x00 },
	{0x0c, 0x0d,  0x03,    0x01,  0x05, 0x00 },

	{0x2c, 0x12,  0x00,    0x01,  0x05, 0x00 }, //GPIO IN CLEAN
	{0x2c, 0x13,  0x00,    0x01,  0x05, 0x00 }, //GPIO IN CLEAN

	//{0x2c, 0x22,  0xc0,    0x01,  0x05, 0x00 }, //tcon_resetGPIO13 | GPIO13_PIN_CTL Register (Address = 0x22)
	//{0x2c, 0x1e,  0xc0,    0x01,  0x05, 0x00 }, //bist_en   GPIO9 | GPIO8_PIN_CTL Register (Address = 0x1E)
	//{0x2c, 0x1d,  0xc0,    0x01,  0x05, 0x00 }, //bl_pwm    GPIO8 | GPIO8_PIN_CTL Register (Address = 0x1D)
	//{0x2c, 0x17,  0xc0,    0x01,  0x05, 0x00 }, //vsp_en    GPIO2 | GPIO2_PIN_CTL Register (Address = 0x17)
	//{0x2c, 0x18,  0xc0,    0x01,  0x05, 0x00 }, //vsn_en    GPIO3 | GPIO3_PIN_CTL Register (Address = 0x18)
	//{0x2c, 0x15,  0xc0,    0x01,  0x05, 0x00 }, //vgh_en    GPIO0 | GPIO0_PIN_CTL Register (Address = 0x15)
	//{0x2c, 0x16,  0xc0,    0x01,  0x05, 0x00 }, //vgl_en	GPIO1 | GPIO1_PIN_CTL Register (Address = 0x16)
	//{0x2c, 0x19,  0xc0,    0x01,  0x05, 0x00 }, //tft_reset GPIO4 | GPIO4_PIN_CTL Register (Address = 0x19)
	//{0x2c, 0x1a,  0xc0,    0x01,  0x05, 0x00 }, //tp_reset  GPIO5 | GPIO5_PIN_CTL Register (Address = 0x1A)
	//{0x2c, 0x1b,  0xc0,    0x01,  0x05, 0x00 }, //tft_pon	GPIO6 | GPIO6_PIN_CTL Register (Address = 0x1B)
	//{0x2c, 0x1c,  0xc0,    0x01,  0x05, 0x00 }, //bl_en     GPIO7 | GPIO7_PIN_CTL Register (Address = 0x1C)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x17,  0xc1,    0x01,  0x05, 0x0a }, //vsp_en    GPIO2 | GPIO2_PIN_CTL Register (Address = 0x17)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x22,  0xc1,    0x01,  0x05, 0x00 }, //tcon_resetGPIO13 | GPIO13_PIN_CTL Register (Address = 0x22)
	{0x2c, 0x18,  0xc1,    0x01,  0x05, 0x00 }, //vsn_en	GPIO3 | GPIO3_PIN_CTL Register (Address = 0x18)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x15,  0xc1,    0x01,  0x05, 0x00 }, //vgh_en    GPIO0 | GPIO0_PIN_CTL Register (Address = 0x15)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x16,  0xc1,    0x01,  0x05, 0x00 }, //vgl_en	GPIO1 | GPIO1_PIN_CTL Register (Address = 0x16)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x19,  0xc1,    0x01,  0x05, 0x00 }, //tft_reset GPIO4 | GPIO4_PIN_CTL Register (Address = 0x1

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x1a,  0xc1,    0x01,  0x05, 0x00 }, //tp_reset	GPIO5 | GPIO5_PIN_CTL Register (Address = 0x1A)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x96 }, //delay 150ms
	{0x2c, 0x1b,  0xc1,    0x01,  0x05, 0x00 }, //tft_pon	GPIO6 | GPIO6_PIN_CTL Register (Address = 0x1B)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x1c,  0xc1,    0x01,  0x05, 0x00 }, //bl_en 	GPIO7 | GPIO7_PIN_CTL Register (Address = 0x1C)

#if 0
	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x1d,  0xc1,    0x01,  0x05, 0x00 }, //bl_en 	GPIO7 | GPIO7_PIN_CTL Register (Address = 0x1C)
#endif
	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x1e,  0xc0,    0x01,  0x05, 0x00 }, //bl_en 	GPIO7 | GPIO7_PIN_CTL Register (Address = 0x1C)

#if 1
	{0x2c, 0x44,  0x81,    0x01,  0x05, 0x14 }, //RX_INT_CTL Register

	{0x0c, 0xc6,  0x21,    0x01,  0x05, 0x00 },
	{0x0c, 0x1b,  0x88,    0x01,  0x05, 0x00 },
	{0x0c, 0x51,  0x83,    0x01,  0x05, 0x00 },

	{0x0c, 0x0d,  0x03,    0x01,  0x05, 0x00 }, //Forward Channel GPIO Enable: 0x03 is 4Num
	{0x0c, 0x15,  0x01,    0x01,  0x05, 0x00 }, //Set GPIO1 as FC GPIO0 input
	{0x0c, 0x3e,  0xff,    0x01,  0x05, 0x00 }, //Enable GPIO1(all) input

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x13,  0xfd,    0x01,  0x05, 0x00 }, //Disable GPIO1 input
	{0x2c, 0x1d,  0x94,    0x01,  0x05, 0x00 }, //Enable output of FC GPIO0 on GPIO8 | i2cset -y 12 0x2c 0x1d 0x94 b //tmp
#endif


	//{0x0c, 0x18,  0x03,    0x01,  0x05, 0x00 },
	//{0x0c, 0x16,  0x10,    0x01,  0x05, 0x00 }, //Set GPIO1
};

unsigned char ub98x_dev_deinit_reg [][6] = {
//	 addr,	reg, value, rbdelay, retry, delay(defore)
	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x1b,  0xc0,    0x01,  0x05, 0x00 }, //tft_pon   GPIO6 | GPIO6_PIN_CTL Register (Address = 0x1B)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0xb4 }, //delay 180ms
	{0x2c, 0x1b,  0xc1,    0x01,  0x05, 0x00 }, //tft_pon   GPIO6 | GPIO6_PIN_CTL Register (Address = 0x1B)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x1c,  0xc0,    0x01,  0x05, 0x00 }, //bl_en     GPIO7 | GPIO7_PIN_CTL Register (Address = 0x1C)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x1b,  0xc0,    0x01,  0x05, 0x00 }, //tft_pon   GPIO6 | GPIO6_PIN_CTL Register (Address = 0x1B)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0xaa }, //delay 170ms
	{0x2c, 0x1a,  0xc0,    0x01,  0x05, 0x00 }, //tp_reset  GPIO5 | GPIO5_PIN_CTL Register (Address = 0x1A)

	{0x2c, 0x19,  0xc0,    0x01,  0x05, 0x00 }, //tft_reset GPIO4 | GPIO4_PIN_CTL Register (Address = 0x19)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x16,  0xc0,    0x01,  0x05, 0x00 }, //vgl_en    GPIO1 | GPIO1_PIN_CTL Register (Address = 0x16)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x15,  0xc0,    0x01,  0x05, 0x00 }, //vgh_en    GPIO0 | GPIO0_PIN_CTL Register (Address = 0x15)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x18,  0xc0,    0x01,  0x05, 0x00 }, //vsn_en    GPIO3 | GPIO3_PIN_CTL Register (Address = 0x18)

	{0x00, 0x00,  0x00,    0x00,  0x00, 0x14 }, //delay 20ms
	{0x2c, 0x17,  0xc0,    0x01,  0x05, 0x00 }, //vsp_en    GPIO2 | GPIO2_PIN_CTL Register (Address = 0x17)
	{0x2c, 0x1c,  0xc0,    0x01,  0x05, 0x00 },
};

//初始化需要透传的I2C设备
unsigned char ub98x_subdev_init_reg [][6] = {
//   addr,  reg, value, rbdelay, retry, delay(defore)
	{0x0c, 0x71,  0x54,    0x01,  0x05, 0x00 }, //backlight i2c 8bit 0x54
	{0x0c, 0x79,  0x54,    0x01,  0x05, 0x00 }, //backlight i2c 8bit 0x54
	{0x0c, 0x89,  0x00,    0x01,  0x05, 0x00 }, //backlight i2c 8bit 0x54

	{0x0c, 0x72,  0x90,    0x01,  0x05, 0x00 }, //touchscreen i2c 8bit 0x90
	{0x0c, 0x7a,  0x90,    0x01,  0x05, 0x00 }, //touchscreen i2c 8bit 0x90
	{0x0c, 0x8a,  0x00,    0x01,  0x05, 0x00 }, //touchscreen i2c 8bit 0x90

	{0x0c, 0x73,  0xa2,    0x01,  0x05, 0x00 }, //touchscreen i2c 8bit 0xa2
	{0x0c, 0x7b,  0xa2,    0x01,  0x05, 0x00 }, //touchscreen i2c 8bit 0xa2
	{0x0c, 0x8b,  0x00,    0x01,  0x05, 0x00 }, //touchscreen i2c 8bit 0xa2
};

static int ub98x_suspend(struct device *dev)
{
	int ret = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	csd_printk_debug("disable irq=%d\n", client->irq);

	disable_irq(client->irq);

	ret = pinctrl_pm_select_sleep_state(dev);
	if(ret < 0)
		csd_printk_error("failed to suspend,ret: (%d)\n", ret);

	return ret;
}

static int ub98x_resume(struct device *dev)
{
	int ret = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	ret = pinctrl_pm_select_default_state(dev);
	if(ret < 0)
		csd_printk_error("failed to resume,ret: (%d)\n", ret);

	csd_printk_debug("enable irq=%d\n", client->irq);

	enable_irq(client->irq);

	return ret;
}

struct dev_pm_ops ub98x_pm_ops = {
	.suspend = ub98x_suspend,
	.resume  = ub98x_resume,
};

static ssize_t ub98x_power_control_show(struct device *d, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 1);
}

static int ub98x_power_up(struct geely_data *ts) {
	int ret = 0;

	if (gpio_is_valid(ts->gpio_vdd1v8_en)) {
		ret = gpio_direction_output(ts->gpio_vdd1v8_en, 1);
		if (ret < 0) {
			return ret;
		}
	}

	if (gpio_is_valid(ts->gpio_vdd1v1_en)) {
		ret = gpio_direction_output(ts->gpio_vdd1v1_en, 1);
		if (ret < 0) {
			return ret;
		}
	}
	mdelay(1000);

	//pdb reset [1/2] , set pdb low.
	if (gpio_is_valid(ts->gpio_pdb)) {
		ret = gpio_direction_output(ts->gpio_pdb, 0);
		if (ret < 0) {
			return ret;
		}
	}
	mdelay(500);

	//pdb reset [2/2] , delay 500ms set pdb high.
	if (gpio_is_valid(ts->gpio_pdb)) {
		ret = gpio_direction_output(ts->gpio_pdb, 1);
		if (ret < 0) {
			return ret;
		}
	}
	mdelay(500);

	if (gpio_is_valid(ts->gpio_bl_en1)) {
		ret = gpio_direction_output(ts->gpio_bl_en1, 1);
		if (ret < 0) {
			return ret;
		}
	}

	if (gpio_is_valid(ts->gpio_bl_en2)) {
		ret = gpio_direction_output(ts->gpio_bl_en2, 1);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int ub98x_power_down(struct geely_data *ts) {
	int ret = 0;
	if (gpio_is_valid(ts->gpio_bl_en1)) {
		ret = gpio_direction_output(ts->gpio_bl_en1, 0);
		if (ret < 0) {
			return ret;
		}
	}

	if (gpio_is_valid(ts->gpio_bl_en2)) {
		ret = gpio_direction_output(ts->gpio_bl_en2, 0);
		if (ret < 0) {
			return ret;
		}
	}

	if (gpio_is_valid(ts->gpio_pdb)) {
		ret = gpio_direction_output(ts->gpio_pdb, 0);
		if (ret < 0) {
			return ret;
		}
	}

	if (gpio_is_valid(ts->gpio_vdd1v8_en)) {
		ret = gpio_direction_output(ts->gpio_vdd1v8_en, 0);
		if (ret < 0) {
			return ret;
		}
	}

	if (gpio_is_valid(ts->gpio_vdd1v1_en)) {
		ret = gpio_direction_output(ts->gpio_vdd1v1_en, 0);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

bool ub983_i2c_read_register(struct i2c_client *client, unsigned char addr, unsigned char reg, unsigned char *data)
{
	int retry = 0;
	int ret = 0;
	unsigned char buf;
	struct geely_data *ts = i2c_get_clientdata(client);
    struct i2c_msg msg[2] =
	{
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &reg,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= &buf,
		}
	};

	mutex_lock(&ts->rw_lock);

	client->addr = addr;

	for(retry=3; retry > 0; retry--)
	{
		ret = i2c_transfer(client->adapter, msg, 2);
		if (ret != 2) {
			csd_printk_debug("%s failed, ret:%d, {0x%02x,0x%02x} \n",__FUNCTION__, ret, client->addr, reg);
		} else {
			*data = buf;
			mutex_unlock(&ts->rw_lock);
			return true;
		}
	}

	mutex_unlock(&ts->rw_lock);
	return false;
}

bool ub983_i2c_write_register(struct i2c_client *client, unsigned char addr, unsigned char reg, unsigned char data)
{
	struct geely_data *ts = i2c_get_clientdata(client);

	u8 tmp[2];
	struct i2c_msg msg;
	int ret;
	// printk("ub983_i2c_write_register, {0x%02x,0x%02x,0x%02x}\n",client->addr , addr, data);
	mdelay(10);
	mutex_lock(&ts->rw_lock);

	tmp[0] = reg;
	tmp[1] = data;

	client->addr = addr;

	msg.addr = client->addr;
	msg.flags = 0;		/* write */
	msg.buf = tmp;
	msg.len = 2;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1)
	{
		csd_printk_debug("%s failed, ret:%d, {0x%02x,0x%02x,0x%02x} \n",__FUNCTION__,ret,client->addr,reg,data);
		mutex_unlock(&ts->rw_lock);
		return false;
	}

	mutex_unlock(&ts->rw_lock);
	return true;
}

bool ub983_i2c_write_with_readback(struct i2c_client *client, unsigned char addr, unsigned char reg, unsigned char data, unsigned long rbdelay, char retry){
	bool ret = false;
	unsigned char readback = 0;
	char remain = 0;

	//printk("ub983_i2c_write_with_readback, {0x%02x,0x%02x,0x%02x,0x%04lx,0x%02x}\n", addr, reg, data, rbdelay, retry);
	for (remain=retry+1;remain>0;remain--) {
		ret = ub983_i2c_write_register(client,addr,reg,data);
		if(ret == true) {
			if (rbdelay > 0) {
				mdelay(rbdelay);
			}

			ret = ub983_i2c_read_register(client,addr,reg,&readback);
			if (ret == true) {
				if (readback == data) {
					return true;
				} else {
					csd_printk_debug("ub983_i2c_write_with_readback, read back data has error,{0x%02x,0x%02x,0x%02x} rb:%d!=data:0x%x, remain:%d\n", addr, reg, data, readback, data, remain);
				}
			} else {
				csd_printk_debug("ub983_i2c_write_with_readback, read back transfer has error,{0x%02x,0x%02x,0x%02x} ret:%d, reg:0x%x, remain:%d\n", addr, reg, data, ret, reg, remain);
			}
		} else {
			csd_printk_debug("ub983_i2c_write_with_readback, transfer has error, ret:%d,{0x%02x,0x%02x,0x%02x} reg:0x%x, remain:%d\n", addr, reg, data, ret, reg, remain);
		}
	}
	return false;
}

int ub98x_reg_array_write(struct i2c_client *client, unsigned char (*array)[6], unsigned int length)
{
	int i = 0;

	for (i=0;i<length;i++) {
		unsigned char delay = array[i][5];
		if (delay > 0) {
			mdelay((unsigned long)delay);
		}

		if (array[i][0] == 0) {
			//just delay
			continue;
		}

		delay = array[i][3];
		if (ub983_i2c_write_with_readback(client, array[i][0], array[i][1], array[i][2], (unsigned long)delay, array[i][4]) == false) {
			return -1;
		}
	}

	return 0;
}

int ub983_indirect_read(struct i2c_client *client, unsigned char addr,unsigned char page, unsigned char reg, unsigned char *data)
{
	unsigned char page_reg = (page << 2) | 1;
	bool ret = false;
	ret = ub983_i2c_write_register(client,addr, 0x40, page_reg);
	if(ret == false) {
		csd_printk_debug("%s, write 0x40 has error,{0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, page, reg);
		return ret;
	}

	ret = ub983_i2c_write_register(client,addr, 0x41, reg);
	if(ret == false) {
		csd_printk_debug("%s, write 0x41 has error,{0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, page, reg);
		return ret;
	}

	ret = ub983_i2c_read_register(client, addr, 0x42, data);
	if(ret == false) {
		csd_printk_debug("%s, read 0x42 has error,{0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, page, reg);
		return ret;
	}

	// printk("%s, read {0x%02x,0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, page, reg, *data);
	//client->addr = back_addr;

	return ret;
}

int ub983_apb_read(struct i2c_client *client, unsigned char addr,unsigned char reg_h, unsigned char reg_l, unsigned int *data)
{
	bool ret = false;
	unsigned char val = 0;
	*data = 0;

	ret = ub983_i2c_write_register(client,addr, 0x49, reg_l);
	if(ret == false) {
		csd_printk_debug("%s, write 0x49 has error,{0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l);
		return ret;
	}

	ret = ub983_i2c_write_register(client,addr, 0x4a, reg_h);
	if(ret == false) {
		csd_printk_debug("%s, write 0x4a has error,{0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l);
		return ret;
	}

	ret = ub983_i2c_write_register(client,addr, 0x48, 0x03);
	if(ret == false) {
		csd_printk_debug("%s, write 0x48 has error,{0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l);
		return ret;
	}

	ret = ub983_i2c_read_register(client,addr , 0x4b, &val);
	if(ret == false) {
		csd_printk_debug("%s, read 0x4b has error,{0x%02x,0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l, val);
		return ret;
	}
	*data = val;


	ret = ub983_i2c_read_register(client,addr, 0x4c, &val);
	if(ret == false) {
		csd_printk_debug("%s, read 0x4c has error,{0x%02x,0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l, val);
		return ret;
	}
	*data = *data | (val << 8);

	ret = ub983_i2c_read_register(client,addr, 0x4d, &val);
	if(ret == false) {
		csd_printk_debug("%s, read 0x4d has error,{0x%02x,0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l, val);
		return ret;
	}
	*data = *data | (val << 16);

	ret = ub983_i2c_read_register(client,addr, 0x4e, &val);
	if(ret == false) {
		csd_printk_debug("%s, read 0x4e has error,{0x%02x,0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l, val);
		return ret;
	}
	*data = *data | (val << 24);
	//printk("%s, read, {0x%02x,0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l, *data);
	//client->addr = back_addr;
	return 0;
}

int ub983_apb_write(struct i2c_client *client, unsigned char addr,unsigned char reg_h, unsigned char reg_l, unsigned int data)
{
	bool ret = false;
	ret = ub983_i2c_write_register(client,addr, 0x49, reg_l);
	if(ret == false) {
		csd_printk_debug("%s, write 0x49 has error,{0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l);
		return ret;
	}

	ret = ub983_i2c_write_register(client,addr, 0x4a, reg_h);
	if(ret == false) {
		csd_printk_debug("%s, write 0x4a has error,{0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l);
		return ret;
	}

	ret = ub983_i2c_write_register(client,addr, 0x4b,  (data & 0x000000ff) >> 0);
	if(ret == false) {
		csd_printk_debug("%s, write 0x4b has error,{0x%02x,0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l, (data & 0x000000ff) >> 0);
		return ret;
	}

	ret = ub983_i2c_write_register(client,addr, 0x4c, (data & 0x0000ff00) >> 8);
	if(ret == false) {
		csd_printk_debug("%s, write 0x4c has error,{0x%02x,0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l, (data & 0x0000ff00) >> 8);
		return ret;
	}

	ret = ub983_i2c_write_register(client,addr, 0x4d, (data & 0x00ff0000) >> 16);
	if(ret == false) {
		csd_printk_debug("%s, write 0x4d has error,{0x%02x,0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l, (data & 0x00ff0000) >> 16);
		return ret;
	}

	ret = ub983_i2c_write_register(client,addr, 0x4e,  (data & 0xff000000) >> 24);
	if(ret == false) {
		csd_printk_debug("%s, write 0x4e has error,{0x%02x,0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l, (data & 0xff000000) >> 24);
		return ret;
	}

	ret = ub983_i2c_write_register(client,addr, 0x48, 0x01);
	if(ret == false) {
		csd_printk_debug("%s, write 0x48 has error,{0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l);
		return ret;
	}

	csd_printk_debug("%s, write, {0x%02x,0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, reg_h, reg_l, data);

	//client->addr = back_addr;
	return 0;
}

static int ub98x_dump_state(struct i2c_client *client)
{
	unsigned char value08 = 0;
	unsigned int value32 = 0;

	ub983_i2c_read_register(client,0x0c, FPD3_ICR, &value08);
	csd_printk_debug("ub98x_dump_state FPD3_ICR : 0x%02x \n",value08);

	ub983_i2c_read_register(client,0x0c, FPD3_ISR, &value08);
	csd_printk_debug("ub98x_dump_state FPD3_ISR : 0x%02x \n",value08);

	ub983_indirect_read(client, 0x0c, PAGE_09, INTR_CTL_DP_RX_PORT, &value08);
	csd_printk_debug("ub98x_dump_state INTR_CTL_DP_RX_PORT : 0x%02x \n",value08);

	ub983_indirect_read(client, 0x0c, PAGE_09, INTR_STS_DP_RX_PORT, &value08);
	csd_printk_debug("ub98x_dump_state INTR_STS_DP_RX_PORT : 0x%02x \n",value08);

	ub983_indirect_read(client, 0x0c, PAGE_09, INTR_STS_FPD4_PORT0, &value08);
	csd_printk_debug("ub98x_dump_state INTR_STS_FPD4_PORT0 : 0x%02x \n",value08);

	ub983_indirect_read(client, 0x0c, PAGE_09, INTR_STS_FPD4_PORT1, &value08);
	csd_printk_debug("ub98x_dump_state INTR_STS_FPD4_PORT1 : 0x%02x \n",value08);

	ub983_indirect_read(client, 0x0c, PAGE_09, INTR_CTL_FPD4_PORT0, &value08);
	csd_printk_debug("ub98x_dump_state INTR_CTL_FPD4_PORT0 : 0x%02x \n",value08);

	ub983_indirect_read(client, 0x0c, PAGE_09, INTR_CTL_FPD4_PORT1, &value08);
	csd_printk_debug("ub98x_dump_state INTR_CTL_FPD4_PORT1 : 0x%02x \n",value08);


	ub983_indirect_read(client, 0x0c, PAGE_12, INTR_CTL_VP_VP0, &value08);
	csd_printk_debug("ub98x_dump_state INTR_CTL_VP_VP0 : 0x%02x \n",value08);

	ub983_indirect_read(client, 0x0c, PAGE_12, INTR_CTL_VP_VP1, &value08);
	csd_printk_debug("ub98x_dump_state INTR_CTL_VP_VP1 : 0x%02x \n",value08);

	ub983_indirect_read(client, 0x0c, PAGE_12, INTR_CTL_VP_VP2, &value08);
	csd_printk_debug("ub98x_dump_state INTR_CTL_VP_VP2 : 0x%02x \n",value08);

	ub983_indirect_read(client, 0x0c, PAGE_12, INTR_CTL_VP_VP3, &value08);
	csd_printk_debug("ub98x_dump_state INTR_CTL_VP_VP3 : 0x%02x \n",value08);


	ub983_indirect_read(client, 0x0c, PAGE_12, INTR_STS_VP_VP0, &value08);
	csd_printk_debug("ub98x_dump_state INTR_STS_VP_VP0 : 0x%02x \n",value08);

	ub983_indirect_read(client, 0x0c, PAGE_12, INTR_STS_VP_VP1, &value08);
	csd_printk_debug("ub98x_dump_state INTR_STS_VP_VP1 : 0x%02x \n",value08);

	ub983_indirect_read(client, 0x0c, PAGE_12, INTR_STS_VP_VP2, &value08);
	csd_printk_debug("ub98x_dump_state INTR_STS_VP_VP2 : 0x%02x \n",value08);

	ub983_indirect_read(client, 0x0c, PAGE_12, INTR_STS_VP_VP3, &value08);
	csd_printk_debug("ub98x_dump_state INTR_STS_VP_VP3 : 0x%02x \n",value08);

	ub983_apb_read(client, 0x0c, ((DISPLAYPORT_INTERRUPT_MASK & 0xff00) >> 16), (DISPLAYPORT_INTERRUPT_MASK & 0x00ff), &value32);
	csd_printk_debug("ub98x_dump_state DISPLAYPORT_INTERRUPT_MASK : 0x%x \n",value32);

	ub983_apb_read(client, 0x0c, ((DISPLAYPORT_INTERRUPT_CAUSE & 0xff00) >> 16), (DISPLAYPORT_INTERRUPT_CAUSE & 0x00ff), &value32);
	csd_printk_debug("ub98x_dump_state DISPLAYPORT_INTERRUPT_CAUSE : 0x%x \n",value32);

	return 0;
}

int ub983_indirect_write(struct i2c_client *client, unsigned char addr,unsigned char page, unsigned char reg, unsigned char data)
{
	unsigned char page_reg = (page << 2) | 0;
	bool ret = false;
	ret = ub983_i2c_write_register(client,addr, 0x40, page_reg);
	if(ret == false) {
		csd_printk_debug("%s, write 0x40 has error,{0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, page, reg);
		return ret;
	}

	ret = ub983_i2c_write_register(client,addr, 0x41, reg);
	if(ret == false) {
		csd_printk_debug("%s, write 0x41 has error,{0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, page, reg);
		return ret;
	}

	ret = ub983_i2c_write_register(client,addr, 0x42, data);
	if(ret == false) {
		csd_printk_debug("%s, write 0x42 has error,{0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, page, reg);
		return ret;
	}

	csd_printk_debug("%s, write {0x%02x,0x%02x,0x%02x,0x%02x} \n",__FUNCTION__, addr, page, reg, data);
	//client->addr = back_addr;

	return ret;
}

unsigned char ub983_read(struct i2c_client *client, unsigned char i2caddr, unsigned char addr) {
	int ret = 0;
	unsigned char buf = 0;
	if (has_error == 1) {
		return 0;
	}

	ret = ub983_i2c_read_register(client,i2caddr,addr,&buf);
	if (ret == 0) {
		has_error = 1;
		csd_printk_debug("ub983_read error! ret(%d) addr(0x%2x) reg(0x%2x)\n",ret,i2caddr,addr);
		return 0;
	}
	//mdelay(10);
	return buf;
}

bool ub983_write(struct i2c_client *client, unsigned char i2caddr, unsigned char addr, unsigned char data) {
	int ret = 0;
	if (has_error == 1) {
		return 0;
	}

	ret = ub983_i2c_write_register(client,i2caddr,addr,data);
	if (ret == 0) {
		has_error = 1;
		csd_printk_debug("ub983_write error! ret(%d) addr(0x%2x) reg(0x%2x) data(0x%2x)\n",ret,i2caddr,addr,data);
		return 0;
	}
	//mdelay(10);
	return 1;
}

static void ub983_regcmd_write(struct i2c_client *client,const struct ub98x_reg_cmds *cmds,int len) {
	int ret, j;
	for(j = 0;j < len;j++){
		ret = ub983_write(client,I2C_7BIT_ADDR(cmds[j].i2c_addr),cmds[j].addr,cmds[j].val);
		if(!ret)
			csd_printk_debug("ub983 reg init has error,line=%d\r\n",__LINE__);
	}
}

static void ub983_regcmd_handler(struct i2c_client *client,char type)
{
	switch(type){
		case UB98X_MODE_HPD_FORCE_TRIGE:
			ub983_regcmd_write(client, reg_init_cmd1, ARRAY_SIZE(reg_init_cmd1));
			break;
		case UB98X_MODE_VP_CONFIG:
			ub983_regcmd_write(client, reg_init_cmd2, ARRAY_SIZE(reg_init_cmd2));
			break;
		case UB98X_MODE_SER_PATGEN_CONFIG:
			ub983_regcmd_write(client, reg_init_cmd3, ARRAY_SIZE(reg_init_cmd3));
			break;
		case UB98X_MODE_SER_TX_LINK_CONFIG:
			ub983_regcmd_write(client, reg_init_cmd4, ARRAY_SIZE(reg_init_cmd4));
			break;
		case UB98X_MODE_DES_CONFIG:
			ub983_regcmd_write(client, reg_init_cmd5, ARRAY_SIZE(reg_init_cmd5));
			break;
		case UB98X_MODE_DES_DP_CONFIG:
			ub983_regcmd_write(client, reg_init_cmd6, ARRAY_SIZE(reg_init_cmd6));
			break;
		case UB98X_MODE_DES_PATGEN_CONFIG:
			ub983_regcmd_write(client, reg_init_cmd7, ARRAY_SIZE(reg_init_cmd7));
			break;
		case UB98X_MODE_DES_DP_CONFIG_PART2:
			ub983_regcmd_write(client, reg_init_cmd8, ARRAY_SIZE(reg_init_cmd8));
			break;
		default:
			break;
	}
}

int ub983_reg_init(struct i2c_client *client,unsigned char mode)
{
	unsigned char HRes = 0;
	unsigned char VRes = 0;
	unsigned char apbData0 = 0;
	unsigned char apbData1 = 0;
	unsigned char apbData2 = 0;
	unsigned char apbData3 = 0;
	unsigned char apbData = 0;
	unsigned char Reg_value = 0;
	unsigned char TEMP_FINAL = 0;
	unsigned char TEMP_FINAL_C = 0;
	unsigned char Efuse_TS_CODE = 0;
	unsigned char Ramp_UP_Range_CODES_Needed = 0;
	unsigned char Ramp_DN_Range_CODES_Needed = 0;
	unsigned char Ramp_UP_CAP_DELTA = 0;
	unsigned char Ramp_DN_CAP_DELTA = 0;
	unsigned char TS_CODE_UP = 0;
	unsigned char rb = 0;
	unsigned char TS_CODE_DN = 0;
	unsigned char I2C_PASS_THROUGH = 0;
	unsigned char I2C_PASS_THROUGH_REG = 0;
	unsigned char I2C_PASS_THROUGH_MASK = 0;
	csd_printk_debug("ub983_reg_init start mode=%x\n",mode);
	ub983_regcmd_write(client, reg_init_cmd0, ARRAY_SIZE(reg_init_cmd0));
	mdelay(0.04*10000);
	;//;// *********************************************;
	;//;// Enable I2C Passthrough;
	;//;// *********************************************;
	I2C_PASS_THROUGH = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x7);
	I2C_PASS_THROUGH_MASK = 0x08;
	I2C_PASS_THROUGH_REG = I2C_PASS_THROUGH | I2C_PASS_THROUGH_MASK;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x07,I2C_PASS_THROUGH_REG) ;//Enable I2C Passthrough;
	ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x1,0x1) ;//Soft reset Des;
	mdelay(0.04*10000);
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x2d,0x1) ;//Select write to port0 reg;
	;//;// *********************************************;
	;//;// Set DP Config;
	;//;// *********************************************;
	if((mode&UB98X_MODE_HPD_FORCE_TRIGE) != 0) {
		csd_printk_debug("ub983_reg_init UB98X_MODE_HPD_FORCE_TRIGE\n");
		ub983_regcmd_handler(client, UB98X_MODE_HPD_FORCE_TRIGE);
		mdelay(1*1000) ;// Allow time after HPD is pulled high for the source to train and provide video (may need to adjust based on source properties*10000);
	}

	;//;// *********************************************;
	;//;// Issue DP video reset after video is available from the DP source;
	;//;// *********************************************;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x49,0x0) ;//Read back Horizontal Resolution from DP APB;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x4a,0x5);
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x48,0x3);
	apbData0 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4b);
	apbData1 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4c);
	apbData2 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4d);
	apbData3 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4e);
	apbData = (apbData3<<24) | (apbData2<<16) | (apbData1<<8) | (apbData0<<0);
	HRes = apbData;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x49,0x14) ;//Read back Vertical Resolution from DP APB;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x4a,0x5);
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x48,0x3);
	apbData0 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4b);
	apbData1 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4c);
	apbData2 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4d);
	apbData3 = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x4e);
	apbData = (apbData3<<24) | (apbData2<<16) | (apbData1<<8) | (apbData0<<0);
	VRes = apbData;
	//print "Detected DP Input Resolution: ", HRes, "x", VRes;
	csd_printk_debug("ub983_reg_init X=%d Y=%d\n",HRes,VRes);
	if( HRes == 0 || VRes == 0) {
			csd_printk_debug("ub983_reg_init warning, no DP Video Input to 983 Detected - try adding more delay after HPD is pulled high in the Set DP Config Section \n");
	//  print "Warning, no DP Video Input to 983 Detected - try adding more delay after HPD is pulled high in the Set DP Config Section";
	}

	if ((mode & UB98X_MODE_VP_CONFIG) != 0) {
		csd_printk_debug("ub983_reg_init UB98X_MODE_VP_CONFIG\n");
		ub983_regcmd_handler(client, UB98X_MODE_VP_CONFIG);
	}

	;//;// *********************************************;
	;//;// Enable PATGEN;
	;//;// *********************************************;
	if ((mode & UB98X_MODE_SER_PATGEN_CONFIG) != 0) {
		csd_printk_debug("ub983_reg_init UB98X_MODE_SER_PATGEN_CONFIG\n");
		ub983_regcmd_handler(client, UB98X_MODE_SER_PATGEN_CONFIG);
	}

	if ((mode & UB98X_MODE_SER_TX_LINK_CONFIG) != 0) {
		csd_printk_debug("ub983_reg_init UB98X_MODE_SER_TX_LINK_CONFIG\n");

		;//;// *********************************************;
		;//;// Configure Serializer TX Link Layer;
		;//;// *********************************************;
		ub983_regcmd_handler(client, UB98X_MODE_SER_TX_LINK_CONFIG);
	}

	if ((mode & UB98X_MODE_DES_CONFIG) != 0) {
		csd_printk_debug("ub983_reg_init UB98X_MODE_DES_CONFIG\n");

		;//;// *********************************************;
		;//;// Override DES 0 eFuse - Can be commented out if using final production silicon;
		;//;// *********************************************;
		ub983_regcmd_handler(client, UB98X_MODE_DES_CONFIG);
		mdelay(0.04*10000);
		;//;// *********************************************;
		;//;// Read Deserializer 0 Temp;
		;//;// *********************************************;
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x40,0x6c);
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x41,0xd);
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x42,0x0);
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x41,0x13);
		TEMP_FINAL = ub983_read(client,I2C_7BIT_ADDR(DESALIAS0),0x42);
		TEMP_FINAL_C = 2*TEMP_FINAL - 273;
		;//;// *********************************************;
		;//;// Set up Deserializer 0 Temp Ramp Optimizations;
		;//;// *********************************************;
		Efuse_TS_CODE = 2;
		Ramp_UP_Range_CODES_Needed = (int)((150-TEMP_FINAL_C)/(190/11)) + 1;
		Ramp_DN_Range_CODES_Needed = (int)((TEMP_FINAL_C-30)/(190/11)) + 1;
		Ramp_UP_CAP_DELTA = Ramp_UP_Range_CODES_Needed - 4;
		Ramp_DN_CAP_DELTA = Ramp_DN_Range_CODES_Needed - 7;
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x40,0x3c);
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x41,0xf5);
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x42,(Efuse_TS_CODE<<4)+1) ;// Override TS_CODE Efuse Code;
		if( Ramp_UP_CAP_DELTA > 0) {
		TS_CODE_UP = Efuse_TS_CODE - Ramp_UP_CAP_DELTA;
		if( TS_CODE_UP < 0) {
			TS_CODE_UP = 0;
		}
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x41,0xf5);
		rb = ub983_read(client,I2C_7BIT_ADDR(DESALIAS0),0x42);
		rb &= 0x8F;
		rb |= (TS_CODE_UP << 4);
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x42,rb);
		rb = ub983_read(client,I2C_7BIT_ADDR(DESALIAS0),0x42);
		rb &= 0xFE;
		rb |= 0x01;
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x42,rb);
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x1,0x1);
		mdelay(0.04*10000);
		}
		if( Ramp_DN_CAP_DELTA > 0) {
		TS_CODE_DN = Efuse_TS_CODE + Ramp_DN_CAP_DELTA;
		if( TS_CODE_DN >= 7) {
			TS_CODE_DN = 7;
		}
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x41,0xf5);
		rb = ub983_read(client,I2C_7BIT_ADDR(DESALIAS0),0x42);
		rb &= 0x8F;
		rb |= (TS_CODE_DN << 4);
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x42,rb);
		rb = ub983_read(client,I2C_7BIT_ADDR(DESALIAS0),0x42);
		rb &= 0xFE;
		rb |= 0x01;
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x42,rb);
		ub983_write(client,I2C_7BIT_ADDR(DESALIAS0),0x1,0x1);
		mdelay(0.04*10000);
		}
	}

	// Clear CRC errors from initial link process;
	Reg_value = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x2);
	Reg_value = Reg_value | 0x20;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x2,Reg_value) ;//CRC Error Reset;
	Reg_value = ub983_read(client,I2C_7BIT_ADDR(SERADDR),0x2);
	Reg_value = Reg_value & 0xdf;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x2,Reg_value) ;//CRC Error Reset Clear;
	ub983_write(client,I2C_7BIT_ADDR(SERADDR),0x2d,0x1);

	if ((mode & UB98X_MODE_DES_DP_CONFIG) != 0) {
		csd_printk_debug("ub983_reg_init UB98X_MODE_DES_DP_CONFIG\n");
		ub983_regcmd_handler(client, UB98X_MODE_DES_DP_CONFIG);
		if ((mode & UB98X_MODE_DES_PATGEN_CONFIG) != 0)
		{
			csd_printk_debug("ub983_reg_init UB98X_MODE_DES_PATGEN_CONFIG\n");
			// Set up deserializer Patgen
			ub983_regcmd_handler(client, UB98X_MODE_DES_PATGEN_CONFIG);
		}
		ub983_regcmd_handler(client, UB98X_MODE_DES_DP_CONFIG_PART2);
	}
	//ub983_i2c_write_register(client,0x1b,0x88);
	//ub983_i2c_write_register(client,0x7 ,0x99);

	return 0;

//init_error:
//return -1;
}

static int ub98x_device_init(struct geely_data *ts, unsigned char mode)
{
	int ret = 0;
	int retry_time = 5;

	ub98x_power_down(ts);
	mdelay(1000);

	csd_printk_debug("ub98x_device_init step 1 ub98x_power_up\n");
	while ((ret = ub98x_power_up(ts)) < 0 && retry_time > 0) {
		csd_printk_debug("ub98x_device_init ub98x_power_up retry! ret:%d ,time:%d\n",ret ,--retry_time);
	}

	if (ret < 0) {
		csd_printk_debug("ub98x_device_init ub98x_power_up fail \n");
		return ret ;
	}

	csd_printk_debug("ub98x_device_init step 2 ub983_reg_init\n");
	while ((ret = ub983_reg_init(ts->client, mode)) < 0 && retry_time > 0) {
		csd_printk_debug("ub98x_device_init ub983_reg_init retry! ret:%d ,time:%d\n",ret ,--retry_time);
	}

	if (ret < 0) {
		csd_printk_debug("ub98x_device_init ub983_reg_init fail \n");
		return ret ;
	}

	csd_printk_debug("ub98x_device_init step 3 ub98x_subdev_init_reg\n");
	while ((ret = ub98x_reg_array_write(ts->client, ub98x_subdev_init_reg, ARRAY_SIZE(ub98x_subdev_init_reg))) < 0 && retry_time > 0) {
		csd_printk_debug("ub98x_device_init ub98x_subdev_init_reg retry! ret:%d ,time:%d\n",ret ,--retry_time);
	}

	if (ret < 0) {
		csd_printk_debug("ub98x_device_init ub98x_subdev_init_reg fail \n");
		return ret ;
	}

	csd_printk_debug("ub98x_device_init step 4 ub98x_dev_init_reg\n");
	while ((ret = ub98x_reg_array_write(ts->client, ub98x_dev_init_reg, ARRAY_SIZE(ub98x_dev_init_reg))) < 0 && retry_time > 0) {
		csd_printk_debug("ub98x_device_init ub98x_dev_init_reg retry! ret:%d ,time:%d\n",ret ,--retry_time);
	}

	if (ret < 0) {
		csd_printk_debug("ub98x_device_init ub98x_dev_init_reg fail \n");
		return ret ;
	}


	ub983_indirect_write(ts->client, 0x0c, PAGE_12, INTR_CTL_VP_VP0, 0xff); //enable all interrupt
	ub983_indirect_write(ts->client, 0x0c, PAGE_12, INTR_CTL_VP_VP1, 0xff); //enable all interrupt
	ub983_indirect_write(ts->client, 0x0c, PAGE_12, INTR_CTL_VP_VP2, 0xff); //enable all interrupt
	ub983_indirect_write(ts->client, 0x0c, PAGE_12, INTR_CTL_VP_VP3, 0xff); //enable all interrupt

	ub983_indirect_write(ts->client, 0x0c, PAGE_09, INTR_CTL_DP_RX_PORT, 0xff); //enable all interrupt
	ub983_indirect_write(ts->client, 0x0c, PAGE_09, INTR_CTL_FPD4_PORT0, 0xff); //enable all interrupt
	ub983_indirect_write(ts->client, 0x0c, PAGE_09, INTR_CTL_FPD4_PORT1, 0xff); //enable all interrupt


	csd_printk_debug("ub98x_device_init end, retry_time:%d\n",retry_time);

	return 0;
}

static int ub98x_init(struct geely_data *ts)
{
	int ret = 0;
	unsigned char mode = 0;
	struct i2c_client *client = ts->client;
	struct device_node *np = client->dev.of_node;
	csd_printk_debug("ub98x_init start \n");

	ts->gpio_vdd1v8_en = of_get_named_gpio_flags(np, "ub98x_vdd1v8_en", 0, NULL);
	if (gpio_is_valid(ts->gpio_vdd1v8_en))
	{
		ret = gpio_request(ts->gpio_vdd1v8_en, "gpio_vdd1v8_en");
		if (ret < 0) {
			dev_err(NULL,"request gpio failed\n");
			return ret;
		}
	}

	ts->gpio_vdd1v1_en = of_get_named_gpio_flags(np, "ub98x_vdd1v1_en", 0, NULL);
	if (gpio_is_valid(ts->gpio_vdd1v1_en))
	{
		ret = gpio_request(ts->gpio_vdd1v1_en, "gpio_vdd1v1_en");
		if (ret < 0) {
			dev_err(NULL,"request gpio failed\n");
			return ret;
		}
	}

	ts->gpio_pdb = of_get_named_gpio_flags(np, "ub98x_pdb", 0, NULL);
	if (gpio_is_valid(ts->gpio_pdb))
	{
		ret = gpio_request(ts->gpio_pdb, "gpio_pdb");
		if (ret < 0) {
			dev_err(NULL,"request gpio failed\n");
			return ret;
		}
	}

	ts->gpio_bl_en1 = of_get_named_gpio_flags(np, "ub98x_bl_en1", 0, NULL);
	if (gpio_is_valid(ts->gpio_bl_en1))
	{
		ret = gpio_request(ts->gpio_bl_en1, "gpio_bl_en1");
		if (ret < 0) {
			dev_err(NULL,"request gpio failed\n");
			return ret;
		}
	}

	ts->gpio_bl_en2 = of_get_named_gpio_flags(np, "ub98x_bl_en2", 0, NULL);
	if (gpio_is_valid(ts->gpio_bl_en2))
	{
		ret = gpio_request(ts->gpio_bl_en2, "gpio_bl_en2");
		if (ret < 0) {
			dev_err(NULL,"request gpio failed\n");
			return ret;
		}
	}

	mode = UB98X_MODE_HPD_FORCE_TRIGE | \
			UB98X_MODE_VP_CONFIG | \
			UB98X_MODE_SER_TX_LINK_CONFIG | \
			UB98X_MODE_DES_CONFIG | \
			UB98X_MODE_DES_DP_CONFIG | \
			UB98X_MODE_SER_PATGEN_CONFIG;

	ret = ub98x_device_init(ts, mode);
	if (ret < 0) {
		dev_err(NULL,"ub98x_device_init failed\n");
		return ret;
	}

	return 0;
}

static ssize_t ub98x_power_control_store(struct device *d, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct geely_data *ts = dev_get_drvdata(d);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		csd_printk_debug("%s is not in decimal form.\n", buf);
	else {
		csd_printk_debug("ub98x_power_control_store , val:%ld.\n", val);

		if (val == 1) {
			ub98x_power_up(ts);
		} else if (val == 0) {
			ub98x_power_down(ts);
		}
	}
	return count;
}

static ssize_t ub98x_reg_dump_show(struct device *d, struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%d\n", 1);
}

static ssize_t ub98x_reg_dump_store(struct device *d, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct geely_data *ts = dev_get_drvdata(d);

	ub98x_dump_state(ts->client);

	return count;
}

static ssize_t ub98x_reinit_show(struct device *d, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 1);
}

static ssize_t ub98x_reinit_store(struct device *d, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct geely_data *ts = dev_get_drvdata(d);
	unsigned long val;
	int retry_time = 0;
	int ret;

	ret = kstrtoul(buf, 16, &val);
	if (ret)
		csd_printk_debug("%s is not in decimal form.\n", buf);
	else {
		csd_printk_debug("ub98x_reinit_store , val:0x%lx.\n", val);
		csd_printk_debug("ub98x_device_init step 2 ub983_reg_init mode=%x \n",ret);
		while ((ret = ub983_reg_init(ts->client, val)) < 0 && retry_time > 0) {
			csd_printk_debug("ub98x_device_init ub983_reg_init retry! ret:%d ,time:%d\n",ret ,--retry_time);
		}

		if (ret < 0) {
			csd_printk_debug("ub98x_device_init ub983_reg_init fail \n");
			return ret ;
		}

		csd_printk_debug("ub98x_device_init step 3 ub98x_subdev_init_reg\n");
		while ((ret = ub98x_reg_array_write(ts->client, ub98x_subdev_init_reg, ARRAY_SIZE(ub98x_subdev_init_reg))) < 0 && retry_time > 0) {
			csd_printk_debug("ub98x_device_init ub98x_subdev_init_reg retry! ret:%d ,time:%d\n",ret ,--retry_time);
		}

		if (ret < 0) {
			csd_printk_debug("ub98x_device_init ub98x_subdev_init_reg fail \n");
			return ret ;
		}

		csd_printk_debug("ub98x_device_init step 4 ub98x_dev_init_reg\n");
		while ((ret = ub98x_reg_array_write(ts->client, ub98x_dev_init_reg, ARRAY_SIZE(ub98x_dev_init_reg))) < 0 && retry_time > 0) {
			csd_printk_debug("ub98x_device_init ub98x_dev_init_reg retry! ret:%d ,time:%d\n",ret ,--retry_time);
		}

		if (ret < 0) {
			csd_printk_debug("ub98x_device_init ub98x_dev_init_reg fail \n");
			return ret ;
		}
		csd_printk_debug("ub98x_device_init end, retry_time:%d\n",retry_time);
	}


	return count;
}

static ssize_t ub98x_reset_show(struct device *d, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 1);
}

static ssize_t ub98x_reset_store(struct device *d, struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct geely_data *ts = dev_get_drvdata(d);
	unsigned char mode = 0;
	mode = UB98X_MODE_HPD_FORCE_TRIGE | \
			UB98X_MODE_VP_CONFIG | \
			UB98X_MODE_SER_TX_LINK_CONFIG | \
			UB98X_MODE_DES_CONFIG | \
			UB98X_MODE_DES_DP_CONFIG;

	ub98x_device_init(ts, mode);
	return count;
}

static ssize_t ub98x_init_show(struct device *d, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", 1);
}

static ssize_t ub98x_init_store(struct device *d, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct geely_data *ts = dev_get_drvdata(d);

	//ub98x_device_init(ts);
	ub98x_init(ts);

	return count;
}

static DEVICE_ATTR(power_control, S_IWUSR | S_IRUGO, ub98x_power_control_show, ub98x_power_control_store);
static DEVICE_ATTR(reg_dump, S_IWUSR | S_IRUGO, ub98x_reg_dump_show, ub98x_reg_dump_store);
static DEVICE_ATTR(reinit, S_IWUSR | S_IRUGO, ub98x_reinit_show, ub98x_reinit_store);
static DEVICE_ATTR(reset, S_IWUSR | S_IRUGO, ub98x_reset_show, ub98x_reset_store);
static DEVICE_ATTR(init, S_IWUSR | S_IRUGO, ub98x_init_show, ub98x_init_store);

static struct attribute *ub98x_attributes[] = {
	&dev_attr_power_control.attr,
	&dev_attr_reg_dump.attr,
	&dev_attr_reinit.attr,
	&dev_attr_reset.attr,
	&dev_attr_init.attr,
	NULL
};

static const struct attribute_group ub98x_group = {
	.attrs = ub98x_attributes,
};

static irqreturn_t ub98x_irq(int irq, void *_dev)
{
	struct geely_data *ts = _dev;
	struct i2c_client *client = ts->client;
	unsigned char value08 = 0;
	unsigned int value32 = 0;

	csd_printk_debug("%s   --------> \n",__FUNCTION__);

	ub983_i2c_read_register(client, 0x0c, FPD3_ISR, &value08);
	csd_printk_debug("%s FPD3_ISR : 0x%02x \n",__FUNCTION__,value08);


	//[6] IS_DP_SINK1_STS_CHANGE:  Interrupt Status on DP stream 1 status change
	//[5] IS_DP_SINK0_STS_CHANGE:  Interrupt Status on DP stream 0 status change
	//[4] IS_DP_LINK_RATE_CHANGE:  Interrupt Status on DP Port Link rate change
	//[3] IS_DP_LANE_COUNT_CHANGE: Interrupt Status on DP Port Lane count change
	//[2] IS_ANY_DP_RX_CORE_INTR:  Interrupt Status on Any DP Core Interrupt
	//[1] IS_DP_LINK_TRAINING_DONE:Interrupt Status on DP Port Link training Done
	//[0] IS_DP_LINK_LOST:         Interrupt Status on DP Port Link Lost
	ub983_indirect_read(client, 0x0c, PAGE_09, INTR_STS_DP_RX_PORT, &value08);
	csd_printk_debug("%s INTR_STS_DP_RX_PORT : 0x%02x \n",__FUNCTION__,value08);


	//[6] IS_RX_LOCK_DET_INT:      Interrupt on Receiver Lock Detect
	//[5] IS_RX_REM_INT:           Interrupt on Remote Receiver interrupt
	//[4] IS_DES_INT:              Set to 1 if the Deserializer has sent an interrupt to the Serializer
	//[3] IS_RX_LOCK_LOST_DET_INT: Interrupt on Receiver Lock Lost Detect
	ub983_indirect_read(client, 0x0c, PAGE_09, INTR_STS_FPD4_PORT0, &value08);
	csd_printk_debug("%s INTR_STS_FPD4_PORT0 : 0x%02x \n",__FUNCTION__,value08);
	if ((value08 & 0x08)) {
		csd_printk_debug("-- %s INTR_STS_FPD4_PORT0 : Receiver Lock Lost Detect \n",__FUNCTION__);
	}
	//[6] IS_RX_LOCK_DET_INT:      Interrupt on Receiver Lock Detect
	//[5] IS_RX_REM_INT:           Interrupt on Remote Receiver interrupt
	//[4] IS_DES_INT:              Set to 1 if the Deserializer has sent an interrupt to the Serializer
	//[3] IS_RX_LOCK_LOST_DET_INT: Interrupt on Receiver Lock Lost Detect
	ub983_indirect_read(client, 0x0c, PAGE_09, INTR_STS_FPD4_PORT1, &value08);
	csd_printk_debug("%s INTR_STS_FPD4_PORT1 : 0x%02x \n",__FUNCTION__,value08);

	if ((value08 & 0x08)) {
		csd_printk_debug("-- %s INTR_STS_FPD4_PORT1 : Receiver Lock Lost Detect \n",__FUNCTION__);
	}

	//[6] IS_CROP_VERT_ERR:        Video Crop Vertical error
	//[5] IS_CROP_HOR_ERR:         Video Crop Horizontal error
	//[4] IS_TIMING_DATA_ERR:      Timing Gen Data Available error
	//[3] IS_TIMING_LINE_ERR:      Timing Gen Line Number error
	//[2] IS_TIMING_STRT_ERR:      Timing Gen Active Start error
	//[1] IS_VP_VBUF_ERR:          Video Buffer error
	//[0] IS_VP_STATUS_CHANGE:     Video Processor Status Changed
	ub983_indirect_read(client, 0x0c, PAGE_12, INTR_STS_VP_VP0, &value08);
	csd_printk_debug("%s INTR_STS_VP_VP0 : 0x%02x \n",__FUNCTION__,value08);

	ub983_indirect_read(client, 0x0c, PAGE_12, INTR_STS_VP_VP1, &value08);
	csd_printk_debug("%s INTR_STS_VP_VP1 : 0x%02x \n",__FUNCTION__,value08);

	ub983_indirect_read(client, 0x0c, PAGE_12, INTR_STS_VP_VP2, &value08);
	csd_printk_debug("%s INTR_STS_VP_VP2 : 0x%02x \n",__FUNCTION__,value08);

	ub983_indirect_read(client, 0x0c, PAGE_12, INTR_STS_VP_VP3, &value08);
	csd_printk_debug("%s INTR_STS_VP_VP3 : 0x%02x \n",__FUNCTION__,value08);

	ub983_apb_read(client, 0x0c, ((DISPLAYPORT_INTERRUPT_CAUSE & 0xff00) >> 16), (DISPLAYPORT_INTERRUPT_CAUSE & 0x00ff), &value32);
	csd_printk_debug("%s DISPLAYPORT_INTERRUPT_CAUSE : 0x%x \n",__FUNCTION__,value32);
	csd_printk_debug("%s   <-------- \n",__FUNCTION__);

	return IRQ_HANDLED;
}

static int ub98x_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct geely_data *ts;
	int ret = 0;
	int ts_irq_gpio = -1;
	u32 touch_screen_addr;
	csd_printk_debug("ub98x_i2c_probe start\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		csd_printk_error("%s: i2c check functionality error\n", DEVICE_NAME);
		return -ENXIO;
	}

	ts = devm_kzalloc(&client->dev, sizeof(struct geely_data), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	mutex_init(&ts->rw_lock);

	if (np) {
		ret = of_property_read_u32(np, "touch_addr", &touch_screen_addr);
		if (ret)
		{
			csd_printk_debug( "not find touch_addr\n");
			return -EINVAL;
		}

		ts_irq_gpio = of_get_named_gpio(np, "ub98x_intb", 0);
		if (gpio_is_valid(ts_irq_gpio)) {
			gpio_request_one(ts_irq_gpio, GPIOF_IN, "ub98x_intb");
			gpio_direction_input(ts_irq_gpio);
			client->irq = gpio_to_irq(ts_irq_gpio);

			ret = devm_request_threaded_irq(&client->dev, client->irq,
							NULL, ub98x_irq,
							IRQF_ONESHOT | IRQ_TYPE_EDGE_FALLING,
							client->name, ts);
			if (ret) {
				csd_printk_error("Failed to register interrupt\n");
				return ret;
			}

			csd_printk_debug("ub98x_i2c_probe ub98x_intb:%d\n",client->irq);
		} else {
			client->irq = -1;
			csd_printk_debug("ub98x_i2c_probe no ub98x_intb\n");
		}
	}
	ret = sysfs_create_group(&client->dev.kobj, &ub98x_group);
	if (ret < 0) {
		csd_printk_debug("ub98x_i2c_probe sysfs_create_group fail \n");
		return ret;
	}

	ret = ub98x_init(ts);

	g_ts = ts;

	client->addr = touch_screen_addr;

	/*
	* Systems using device tree should set up wakeup via DTS,
	* the rest will configure device as wakeup source by default.
	*/
	if (!client->dev.of_node)
		device_init_wakeup(&client->dev, true);

#ifdef CONFIG_IPO_MANAGER
	if(ipo_manager_register_device(ts->input_name, &client->dev, IPO_MANAGER_DEVICE_TYPE_TOUCHPAD, true) < 0)
		csd_printk_error("ipo_manager_register_device geely_ts(%s) error\n", ts->input_name);
#endif

	csd_printk_debug( "geely_i2c_probe end client->irq %d\n", client->irq);
	return 0;
}

static const struct i2c_device_id ub98x_i2c_id[] = {
	{ "ecarx,ser-ds90ub98x", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ub98x_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id ub98x_of_match[] = {
	{ .compatible = "ecarx,ser-ds90ub98x" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ub98x_of_match);
#endif

static struct i2c_driver ub98x_i2c_driver = {
	.probe = ub98x_i2c_probe,
	.id_table = ub98x_i2c_id,
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = of_match_ptr(ub98x_of_match),
		.pm = &ub98x_pm_ops,
	},
};

module_i2c_driver(ub98x_i2c_driver);

MODULE_AUTHOR("liam chen <liam.chen@siengine.com>");
MODULE_AUTHOR("Haijun Ma <haijunma@ecarx.com.cn>");
MODULE_DESCRIPTION("Geely CTP Touchscreen driver");
MODULE_VERSION("1.0");
MODULE_LICENSE("GPL");
