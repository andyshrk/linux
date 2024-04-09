#ifndef __GDC_HW_H__
#define __GDC_HW_H__

/**
 *   Configure the output gdc configuration address/size and buffer address/size; resolution; and sequential mode.
 *
 *
 *   @param  gdc_settings - overall gdc settings and state
 *
 *   @return 0 - success
 *           -1 - fail.
 */
int gdc_init(gdc_settings_t *gdc_settings);
/**
 *   This function stops the gdc block
 *
 *   @param  gdc_settings - overall gdc settings and state
 *
 */
void gdc_stop(gdc_settings_t *gdc_settings);

/**
 *   This function starts the gdc block
 *
 *   Writing 0->1 transition is necessary for trigger
 *
 *   @param  gdc_settings - overall gdc settings and state
 *
 */
void gdc_start(gdc_settings_t *gdc_settings);

/**
 *   This function points gdc to its input resolution and yuv address and offsets
 *
 *   Shown inputs to GDC are Y and UV plane address and offsets
 *
 *   @param  gdc_settings - overall gdc settings and state
 *   @param  num_input -  number of input addresses in the array to be processed by gdc
 *   @param  input_addr - input addresses in the array to be processed by gdc
 *
 *   @return 0 - success
 *           -1 - no interrupt from GDC.
 */

int gdc_process(gdc_settings_t *gdc_settings, u32 num_input,  u8 flag);

/**
 *   This function gets the GDC ouput frame addresses and offsets and updates the frame buffer via callback if it is available
 *
 *   Shown ouputs to GDC are Y and UV plane address and offsets
 *
 *   @param  gdc_settings - overall gdc settings and state
 *   @param  num_input -  number of input addresses in the array to be processed by gdc
 *
 *   @return 0 - success
 *           -1 - unexpected interrupt from GDC.
 */
int gdc_get_frame(gdc_settings_t *gdc_settings, u32 num_input);

/**
 *   gdc_get_status
 *
 *   This function is going to get gdc hardware regester status.
 *
 *   @param gdc_settings - overall gdc settings and state
 *   @return succefful 0 ; error negtive
 */

uint32_t gdc_get_status( gdc_settings_t *gdc_settings);

/**
 *   gdc hardware initialiation
 *
 *   This function is going setting gdc hardware regester.
 *
 *   @param se_gdc - platform struct device
 *   @return succefful 0 ; error negtive
 */
int gdc_fw_init(struct se_gdc *se_gdc);


/**
 *   gdc hardware exit
 *
 *   This function is going setting gdc hardware regester.
 *
 *   @param se_gdc - platform struct device
 *   @return succefful 0 ; error negtive
 */

void gdc_fw_exit(struct se_gdc *se_gdc);

/**
 *   updata_buffer_to_gdc_hw
 *
 *   This function is going setting gdc hardware regester.
 *
 *   @param se_gdc - platform struct device
 *   @return succefful 0 ; error negtive
 */

int updata_buffer_to_gdc_hw(struct se_gdc *se_gdc);


/**
 *   reload_fw_paramters
 *
 *   This function is going reload gdc caculate paramters for new settings.
 *
 *   @param se_gdc   - platform struct device
 *   @param ppara    - pointer to fw data space
 *   @param para_len - fw data length
 *   @return succefful 0 ; error negtive
 */

int reload_fw_paramters(struct se_gdc *se_gdc, const u8 *ppara, u32 para_len);

#endif
