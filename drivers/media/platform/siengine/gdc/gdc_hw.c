#include <asm/io.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include "gdc_driver.h"
#include "system_log.h"
#include "gdc_reg.h"

#define CONFIG_PARAM_OFFSET 0

struct _gdc_config_param {
    const unsigned char * gdc_sequence;
    u32 gdc_sequence_size;
    u32 total_planes;
    u8  sequential_mode;
    u8  div_width; //use in dividing UV dimensions; actually a shift right
    u8  div_height; //use in dividing UV dimensions; actually a shift right
};

//settings for each test case
struct _gdc_config_param gdc_config_param[gdc_format_max]= {
    {
        //gdc_yuv420_planar
        .total_planes=3,    //total_planes
        .sequential_mode=0, //plane_sequential_processing
        .div_width=1,       //div_width
        .div_height=1       //div_height
    },
    {
        //gdc_yuv420_semiplanar NV12
        .total_planes=2,    //total_planes
        .sequential_mode=0, //plane_sequential_processing
        .div_width=0,       //div_width
        .div_height=1       //div_height
    },
    {
        //gdc_yuv420_semiplanar NV16
        .total_planes=2,    //total_planes
        .sequential_mode=0, //plane_sequential_processing
        .div_width=0,       //div_width
        .div_height=1       //div_height

    },
};

int gdc_init(gdc_settings_t *gdc_settings)
{
    gdc_settings->is_waiting_gdc = 0;

    if (gdc_settings->gdc_config.output_width == 0 || gdc_settings->gdc_config.output_height == 0) {
        LOG( LOG_ERR, "Wrong GDC output resolution.\n" );
        return -1;
    }

    //set the configuration address and size to the gdc block
    gdc_config_addr_write(gdc_settings->base_gdc, (uintptr_t)gdc_settings->fw_address_pa);
    gdc_config_size_write(gdc_settings->base_gdc, gdc_settings->gdc_config.config_size / CONFIG_MEM_ALIGN);

    //set the gdc in and output resolution
    gdc_datain_width_write(gdc_settings->base_gdc, gdc_settings->gdc_config.input_width);
    gdc_datain_height_write(gdc_settings->base_gdc, gdc_settings->gdc_config.input_height);
    gdc_dataout_width_write(gdc_settings->base_gdc, gdc_settings->gdc_config.output_width);
    gdc_dataout_height_write(gdc_settings->base_gdc, gdc_settings->gdc_config.output_height);

    return 0;
}

void gdc_stop(gdc_settings_t *gdc_settings)
{
    gdc_settings->is_waiting_gdc = 0;
    gdc_start_flag_write(gdc_settings->base_gdc, 0);
}

void gdc_start(gdc_settings_t *gdc_settings)
{
    gdc_start_flag_write(gdc_settings->base_gdc, 0); //do a stop for sync
    gdc_start_flag_write(gdc_settings->base_gdc, 1);
    gdc_settings->is_waiting_gdc = 1;
}

int gdc_process( gdc_settings_t *gdc_settings, u32 num_input, u8 flag)
{
    u32 lineoffset, height;

    if (!gdc_settings->is_waiting_gdc) {
        LOG(LOG_INFO,"starting GDC process.\n");

        if (num_input == 0 || num_input > ACAMERA_GDC_MAX_INPUT) {
            LOG(LOG_CRIT,"GDC number of input invalid %d.\n",num_input);
            return -1;
        }

        if (num_input != gdc_settings->gdc_config.total_planes) {
            LOG(LOG_CRIT,"GDC number of input less than planes %d.\n",num_input);
            return -1;
        }

        //process input addresses
        if (num_input >= 1) {
            lineoffset = gdc_settings->gdc_config.input_width;
            height = gdc_settings->gdc_config.input_height;
            gdc_data1in_addr_write(gdc_settings->base_gdc, (uintptr_t)gdc_settings->input_address_pa[0]);
            gdc_data1in_line_offset_write(gdc_settings->base_gdc, lineoffset);
        }

        if (num_input >= 2 && gdc_settings->gdc_config.sequential_mode == 0) { //only processed if not in toggle mode
            lineoffset = gdc_settings->gdc_config.output_width >> gdc_settings->gdc_config.div_width;
            height = gdc_settings->gdc_config.output_height >> gdc_settings->gdc_config.div_height;
            gdc_data2in_addr_write(gdc_settings->base_gdc, (uintptr_t)gdc_settings->input_address_pa[1]);

            if (flag == 1)
                gdc_data2in_line_offset_write(gdc_settings->base_gdc, lineoffset*2);//*2 for yuv422sp nv16
            else
                gdc_data2in_line_offset_write(gdc_settings->base_gdc, lineoffset);//for yuv420sp nv12
        }

        if(num_input >= 3 && gdc_settings->gdc_config.sequential_mode == 0) { //only processed if not in toggle mode
            lineoffset = gdc_settings->gdc_config.output_width >> gdc_settings->gdc_config.div_width;
            height = gdc_settings->gdc_config.output_height >> gdc_settings->gdc_config.div_height;
            gdc_data3in_addr_write(gdc_settings->base_gdc, (uintptr_t)gdc_settings->input_address_pa[2]);
            gdc_data3in_line_offset_write(gdc_settings->base_gdc, lineoffset);
        }

        //outputs
        if (num_input >= 1) {
            lineoffset = gdc_settings->gdc_config.output_width;
            height=gdc_settings->gdc_config.output_height;
            gdc_data1out_addr_write(gdc_settings->base_gdc, (uintptr_t)gdc_settings->output_address_pa[0]);
            gdc_data1out_line_offset_write(gdc_settings->base_gdc, lineoffset);
        }

        if (num_input >= 2 && gdc_settings->gdc_config.sequential_mode == 0) {
            lineoffset = gdc_settings->gdc_config.output_width >> gdc_settings->gdc_config.div_width;
            height = gdc_settings->gdc_config.output_height >> gdc_settings->gdc_config.div_height;
            gdc_data2out_addr_write(gdc_settings->base_gdc, (uintptr_t)gdc_settings->output_address_pa[1]);
            gdc_data2out_line_offset_write(gdc_settings->base_gdc, lineoffset);
        }

        if (num_input >= 3 && gdc_settings->gdc_config.sequential_mode == 0) {
            lineoffset = gdc_settings->gdc_config.output_width >> gdc_settings->gdc_config.div_width;
            height = gdc_settings->gdc_config.output_height >> gdc_settings->gdc_config.div_height;
            gdc_data3out_addr_write(gdc_settings->base_gdc, (uintptr_t)gdc_settings->output_address_pa[2]);
            gdc_data3out_line_offset_write(gdc_settings->base_gdc, lineoffset);
        }

        LOG(LOG_INFO, "gdc process over!\n");
        return 0;
    } else {
        gdc_start_flag_write(gdc_settings->base_gdc, 0);
        LOG(LOG_CRIT, "No interrupt from GDC block. Still waiting...\n");
        gdc_start_flag_write(gdc_settings->base_gdc, 1);

        return -1;
    }
}

u32 gdc_get_status(gdc_settings_t *gdc_settings)
{
    u32 status = gdc_status_read(gdc_settings->base_gdc);
    LOG(LOG_CRIT, "status = 0x%2x", status);
    return status;
}

int gdc_get_frame(gdc_settings_t *gdc_settings, u32 num_input)
{
    u32 out_addr[ACAMERA_GDC_MAX_INPUT];
    u32 out_lineoffset[ACAMERA_GDC_MAX_INPUT];

    if(gdc_settings == NULL) {
        LOG(LOG_CRIT, "gdc_get_frame gdc_settings is NULL, num_input:%d", num_input);
        return -1;
    }

    if (gdc_settings->is_waiting_gdc) {
        if (num_input >= 1) {
            out_addr[0] = gdc_data1out_addr_read(gdc_settings->base_gdc);
            out_lineoffset[0] = gdc_data1out_line_offset_read(gdc_settings->base_gdc);
        }

        if (num_input >= 2) {
            out_addr[1] = gdc_data2out_addr_read(gdc_settings->base_gdc);
            out_lineoffset[1] = gdc_data2out_line_offset_read(gdc_settings->base_gdc);
        }

        if (num_input >= 3) {
            out_addr[2] = gdc_data3out_addr_read(gdc_settings->base_gdc);
            out_lineoffset[2] = gdc_data3out_line_offset_read(gdc_settings->base_gdc);
        }

        LOG(LOG_INFO, "gdc0 interrupt normal!.\n");
        //done of the current frame and stop gdc block
        //gdc_stop(gdc_settings);
        return 0;
    } else {
        LOG(LOG_CRIT, "Unexpected interrupt from GDC.\n");
        return -1;
    }
}

int updata_buffer_to_gdc_hw(struct se_gdc *se_gdc)
{
    int ret = 0;

    LOG(LOG_INFO, "update_buffer_to_gdc_hw...\n");
    ret = gdc_process(&se_gdc->gdc_settings, se_gdc->gdc_settings.gdc_config.total_planes, se_gdc->p_gdc_ctx->in.fmt.flags);
    if (ret < 0)
        return -EINVAL;

    return ret;
}

//we need to copy the gdc configuration sequence to the gdc config address
u32 gdc_load_settings_to_memory(struct se_gdc *se_gdc, u8 *config_mem_start, const u8 *config_settings_start, u32 config_size)
{
    u32 i = 0;
    unsigned long flags;
    LOG(LOG_INFO, "config_size = %d\n", config_size);

    spin_lock_irqsave(&se_gdc->ctrl_lock, flags);
    for (i = 0; i < config_size; i++) {
        config_mem_start[i] = config_settings_start[i];
        if(config_mem_start[i] != config_settings_start[i]) {
            LOG(LOG_CRIT, "GDC config mismatch index %d, values 0x%x vs 0x%x\n",
                i, config_mem_start[i], config_settings_start[i]);
            spin_unlock_irqrestore(&se_gdc->ctrl_lock, flags);
            return 0;
        }
    }
    spin_unlock_irqrestore(&se_gdc->ctrl_lock, flags);

    return config_size;
}

static int alloc_mem_for_fw(gdc_settings_t *gdc_settings)
{
    gdc_settings->fw_address_va = dma_alloc_coherent(gdc_settings->dev, PAGE_ALIGN(gdc_settings->gdc_config.fw_alloc_mem_size),
                                  &gdc_settings->fw_address_pa, GFP_KERNEL);
    if (gdc_settings->fw_address_va == NULL) {
        LOG( LOG_CRIT, "dma_alloc_coherent(size 0x%x): failed", gdc_settings->gdc_config.fw_alloc_mem_size);
        gdc_settings->fw_address_va = NULL;
        return -1;
    } else {
        LOG(LOG_INFO, "dma_pa = 0x%llx\n", (u64)gdc_settings->fw_address_pa);
        LOG(LOG_INFO, "dma_va = 0x%pK\n", gdc_settings->fw_address_va);
        return 0;
    }
}

int gdc_fw_init(struct se_gdc *se_gdc)
{
    u8 fmt_index = se_gdc->p_gdc_ctx->in.fmt_index;

    gdc_settings_t *gdc_settings = &se_gdc->gdc_settings;

    gdc_settings->gdc_config.input_width = se_gdc->p_gdc_ctx->in.width;
    gdc_settings->gdc_config.input_height = se_gdc->p_gdc_ctx->in.height;
    gdc_settings->gdc_config.output_width = se_gdc->p_gdc_ctx->out.width;
    gdc_settings->gdc_config.output_height = se_gdc->p_gdc_ctx->out.height;
    gdc_settings->gdc_config.total_planes = gdc_config_param[fmt_index].total_planes;
    gdc_settings->gdc_config.sequential_mode = gdc_config_param[fmt_index].sequential_mode;
    gdc_settings->gdc_config.div_width = gdc_config_param[fmt_index].div_width;
    gdc_settings->gdc_config.div_height = gdc_config_param[fmt_index].div_height;
    gdc_settings->gdc_config.fw_alloc_mem_size = 0;

    LOG(LOG_INFO, "fmt_index = %d", fmt_index);
    LOG(LOG_INFO, "input_width = %d", gdc_settings->gdc_config.input_width);
    LOG(LOG_INFO, "input_height = %d", gdc_settings->gdc_config.input_height);
    LOG(LOG_INFO, "Done config gdc parameters.. return 0\n");
    return 0;
}

void gdc_fw_exit(struct se_gdc *se_gdc)
{
    if (se_gdc->gdc_settings.fw_address_va != NULL) {
        dma_free_coherent(&se_gdc->pdev->dev, se_gdc->gdc_settings.gdc_config.fw_alloc_mem_size,
                          se_gdc->gdc_settings.fw_address_va, se_gdc->gdc_settings.fw_address_pa);
        se_gdc->gdc_settings.fw_address_va = NULL;
    }
}

int reload_fw_paramters(struct se_gdc *se_gdc, const u8 *ppara, u32 para_len)
{
    int ret = 0;
    u32 re_count = 0;

    gdc_settings_t *gdc_settings = &se_gdc->gdc_settings;

    if (gdc_settings->gdc_config.fw_alloc_mem_size < para_len) {
        gdc_fw_exit(se_gdc);
        LOG(LOG_INFO, "re alloc mem space\n");
        gdc_settings->gdc_config.fw_alloc_mem_size = para_len;
        ret = alloc_mem_for_fw(gdc_settings);
        if (ret != 0) {
            LOG(LOG_ERR, "Failed to reallco fw mem space size = %d\n", para_len);
            goto result;
        }
    }

    re_count = gdc_load_settings_to_memory(se_gdc, (u8 *)(gdc_settings->fw_address_va),
                  ppara, para_len);
    if (re_count != para_len) {
        LOG(LOG_ERR, "fw reload failed\n");
        ret = -EINVAL;
        goto result;
    }
    gdc_settings->gdc_config.config_size = para_len;

    LOG(LOG_INFO, "reload fw paramters done\n");

    ret = gdc_init(gdc_settings);
    if (ret != 0) {
        LOG(LOG_ERR, "Failed to reconfig gdc block\n");
        goto result;
    }

result:
    return ret;
}
