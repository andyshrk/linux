#ifndef __ACAMERA_GDC_CONFIG_H__
#define __ACAMERA_GDC_CONFIG_H__

#include "system_log.h"
#include <asm/io.h>

static u32 system_gdc_read_32(u64 addr)
{
    u32 result = 0;

    result = readl((void __iomem *)addr);
    LOG(LOG_DEBUG, "addr = 0x%llx\n", addr);
    return result;
}

static void system_gdc_write_32(u64 addr, u32 data)
{
    writel(data, (void __iomem *)addr);
    LOG(LOG_DEBUG, "addr = 0x%llx, data = 0x%2x\n", addr, data);
}

// ------------------------------------------------------------------------------ //
// Register: API
// ------------------------------------------------------------------------------ //

#define ACAMERA_GDC_ID_API_DEFAULT (0x0)
#define ACAMERA_GDC_ID_API_DATASIZE (32)
#define ACAMERA_GDC_ID_API_OFFSET (0x0)
#define ACAMERA_GDC_ID_API_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 acamera_gdc_id_api_read(u64 base)
{
    return system_gdc_read_32(base);
}
// ------------------------------------------------------------------------------ //
// Register: Product
// ------------------------------------------------------------------------------ //

#define ACAMERA_GDC_ID_PRODUCT_DEFAULT (0x0)
#define ACAMERA_GDC_ID_PRODUCT_DATASIZE (32)
#define ACAMERA_GDC_ID_PRODUCT_OFFSET (0x4)
#define ACAMERA_GDC_ID_PRODUCT_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_id_product_read(u64 base)
{
    return system_gdc_read_32(base+ACAMERA_GDC_ID_PRODUCT_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: Version
// ------------------------------------------------------------------------------ //

#define ACAMERA_GDC_ID_VERSION_DEFAULT (0x0)
#define ACAMERA_GDC_ID_VERSION_DATASIZE (32)
#define ACAMERA_GDC_ID_VERSION_OFFSET (0x8)
#define ACAMERA_GDC_ID_VERSION_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_id_version_read(u64 base)
{
    return system_gdc_read_32(base + ACAMERA_GDC_ID_VERSION_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: Revision
// ------------------------------------------------------------------------------ //

#define ACAMERA_GDC_ID_REVISION_DEFAULT (0x0)
#define ACAMERA_GDC_ID_REVISION_DATASIZE (32)
#define ACAMERA_GDC_ID_REVISION_OFFSET (0xc)
#define ACAMERA_GDC_ID_REVISION_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_id_revision_read(u64 base)
{
    return system_gdc_read_32(base+ACAMERA_GDC_ID_REVISION_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Group: GDC
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// GDC controls
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Register: config addr
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Base address of configuration stream (in bytes, AXI word aligned)
// ------------------------------------------------------------------------------ //

#define GDC_CONFIG_ADDR_DEFAULT (0x0)
#define GDC_CONFIG_ADDR_DATASIZE (32)
#define GDC_CONFIG_ADDR_OFFSET (0x10)
#define GDC_CONFIG_ADDR_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_config_addr_write(u64 base, u32 data)
{
    system_gdc_write_32(base + GDC_CONFIG_ADDR_OFFSET, data);
}
static __inline u32 gdc_config_addr_read(u64 base)
{
    return system_gdc_read_32(base + GDC_CONFIG_ADDR_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: config size
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Size of the configuration stream (in bytes, 32 bit word granularity)
// ------------------------------------------------------------------------------ //

#define GDC_CONFIG_SIZE_DEFAULT (0x0)
#define GDC_CONFIG_SIZE_DATASIZE (32)
#define GDC_CONFIG_SIZE_OFFSET (0x14)
#define GDC_CONFIG_SIZE_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_config_size_write(u64 base, u32 data)
{
    system_gdc_write_32(base + GDC_CONFIG_SIZE_OFFSET, data);
}
static __inline u32 gdc_config_size_read(u64 base)
{
    return system_gdc_read_32(base + GDC_CONFIG_SIZE_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: datain width
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Width of the input image (in pixels)
// ------------------------------------------------------------------------------ //

#define GDC_DATAIN_WIDTH_DEFAULT (0x0)
#define GDC_DATAIN_WIDTH_DATASIZE (16)
#define GDC_DATAIN_WIDTH_OFFSET (0x20)
#define GDC_DATAIN_WIDTH_MASK (0xffff)

// args: data (16-bit)
static __inline void gdc_datain_width_write(u64 base, u16 data)
{
    u32 curr = system_gdc_read_32(base + GDC_DATAIN_WIDTH_OFFSET);
    system_gdc_write_32(base + GDC_DATAIN_WIDTH_OFFSET,
                        (((u32) (data & GDC_DATAIN_WIDTH_MASK)) << 0) |
                        (curr & (~GDC_DATAIN_WIDTH_MASK)));
}
static __inline u16 gdc_datain_width_read(u64 base)
{
    return (u16)((system_gdc_read_32(base + GDC_DATAIN_WIDTH_OFFSET) &
                  GDC_DATAIN_WIDTH_MASK) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: datain_height
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Height of the input image (in pixels)
// ------------------------------------------------------------------------------ //

#define GDC_DATAIN_HEIGHT_DEFAULT (0x0)
#define GDC_DATAIN_HEIGHT_DATASIZE (16)
#define GDC_DATAIN_HEIGHT_OFFSET (0x24)
#define GDC_DATAIN_HEIGHT_MASK (0xffff)

// args: data (16-bit)
static __inline void gdc_datain_height_write(u64 base, u16 data)
{
    u32 curr = system_gdc_read_32(base + GDC_DATAIN_HEIGHT_OFFSET);
    system_gdc_write_32(base + GDC_DATAIN_HEIGHT_OFFSET, (((u32) (data & GDC_DATAIN_HEIGHT_MASK)) << 0) | (curr & (!GDC_DATAIN_HEIGHT_MASK)));
}
static __inline u16 gdc_datain_height_read(u64 base)
{
    return (u16)((system_gdc_read_32(base + GDC_DATAIN_HEIGHT_OFFSET) & GDC_DATAIN_HEIGHT_MASK) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: data1in addr
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Base address of the 1st plane in the input frame buffer  (in bytes, AXI word aligned)
// ------------------------------------------------------------------------------ //

#define GDC_DATA1IN_ADDR_DEFAULT (0x0)
#define GDC_DATA1IN_ADDR_DATASIZE (32)
#define GDC_DATA1IN_ADDR_OFFSET (0x28)
#define GDC_DATA1IN_ADDR_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_data1in_addr_write(u64 base, u32 data)
{
    system_gdc_write_32(base+GDC_DATA1IN_ADDR_OFFSET, data);
}
static __inline u32 gdc_data1in_addr_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DATA1IN_ADDR_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: data1in line offset
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Address difference between adjacent lines for the 1st plane in the input frame buffer (in bytes, AXI word aligned)
// ------------------------------------------------------------------------------ //

#define GDC_DATA1IN_LINE_OFFSET_DEFAULT (0x0)
#define GDC_DATA1IN_LINE_OFFSET_DATASIZE (32)
#define GDC_DATA1IN_LINE_OFFSET_OFFSET (0x2c)
#define GDC_DATA1IN_LINE_OFFSET_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_data1in_line_offset_write(u64 base, u32 data)
{
    system_gdc_write_32(base+GDC_DATA1IN_LINE_OFFSET_OFFSET, data);
}
static __inline u32 gdc_data1in_line_offset_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DATA1IN_LINE_OFFSET_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: data2in addr
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Address of the 2nd plane in the input frame buffer  (in bytes, AXI word aligned)
// ------------------------------------------------------------------------------ //

#define GDC_DATA2IN_ADDR_DEFAULT (0x0)
#define GDC_DATA2IN_ADDR_DATASIZE (32)
#define GDC_DATA2IN_ADDR_OFFSET (0x30)
#define GDC_DATA2IN_ADDR_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_data2in_addr_write(u64 base, u32 data)
{
    system_gdc_write_32(base+GDC_DATA2IN_ADDR_OFFSET, data);
}
static __inline u32 gdc_data2in_addr_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DATA2IN_ADDR_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: data2in line offset
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Adress difference between adjacent lines for the 2nd plane in the input frame buffer (in bytes, AXI word aligned)
// ------------------------------------------------------------------------------ //

#define GDC_DATA2IN_LINE_OFFSET_DEFAULT (0x0)
#define GDC_DATA2IN_LINE_OFFSET_DATASIZE (32)
#define GDC_DATA2IN_LINE_OFFSET_OFFSET (0x34)
#define GDC_DATA2IN_LINE_OFFSET_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_data2in_line_offset_write(u64 base, u32 data)
{
    system_gdc_write_32(base+GDC_DATA2IN_LINE_OFFSET_OFFSET, data);
}
static __inline u32 gdc_data2in_line_offset_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DATA2IN_LINE_OFFSET_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: data3in addr
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Base address of the 3rd plane in the input frame buffer  (in bytes, AXI word aligned)
// ------------------------------------------------------------------------------ //

#define GDC_DATA3IN_ADDR_DEFAULT (0x0)
#define GDC_DATA3IN_ADDR_DATASIZE (32)
#define GDC_DATA3IN_ADDR_OFFSET (0x38)
#define GDC_DATA3IN_ADDR_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_data3in_addr_write(u64 base, u32 data)
{
    system_gdc_write_32(base+GDC_DATA3IN_ADDR_OFFSET, data);
}
static __inline u32 gdc_data3in_addr_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DATA3IN_ADDR_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: data3in line offset
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Address difference between adjacent lines for the 3rd plane in the input frame buffer (in bytes, AXI word aligned)
// ------------------------------------------------------------------------------ //

#define GDC_DATA3IN_LINE_OFFSET_DEFAULT (0x0)
#define GDC_DATA3IN_LINE_OFFSET_DATASIZE (32)
#define GDC_DATA3IN_LINE_OFFSET_OFFSET (0x3c)
#define GDC_DATA3IN_LINE_OFFSET_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_data3in_line_offset_write(u64 base, u32 data)
{
    system_gdc_write_32(base+GDC_DATA3IN_LINE_OFFSET_OFFSET, data);
}
static __inline u32 gdc_data3in_line_offset_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DATA3IN_LINE_OFFSET_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: dataout width
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Width of the output image (in pixels)
// ------------------------------------------------------------------------------ //

#define GDC_DATAOUT_WIDTH_DEFAULT (0x0)
#define GDC_DATAOUT_WIDTH_DATASIZE (16)
#define GDC_DATAOUT_WIDTH_OFFSET (0x40)
#define GDC_DATAOUT_WIDTH_MASK (0xffff)

// args: data (16-bit)
static __inline void gdc_dataout_width_write(u64 base, u16 data)
{
    u32 curr = system_gdc_read_32(base+GDC_DATAOUT_WIDTH_OFFSET);
    system_gdc_write_32(base+GDC_DATAOUT_WIDTH_OFFSET, (((u32) (data & GDC_DATAOUT_WIDTH_MASK)) << 0) | (curr & (~GDC_DATAOUT_WIDTH_MASK)));
}
static __inline u16 gdc_dataout_width_read(u64 base)
{
    return (u16)((system_gdc_read_32(base+GDC_DATAOUT_WIDTH_OFFSET) & GDC_DATAOUT_WIDTH_MASK) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: dataout height
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Height of the output image (in pixels)
// ------------------------------------------------------------------------------ //

#define GDC_DATAOUT_HEIGHT_DEFAULT (0x0)
#define GDC_DATAOUT_HEIGHT_DATASIZE (16)
#define GDC_DATAOUT_HEIGHT_OFFSET (0x44)
#define GDC_DATAOUT_HEIGHT_MASK (0xffff)

// args: data (16-bit)
static __inline void gdc_dataout_height_write(u64 base, u16 data)
{
    u32 curr = system_gdc_read_32(base+GDC_DATAOUT_HEIGHT_OFFSET);
    system_gdc_write_32(base+GDC_DATAOUT_HEIGHT_OFFSET, (((u32) (data & GDC_DATAOUT_HEIGHT_MASK)) << 0) | (curr & (~GDC_DATAOUT_HEIGHT_MASK)));
}
static __inline u16 gdc_dataout_height_read(u64 base)
{
    return (u16)((system_gdc_read_32(base+GDC_DATAOUT_HEIGHT_OFFSET) & GDC_DATAOUT_HEIGHT_MASK) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: data1out addr
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Base address of the 1st plane in the output frame buffer  (in bytes, AXI word aligned)
// ------------------------------------------------------------------------------ //

#define GDC_DATA1OUT_ADDR_DEFAULT (0x0)
#define GDC_DATA1OUT_ADDR_DATASIZE (32)
#define GDC_DATA1OUT_ADDR_OFFSET (0x48)
#define GDC_DATA1OUT_ADDR_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_data1out_addr_write(u64 base, u32 data)
{
    system_gdc_write_32(base+GDC_DATA1OUT_ADDR_OFFSET, data);
}
static __inline u32 gdc_data1out_addr_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DATA1OUT_ADDR_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: data1out line offset
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Address difference between adjacent lines for the 1st plane in the output frame buffer (in bytes, AXI word aligned)
// ------------------------------------------------------------------------------ //

#define GDC_DATA1OUT_LINE_OFFSET_DEFAULT (0x0)
#define GDC_DATA1OUT_LINE_OFFSET_DATASIZE (32)
#define GDC_DATA1OUT_LINE_OFFSET_OFFSET (0x4c)
#define GDC_DATA1OUT_LINE_OFFSET_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_data1out_line_offset_write(u64 base, u32 data)
{
    system_gdc_write_32(base+GDC_DATA1OUT_LINE_OFFSET_OFFSET, data);
}
static __inline u32 gdc_data1out_line_offset_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DATA1OUT_LINE_OFFSET_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: data2out addr
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Base address of the 2nd plane in the output frame buffer  (in bytes, AXI word aligned)
// ------------------------------------------------------------------------------ //

#define GDC_DATA2OUT_ADDR_DEFAULT (0x0)
#define GDC_DATA2OUT_ADDR_DATASIZE (32)
#define GDC_DATA2OUT_ADDR_OFFSET (0x50)
#define GDC_DATA2OUT_ADDR_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_data2out_addr_write(u64 base, u32 data)
{
    system_gdc_write_32(base+GDC_DATA2OUT_ADDR_OFFSET, data);
}
static __inline u32 gdc_data2out_addr_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DATA2OUT_ADDR_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: data2out line offset
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Address difference between adjacent lines for the 2ndt plane in the output frame buffer (in bytes, AXI word aligned)
// ------------------------------------------------------------------------------ //

#define GDC_DATA2OUT_LINE_OFFSET_DEFAULT (0x0)
#define GDC_DATA2OUT_LINE_OFFSET_DATASIZE (32)
#define GDC_DATA2OUT_LINE_OFFSET_OFFSET (0x54)
#define GDC_DATA2OUT_LINE_OFFSET_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_data2out_line_offset_write(u64 base, u32 data)
{
    system_gdc_write_32(base+GDC_DATA2OUT_LINE_OFFSET_OFFSET, data);
}
static __inline u32 gdc_data2out_line_offset_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DATA2OUT_LINE_OFFSET_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: data3out addr
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Base address of the 3rd plane in the output frame buffer  (in bytes, AXI word aligned)
// ------------------------------------------------------------------------------ //

#define GDC_DATA3OUT_ADDR_DEFAULT (0x0)
#define GDC_DATA3OUT_ADDR_DATASIZE (32)
#define GDC_DATA3OUT_ADDR_OFFSET (0x58)
#define GDC_DATA3OUT_ADDR_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_data3out_addr_write(u64 base, u32 data)
{
    system_gdc_write_32(base+GDC_DATA3OUT_ADDR_OFFSET, data);
}
static __inline u32 gdc_data3out_addr_read(u64 base)
{
    return system_gdc_read_32(base + GDC_DATA3OUT_ADDR_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: data3out line offset
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Address difference between adjacent lines for the 3rd plane in the output frame buffer (in bytes, AXI word aligned)
// ------------------------------------------------------------------------------ //

#define GDC_DATA3OUT_LINE_OFFSET_DEFAULT (0x0)
#define GDC_DATA3OUT_LINE_OFFSET_DATASIZE (32)
#define GDC_DATA3OUT_LINE_OFFSET_OFFSET (0x5c)
#define GDC_DATA3OUT_LINE_OFFSET_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_data3out_line_offset_write(u64 base, u32 data)
{
    system_gdc_write_32(base+GDC_DATA3OUT_LINE_OFFSET_OFFSET, data);
}
static __inline u32 gdc_data3out_line_offset_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DATA3OUT_LINE_OFFSET_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: status
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// word with status fields:
// ------------------------------------------------------------------------------ //

#define GDC_STATUS_DEFAULT (0x0)
#define GDC_STATUS_DATASIZE (32)
#define GDC_STATUS_OFFSET (0x60)
#define GDC_STATUS_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_status_read(u64 base)
{
    return system_gdc_read_32(base+GDC_STATUS_OFFSET);
}

// args: data (32-bit)
static __inline void gdc_status_write(u64 base, u32 data)
{
    system_gdc_write_32(base+GDC_STATUS_OFFSET, data);
}

// ------------------------------------------------------------------------------ //
// Register: busy
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Busy 1 = processing in progress, 0 = ready for next image
// ------------------------------------------------------------------------------ //

#define GDC_BUSY_DEFAULT (0x0)
#define GDC_BUSY_DATASIZE (1)
#define GDC_BUSY_OFFSET (0x60)
#define GDC_BUSY_MASK (0x1)

// args: data (1-bit)
static __inline void gdc_busy_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_BUSY_OFFSET);
    system_gdc_write_32(base+GDC_BUSY_OFFSET, (((u32) (data & GDC_BUSY_MASK)) << 0) | (curr & (~GDC_BUSY_MASK)));
}
static __inline u8 gdc_busy_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_BUSY_OFFSET) & GDC_BUSY_MASK) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: error
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Error flag: last operation was finished with error (see bits 15:8)
// ------------------------------------------------------------------------------ //

#define GDC_ERROR_DEFAULT (0x0)
#define GDC_ERROR_DATASIZE (1)
#define GDC_ERROR_OFFSET (0x60)
#define GDC_ERROR_MASK (0x2)

// args: data (1-bit)
static __inline void gdc_error_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_ERROR_OFFSET);
    system_gdc_write_32(base+GDC_ERROR_OFFSET, (((u32) (data & 0x1)) << 1) | (curr & GDC_ERROR_MASK));
}
static __inline u8 gdc_error_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_ERROR_OFFSET) & GDC_ERROR_MASK) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: Reserved for future use 1
// ------------------------------------------------------------------------------ //

#define GDC_RESERVED_FOR_FUTURE_USE_1_DEFAULT (0x0)
#define GDC_RESERVED_FOR_FUTURE_USE_1_DATASIZE (6)
#define GDC_RESERVED_FOR_FUTURE_USE_1_OFFSET (0x60)
#define GDC_RESERVED_FOR_FUTURE_USE_1_MASK (0xfc)

// args: data (6-bit)
static __inline void gdc_reserved_for_future_use_1_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_RESERVED_FOR_FUTURE_USE_1_OFFSET);
    system_gdc_write_32(base+GDC_RESERVED_FOR_FUTURE_USE_1_OFFSET, (((u32) (data & 0x3f)) << 2) | (curr & (~GDC_RESERVED_FOR_FUTURE_USE_1_MASK)));
}
static __inline u8 gdc_reserved_for_future_use_1_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_RESERVED_FOR_FUTURE_USE_1_OFFSET) & GDC_RESERVED_FOR_FUTURE_USE_1_MASK) >> 2);
}
// ------------------------------------------------------------------------------ //
// Register: configuration error
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Configuration error (wrong configuration stream)
// ------------------------------------------------------------------------------ //

#define GDC_CONFIGURATION_ERROR_DEFAULT (0x0)
#define GDC_CONFIGURATION_ERROR_DATASIZE (1)
#define GDC_CONFIGURATION_ERROR_OFFSET (0x60)
#define GDC_CONFIGURATION_ERROR_MASK (0x100)

// args: data (1-bit)
static __inline void gdc_configuration_error_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_CONFIGURATION_ERROR_OFFSET);
    system_gdc_write_32(base+GDC_CONFIGURATION_ERROR_OFFSET, (((u32) (data & 0x1)) << 8) | (curr & (~GDC_CONFIGURATION_ERROR_MASK)));
}
static __inline u8 gdc_configuration_error_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_CONFIGURATION_ERROR_OFFSET) & GDC_CONFIGURATION_ERROR_MASK) >> 8);
}
// ------------------------------------------------------------------------------ //
// Register: user abort
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// User abort (stop/reset command)
// ------------------------------------------------------------------------------ //

#define GDC_USER_ABORT_DEFAULT (0x0)
#define GDC_USER_ABORT_DATASIZE (1)
#define GDC_USER_ABORT_OFFSET (0x60)
#define GDC_USER_ABORT_MASK (0x200)

// args: data (1-bit)
static __inline void gdc_user_abort_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_USER_ABORT_OFFSET);
    system_gdc_write_32(base+GDC_USER_ABORT_OFFSET, (((u32) (data & 0x1)) << 9) | (curr & (~GDC_USER_ABORT_MASK)));
}
static __inline u8 gdc_user_abort_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_USER_ABORT_OFFSET) & GDC_USER_ABORT_MASK) >> 9);
}
// ------------------------------------------------------------------------------ //
// Register: AXI reader error
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// AXI reader error (e.g. error code returned by fabric)
// ------------------------------------------------------------------------------ //

#define GDC_AXI_READER_ERROR_DEFAULT (0x0)
#define GDC_AXI_READER_ERROR_DATASIZE (1)
#define GDC_AXI_READER_ERROR_OFFSET (0x60)
#define GDC_AXI_READER_ERROR_MASK (0x400)

// args: data (1-bit)
static __inline void gdc_axi_reader_error_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_AXI_READER_ERROR_OFFSET);
    system_gdc_write_32(base+GDC_AXI_READER_ERROR_OFFSET, (((u32) (data & 0x1)) << 10) | (curr & (~GDC_AXI_READER_ERROR_MASK)));
}
static __inline u8 gdc_axi_reader_error_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_AXI_READER_ERROR_OFFSET) & GDC_AXI_READER_ERROR_MASK) >> 10);
}
// ------------------------------------------------------------------------------ //
// Register: AXI writer error
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// AXI writer error
// ------------------------------------------------------------------------------ //

#define GDC_AXI_WRITER_ERROR_DEFAULT (0x0)
#define GDC_AXI_WRITER_ERROR_DATASIZE (1)
#define GDC_AXI_WRITER_ERROR_OFFSET (0x60)
#define GDC_AXI_WRITER_ERROR_MASK (0x800)

// args: data (1-bit)
static __inline void gdc_axi_writer_error_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_AXI_WRITER_ERROR_OFFSET);
    system_gdc_write_32(base+GDC_AXI_WRITER_ERROR_OFFSET, (((u32) (data & 0x1)) << 11) | (curr & (~GDC_AXI_WRITER_ERROR_MASK)));
}
static __inline u8 gdc_axi_writer_error_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_AXI_WRITER_ERROR_OFFSET) & GDC_AXI_WRITER_ERROR_MASK) >> 11);
}
// ------------------------------------------------------------------------------ //
// Register: Unaligned access
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Unaligned access (address pointer is not aligned)
// ------------------------------------------------------------------------------ //

#define GDC_UNALIGNED_ACCESS_DEFAULT (0x0)
#define GDC_UNALIGNED_ACCESS_DATASIZE (1)
#define GDC_UNALIGNED_ACCESS_OFFSET (0x60)
#define GDC_UNALIGNED_ACCESS_MASK (0x1000)

// args: data (1-bit)
static __inline void gdc_unaligned_access_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_UNALIGNED_ACCESS_OFFSET);
    system_gdc_write_32(base+GDC_UNALIGNED_ACCESS_OFFSET, (((u32) (data & 0x1)) << 12) | (curr & (~GDC_UNALIGNED_ACCESS_MASK)));
}
static __inline u8 gdc_unaligned_access_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_UNALIGNED_ACCESS_OFFSET) & GDC_UNALIGNED_ACCESS_MASK) >> 12);
}
// ------------------------------------------------------------------------------ //
// Register: Incompatible configuration
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Incompatible configuration (request of unimplemented mode of operation, e.g. unsupported image format, unsupported module mode in the configuration stream)
// ------------------------------------------------------------------------------ //

#define GDC_INCOMPATIBLE_CONFIGURATION_DEFAULT (0x0)
#define GDC_INCOMPATIBLE_CONFIGURATION_DATASIZE (1)
#define GDC_INCOMPATIBLE_CONFIGURATION_OFFSET (0x60)
#define GDC_INCOMPATIBLE_CONFIGURATION_MASK (0x2000)

// args: data (1-bit)
static __inline void gdc_incompatible_configuration_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_INCOMPATIBLE_CONFIGURATION_OFFSET);
    system_gdc_write_32(base+GDC_INCOMPATIBLE_CONFIGURATION_OFFSET, (((u32) (data & 0x1)) << 13) | (curr & (~GDC_INCOMPATIBLE_CONFIGURATION_MASK)));
}
static __inline u8 gdc_incompatible_configuration_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_INCOMPATIBLE_CONFIGURATION_OFFSET) & GDC_INCOMPATIBLE_CONFIGURATION_MASK) >> 13);
}
// ------------------------------------------------------------------------------ //
// Register: Reserved for future use 2
// ------------------------------------------------------------------------------ //

#define GDC_RESERVED_FOR_FUTURE_USE_2_DEFAULT (0x0)
#define GDC_RESERVED_FOR_FUTURE_USE_2_DATASIZE (18)
#define GDC_RESERVED_FOR_FUTURE_USE_2_OFFSET (0x60)
#define GDC_RESERVED_FOR_FUTURE_USE_2_MASK (0xffffc000)
#define GDC_RESERVED_FOR_FUTURE_USE_2_DATA_MASK (0x3ffff)
#define GDC_RESERVED_FOR_FUTURE_USE_2_DATA_SHIFT (14)

// args: data (18-bit)
static __inline void gdc_reserved_for_future_use_2_write(u64 base, u32 data)
{
    u32 curr = system_gdc_read_32(base+GDC_RESERVED_FOR_FUTURE_USE_2_OFFSET);
    system_gdc_write_32(base+GDC_RESERVED_FOR_FUTURE_USE_2_OFFSET,
                        (((u32) (data & GDC_RESERVED_FOR_FUTURE_USE_2_DATA_MASK)) << GDC_RESERVED_FOR_FUTURE_USE_2_DATA_SHIFT) | (curr & (~GDC_RESERVED_FOR_FUTURE_USE_2_MASK)));
}
static __inline u32 gdc_reserved_for_future_use_2_read(u64 base)
{
    return (u32)((system_gdc_read_32(base+GDC_RESERVED_FOR_FUTURE_USE_2_OFFSET) & GDC_RESERVED_FOR_FUTURE_USE_2_MASK) >> GDC_RESERVED_FOR_FUTURE_USE_2_DATA_SHIFT);
}
// ------------------------------------------------------------------------------ //
// Register: config
// ------------------------------------------------------------------------------ //

#define GDC_CONFIG_DEFAULT (0x0)
#define GDC_CONFIG_DATASIZE (32)
#define GDC_CONFIG_OFFSET (0x64)
#define GDC_CONFIG_MASK (0xffffffff)

// args: data (32-bit)
static __inline void gdc_config_write(u64 base, u32 data)
{
    system_gdc_write_32(base+GDC_CONFIG_OFFSET, data);
}
static __inline u32 gdc_config_read(u64 base)
{
    return system_gdc_read_32(base+GDC_CONFIG_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: start flag
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Start flag: transition from 0 to 1 latches the data on the configuration ports and starts the processing
// ------------------------------------------------------------------------------ //

#define GDC_START_FLAG_DEFAULT (0x0)
#define GDC_START_FLAG_DATASIZE (1)
#define GDC_START_FLAG_OFFSET (0x64)
#define GDC_START_FLAG_MASK (0x1)

// args: data (1-bit)
static __inline void gdc_start_flag_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_START_FLAG_OFFSET);
    system_gdc_write_32(base+GDC_START_FLAG_OFFSET, (((u32) (data & GDC_START_FLAG_MASK)) << 0) | (curr & (~GDC_START_FLAG_MASK)));
}

static __inline u8 gdc_start_flag_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_START_FLAG_OFFSET) & GDC_START_FLAG_MASK) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: stop flag
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Stop/reset flag: 0 - normal operation, 1 means to initiate internal cleanup procedure to abandon the current frame and prepare for processing of the next frame. The busy flag in status word should be cleared at the end of this process
// ------------------------------------------------------------------------------ //

#define GDC_STOP_FLAG_DEFAULT (0x0)
#define GDC_STOP_FLAG_DATASIZE (1)
#define GDC_STOP_FLAG_OFFSET (0x64)
#define GDC_STOP_FLAG_MASK (0x2)

// args: data (1-bit)
static __inline void gdc_stop_flag_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_STOP_FLAG_OFFSET);
    system_gdc_write_32(base+GDC_STOP_FLAG_OFFSET, (((u32) (data & 0x1)) << 1) | (curr & (~GDC_STOP_FLAG_MASK)));
}
static __inline u8 gdc_stop_flag_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_STOP_FLAG_OFFSET) & GDC_STOP_FLAG_MASK) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: Reserved for future use 3
// ------------------------------------------------------------------------------ //

#define GDC_RESERVED_FOR_FUTURE_USE_3_DEFAULT (0x0)
#define GDC_RESERVED_FOR_FUTURE_USE_3_DATASIZE (30)
#define GDC_RESERVED_FOR_FUTURE_USE_3_OFFSET (0x64)
#define GDC_RESERVED_FOR_FUTURE_USE_3_MASK (0xfffffffc)
#define GDC_RESERVED_FOR_FUTURE_USE_3_DATA_MASK (0x3fffffff)
#define GDC_RESERVED_FOR_FUTURE_USE_3_DATA_SHIFT (2)

// args: data (30-bit)
static __inline void gdc_reserved_for_future_use_3_write(u64 base, u32 data)
{
    u32 curr = system_gdc_read_32(base+GDC_RESERVED_FOR_FUTURE_USE_3_OFFSET);
    system_gdc_write_32(base+GDC_RESERVED_FOR_FUTURE_USE_3_OFFSET,
                        (((u32) (data & GDC_RESERVED_FOR_FUTURE_USE_3_DATA_MASK)) <<
                         GDC_RESERVED_FOR_FUTURE_USE_3_DATA_SHIFT) |
                        (curr & (~GDC_RESERVED_FOR_FUTURE_USE_3_MASK)));
}
static __inline u32 gdc_reserved_for_future_use_3_read(u64 base)
{
    return (u32)((system_gdc_read_32(0x208064L) & GDC_RESERVED_FOR_FUTURE_USE_3_MASK) >> GDC_RESERVED_FOR_FUTURE_USE_3_DATA_SHIFT);
}
// ------------------------------------------------------------------------------ //
// Register: Capability mask
// ------------------------------------------------------------------------------ //

#define GDC_CAPABILITY_MASK_DEFAULT (0x0)
#define GDC_CAPABILITY_MASK_DATASIZE (32)
#define GDC_CAPABILITY_MASK_OFFSET (0x68)
#define GDC_CAPABILITY_MASK_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_capability_mask_read(u64 base)
{
    return system_gdc_read_32(base+GDC_CAPABILITY_MASK_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: Eight bit data suppoirted
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 8 bit data supported
// ------------------------------------------------------------------------------ //

#define GDC_EIGHT_BIT_DATA_SUPPOIRTED_DEFAULT (0x0)
#define GDC_EIGHT_BIT_DATA_SUPPOIRTED_DATASIZE (1)
#define GDC_EIGHT_BIT_DATA_SUPPOIRTED_OFFSET (0x68)
#define GDC_EIGHT_BIT_DATA_SUPPOIRTED_MASK (0x1)

// args: data (1-bit)
static __inline void gdc_eight_bit_data_suppoirted_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_EIGHT_BIT_DATA_SUPPOIRTED_OFFSET);
    system_gdc_write_32(base+GDC_EIGHT_BIT_DATA_SUPPOIRTED_OFFSET, (((u32) (data & 0x1)) << 0) | (curr & (~GDC_EIGHT_BIT_DATA_SUPPOIRTED_MASK)));
}
static __inline u8 gdc_eight_bit_data_suppoirted_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_EIGHT_BIT_DATA_SUPPOIRTED_OFFSET) & GDC_EIGHT_BIT_DATA_SUPPOIRTED_MASK) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: Ten bit data supported
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 10 bit data supported
// ------------------------------------------------------------------------------ //

#define GDC_TEN_BIT_DATA_SUPPORTED_DEFAULT (0x0)
#define GDC_TEN_BIT_DATA_SUPPORTED_DATASIZE (1)
#define GDC_TEN_BIT_DATA_SUPPORTED_OFFSET (0x68)
#define GDC_TEN_BIT_DATA_SUPPORTED_MASK (0x2)

// args: data (1-bit)
static __inline void gdc_ten_bit_data_supported_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_TEN_BIT_DATA_SUPPORTED_OFFSET);
    system_gdc_write_32(base+GDC_TEN_BIT_DATA_SUPPORTED_OFFSET, (((u32) (data & 0x1)) << 1) | (curr & (~GDC_TEN_BIT_DATA_SUPPORTED_MASK)));
}
static __inline u8 gdc_ten_bit_data_supported_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_TEN_BIT_DATA_SUPPORTED_OFFSET) & GDC_TEN_BIT_DATA_SUPPORTED_MASK) >> 1);
}
// ------------------------------------------------------------------------------ //
// Register: Grayscale supported
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// grayscale supported
// ------------------------------------------------------------------------------ //

#define GDC_GRAYSCALE_SUPPORTED_DEFAULT (0x0)
#define GDC_GRAYSCALE_SUPPORTED_DATASIZE (1)
#define GDC_GRAYSCALE_SUPPORTED_OFFSET (0x68)
#define GDC_GRAYSCALE_SUPPORTED_MASK (0x4)

// args: data (1-bit)
static __inline void gdc_grayscale_supported_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_GRAYSCALE_SUPPORTED_OFFSET);
    system_gdc_write_32(base+GDC_GRAYSCALE_SUPPORTED_OFFSET, (((u32) (data & 0x1)) << 2) | (curr & (~GDC_GRAYSCALE_SUPPORTED_MASK)));
}
static __inline u8 gdc_grayscale_supported_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_GRAYSCALE_SUPPORTED_OFFSET) & GDC_GRAYSCALE_SUPPORTED_MASK) >> 2);
}
// ------------------------------------------------------------------------------ //
// Register: RGBA888 supported
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// RGBA8:8:8/YUV4:4:4 mode supported
// ------------------------------------------------------------------------------ //

#define GDC_RGBA888_SUPPORTED_DEFAULT (0x0)
#define GDC_RGBA888_SUPPORTED_DATASIZE (1)
#define GDC_RGBA888_SUPPORTED_OFFSET (0x68)
#define GDC_RGBA888_SUPPORTED_MASK (0x8)

// args: data (1-bit)
static __inline void gdc_rgba888_supported_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_RGBA888_SUPPORTED_OFFSET);
    system_gdc_write_32(base+GDC_RGBA888_SUPPORTED_OFFSET, (((u32) (data & 0x1)) << 3) | (curr & (~GDC_RGBA888_SUPPORTED_MASK)));
}
static __inline u8 gdc_rgba888_supported_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_RGBA888_SUPPORTED_OFFSET) & GDC_RGBA888_SUPPORTED_MASK) >> 3);
}
// ------------------------------------------------------------------------------ //
// Register: RGB YUV444 planar supported
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// RGB/YUV444 planar modes supported
// ------------------------------------------------------------------------------ //

#define GDC_RGB_YUV444_PLANAR_SUPPORTED_DEFAULT (0x0)
#define GDC_RGB_YUV444_PLANAR_SUPPORTED_DATASIZE (1)
#define GDC_RGB_YUV444_PLANAR_SUPPORTED_OFFSET (0x68)
#define GDC_RGB_YUV444_PLANAR_SUPPORTED_MASK (0x10)

// args: data (1-bit)
static __inline void gdc_rgb_yuv444_planar_supported_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_RGB_YUV444_PLANAR_SUPPORTED_OFFSET);
    system_gdc_write_32(base+GDC_RGB_YUV444_PLANAR_SUPPORTED_OFFSET, (((u32) (data & 0x1)) << 4) | (curr & (~GDC_RGB_YUV444_PLANAR_SUPPORTED_MASK)));
}
static __inline u8 gdc_rgb_yuv444_planar_supported_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_RGB_YUV444_PLANAR_SUPPORTED_OFFSET) & GDC_RGB_YUV444_PLANAR_SUPPORTED_MASK) >> 4);
}
// ------------------------------------------------------------------------------ //
// Register: YUV semiplanar supported
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// YUV semiplanar modes supported
// ------------------------------------------------------------------------------ //

#define GDC_YUV_SEMIPLANAR_SUPPORTED_DEFAULT (0x0)
#define GDC_YUV_SEMIPLANAR_SUPPORTED_DATASIZE (1)
#define GDC_YUV_SEMIPLANAR_SUPPORTED_OFFSET (0x68)
#define GDC_YUV_SEMIPLANAR_SUPPORTED_MASK (0x20)

// args: data (1-bit)
static __inline void gdc_yuv_semiplanar_supported_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_YUV_SEMIPLANAR_SUPPORTED_OFFSET);
    system_gdc_write_32(base+GDC_YUV_SEMIPLANAR_SUPPORTED_OFFSET, (((u32) (data & 0x1)) << 5) | (curr & (~GDC_YUV_SEMIPLANAR_SUPPORTED_MASK)));
}
static __inline u8 gdc_yuv_semiplanar_supported_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_YUV_SEMIPLANAR_SUPPORTED_OFFSET) & GDC_YUV_SEMIPLANAR_SUPPORTED_MASK) >> 5);
}
// ------------------------------------------------------------------------------ //
// Register: YUV422 linear mode supported
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// YUV4:2:2 linear mode supported (16 bit/pixel)
// ------------------------------------------------------------------------------ //

#define GDC_YUV422_LINEAR_MODE_SUPPORTED_DEFAULT (0x0)
#define GDC_YUV422_LINEAR_MODE_SUPPORTED_DATASIZE (1)
#define GDC_YUV422_LINEAR_MODE_SUPPORTED_OFFSET (0x68)
#define GDC_YUV422_LINEAR_MODE_SUPPORTED_MASK (0x40)

// args: data (1-bit)
static __inline void gdc_yuv422_linear_mode_supported_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_YUV422_LINEAR_MODE_SUPPORTED_OFFSET);
    system_gdc_write_32(base+GDC_YUV422_LINEAR_MODE_SUPPORTED_OFFSET, (((u32) (data & 0x1)) << 6) | (curr & (~GDC_YUV422_LINEAR_MODE_SUPPORTED_MASK)));
}
static __inline u8 gdc_yuv422_linear_mode_supported_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_YUV422_LINEAR_MODE_SUPPORTED_OFFSET) & GDC_YUV422_LINEAR_MODE_SUPPORTED_MASK) >> 6);
}
// ------------------------------------------------------------------------------ //
// Register: RGB10_10_10 supported
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// RGB10:10:10 mode supported
// ------------------------------------------------------------------------------ //

#define GDC_RGB10_10_10_SUPPORTED_DEFAULT (0x0)
#define GDC_RGB10_10_10_SUPPORTED_DATASIZE (1)
#define GDC_RGB10_10_10_SUPPORTED_OFFSET (0x68)
#define GDC_RGB10_10_10_SUPPORTED_MASK (0x80)

// args: data (1-bit)
static __inline void gdc_rgb10_10_10_supported_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_RGB10_10_10_SUPPORTED_OFFSET);
    system_gdc_write_32(base+GDC_RGB10_10_10_SUPPORTED_OFFSET, (((u32) (data & 0x1)) << 7) | (curr & (~GDC_RGB10_10_10_SUPPORTED_MASK)));
}
static __inline u8 gdc_rgb10_10_10_supported_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_RGB10_10_10_SUPPORTED_OFFSET) & GDC_RGB10_10_10_SUPPORTED_MASK) >> 7);
}
// ------------------------------------------------------------------------------ //
// Register: Bicubic interpolation supported
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// 4 tap bicubic interpolation supported
// ------------------------------------------------------------------------------ //

#define GDC_BICUBIC_INTERPOLATION_SUPPORTED_DEFAULT (0x0)
#define GDC_BICUBIC_INTERPOLATION_SUPPORTED_DATASIZE (1)
#define GDC_BICUBIC_INTERPOLATION_SUPPORTED_OFFSET (0x68)
#define GDC_BICUBIC_INTERPOLATION_SUPPORTED_MASK (0x100)

// args: data (1-bit)
static __inline void gdc_bicubic_interpolation_supported_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_BICUBIC_INTERPOLATION_SUPPORTED_OFFSET);
    system_gdc_write_32(base+GDC_BICUBIC_INTERPOLATION_SUPPORTED_OFFSET, (((u32) (data & 0x1)) << 8) | (curr & (~GDC_BICUBIC_INTERPOLATION_SUPPORTED_MASK)));
}
static __inline u8 gdc_bicubic_interpolation_supported_read(u64 base)
{
    return (u8)((system_gdc_read_32(0x208068L) & GDC_BICUBIC_INTERPOLATION_SUPPORTED_MASK) >> 8);
}
// ------------------------------------------------------------------------------ //
// Register: Bilinear interpolation mode 1 supported
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// bilinear interpolation mode 1 supported {for U,V components}
// ------------------------------------------------------------------------------ //

#define GDC_BILINEAR_INTERPOLATION_MODE_1_SUPPORTED_DEFAULT (0x0)
#define GDC_BILINEAR_INTERPOLATION_MODE_1_SUPPORTED_DATASIZE (1)
#define GDC_BILINEAR_INTERPOLATION_MODE_1_SUPPORTED_OFFSET (0x68)
#define GDC_BILINEAR_INTERPOLATION_MODE_1_SUPPORTED_MASK (0x200)

// args: data (1-bit)
static __inline void gdc_bilinear_interpolation_mode_1_supported_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_BILINEAR_INTERPOLATION_MODE_1_SUPPORTED_OFFSET);
    system_gdc_write_32(base+GDC_BILINEAR_INTERPOLATION_MODE_1_SUPPORTED_OFFSET, (((u32) (data & 0x1)) << 9) | (curr & (~GDC_BILINEAR_INTERPOLATION_MODE_1_SUPPORTED_MASK)));
}
static __inline u8 gdc_bilinear_interpolation_mode_1_supported_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_BILINEAR_INTERPOLATION_MODE_1_SUPPORTED_OFFSET) & GDC_BILINEAR_INTERPOLATION_MODE_1_SUPPORTED_MASK) >> 9);
}
// ------------------------------------------------------------------------------ //
// Register: Bilinear interpolation mode 2 supported
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// bilinear interpolation mode 2 supported {for U,V components}
// ------------------------------------------------------------------------------ //

#define GDC_BILINEAR_INTERPOLATION_MODE_2_SUPPORTED_DEFAULT (0x0)
#define GDC_BILINEAR_INTERPOLATION_MODE_2_SUPPORTED_DATASIZE (1)
#define GDC_BILINEAR_INTERPOLATION_MODE_2_SUPPORTED_OFFSET (0x68)
#define GDC_BILINEAR_INTERPOLATION_MODE_2_SUPPORTED_MASK (0x400)

// args: data (1-bit)
static __inline void gdc_bilinear_interpolation_mode_2_supported_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_BILINEAR_INTERPOLATION_MODE_2_SUPPORTED_OFFSET);
    system_gdc_write_32(base+GDC_BILINEAR_INTERPOLATION_MODE_2_SUPPORTED_OFFSET, (((u32) (data & 0x1)) << 10) | (curr & (~GDC_BILINEAR_INTERPOLATION_MODE_2_SUPPORTED_MASK)));
}
static __inline u8 gdc_bilinear_interpolation_mode_2_supported_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_BILINEAR_INTERPOLATION_MODE_2_SUPPORTED_OFFSET) & GDC_BILINEAR_INTERPOLATION_MODE_2_SUPPORTED_MASK) >> 10);
}
// ------------------------------------------------------------------------------ //
// Register: Output of interpolation coordinates supported
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// output of interpolation coordinates is supported
// ------------------------------------------------------------------------------ //

#define GDC_OUTPUT_OF_INTERPOLATION_COORDINATES_SUPPORTED_DEFAULT (0x0)
#define GDC_OUTPUT_OF_INTERPOLATION_COORDINATES_SUPPORTED_DATASIZE (1)
#define GDC_OUTPUT_OF_INTERPOLATION_COORDINATES_SUPPORTED_OFFSET (0x68)
#define GDC_OUTPUT_OF_INTERPOLATION_COORDINATES_SUPPORTED_MASK (0x800)

// args: data (1-bit)
static __inline void gdc_output_of_interpolation_coordinates_supported_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_OUTPUT_OF_INTERPOLATION_COORDINATES_SUPPORTED_OFFSET);
    system_gdc_write_32(base+GDC_OUTPUT_OF_INTERPOLATION_COORDINATES_SUPPORTED_OFFSET, (((u32) (data & 0x1)) << 11) | (curr & (~GDC_OUTPUT_OF_INTERPOLATION_COORDINATES_SUPPORTED_MASK)));
}
static __inline u8 gdc_output_of_interpolation_coordinates_supported_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_OUTPUT_OF_INTERPOLATION_COORDINATES_SUPPORTED_OFFSET) & GDC_OUTPUT_OF_INTERPOLATION_COORDINATES_SUPPORTED_MASK) >> 11);
}
// ------------------------------------------------------------------------------ //
// Register: Reserved for future use 4
// ------------------------------------------------------------------------------ //

#define GDC_RESERVED_FOR_FUTURE_USE_4_DEFAULT (0x0)
#define GDC_RESERVED_FOR_FUTURE_USE_4_DATASIZE (4)
#define GDC_RESERVED_FOR_FUTURE_USE_4_OFFSET (0x68)
#define GDC_RESERVED_FOR_FUTURE_USE_4_MASK (0xf000)

// args: data (4-bit)
static __inline void gdc_reserved_for_future_use_4_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_RESERVED_FOR_FUTURE_USE_4_OFFSET);
    system_gdc_write_32(base+GDC_RESERVED_FOR_FUTURE_USE_4_OFFSET, (((u32) (data & 0xf)) << 12) | (curr & (~GDC_RESERVED_FOR_FUTURE_USE_4_MASK)));
}
static __inline u8 gdc_reserved_for_future_use_4_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_RESERVED_FOR_FUTURE_USE_4_OFFSET) & GDC_RESERVED_FOR_FUTURE_USE_4_MASK) >> 12);
}
// ------------------------------------------------------------------------------ //
// Register: Size of output cache
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// log2(size of output cache in lines)-5 (0 - 32lines, 1 - 64 lines etc)
// ------------------------------------------------------------------------------ //

#define GDC_SIZE_OF_OUTPUT_CACHE_DEFAULT (0x0)
#define GDC_SIZE_OF_OUTPUT_CACHE_DATASIZE (3)
#define GDC_SIZE_OF_OUTPUT_CACHE_OFFSET (0x68)
#define GDC_SIZE_OF_OUTPUT_CACHE_MASK (0x70000)

// args: data (3-bit)
static __inline void gdc_size_of_output_cache_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_SIZE_OF_OUTPUT_CACHE_OFFSET);
    system_gdc_write_32(base+GDC_SIZE_OF_OUTPUT_CACHE_OFFSET, (((u32) (data & 0x7)) << 16) | (curr & (~GDC_SIZE_OF_OUTPUT_CACHE_MASK)));
}
static __inline u8 gdc_size_of_output_cache_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_SIZE_OF_OUTPUT_CACHE_OFFSET) & GDC_SIZE_OF_OUTPUT_CACHE_MASK) >> 16);
}
// ------------------------------------------------------------------------------ //
// Register: Size of tile cache
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// log2(size of tile cache in 16x16 clusters)
// ------------------------------------------------------------------------------ //

#define GDC_SIZE_OF_TILE_CACHE_DEFAULT (0x0)
#define GDC_SIZE_OF_TILE_CACHE_DATASIZE (5)
#define GDC_SIZE_OF_TILE_CACHE_OFFSET (0x68)
#define GDC_SIZE_OF_TILE_CACHE_MASK (0xf80000)

// args: data (5-bit)
static __inline void gdc_size_of_tile_cache_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_SIZE_OF_TILE_CACHE_OFFSET);
    system_gdc_write_32(base+GDC_SIZE_OF_TILE_CACHE_OFFSET, (((u32) (data & 0x1f)) << 19) | (curr & (~GDC_SIZE_OF_TILE_CACHE_MASK)));
}
static __inline u8 gdc_size_of_tile_cache_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_SIZE_OF_TILE_CACHE_OFFSET) & GDC_SIZE_OF_TILE_CACHE_MASK) >> 19);
}
// ------------------------------------------------------------------------------ //
// Register: Nuimber of polyphase filter banks
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// log2(number of polyphase filter banks)
// ------------------------------------------------------------------------------ //

#define GDC_NUIMBER_OF_POLYPHASE_FILTER_BANKS_DEFAULT (0x0)
#define GDC_NUIMBER_OF_POLYPHASE_FILTER_BANKS_DATASIZE (3)
#define GDC_NUIMBER_OF_POLYPHASE_FILTER_BANKS_OFFSET (0x68)
#define GDC_NUIMBER_OF_POLYPHASE_FILTER_BANKS_MASK (0x7000000)

// args: data (3-bit)
static __inline void gdc_nuimber_of_polyphase_filter_banks_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_NUIMBER_OF_POLYPHASE_FILTER_BANKS_OFFSET);
    system_gdc_write_32(base+GDC_NUIMBER_OF_POLYPHASE_FILTER_BANKS_OFFSET, (((u32) (data & 0x7)) << 24) | (curr & (GDC_NUIMBER_OF_POLYPHASE_FILTER_BANKS_MASK)));
}
static __inline u8 gdc_nuimber_of_polyphase_filter_banks_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_NUIMBER_OF_POLYPHASE_FILTER_BANKS_OFFSET) & GDC_NUIMBER_OF_POLYPHASE_FILTER_BANKS_MASK) >> 24);
}
// ------------------------------------------------------------------------------ //
// Register: AXI data width
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// log2(AXI_DATA_WIDTH)-5
// ------------------------------------------------------------------------------ //

#define GDC_AXI_DATA_WIDTH_DEFAULT (0x0)
#define GDC_AXI_DATA_WIDTH_DATASIZE (3)
#define GDC_AXI_DATA_WIDTH_OFFSET (0x68)
#define GDC_AXI_DATA_WIDTH_MASK (0x38000000)

// args: data (3-bit)
static __inline void gdc_axi_data_width_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_AXI_DATA_WIDTH_OFFSET);
    system_gdc_write_32(base+GDC_AXI_DATA_WIDTH_OFFSET, (((u32) (data & 0x7)) << 27) | (curr & (~GDC_AXI_DATA_WIDTH_MASK)));
}
static __inline u8 gdc_axi_data_width_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_AXI_DATA_WIDTH_OFFSET) & GDC_AXI_DATA_WIDTH_MASK) >> 27);
}
// ------------------------------------------------------------------------------ //
// Register: Reserved for future use 5
// ------------------------------------------------------------------------------ //

#define GDC_RESERVED_FOR_FUTURE_USE_5_DEFAULT (0x0)
#define GDC_RESERVED_FOR_FUTURE_USE_5_DATASIZE (2)
#define GDC_RESERVED_FOR_FUTURE_USE_5_OFFSET (0x68)
#define GDC_RESERVED_FOR_FUTURE_USE_5_MASK (0xc0000000)

// args: data (2-bit)
static __inline void gdc_reserved_for_future_use_5_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+GDC_RESERVED_FOR_FUTURE_USE_5_OFFSET);
    system_gdc_write_32(base+GDC_RESERVED_FOR_FUTURE_USE_5_OFFSET, (((u32) (data & 0x3)) << 30) | (curr & (~GDC_RESERVED_FOR_FUTURE_USE_5_MASK)));
}
static __inline u8 gdc_reserved_for_future_use_5_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+GDC_RESERVED_FOR_FUTURE_USE_5_OFFSET) & GDC_RESERVED_FOR_FUTURE_USE_5_MASK) >> 30);
}
// ------------------------------------------------------------------------------ //
// Register: default ch1
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Default value for 1st data channel (Y/R color) to fill missing pixels (when coordinated are out of bound). LSB aligned
// ------------------------------------------------------------------------------ //

#define GDC_DEFAULT_CH1_DEFAULT (0x0)
#define GDC_DEFAULT_CH1_DATASIZE (12)
#define GDC_DEFAULT_CH1_OFFSET (0x70)
#define GDC_DEFAULT_CH1_MASK (0xfff)

// args: data (12-bit)
static __inline void gdc_default_ch1_write(u64 base, u16 data)
{
    u32 curr = system_gdc_read_32(base+GDC_DEFAULT_CH1_OFFSET);
    system_gdc_write_32(base+GDC_DEFAULT_CH1_OFFSET, (((u32) (data & GDC_DEFAULT_CH1_MASK)) << 0) | (curr & (~GDC_DEFAULT_CH1_MASK)));
}
static __inline u16 gdc_default_ch1_read(u64 base)
{
    return (u16)((system_gdc_read_32(base+GDC_DEFAULT_CH1_OFFSET) & GDC_DEFAULT_CH1_MASK) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: default ch2
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Default value for 2nd data channel (U/G color) to fill missing pixels (when coordinated are out of bound) LSB aligned
// ------------------------------------------------------------------------------ //

#define GDC_DEFAULT_CH2_DEFAULT (0x0)
#define GDC_DEFAULT_CH2_DATASIZE (12)
#define GDC_DEFAULT_CH2_OFFSET (0x74)
#define GDC_DEFAULT_CH2_MASK (0xfff)

// args: data (12-bit)
static __inline void gdc_default_ch2_write(u64 base, u16 data)
{
    u32 curr = system_gdc_read_32(base+GDC_DEFAULT_CH2_OFFSET);
    system_gdc_write_32(base+GDC_DEFAULT_CH2_OFFSET, (((u32) (data & GDC_DEFAULT_CH2_MASK)) << 0) | (curr & (~GDC_DEFAULT_CH2_MASK)));
}
static __inline u16 gdc_default_ch2_read(u64 base)
{
    return (u16)((system_gdc_read_32(base+GDC_DEFAULT_CH2_OFFSET) & GDC_DEFAULT_CH2_MASK) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: default ch3
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Default value for 3rd data channel (V/B color) to fill missing pixels (when coordinated are out of bound) LSB aligned
// ------------------------------------------------------------------------------ //

#define GDC_DEFAULT_CH3_DEFAULT (0x0)
#define GDC_DEFAULT_CH3_DATASIZE (12)
#define GDC_DEFAULT_CH3_OFFSET (0x78)
#define GDC_DEFAULT_CH3_MASK (0xfff)

// args: data (12-bit)
static __inline void gdc_default_ch3_write(u64 base, u16 data)
{
    u32 curr = system_gdc_read_32(base+GDC_DEFAULT_CH3_OFFSET);
    system_gdc_write_32(base+GDC_DEFAULT_CH3_OFFSET, (((u32) (data & GDC_DEFAULT_CH3_MASK)) << 0) | (curr & (~GDC_DEFAULT_CH3_MASK)));
}
static __inline u16 gdc_default_ch3_read(u64 base)
{
    return (u16)((system_gdc_read_32(base+GDC_DEFAULT_CH3_OFFSET) & GDC_DEFAULT_CH3_MASK) >> 0);
}
// ------------------------------------------------------------------------------ //
// Group: GDC diagnostics
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Register: cfg_stall_count0
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Cycles spent on stalls on configutation FIFO to tile reader
// ------------------------------------------------------------------------------ //

#define GDC_DIAGNOSTICS_CFG_STALL_COUNT0_DEFAULT (0x0)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT0_DATASIZE (32)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT0_OFFSET (0x80)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT0_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_diagnostics_cfg_stall_count0_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DIAGNOSTICS_CFG_STALL_COUNT0_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: cfg_stall_count1
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Cycles spent on stalls on configutation FIFO to CIM
// ------------------------------------------------------------------------------ //

#define GDC_DIAGNOSTICS_CFG_STALL_COUNT1_DEFAULT (0x0)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT1_DATASIZE (32)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT1_OFFSET (0x84)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT1_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_diagnostics_cfg_stall_count1_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DIAGNOSTICS_CFG_STALL_COUNT1_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: cfg_stall_count2
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Cycles spent on stalls on configutation FIFO to PIM
// ------------------------------------------------------------------------------ //

#define GDC_DIAGNOSTICS_CFG_STALL_COUNT2_DEFAULT (0x0)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT2_DATASIZE (32)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT2_OFFSET (0x88)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT2_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_diagnostics_cfg_stall_count2_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DIAGNOSTICS_CFG_STALL_COUNT2_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: cfg_stall_count3
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Cycles spent on stalls on configutation FIFO to write cache
// ------------------------------------------------------------------------------ //

#define GDC_DIAGNOSTICS_CFG_STALL_COUNT3_DEFAULT (0x0)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT3_DATASIZE (32)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT3_OFFSET (0x8c)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT3_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_diagnostics_cfg_stall_count3_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DIAGNOSTICS_CFG_STALL_COUNT3_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: cfg_stall_count4
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Cycles spent on stalls on configutation FIFO to tile writer
// ------------------------------------------------------------------------------ //

#define GDC_DIAGNOSTICS_CFG_STALL_COUNT4_DEFAULT (0x0)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT4_DATASIZE (32)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT4_OFFSET (0x90)
#define GDC_DIAGNOSTICS_CFG_STALL_COUNT4_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_diagnostics_cfg_stall_count4_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DIAGNOSTICS_CFG_STALL_COUNT4_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: int_read_stall_count
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Cycles spent on waiting on pixel interpolator read pixel stream
// ------------------------------------------------------------------------------ //

#define GDC_DIAGNOSTICS_INT_READ_STALL_COUNT_DEFAULT (0x0)
#define GDC_DIAGNOSTICS_INT_READ_STALL_COUNT_DATASIZE (32)
#define GDC_DIAGNOSTICS_INT_READ_STALL_COUNT_OFFSET (0x94)
#define GDC_DIAGNOSTICS_INT_READ_STALL_COUNT_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_diagnostics_int_read_stall_count_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DIAGNOSTICS_INT_READ_STALL_COUNT_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: int_coord_stall_count
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Cycles spent on waiting on coordinate stream of pixel interpolator
// ------------------------------------------------------------------------------ //

#define GDC_DIAGNOSTICS_INT_COORD_STALL_COUNT_DEFAULT (0x0)
#define GDC_DIAGNOSTICS_INT_COORD_STALL_COUNT_DATASIZE (32)
#define GDC_DIAGNOSTICS_INT_COORD_STALL_COUNT_OFFSET (0x98)
#define GDC_DIAGNOSTICS_INT_COORD_STALL_COUNT_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_diagnostics_int_coord_stall_count_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DIAGNOSTICS_INT_COORD_STALL_COUNT_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: int_write_wait_count
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Cycles spent on waiting on pixel interpolator output pixel stream
// ------------------------------------------------------------------------------ //

#define GDC_DIAGNOSTICS_INT_WRITE_WAIT_COUNT_DEFAULT (0x0)
#define GDC_DIAGNOSTICS_INT_WRITE_WAIT_COUNT_DATASIZE (32)
#define GDC_DIAGNOSTICS_INT_WRITE_WAIT_COUNT_OFFSET (0x9c)
#define GDC_DIAGNOSTICS_INT_WRITE_WAIT_COUNT_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_diagnostics_int_write_wait_count_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DIAGNOSTICS_INT_WRITE_WAIT_COUNT_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: wrt_write_wait_count
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Cycles spent on waiting on sending word from write cache to tile writer
// ------------------------------------------------------------------------------ //

#define GDC_DIAGNOSTICS_WRT_WRITE_WAIT_COUNT_DEFAULT (0x0)
#define GDC_DIAGNOSTICS_WRT_WRITE_WAIT_COUNT_DATASIZE (32)
#define GDC_DIAGNOSTICS_WRT_WRITE_WAIT_COUNT_OFFSET (0xa0)
#define GDC_DIAGNOSTICS_WRT_WRITE_WAIT_COUNT_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_diagnostics_wrt_write_wait_count_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DIAGNOSTICS_WRT_WRITE_WAIT_COUNT_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Register: int_dual_count
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Number of beats on output of tile writer interface where 2 pixels were interpolated.
// ------------------------------------------------------------------------------ //

#define GDC_DIAGNOSTICS_INT_DUAL_COUNT_DEFAULT (0x0)
#define GDC_DIAGNOSTICS_INT_DUAL_COUNT_DATASIZE (32)
#define GDC_DIAGNOSTICS_INT_DUAL_COUNT_OFFSET (0xa4)
#define GDC_DIAGNOSTICS_INT_DUAL_COUNT_MASK (0xffffffff)

// args: data (32-bit)
static __inline u32 gdc_diagnostics_int_dual_count_read(u64 base)
{
    return system_gdc_read_32(base+GDC_DIAGNOSTICS_INT_DUAL_COUNT_OFFSET);
}
// ------------------------------------------------------------------------------ //
// Group: AXI Settings
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Register: config reader max arlen
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Maximum value to use for arlen (axi burst length). "0000"= max 1 transfer/burst , upto "1111"= max 16 transfers/burst
// ------------------------------------------------------------------------------ //

#define ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_MAX_ARLEN_DEFAULT (0xF)
#define ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_MAX_ARLEN_DATASIZE (4)
#define ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_MAX_ARLEN_OFFSET (0xa8)
#define ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_MAX_ARLEN_MASK (0xf)

// args: data (4-bit)
static __inline void acamera_gdc_axi_settings_config_reader_max_arlen_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_MAX_ARLEN_OFFSET);
    system_gdc_write_32(base+ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_MAX_ARLEN_OFFSET, (((u32) (data & 0xf)) << 0) | (curr & (~ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_MAX_ARLEN_OFFSET)));
}
static __inline u8 acamera_gdc_axi_settings_config_reader_max_arlen_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_MAX_ARLEN_OFFSET) & 0xf) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: config reader fifo watermark
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Number of words space in fifo before AXI read burst(s) start (legal values = max_burst_length(max_arlen+1) to 2**fifo_aw, but workable value for your system are probably less!). Allowing n back to back bursts to generated if watermark is set to n*burst length. Burst(s) continue while fifo has enough space for next burst.
// ------------------------------------------------------------------------------ //

#define ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_FIFO_WATERMARK_DEFAULT (0x10)
#define ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_FIFO_WATERMARK_DATASIZE (8)
#define ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_FIFO_WATERMARK_OFFSET (0xa8)
#define ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_FIFO_WATERMARK_MASK (0xff00)

// args: data (8-bit)
static __inline void acamera_gdc_axi_settings_config_reader_fifo_watermark_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_FIFO_WATERMARK_OFFSET);
    system_gdc_write_32(base+ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_FIFO_WATERMARK_OFFSET, (((u32) (data & 0xff)) << 8) | (curr & (~ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_FIFO_WATERMARK_MASK)));
}
static __inline u8 acamera_gdc_axi_settings_config_reader_fifo_watermark_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_FIFO_WATERMARK_OFFSET) & ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_FIFO_WATERMARK_MASK) >> 8);
}
// ------------------------------------------------------------------------------ //
// Register: config reader rxact maxostand
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Max outstanding read transactions (bursts) allowed. zero means no maximum(uses fifo size as max)
// ------------------------------------------------------------------------------ //

#define ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_RXACT_MAXOSTAND_DEFAULT (0x00)
#define ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_RXACT_MAXOSTAND_DATASIZE (8)
#define ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_RXACT_MAXOSTAND_OFFSET (0xa8)
#define ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_RXACT_MAXOSTAND_MASK (0xff0000)

// args: data (8-bit)
static __inline void acamera_gdc_axi_settings_config_reader_rxact_maxostand_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_RXACT_MAXOSTAND_OFFSET);
    system_gdc_write_32(base+ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_RXACT_MAXOSTAND_OFFSET, (((u32) (data & 0xff)) << 16) | (curr & (~ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_RXACT_MAXOSTAND_MASK)));
}
static __inline u8 acamera_gdc_axi_settings_config_reader_rxact_maxostand_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_RXACT_MAXOSTAND_OFFSET) & ACAMERA_GDC_AXI_SETTINGS_CONFIG_READER_RXACT_MAXOSTAND_MASK) >> 16);
}
// ------------------------------------------------------------------------------ //
// Register: tile reader max arlen
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Maximum value to use for arlen (axi burst length). "0000"= max 1 transfer/burst , upto "1111"= max 16 transfers/burst
// ------------------------------------------------------------------------------ //

#define ACAMERA_GDC_AXI_SETTINGS_TILE_READER_MAX_ARLEN_DEFAULT (0xF)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_READER_MAX_ARLEN_DATASIZE (4)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_READER_MAX_ARLEN_OFFSET (0xac)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_READER_MAX_ARLEN_MASK (0xf)

// args: data (4-bit)
static __inline void acamera_gdc_axi_settings_tile_reader_max_arlen_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_READER_MAX_ARLEN_OFFSET);
    system_gdc_write_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_READER_MAX_ARLEN_OFFSET, (((u32) (data & 0xf)) << 0) | (curr & (~ACAMERA_GDC_AXI_SETTINGS_TILE_READER_MAX_ARLEN_MASK)));
}
static __inline u8 acamera_gdc_axi_settings_tile_reader_max_arlen_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_READER_MAX_ARLEN_OFFSET) & ACAMERA_GDC_AXI_SETTINGS_TILE_READER_MAX_ARLEN_MASK) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: tile reader fifo watermark
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Number of words space in fifo before AXI read burst(s) start (legal values = max_burst_length(max_arlen+1) to 2**fifo_aw, but workable value for your system are probably less!). Allowing n back to back bursts to generated if watermark is set to n*burst length. Burst(s) continue while fifo has enough space for next burst.
// ------------------------------------------------------------------------------ //

#define ACAMERA_GDC_AXI_SETTINGS_TILE_READER_FIFO_WATERMARK_DEFAULT (0x10)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_READER_FIFO_WATERMARK_DATASIZE (8)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_READER_FIFO_WATERMARK_OFFSET (0xac)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_READER_FIFO_WATERMARK_MASK (0xff00)

// args: data (8-bit)
static __inline void acamera_gdc_axi_settings_tile_reader_fifo_watermark_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_READER_FIFO_WATERMARK_OFFSET);
    system_gdc_write_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_READER_FIFO_WATERMARK_OFFSET, (((u32) (data & 0xff)) << 8) | (curr & (~ACAMERA_GDC_AXI_SETTINGS_TILE_READER_FIFO_WATERMARK_MASK)));
}
static __inline u8 acamera_gdc_axi_settings_tile_reader_fifo_watermark_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_READER_FIFO_WATERMARK_OFFSET) & ACAMERA_GDC_AXI_SETTINGS_TILE_READER_FIFO_WATERMARK_MASK) >> 8);
}
// ------------------------------------------------------------------------------ //
// Register: tile reader rxact maxostand
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Max outstanding read transactions (bursts) allowed. zero means no maximum(uses fifo size as max).
// ------------------------------------------------------------------------------ //

#define ACAMERA_GDC_AXI_SETTINGS_TILE_READER_RXACT_MAXOSTAND_DEFAULT (0x00)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_READER_RXACT_MAXOSTAND_DATASIZE (8)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_READER_RXACT_MAXOSTAND_OFFSET (0xac)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_READER_RXACT_MAXOSTAND_MASK (0xff0000)

// args: data (8-bit)
static __inline void acamera_gdc_axi_settings_tile_reader_rxact_maxostand_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_READER_RXACT_MAXOSTAND_OFFSET);
    system_gdc_write_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_READER_RXACT_MAXOSTAND_OFFSET, (((u32) (data & 0xff)) << 16) | (curr & (~ACAMERA_GDC_AXI_SETTINGS_TILE_READER_RXACT_MAXOSTAND_MASK)));
}
static __inline u8 acamera_gdc_axi_settings_tile_reader_rxact_maxostand_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_READER_RXACT_MAXOSTAND_OFFSET) & ACAMERA_GDC_AXI_SETTINGS_TILE_READER_RXACT_MAXOSTAND_MASK) >> 16);
}
// ------------------------------------------------------------------------------ //
// Register: tile writer max awlen
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Maximum value to use for awlen (axi burst length). "0000"= max 1 transfer/burst , upto "1111"= max 16 transfers/burst
// ------------------------------------------------------------------------------ //

#define ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_MAX_AWLEN_DEFAULT (0xF)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_MAX_AWLEN_DATASIZE (4)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_MAX_AWLEN_OFFSET (0xb0)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_MAX_AWLEN_MASK (0xf)

// args: data (4-bit)
static __inline void acamera_gdc_axi_settings_tile_writer_max_awlen_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_MAX_AWLEN_OFFSET);
    system_gdc_write_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_MAX_AWLEN_OFFSET, (((u32) (data & 0xf)) << 0) | (curr & (~ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_MAX_AWLEN_MASK)));
}
static __inline u8 acamera_gdc_axi_settings_tile_writer_max_awlen_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_MAX_AWLEN_OFFSET) & ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_MAX_AWLEN_MASK) >> 0);
}
// ------------------------------------------------------------------------------ //
// Register: tile writer fifo watermark
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Number of words in fifo before AXI write burst(s) start (legal values = max_burst_length(max_awlen+1) to 2**fifo_aw, but workable value for your system are probably less!). Allowing n back to back bursts to generated if watermark is set to n*burst length. Burst(s) continue while fifo has enough for next burst.
// ------------------------------------------------------------------------------ //

#define ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_FIFO_WATERMARK_DEFAULT (0x10)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_FIFO_WATERMARK_DATASIZE (8)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_FIFO_WATERMARK_OFFSET (0xb0)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_FIFO_WATERMARK_MASK (0xff00)

// args: data (8-bit)
static __inline void acamera_gdc_axi_settings_tile_writer_fifo_watermark_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_FIFO_WATERMARK_OFFSET);
    system_gdc_write_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_FIFO_WATERMARK_OFFSET, (((u32) (data & 0xff)) << 8) | (curr & (~ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_FIFO_WATERMARK_MASK)));
}
static __inline u8 acamera_gdc_axi_settings_tile_writer_fifo_watermark_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_FIFO_WATERMARK_OFFSET) & ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_FIFO_WATERMARK_MASK) >> 8);
}
// ------------------------------------------------------------------------------ //
// Register: tile writer wxact maxostand
// ------------------------------------------------------------------------------ //

// ------------------------------------------------------------------------------ //
// Max outstanding write transactions (bursts) allowed. zero means no maximum(uses internal limit of 2048)
// ------------------------------------------------------------------------------ //

#define ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_WXACT_MAXOSTAND_DEFAULT (0x00)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_WXACT_MAXOSTAND_DATASIZE (8)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_WXACT_MAXOSTAND_OFFSET (0xb0)
#define ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_WXACT_MAXOSTAND_MASK (0xff0000)

// args: data (8-bit)
static __inline void acamera_gdc_axi_settings_tile_writer_wxact_maxostand_write(u64 base, u8 data)
{
    u32 curr = system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_WXACT_MAXOSTAND_OFFSET);
    system_gdc_write_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_WXACT_MAXOSTAND_OFFSET, (((u32) (data & 0xff)) << 16) | (curr & (~ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_WXACT_MAXOSTAND_MASK)));
}
static __inline u8 acamera_gdc_axi_settings_tile_writer_wxact_maxostand_read(u64 base)
{
    return (u8)((system_gdc_read_32(base+ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_WXACT_MAXOSTAND_OFFSET) & ACAMERA_GDC_AXI_SETTINGS_TILE_WRITER_WXACT_MAXOSTAND_MASK) >> 16);
}
// ------------------------------------------------------------------------------ //
#endif //__ACAMERA_GDC_CONFIG_H__
