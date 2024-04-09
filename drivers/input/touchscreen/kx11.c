/*
 * Copyright (C) 2021 Siengine Technology Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/termios.h>
#include <linux/of_platform.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input/mt.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/syscalls.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#define MAX_RX_LEN			PAGE_SIZE

#define PANEL_DEFAULT_WIDTH		1920
#define PANEL_DEFAULT_HEIGHT		1080

#define TP_MAX				10

#define FLG_MOVE			0x10
#define FLG_UP				0x20
#define FLG_DOWN			0x40

#define FRAME_SYNC				0x79
#define FRAME_SOC_ADDR			0x6D

#define TOUCH_OPCODE			(0x01)
#define CRC32_POLY				0xEDB88320
#define PROTOCOL_HEADER_LEN		(5)
#define PKG_LEN(pkg)			(pkg->payload_size + PROTOCOL_HEADER_LEN)

struct touch_point_param {
	unsigned int id;
	unsigned int x;
	unsigned int y;
	unsigned int flg;
};

struct touch_struct {
	unsigned int mask;
	struct touch_point_param point[TP_MAX];
};

/* package type */
enum pkg_type {
	PKG_TYPE_REQST = 0,
	PKG_TYPE_RESPN,
	PKG_TYPE_NOTIF,
	PKG_TYPE_ACK,
	PKG_TYPE_NAK,
	PKG_TYPE_MAX = PKG_TYPE_NAK
};

enum pkg_module_id {
	PKG_MID_NORMAL = 1,
	PKG_MID_TOUCH,
	PKG_MID_UPDATE,
	PKG_MID_TOUCHUPDATE,
	PKG_MID_DTC,
	PKG_MID_MAX
};

#pragma pack (1)
struct pkg {
	unsigned char frame_sync;
	unsigned char frame_soc_address;
	unsigned char fixed;
	unsigned char type;
	unsigned char payload_size;
	unsigned char module_id; //module id
	unsigned char opid_m; //opcode id
	unsigned char opid_l;
	unsigned char data[1];
};

struct pkg_touch_point_param {
	unsigned char id;
	unsigned char state;
	unsigned char x_lsb;
	unsigned char x_msb;
	unsigned char y_lsb;
	unsigned char y_msb;
};

struct pkg_payload {
	unsigned char num;
	struct pkg_touch_point_param point[0];
};
#pragma pack()

struct max967xx_data {
	const char *uart_path;
	const char *uart_cfg;
	const char *phys_port;
	unsigned int max_x;
	unsigned int max_y;
};

struct max967xx {
	struct device *dev;
	mm_segment_t old_fs;
	struct file *filp;
	bool uart_state;
	struct max967xx_data pdata;
	struct input_dev *input_dev;
	struct task_struct *kthread;
	bool exit;
	struct mutex exit_lock;
	char * buf;
};

struct data_bits {
	char c;
	unsigned int flag;
};

const static struct data_bits data_bits_map[] = {
	{'5', CS5},
	{'6', CS6},
	{'7', CS7},
	{'8', CS8},
};

static unsigned long crc32_table[256];
static int crc32_table_inited = 0;

static int data_bits_to_flag(const char c)
{
	int len = sizeof(data_bits_map) / sizeof(struct data_bits);
	int i;

	for (i = 0; i < len; ++i) {
		if (data_bits_map[i].c == c)
			return data_bits_map[i].flag;
	}

	pr_err("%s invaild data bit %c\n", __func__, c);

	return -1;
}

static int parity_config(struct ktermios *termios_p, const char c)
{
	switch (c) {
	case 'n':
	case 'N':
		termios_p->c_cflag &= ~PARENB; /* Clear parity enable */
		termios_p->c_iflag &= ~INPCK; /* Enable parity checking */
		break;
	case 'o':
	case 'O':
		termios_p->c_cflag |= (PARODD | PARENB); /* add parity */
		break;
	case 'e':
	case 'E':
		termios_p->c_cflag |= PARENB; /* Enable parity */
		break;
	case 'S':
		break;
	default:
		return -1;
	}

	return 0;
}

static int stop_bit_config(struct ktermios *termios_p, const char c)
{
	switch (c) {
	case '1':
		termios_p->c_cflag &= ~CSTOPB;
		break;
	case '2':
		termios_p->c_cflag |= CSTOPB;
		break;
	default:
		return -1;
	}

	return 0;
}

static void control_flow_config(struct ktermios *termios_p, const char c)
{
	switch (c) {
	case 's':
	case 'S':
		//Enable Software Flow Control
		termios_p->c_iflag |= (IXON | IXOFF | IXANY);
		break;
	case 'h':
	case 'H':
		//Enable hardware flow control
		termios_p->c_cflag |= CRTSCTS;
		break;
	default:
		termios_p->c_cflag |= CLOCAL;
		break;
	}
}

static int max967xx_uart_config(struct max967xx *max967xx)
{
	struct ktermios termios_p;
	struct tty_struct *tty;
	struct tty_ldisc *ld;
	long baudrate = 0;
	int i;
	const char *cfg = max967xx->pdata.uart_cfg;
	int ret;

	memset(&termios_p, 0, sizeof(termios_p));

	dev_info(max967xx->dev, "%s:cfg %s\n", __func__, cfg);

	/* Get baud_rate and set it up */
	for (i = 0; i < strlen(cfg); i++) {
		if ('0'> cfg[i] || '9' < cfg[i])
			break;
		baudrate = baudrate * 10 + cfg[i] - '0';
	}

	if (baudrate) {
		termios_p.c_cflag =  BOTHER;
		termios_p.c_ispeed = baudrate;
		termios_p.c_ospeed = baudrate;
	}

	/* skip a ' ' */
	if (++i >= strlen(cfg))
		return -1;

	ret = data_bits_to_flag(cfg[i]);
	if (ret == -1)
		return ret;

	termios_p.c_cflag |= ret;

	if (++i >= strlen(cfg))
		return -1;

	ret = parity_config(&termios_p, cfg[i]);
	if (ret)
		return ret;

	if (++i >= strlen(cfg))
		return -1;

	ret = stop_bit_config(&termios_p, cfg[i]);
	if (ret)
		return ret;

	termios_p.c_cflag |= CREAD;
	termios_p.c_iflag = IGNPAR | IGNBRK;

	if (++i < strlen(cfg))
		control_flow_config(&termios_p, cfg[i]);

	termios_p.c_oflag = 0;
	termios_p.c_lflag = 0;
	termios_p.c_cc[VTIME] = 1; //read 100ms time out
	termios_p.c_cc[VMIN] = 0;
	termios_p.c_cc[18] = 1;

	max967xx->old_fs = get_fs();
	set_fs(KERNEL_DS);
	tty = ((struct tty_file_private*)max967xx->filp->private_data)->tty;
	ld = tty_ldisc_ref(tty);
	if ( ld && ld->ops->flush_buffer ) {
		ld->ops->flush_buffer(tty);
		tty_ldisc_deref(ld);
	}
	/* set uart port attribute */
	tty_set_termios(tty, &termios_p);
	tty_perform_flush(tty, TCOFLUSH);
	tty_perform_flush(tty, TCIFLUSH);
	set_fs(max967xx->old_fs);

	return 0;
}

static int max967xx_uart_enable(struct max967xx *max967xx, bool enable)
{
	const char *path = max967xx->pdata.uart_path;

	if (enable) {
		max967xx->filp = filp_open(path, O_RDWR | O_NOCTTY, 0644);
		if (IS_ERR(max967xx->filp))
			return -EPERM;

		max967xx_uart_config(max967xx);
	} else {
		filp_close(max967xx->filp, NULL);
	}

	max967xx->uart_state = enable;

	return 0;
}

static size_t max967xx_uart_read(struct max967xx *max967xx,  unsigned char *buf, size_t size)
{
	loff_t pos = 0;
	int ret;
	size_t len = 0;

	if (!max967xx->uart_state) {
		ret = max967xx_uart_enable(max967xx, true);
		if (ret < 0)
			return ret;
	}

	max967xx->old_fs = get_fs();
	set_fs(KERNEL_DS);

	do {
		mutex_lock(&max967xx->exit_lock);

		if (max967xx->exit) {
			mutex_unlock(&max967xx->exit_lock);
			break;
		}

		mutex_unlock(&max967xx->exit_lock);

		len = vfs_read(max967xx->filp, buf, size, &pos);

	} while (len <= 0);

	set_fs(max967xx->old_fs);

	return len;
}

int kx11_parse_dt(struct device_node *np, struct max967xx *max967xx)
{
	struct max967xx_data *pdata = &max967xx->pdata;

	if (of_property_read_string(np, "uart-path", &pdata->uart_path)) {
		dev_err(max967xx->dev, "%s: get uart-path fail in dts\n", __func__);
		return -EINVAL;
	}

	if (of_property_read_string(np, "uart-config", &pdata->uart_cfg)) {
		dev_err(max967xx->dev, "%s: get uart-config fail in dts\n", __func__);
		return -EINVAL;
	}

	if (of_property_read_u32_index(np, "ts-max-x", 0, &(pdata->max_x)))
		pdata->max_x = 1920;

	if (of_property_read_u32_index(np, "ts-max-y", 0, &(pdata->max_y)))
		pdata->max_y = 1080;

	if (of_property_read_string(np, "phys-port", &pdata->phys_port)) {
		dev_err(max967xx->dev, "%s: get phys-port fail in dts\n", __func__);
		//return -EINVAL;
	}

    return 0;
}

static int local_input_open(struct input_dev *dev)
{
	return 0;
}

static void local_input_close(struct input_dev *dev)
{
	return;
}

static int request_input_dev(struct max967xx *max967xx)
{
	int ret;
	int max_x = max967xx->pdata.max_x;
	int max_y = max967xx->pdata.max_y;

	max967xx->input_dev = devm_input_allocate_device(max967xx->dev);
	if (!max967xx->input_dev) {
		dev_err(max967xx->dev, "%s Failed to allocate input device.", __func__);
		return -ENOMEM;
	}
	if(max967xx->pdata.phys_port)
		max967xx->input_dev->phys = max967xx->pdata.phys_port;
	else
		max967xx->input_dev->phys = "UART";
	max967xx->input_dev->name = "KX11";
	max967xx->input_dev->id.bustype = BUS_RS232;
	max967xx->input_dev->id.vendor  = 0x8000;
	max967xx->input_dev->id.product = 0x0400;
	max967xx->input_dev->id.version = 0x0010;
	max967xx->input_dev->open  = local_input_open;
	max967xx->input_dev->close = local_input_close;

	input_set_abs_params(max967xx->input_dev, ABS_MT_POSITION_X,  0, max_x, 0 ,0);
	input_set_abs_params(max967xx->input_dev, ABS_MT_POSITION_Y,  0, max_y, 0 ,0);
	input_mt_init_slots(max967xx->input_dev, TP_MAX, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);

	input_set_drvdata(max967xx->input_dev, max967xx);

	ret = input_register_device(max967xx->input_dev);
	if (ret) {
		dev_err(max967xx->dev, "[ERR] %s: register %s input failed\n",
			__func__, max967xx->input_dev->name);
		return ret;
	}

	return 0;
}

static void create_crc32_table(void) {
	int i = 0, j = 0;
	unsigned long crc = 0;

	if ( crc32_table_inited )
		return;
	for(i = 0;i < 256;i++ ) {
		crc = i;
		for(j = 8;j > 0;j--) {
			if( crc&1 ) {
				crc = (crc>>1) ^ CRC32_POLY;
			} else {
				crc >>= 1;
			}
		}
		crc32_table[i] = crc;
	}
	crc32_table_inited = 1;
}

static unsigned long calc_crc32(char *data, int size) {
	unsigned long crc = 0;
	int i = 0;
	crc = 0xffffffff;

	for(i = 0; i < size; i++) {
		crc = ((crc >> 8) & 0x00FFFFFF) ^ crc32_table[(int)(crc ^ data[i]) & 0xff];
	}
	return ~crc;
}

static int check_crc32_isvalid(char *buf, int len) {
	unsigned long crcValue;
	int result = 0;

	if ( NULL == buf || len < 12)
		return 0;

	crcValue = calc_crc32(buf, len - 4);
	result = ((buf[len - 1] == ((crcValue >> 0) & 0xFF)) &&
				(buf[len - 2] == ((crcValue >> 8) & 0xFF)) &&
				(buf[len - 3] == ((crcValue >> 16) & 0xFF)) &&
				(buf[len - 4] == ((crcValue >> 24) & 0xFF)));
	//printk("%s crcValue = 0x%08x, crc result = %d \n",
	//			__func__, crcValue, result);

	return result;
}

static int __pkg_check(unsigned char *data, size_t len)
{
	struct pkg *pkg = (struct pkg *)data;
	int pkg_len;

	if (!pkg || !len || (sizeof(*pkg) > len))
		return -EINVAL;

	pkg_len = PKG_LEN(pkg);
	if (   (pkg_len > len)\
	    || (FRAME_SYNC != pkg->frame_sync)\
	    || (FRAME_SOC_ADDR != pkg->frame_soc_address)\
	    || (0x00 != pkg->fixed)\
	    || (PKG_TYPE_NOTIF != pkg->type)\
	    || (PKG_MID_TOUCH != pkg->module_id)\
		|| (0x01 != pkg->opid_l))
		return -EINVAL;

	return pkg_len;
}

static bool retrieve_pkg(unsigned char *data, size_t size, size_t* plen)
{
	struct pkg *pkg;
	int idx, ret;

	for(idx = 0; idx < size; idx++) {
		ret =  __pkg_check(data + idx, size - idx);
		if (ret <= 0)
			continue;
		else if (idx)
			break;

		*plen = (size_t)ret;
		pkg = (struct pkg *)(data + idx);

		//touch message
		if((pkg->type == PKG_TYPE_NOTIF) && (pkg->module_id == PKG_MID_TOUCH))
			return true;

		return false;
	}

	*plen = (size_t)idx;
	return false;
}

static int pkg2tps(struct pkg *pkg, struct touch_struct *tps)
{
	struct touch_point_param *tp;
	struct pkg_payload *touch;
	int idx;

	tps->mask = 0;
	touch = (struct pkg_payload*)(pkg->data);

	for (idx = 0; idx < touch->num; idx++) {
		if (touch->point[idx].id > TP_MAX)
			return -EINVAL;

		tp = tps->point + (touch->point[idx].id - 1);

		tp->id  = touch->point[idx].id - 1;
		tp->x   = touch->point[idx].x_msb;
		tp->x   = (tp->x << 8) + touch->point[idx].x_lsb;
		tp->y   = touch->point[idx].y_msb;
		tp->y   = (tp->y << 8) + touch->point[idx].y_lsb;
		tp->flg = touch->point[idx].state;

		tps->mask |= (1 << (touch->point[idx].id - 1));
	}

	return 0;
}


static void touch_event_report(struct max967xx *max967xx, struct touch_struct *tps)
{
	unsigned int idx;

	if (!tps->mask)
		return;

	for (idx = 0; idx < TP_MAX; idx++) {
		if (~tps->mask & (1 << idx))
			continue;

		if (tps->point[idx].flg != FLG_UP) {
			input_mt_slot(max967xx->input_dev, tps->point[idx].id);
			input_mt_report_slot_state(max967xx->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(max967xx->input_dev ,ABS_MT_POSITION_X,
						tps->point[idx].x);
			input_report_abs(max967xx->input_dev ,ABS_MT_POSITION_Y,
						tps->point[idx].y);
		} else {
			input_mt_report_slot_state(max967xx->input_dev, MT_TOOL_FINGER, false);
		}

                dev_dbg(max967xx->dev, "{ID:%d %s [%04d,%04d]} \n", \
                                tps->point[idx].id,\
                                (tps->point[idx].flg == FLG_UP) ? "Up" : "Down",\
                                tps->point[idx].x, tps->point[idx].y);
	}

	input_mt_sync_frame(max967xx->input_dev);
	input_sync(max967xx->input_dev);
}

static int kthread_tp (void *data)
{
	struct max967xx *max967xx = (struct max967xx *)data;
	ssize_t rxd = 0;
	size_t len = 0;
	char *buf = NULL;
	struct touch_struct tps;
	bool is_touch;
	int ret;
	int touch_frame_len = 0;

	create_crc32_table();

	//wait uart ready
	while (!max967xx->filp) {
		ret = max967xx_uart_enable(max967xx, 1);
		if (ret) {
			msleep(200);
			max967xx->filp = NULL;
		} else {
			dev_info(max967xx->dev, "%s %s open\n",
					__func__, max967xx->pdata.uart_path);
		}
	}

	max967xx->exit = false;

	mutex_init(&max967xx->exit_lock);

	while (1) {
		buf = max967xx->buf;
		rxd = max967xx_uart_read(max967xx, buf, MAX_RX_LEN);

		mutex_lock(&max967xx->exit_lock);
		if (max967xx->exit) {
			mutex_unlock(&max967xx->exit_lock);
			break;
		}
		mutex_unlock(&max967xx->exit_lock);

		while ( rxd > 8 ) {
			if (( buf[0] != 0x79 || buf[1] != 0x6D || buf[2] != 0x00 )) {
				rxd -= 1;
				buf += 1;
				continue;
			}
			touch_frame_len = buf[4] + PROTOCOL_HEADER_LEN;
			if ( (rxd >= touch_frame_len) && check_crc32_isvalid(buf, touch_frame_len)) {
				is_touch = retrieve_pkg(buf, touch_frame_len, &len);
				if (is_touch) {
					pkg2tps((struct pkg*)buf, &tps);
					touch_event_report(max967xx, &tps);
				} else {
					dev_err(max967xx->dev, "%s get invalid frame\n", __func__);
				}
			} else {
				dev_info(max967xx->dev, "%s received invalid data, Read Len = %d \n", __func__, touch_frame_len);
			}

			buf += touch_frame_len;
			rxd -= touch_frame_len;
		}

	}

	return 0;
}

static int kx11_panel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max967xx *max967xx = NULL;
	int ret;

	max967xx = devm_kzalloc(dev, sizeof(*max967xx), GFP_KERNEL);
	if (!max967xx)
		return -ENOMEM;

	max967xx->dev = dev;

	platform_set_drvdata(pdev, max967xx);

	ret = kx11_parse_dt(dev->of_node, max967xx);
	if (ret) {
		dev_err(dev, "%s: failed to parse device-tree!\n", __func__);
		goto parse_dt_failed;
	}

	max967xx->uart_state = 0;

	ret = request_input_dev(max967xx);
	if (ret)
		goto parse_dt_failed;

	max967xx->buf = (char *)kzalloc(MAX_RX_LEN, GFP_KERNEL);
	if (!max967xx->buf) {
		dev_err(max967xx->dev, "alloc mem failed\n");
		ret = -ENOMEM;
		goto parse_dt_failed;
	}

	max967xx->kthread = kthread_run(kthread_tp, max967xx, "tp-thread");
	if (!max967xx->kthread) {
		dev_err(dev, "%s thread create fail\n", __func__);
		ret = -1;
		goto parse_dt_failed;
	}

	return 0;

parse_dt_failed:
	devm_kfree(&pdev->dev, max967xx);
	return ret;
}

static int kx11_panel_remove(struct platform_device *pdev)
{
	struct max967xx *max967xx = dev_get_drvdata(&pdev->dev);

	mutex_lock(&max967xx->exit_lock);
	max967xx->exit = true;
	mutex_unlock(&max967xx->exit_lock);

	kthread_stop(max967xx->kthread);
	max967xx->kthread = NULL;
	kfree(max967xx->buf);
	devm_kfree(&pdev->dev, max967xx);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id kx11_dt_ids[] = {
	{ .compatible = "siengine,kx11", },
	{ }
};
MODULE_DEVICE_TABLE(of, kx11_dt_ids);
#endif

static struct platform_driver kx11_driver = {
	.probe = kx11_panel_probe,
	.remove = kx11_panel_remove,
	.driver = {
		.name = "kx11_panel",
		.of_match_table = kx11_dt_ids,
	},
};

module_platform_driver(kx11_driver);

MODULE_AUTHOR("siengine");
MODULE_DESCRIPTION("KX11 panel driver");
MODULE_LICENSE("GPL");
