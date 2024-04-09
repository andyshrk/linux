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

#define FLG_UP				0x20
#define FLG_DOWN			0x40
#define FLG_MOVE			0x60

#define PKG_FRAME_SYNC			0x79
#define PKG_MAGIC_HIGH			0x55
#define PKG_MAGIC_LOW			0xAA

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

#define PKG_LEN(pkg)	(((pkg->len_msb) << 8) + (pkg->len_lsb))

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
	unsigned char magic_high;
	unsigned char magic_low;
	unsigned char type;
	unsigned char len_msb;
	unsigned char len_lsb;
	unsigned char mid; //module id
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
	bool suspend;
	struct completion filp_close_done;
};

struct baudrate_data {
	long baudrate;
	unsigned int flag;
};

const static struct baudrate_data baudrate_map[] = {
	{300, B300},
	{600, B600},
	{1200, B1200},
	{2400, B2400},
	{4800, B4800},
	{9600, B9600},
	{19200, B19200},
	{38400, B38400},
	{57600, B57600},
	{115200, B115200},
	{230400, B230400},
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

static int baudrate_to_flag(const long baudrate)
{
	int len = sizeof(baudrate_map) / sizeof(struct baudrate_data);
	int i;

	for (i = 0; i < len; ++i) {
		if (baudrate_map[i].baudrate == baudrate)
			return baudrate_map[i].flag;
	}

	pr_err("%s invaild baudrate %ld, use 9600 as default\n", __func__, baudrate);

	return B9600;
}

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

	termios_p.c_cflag = baudrate_to_flag(baudrate);

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
	termios_p.c_cflag |= CLOCAL;

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

		while (max967xx->suspend == 1) {

			if (max967xx->filp != NULL) {
				filp_close(max967xx->filp, NULL);
				max967xx->filp = NULL;
				max967xx->uart_state = 0;
				complete(&max967xx->filp_close_done);
			}

			msleep(10);
		}

		if (max967xx->uart_state == 0) {
			const char *path = max967xx->pdata.uart_path;
			max967xx->filp = filp_open(path, O_RDWR | O_NOCTTY, 0644);
			if (IS_ERR(max967xx->filp))
				return -EPERM;
			max967xx->uart_state = 1;
		}

		len = vfs_read(max967xx->filp, buf, size, &pos);
	} while (len <= 0);

	set_fs(max967xx->old_fs);

	return len;
}

#if 0
static int max967xx_uart_write(struct max967xx *max967xx, unsigned char *buf, size_t size)
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

	len = vfs_write(max967xx->filp, buf, size, &pos);

	set_fs(max967xx->old_fs);

	return len;
}
#endif

int max967xx_parse_dt(struct device_node *np, struct max967xx *max967xx)
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
	max967xx->input_dev->name = "CC11";
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

static unsigned char pkg_crc_calculate(unsigned char *data, size_t len)
{
	unsigned char crc = 0;
	size_t idx = 0;

	if (!data || !len)
		return 0;

	for (idx = 0; idx < len; ++idx)
		crc += data[idx];

	return crc;
}

static int __pkg_check(unsigned char *data, size_t len)
{
	struct pkg *pkg = (struct pkg *)data;
	int pkg_len;

	if (!pkg || !len || (sizeof(*pkg) > len))
		return -EINVAL;

	pkg_len = PKG_LEN(pkg) + offsetof(struct pkg, mid);
	if (   (pkg_len > len)\
	    || (PKG_FRAME_SYNC != pkg->frame_sync)\
	    || (PKG_MAGIC_HIGH != pkg->magic_high)\
	    || (PKG_MAGIC_LOW != pkg->magic_low)\
	    || (PKG_TYPE_MAX <= pkg->type)\
	    || (PKG_MID_MAX <= pkg->mid))
		return -EINVAL;

	if (pkg_crc_calculate(data, pkg_len - 1) != data[pkg_len - 1])
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
		if((pkg->type == PKG_TYPE_NOTIF) && (pkg->mid == PKG_MID_TOUCH))
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
		}

                dev_dbg(max967xx->dev, "{ID:%d %s [%04d,%04d]} \n", \
                                tps->point[idx].id,\
                                (tps->point[idx].flg == FLG_UP) ? "Up" : "Down",\
                                tps->point[idx].x, tps->point[idx].y);
	}

	input_mt_sync_frame(max967xx->input_dev);
	input_sync(max967xx->input_dev);
}

#if 0
static int panel_enable(struct max967xx *max967xx, bool enable)
{
	unsigned char req[] = {PKG_FRAME_SYNC, PKG_MAGIC_HIGH, PKG_MAGIC_LOW,\
		PKG_TYPE_REQST, 0, 5, PKG_MID_NORMAL, 0, 1, !enable, 0};
	struct pkg *cmd = (struct pkg *)req;
	size_t len = PKG_LEN(cmd) + offsetof(struct pkg, mid);
	int retry;
	const int buf_len = 100;
	unsigned char buf[buf_len];
	unsigned char ack[] = {PKG_FRAME_SYNC, PKG_MAGIC_HIGH, PKG_MAGIC_LOW,\
		PKG_TYPE_ACK, 0, 4, PKG_MID_NORMAL, 0, 1, !enable, 0};
	struct pkg *ack_cmd = (struct pkg *)ack;
	size_t ack_len = PKG_LEN(ack_cmd) + offsetof(struct pkg, mid);

	req[len - 1] = pkg_crc_calculate(req, len - 1);

	ack[ack_len - 1] = pkg_crc_calculate(ack, ack_len - 1);

	for (retry = 0; retry < 5; retry++) {
		//enable soc->csd
		//79 55 aa 00 00 05 01 00 01 00 crc
		if (len != max967xx_uart_write(max967xx, req, len)) {
			dev_err(max967xx->dev, "%s enable panel fail, retry %d\n",
				__func__, retry);
		} else {
			//ack csd->soc
			//79 55 aa 03 00 04 01 00 01 crc
			max967xx_uart_read(max967xx, buf, buf_len);
			//enable done csd->soc
			//79 55 aa 01 00 05 01 00 01 00 crc
			max967xx_uart_read(max967xx, buf, buf_len);
			//ack soc->csd
			//79 55 aa 03 00 04 01 00 01 crc
			if (ack_len != max967xx_uart_write(max967xx, ack, ack_len)) {
				dev_err(max967xx->dev, "%s ack fail, retry %d\n",
					__func__, retry);
			}

			dev_info(max967xx->dev, "%s enable panel pass\n", __func__);
			break;
		}
	}

	if (retry >= 5) {
		dev_err(max967xx->dev, "%s enable panel fail\n", __func__);
		return -1;
	}

	return 0;
}
#endif

static int kthread_tp (void *data)
{
	struct max967xx *max967xx = (struct max967xx *)data;
	ssize_t rxd = 0, optr = 0;
	size_t len = 0;
	char *buf = NULL;
	struct touch_struct tps;
	bool is_touch;
	int ret;

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

	//ret = panel_enable(max967xx, 1);
	//if (ret)
		//return ret;

	while (1) {
		buf = max967xx->buf;
		while (optr >= rxd) {
			rxd = max967xx_uart_read(max967xx, buf, MAX_RX_LEN);
			optr = 0;
		}

		mutex_lock(&max967xx->exit_lock);
		if (max967xx->exit) {
			mutex_unlock(&max967xx->exit_lock);
			break;
		}
		mutex_unlock(&max967xx->exit_lock);

		is_touch = retrieve_pkg(buf + optr, rxd - optr, &len);
		if (is_touch) {
			pkg2tps((struct pkg*)(buf + optr), &tps);
			touch_event_report(max967xx, &tps);
		} else {
			dev_dbg(max967xx->dev, "%s get invalid frame\n", __func__);
		}

		optr += len;
	}

	return 0;
}

static int max967xx_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct max967xx *max967xx = NULL;
	int ret;

	max967xx = devm_kzalloc(dev, sizeof(*max967xx), GFP_KERNEL);
	if (!max967xx)
		return -ENOMEM;

	max967xx->dev = dev;

	platform_set_drvdata(pdev, max967xx);

	ret = max967xx_parse_dt(dev->of_node, max967xx);
	if (ret) {
		dev_err(dev, "%s: failed to parse device-tree!\n", __func__);
		goto parse_dt_failed;
	}

	max967xx->uart_state = 0;
	max967xx->suspend = false;

	ret = request_input_dev(max967xx);
	if (ret)
		goto parse_dt_failed;

	max967xx->buf = (char *)kzalloc(MAX_RX_LEN, GFP_KERNEL);
	if (!max967xx->buf) {
		dev_err(max967xx->dev, "alloc mem failed\n");
		ret = -ENOMEM;
		goto parse_dt_failed;
	}

	init_completion(&max967xx->filp_close_done);

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

static int max967xx_remove(struct platform_device *pdev)
{
	struct max967xx *max967xx = dev_get_drvdata(&pdev->dev);

	mutex_lock(&max967xx->exit_lock);
	max967xx->exit = true;
	mutex_unlock(&max967xx->exit_lock);

	max967xx->suspend = true;
	kthread_stop(max967xx->kthread);
	max967xx->kthread = NULL;
	kfree(max967xx->buf);
	devm_kfree(&pdev->dev, max967xx);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id max967xx_dt_ids[] = {
	{ .compatible = "max,max967xx", },
	{ }
};
MODULE_DEVICE_TABLE(of, max967xx_dt_ids);
#endif

#ifdef CONFIG_PM_SLEEP
static int max967xx_resume(struct device *dev)
{
	dev_dbg(dev, "max967xx_resume... \n");
	struct max967xx *max967xx = dev_get_drvdata(dev);
	max967xx->suspend = false;

	return 0;
}

#if 0
static int max967xx_suspend(struct device *dev)
{
	dev_dbg(dev, "max967xx_suspend... \n");
	struct max967xx *max967xx = dev_get_drvdata(dev);
	max967xx->suspend = true;
	return 0;
}
#endif

static int max967xx_repare_suspend(struct device *dev)
{
	dev_dbg(dev, "max967xx_repare_suspend... \n");
	struct max967xx *max967xx = dev_get_drvdata(dev);
	max967xx->suspend = true;

	wait_for_completion(&max967xx->filp_close_done);
	return 0;
}

#if 0
static const struct dev_pm_ops max967xx_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(max967xx_suspend, max967xx_resume)
};
#endif

static const struct dev_pm_ops max967xx_pm = {
	.prepare = max967xx_repare_suspend,
	.resume  = max967xx_resume,
};
#endif

static struct platform_driver max967xx_driver = {
	.probe = max967xx_probe,
	.remove = max967xx_remove,
	.driver = {
		#ifdef CONFIG_PM_SLEEP
		.pm = &max967xx_pm,
		#endif
		.name = "max967xx",
		.of_match_table = max967xx_dt_ids,
	},
};

module_platform_driver(max967xx_driver);

MODULE_AUTHOR("siengine");
MODULE_DESCRIPTION("CC11 panel driver");
MODULE_LICENSE("GPL");
