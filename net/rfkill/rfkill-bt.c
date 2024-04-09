// SPDX-License-Identifier: GPL-2.0
// (C) 2019-2021 Siengine, Inc. (www.siengine.com)

/*
 * Siengine rfkill driver for bluetooth
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/rfkill.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/rfkill-bt.h>
#include <linux/interrupt.h>
#include <asm/irq.h>
#include <linux/suspend.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <dt-bindings/gpio/gpio.h>
#include <uapi/linux/rfkill.h>
#include <linux/cdev.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif

#if 0
#define DBG(x...) pr_info("[BT_RFKILL]: " x)
#else
#define DBG(x...)
#endif

#define LOG(x...) pr_info("[BT_RFKILL]: " x)

#define BT_WAKEUP_TIMEOUT 10000
#define BT_IRQ_WAKELOCK_TIMEOUT (10 * 1000)

#define BT_BLOCKED true
#define BT_UNBLOCK false
#define BT_SLEEP true
#define BT_WAKEUP false

enum {
	IOMUX_FNORMAL = 0,
	IOMUX_FGPIO,
	IOMUX_FMUX,
};

struct rfkill_data {
	struct rfkill_platform_data *pdata;
	struct platform_device *pdev;
	struct rfkill *rfkill_dev;
	struct delayed_work bt_sleep_delay_work;
	int irq_req;
};

static struct rfkill_data *g_rfkill = NULL;

static const char bt_name[] = "bt_default";
static char writebuf[1];
static int gpio_btm2 = 0;
static struct class *btm2_class;
static struct cdev *btm2_cdev = NULL;
static dev_t btm2_devid = 0;
static dev_t btm2_major;
#define BTM2_MAX_CHANNELS	1

static int gpio_fm = 0;
static struct class *fm_class;
static struct cdev *fm_cdev = NULL;
static dev_t fm_devid = 0;
static dev_t fm_major;
#define FM_MAX_CHANNELS	1

static irqreturn_t rfkill_wake_host_irq(int irq, void *dev)
{
	return IRQ_HANDLED;
}

static int rfkill_setup_gpio(struct platform_device *pdev,
				struct rfkill_gpio *gpio, const char *prefix,
				const char *name)
{
	if (gpio_is_valid(gpio->io)) {
		int ret = 0;

		sprintf(gpio->name, "%s_%s", prefix, name);
		ret = devm_gpio_request(&pdev->dev, gpio->io, gpio->name);
		if (ret) {
			LOG("Failed to get %s gpio.\n", gpio->name);
			return -1;
		}
	}

	return 0;
}

static int rfkill_setup_wake_irq(struct rfkill_data *rfkill, int flag)
{
	int ret = 0;
	struct rfkill_irq *irq = &rfkill->pdata->wake_host_irq;

	if (!flag) {
		rfkill->irq_req = 0;
		ret = rfkill_setup_gpio(rfkill->pdev, &irq->gpio,
					   rfkill->pdata->name, "wake_host");
		if (ret)
			goto fail1;
	}
	if (gpio_is_valid(irq->gpio.io)) {
		if (rfkill->irq_req) {
			rfkill->irq_req = 0;
			free_irq(irq->irq, rfkill);
		}
		LOG("Request irq for bt wakeup host\n");
		irq->irq = gpio_to_irq(irq->gpio.io);
		sprintf(irq->name, "%s_irq", irq->gpio.name);
		ret = request_irq(irq->irq, rfkill_wake_host_irq,
				  (irq->gpio.enable == GPIO_ACTIVE_LOW) ?
					  IRQF_TRIGGER_FALLING :
					  IRQF_TRIGGER_RISING,
				  irq->name, rfkill);
		if (ret)
			goto fail2;
		rfkill->irq_req = 1;
		LOG("** disable irq\n");
		disable_irq(irq->irq);
		ret = enable_irq_wake(irq->irq);
		if (ret)
			goto fail3;
	}

	return ret;

fail3:
	free_irq(irq->irq, rfkill);
fail2:
	gpio_free(irq->gpio.io);
fail1:
	return ret;
}

static inline void rfkill_sleep_bt_internal(struct rfkill_data *rfkill,
					       bool sleep)
{
	struct rfkill_gpio *wake = &rfkill->pdata->wake_gpio;
	if (!gpio_is_valid(wake->io)) {
		DBG("*** Not support bt wakeup and sleep\n");
		return;
	}
	DBG("*** bt sleep: %d ***\n", sleep);

	if (!sleep) {
		DBG("HOST_UART_TX pull down 10us\n");
		if (rfkill_setup_gpio(rfkill->pdev, wake,
					 rfkill->pdata->name, "wake") != 0) {
			return;
		}

		gpio_direction_output(wake->io, wake->enable);
		usleep_range(10, 20);
		gpio_direction_output(wake->io, !wake->enable);

		gpio_free(wake->io);
	}
}

void rfkill_sleep_bt(bool sleep)
{
	struct rfkill_data *rfkill = g_rfkill;
	struct rfkill_gpio *wake;
	bool ret;

	DBG("Enter %s\n", __func__);

	if (!rfkill) {
		LOG("*** RFKILL is empty???\n");
		return;
	}

	wake = &rfkill->pdata->wake_gpio;
	if (!gpio_is_valid(wake->io)) {
		DBG("*** Not support bt wakeup and sleep\n");
		return;
	}

	ret = cancel_delayed_work_sync(&rfkill->bt_sleep_delay_work);

	rfkill_sleep_bt_internal(rfkill, sleep);

}
EXPORT_SYMBOL(rfkill_sleep_bt);

static int bt_power_state = 0;
int rfkill_get_bt_power_state(int *power, bool *toggle)
{
	struct rfkill_data *mrfkill = g_rfkill;

	if (!mrfkill) {
		LOG("%s: rfkill-bt driver has not Successful initialized\n", __func__);
		return -1;
	}

	*toggle = mrfkill->pdata->power_toggle;
	*power = bt_power_state;

	return 0;
}

static int rfkill_set_power(void *data, bool blocked)
{
	struct rfkill_data *rfkill = data;
	struct rfkill_gpio *wake_host = &rfkill->pdata->wake_host_irq.gpio;
	struct rfkill_gpio *poweron = &rfkill->pdata->poweron_gpio;
	struct rfkill_gpio *reset = &rfkill->pdata->reset_gpio;
	struct rfkill_gpio *rts = &rfkill->pdata->rts_gpio;
	struct pinctrl *pinctrl = rfkill->pdata->pinctrl;
	int wifi_power = 0;
	bool toggle = false;

	toggle = rfkill->pdata->power_toggle;

	if (!blocked) {

		rfkill_sleep_bt(BT_WAKEUP); // ensure bt is wakeup

		if (gpio_is_valid(wake_host->io)) {
			LOG("%s: set bt wake_host high!\n", __func__);
			gpio_direction_output(wake_host->io, 1);
			msleep(20);
		}

		if (gpio_is_valid(poweron->io)) {
			if (gpio_get_value_cansleep(poweron->io) == !poweron->enable) {
				gpio_direction_output(poweron->io,
						      !poweron->enable);
				msleep(20);
				gpio_direction_output(poweron->io,
						      poweron->enable);
				msleep(20);
				if (gpio_is_valid(wake_host->io))
					gpio_direction_input(wake_host->io);
			}
		}

		if (gpio_is_valid(reset->io)) {
			if (gpio_get_value_cansleep(reset->io) == !reset->enable) {
				gpio_direction_output(reset->io,
						      !reset->enable);
				msleep(20);
				gpio_direction_output(reset->io, reset->enable);
			}
		}

		if (pinctrl && gpio_is_valid(rts->io)) {
			pinctrl_select_state(pinctrl, rts->gpio_state);
			LOG("ENABLE UART_RTS\n");
			gpio_direction_output(rts->io, rts->enable);
			msleep(100);
			LOG("DISABLE UART_RTS\n");
			gpio_direction_output(rts->io, !rts->enable);
			pinctrl_select_state(pinctrl, rts->default_state);
		}

		bt_power_state = 1;
		LOG("bt turn on power\n");
		rfkill_setup_wake_irq(rfkill, 1);
	} else {
		if (gpio_is_valid(poweron->io)) {
			if (gpio_get_value_cansleep(poweron->io) == poweron->enable) {
				gpio_direction_output(poweron->io,
						      !poweron->enable);
				msleep(20);
			}
		}

		bt_power_state = 0;
		LOG("bt shut off power\n");
		if (gpio_is_valid(reset->io)) {
			if (gpio_get_value_cansleep(reset->io) == reset->enable) {
				gpio_direction_output(reset->io,
						      !reset->enable);
				msleep(20);
			}
		}
		if (toggle) {
			if (!wifi_power) {
				LOG("%s: bt will set vbat to low\n", __func__);
			} else {
				LOG("%s: bt shouldn't control the vbat\n", __func__);
			}
		}
	}

	return 0;
}

static int rfkill_pm_prepare(struct device *dev)
{
	struct rfkill_data *rfkill = g_rfkill;
	struct rfkill_gpio *rts;
	struct rfkill_irq *wake_host_irq;
	struct pinctrl *pinctrl = rfkill->pdata->pinctrl;

	DBG("Enter %s\n", __func__);

	if (!rfkill)
		return 0;

	rts = &rfkill->pdata->rts_gpio;
	wake_host_irq = &rfkill->pdata->wake_host_irq;

	//To prevent uart to receive bt data when suspended
	if (pinctrl && gpio_is_valid(rts->io)) {
		DBG("Disable UART_RTS\n");
		pinctrl_select_state(pinctrl, rts->gpio_state);
		gpio_direction_output(rts->io, !rts->enable);
	}

	// enable bt wakeup host
	if (gpio_is_valid(wake_host_irq->gpio.io) && bt_power_state) {
		DBG("enable irq for bt wakeup host\n");
		enable_irq(wake_host_irq->irq);
	}

	return 0;
}

static void rfkill_pm_complete(struct device *dev)
{
	struct rfkill_data *rfkill = g_rfkill;
	struct rfkill_irq *wake_host_irq;
	struct rfkill_gpio *rts;
	struct pinctrl *pinctrl = rfkill->pdata->pinctrl;

	DBG("Enter %s\n", __func__);

	if (!rfkill)
		return;

	wake_host_irq = &rfkill->pdata->wake_host_irq;
	rts = &rfkill->pdata->rts_gpio;

	if (gpio_is_valid(wake_host_irq->gpio.io) && bt_power_state) {
		LOG("** disable irq\n");
		disable_irq(wake_host_irq->irq);
	}

	if (pinctrl && gpio_is_valid(rts->io)) {
		DBG("Enable UART_RTS\n");
		gpio_direction_output(rts->io, rts->enable);
		pinctrl_select_state(pinctrl, rts->default_state);
	}
}

static const struct rfkill_ops rfkill_ops = {
	.set_block = rfkill_set_power,
};

#define PROC_DIR "bluetooth/sleep"

static struct proc_dir_entry *bluetooth_dir, *sleep_dir;

static ssize_t bluesleep_read_proc_lpm(struct file *file, char __user *buffer,
				       size_t count, loff_t *data)
{
	return sprintf(buffer, "unsupported to read\n");
}

static ssize_t bluesleep_write_proc_lpm(struct file *file,
					const char __user *buffer, size_t count,
					loff_t *data)
{
	return count;
}

static ssize_t bluesleep_read_proc_btwrite(struct file *file,
					   char __user *buffer, size_t count,
					   loff_t *data)
{
	return sprintf(buffer, "unsupported to read\n");
}

static ssize_t bluesleep_write_proc_btwrite(struct file *file,
					    const char __user *buffer,
					    size_t count, loff_t *data)
{
	char b;

	if (count < 1)
		return -EINVAL;

	if (copy_from_user(&b, buffer, 1))
		return -EFAULT;

	DBG("btwrite %c\n", b);
	/* HCI_DEV_WRITE */
	if (b != '0')
		rfkill_sleep_bt(BT_WAKEUP);
	else
		rfkill_sleep_bt(BT_SLEEP);

	return count;
}

#ifdef CONFIG_OF
static int bluetooth_platdata_parse_dt(struct device *dev,
				       struct rfkill_platform_data *data)
{
	struct device_node *node = dev->of_node;
	int gpio;
	enum of_gpio_flags flags;

	if (!node)
		return -ENODEV;

	memset(data, 0, sizeof(*data));

	if (of_find_property(node, "wifi-bt-power-toggle", NULL)) {
		data->power_toggle = true;
		LOG("%s: get property wifi-bt-power-toggle.\n", __func__);
	} else {
		data->power_toggle = false;
	}

	gpio = of_get_named_gpio_flags(node, "uart_rts_gpios", 0, &flags);
	if (gpio_is_valid(gpio)) {
		data->rts_gpio.io = gpio;
		data->rts_gpio.enable = (flags == GPIO_ACTIVE_HIGH) ? 1 : 0;
		LOG("%s: get property: uart_rts_gpios = %d.\n", __func__, gpio);
		data->pinctrl = devm_pinctrl_get(dev);
		if (!IS_ERR(data->pinctrl)) {
			data->rts_gpio.default_state =
				pinctrl_lookup_state(data->pinctrl, "default");
			data->rts_gpio.gpio_state =
				pinctrl_lookup_state(data->pinctrl, "rts_gpio");
		} else {
			data->pinctrl = NULL;
			LOG("%s: dts does't define the uart rts iomux.\n",__func__);
			return -EINVAL;
		}
	} else {
		data->pinctrl = NULL;
		data->rts_gpio.io = -EINVAL;
		LOG("%s: uart_rts_gpios is no-in-use.\n", __func__);
	}

	gpio = of_get_named_gpio_flags(node, "BT,power_gpio", 0, &flags);
	if (gpio_is_valid(gpio)) {
		data->poweron_gpio.io = gpio;
		data->poweron_gpio.enable = (flags == GPIO_ACTIVE_HIGH) ? 1 : 0;
		LOG("%s: get property: BT,power_gpio = %d.\n", __func__, gpio);
	} else {
		data->poweron_gpio.io = -1;
	}
	gpio = of_get_named_gpio_flags(node, "BT,reset_gpio", 0, &flags);
	if (gpio_is_valid(gpio)) {
		data->reset_gpio.io = gpio;
		data->reset_gpio.enable = (flags == GPIO_ACTIVE_HIGH) ? 1 : 0;
		LOG("%s: get property: BT,reset_gpio = %d.\n", __func__, gpio);
	} else {
		data->reset_gpio.io = -1;
	}
	gpio = of_get_named_gpio_flags(node, "btm2,reset_gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		gpio_btm2 = gpio;
		gpio_direction_output(gpio_btm2, 0);
	} else {
		LOG("%s: get btm2,reset_gpio failed\n", __func__);
	}
	gpio = of_get_named_gpio_flags(node, "fm,reset_gpio", 0, &flags);
	if(gpio_is_valid(gpio)) {
		gpio_fm = gpio;
		gpio_direction_output(gpio_fm, 0);
	} else {
		LOG("%s: get fm,reset_gpio failed\n", __func__);
	}
	gpio = of_get_named_gpio_flags(node, "BT,wake_gpio", 0, &flags);
	if (gpio_is_valid(gpio)) {
		data->wake_gpio.io = gpio;
		data->wake_gpio.enable = (flags == GPIO_ACTIVE_HIGH) ? 1 : 0;
		LOG("%s: get property: BT,wake_gpio = %d.\n", __func__, gpio);
	} else {
		data->wake_gpio.io = -1;
	}
	gpio = of_get_named_gpio_flags(node, "BT,wake_host_irq", 0, &flags);
	if (gpio_is_valid(gpio)) {
		data->wake_host_irq.gpio.io = gpio;
		data->wake_host_irq.gpio.enable = flags;
		LOG("%s: get property: BT,wake_host_irq = %d.\n", __func__,
		    gpio);
	} else {
		data->wake_host_irq.gpio.io = -1;
	}

	return 0;
}
#endif //CONFIG_OF

static const struct file_operations bluesleep_lpm = {
	.owner = THIS_MODULE,
	.read = bluesleep_read_proc_lpm,
	.write = bluesleep_write_proc_lpm,
};

static const struct file_operations bluesleep_btwrite = {
	.owner = THIS_MODULE,
	.read = bluesleep_read_proc_btwrite,
	.write = bluesleep_write_proc_btwrite,
};

static int fm_open(struct inode *inode, struct file *file){
    return 0;
}

static ssize_t fm_write(struct file *file,
          const char __user *buf, size_t count, loff_t *ppos){
	printk(KERN_EMERG "fm write.\n");
	int ret = 0;
	ret = copy_from_user(writebuf, buf, 1);
	if(writebuf[0] == '1') {
		if(gpio_is_valid(gpio_fm)) {
			gpio_direction_output(gpio_fm, 1);
		}
	}
	else {
		if(gpio_is_valid(gpio_fm)) {
			gpio_direction_output(gpio_fm, 0);
		}
	}
	writebuf[0] = '0';
    return 0;
}

static struct file_operations fm_flops = {
    .owner  =  THIS_MODULE,
    .open   =  fm_open,
    .write  =  fm_write,
};

static int fm_add_devnode(dev_t minor)
{
	int ret;
	struct device *devp;

	if (!fm_cdev) {
		fm_cdev = cdev_alloc();
		if(NULL == fm_cdev){
			return -ENOMEM;
		}

		cdev_init(fm_cdev, &fm_flops);
	}

	if (!fm_devid) {
		ret = alloc_chrdev_region(&fm_devid, 0, FM_MAX_CHANNELS, "fm");
		if (ret)
			return ret;

		ret = cdev_add(fm_cdev, fm_devid, FM_MAX_CHANNELS);
		fm_major = MAJOR(fm_devid);
	}

	if (!fm_class) {
		fm_class = class_create(THIS_MODULE, "fm");
		if (IS_ERR(fm_class)) {
			return PTR_ERR(fm_class);
		}
	}

	devp = device_create(fm_class, NULL, MKDEV(fm_major, minor), NULL, "fm");
	if(IS_ERR(devp)){
		return PTR_ERR(devp);
	}

	return 0;
}

static int btm_open(struct inode *inode, struct file *file){
    return 0;
}

static ssize_t btm_write(struct file *file,
          const char __user *buf, size_t count, loff_t *ppos){
	printk(KERN_EMERG "btm2 write.\n");
	int ret = 0;
	ret = copy_from_user(writebuf, buf, 1);
	if(writebuf[0] == '1') {
		if(gpio_is_valid(gpio_btm2)) {
			gpio_direction_output(gpio_btm2, 1);
		}
	}
	else {
		if(gpio_is_valid(gpio_btm2)) {
			gpio_direction_output(gpio_btm2, 0);
		}
	}
	writebuf[0] = '0';
    return 0;
}

static struct file_operations btm2_flops = {
    .owner  =  THIS_MODULE,
    .open   =  btm_open,
    .write  =  btm_write,
};

static int btm2_add_devnode(dev_t minor)
{
	int ret;
	struct device *devp;

	if (!btm2_cdev) {
		btm2_cdev = cdev_alloc();
		if(NULL == btm2_cdev){
			return -ENOMEM;
		}

		cdev_init(btm2_cdev, &btm2_flops);
	}

	if (!btm2_devid) {
		ret = alloc_chrdev_region(&btm2_devid, 0, BTM2_MAX_CHANNELS, "btm2");
		if (ret)
			return ret;

		ret = cdev_add(btm2_cdev, btm2_devid, BTM2_MAX_CHANNELS);
		btm2_major = MAJOR(btm2_devid);
	}

	if (!btm2_class) {
		btm2_class = class_create(THIS_MODULE, "btm2");
		if (IS_ERR(btm2_class)) {
			return PTR_ERR(btm2_class);
		}
	}

	devp = device_create(btm2_class, NULL, MKDEV(btm2_major, minor), NULL, "btm2");
	if(IS_ERR(devp)){
		return PTR_ERR(devp);
	}

	return 0;
}

static int rfkill_probe(struct platform_device *pdev)
{
	struct rfkill_data *rfkill;
	struct rfkill_platform_data *pdata = pdev->dev.platform_data;
	int ret = 0;

	DBG("Enter %s\n", __func__);

	if (!pdata) {
		pdata = devm_kzalloc(&pdev->dev,
				     sizeof(struct rfkill_platform_data),
				     GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
#ifdef CONFIG_OF
		ret = bluetooth_platdata_parse_dt(&pdev->dev, pdata);
		if (ret < 0) {
			LOG("%s: No platform data specified\n", __func__);
			return ret;
		}
#endif
	}

	pdata->name = (char *)bt_name;
	pdata->type = RFKILL_TYPE_BLUETOOTH;

	rfkill = devm_kzalloc(&pdev->dev, sizeof(*rfkill), GFP_KERNEL);
	if (!rfkill)
		return -ENOMEM;

	rfkill->pdata = pdata;
	rfkill->pdev = pdev;
	g_rfkill = rfkill;

	bluetooth_dir = proc_mkdir("bluetooth", NULL);
	if (!bluetooth_dir) {
		LOG("Unable to create /proc/bluetooth directory");
		return -ENOMEM;
	}

	sleep_dir = proc_mkdir("sleep", bluetooth_dir);
	if (!sleep_dir) {
		LOG("Unable to create /proc/%s directory", PROC_DIR);
		return -ENOMEM;
	}

	DBG("init gpio\n");

	ret = rfkill_setup_gpio(pdev, &pdata->poweron_gpio, pdata->name,
				   "poweron");
	if (ret)
		goto fail_gpio;

	ret = rfkill_setup_gpio(pdev, &pdata->reset_gpio, pdata->name,
				   "reset");
	if (ret)
		goto fail_gpio;

	ret = rfkill_setup_gpio(pdev, &pdata->wake_gpio, pdata->name,
				   "wake");
	if (ret)
		goto fail_gpio;

	ret = rfkill_setup_gpio(pdev, &pdata->rts_gpio, rfkill->pdata->name,
				   "rts");
	if (ret)
		goto fail_gpio;

	ret = rfkill_setup_wake_irq(rfkill, 0);

	DBG("setup rfkill\n");
	rfkill->rfkill_dev = rfkill_alloc(pdata->name, &pdev->dev, pdata->type,
					  &rfkill_ops, rfkill);
	if (!rfkill->rfkill_dev)
		goto fail_alloc;

	rfkill_set_states(rfkill->rfkill_dev, BT_BLOCKED, false);
	ret = rfkill_register(rfkill->rfkill_dev);
	if (ret < 0)
              goto fail_rfkill;

	// bt turn off power
	if (gpio_is_valid(pdata->poweron_gpio.io)) {
		gpio_direction_output(pdata->poweron_gpio.io,
				      !pdata->poweron_gpio.enable);
	}
	if (gpio_is_valid(pdata->reset_gpio.io)) {
		gpio_direction_output(pdata->reset_gpio.io,
				      !pdata->reset_gpio.enable);
	}

	platform_set_drvdata(pdev, rfkill);

	LOG("%s device registered.\n", pdata->name);
	btm2_add_devnode(0);
	fm_add_devnode(0);
	return 0;

fail_rfkill:
	rfkill_destroy(rfkill->rfkill_dev);
fail_alloc:

	remove_proc_entry("btwrite", sleep_dir);
	remove_proc_entry("lpm", sleep_dir);

fail_gpio:

	g_rfkill = NULL;
	return ret;
}

static int rfkill_remove(struct platform_device *pdev)
{
	struct rfkill_data *rfkill = platform_get_drvdata(pdev);

	LOG("Enter %s\n", __func__);
	device_destroy(btm2_class, MKDEV(btm2_major, 0));
	class_destroy(btm2_class);
	unregister_chrdev_region(btm2_devid, BTM2_MAX_CHANNELS);
	cdev_del(btm2_cdev);
	device_destroy(fm_class, MKDEV(fm_major, 0));
	class_destroy(fm_class);
	unregister_chrdev_region(fm_devid, FM_MAX_CHANNELS);
	cdev_del(fm_cdev);

	cancel_delayed_work_sync(&rfkill->bt_sleep_delay_work);
	rfkill_unregister(rfkill->rfkill_dev);
	rfkill_destroy(rfkill->rfkill_dev);

	// free gpio
	if (gpio_is_valid(rfkill->pdata->rts_gpio.io))
		gpio_free(rfkill->pdata->rts_gpio.io);

	if (gpio_is_valid(rfkill->pdata->wake_host_irq.gpio.io)) {
		free_irq(rfkill->pdata->wake_host_irq.irq, rfkill);
	}

	if (gpio_is_valid(rfkill->pdata->reset_gpio.io))
		gpio_free(rfkill->pdata->reset_gpio.io);

	if (gpio_is_valid(rfkill->pdata->poweron_gpio.io))
		gpio_free(rfkill->pdata->poweron_gpio.io);
	clk_disable_unprepare(rfkill->pdata->ext_clk);

	g_rfkill = NULL;

	return 0;
}

static const struct dev_pm_ops rfkill_pm_ops = {
	.prepare = rfkill_pm_prepare,
	.complete = rfkill_pm_complete,
};

#ifdef CONFIG_OF
static struct of_device_id bt_platdata_of_match[] = {
	{ .compatible = "bluetooth-platdata" },
	{}
};
MODULE_DEVICE_TABLE(of, bt_platdata_of_match);
#endif //CONFIG_OF

static struct platform_driver rfkill_driver = {
	.probe = rfkill_probe,
	.remove = rfkill_remove,
	.driver = {
		.name = "rfkill_bt",
		.owner = THIS_MODULE,
		.pm = &rfkill_pm_ops,
        .of_match_table = of_match_ptr(bt_platdata_of_match),
	},
};

static int __init rfkill_init(void)
{
	LOG("Enter %s\n", __func__);
	return platform_driver_register(&rfkill_driver);
}

static void __exit rfkill_exit(void)
{
	LOG("Enter %s\n", __func__);
	platform_driver_unregister(&rfkill_driver);

}

module_init(rfkill_init);
module_exit(rfkill_exit);

MODULE_DESCRIPTION("siengine rfkill for Bluetooth v0.1");
MODULE_AUTHOR("Yuhang Zeng <Yuhang.Zeng@siengine.com>");
MODULE_LICENSE("GPL");
