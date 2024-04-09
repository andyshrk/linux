/**
 * dwc3-siengine.c - Siengine DWC3 Specific Glue layer
 *
 * Copyright (c) Siengine Co., Ltd.
 *
 * GNU General Public License for more details.
 */

#include <linux/async.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/usb/otg.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/extcon.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/reset-controller.h>
#include <linux/iopoll.h>

#include "core.h"
#include "io.h"
#include "../host/xhci.h"


#define  PERIPHERAL_DISCONNECT_TIMEOUT	1000000 /* us */
#define  WAIT_FOR_HCD_READY_TIMEOUT	5000000 /* us */
#define  XHCI_TSTCTRL_MASK		(0xf << 28)

struct dwc3_siengine {
	struct platform_device	*usb2_phy;
	struct platform_device	*usb3_phy;
	struct device		*dev;

	struct clk		*clk;
	struct clk		*usb3pipe_clk;
	struct clk		*usb3utmi_clk;
	struct reset_control	*reset;
	struct regulator	*vdd33;
	struct regulator	*vdd10;
	bool   			force_mode;
	struct extcon_dev	*edev;
	struct dwc3		*dwc;
	struct usb_hcd		*hcd;
	struct notifier_block	device_nb;
	struct notifier_block	host_nb;
	struct work_struct	otg_work;
	struct mutex		lock;

	bool			skip_suspend;
	bool			suspended;
	bool			reset_on_resume;
	bool			is_phy_on;
};


static int dwc3_se1000_remove_child(struct device *dev, void *unused)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}


static ssize_t dwc3_mode_show(struct device *device,
			struct device_attribute *attr, char *buf)
{
	struct dwc3_siengine	*se1000 = dev_get_drvdata(device);
	struct dwc3		*dwc = se1000->dwc;
	int			ret;

	switch (dwc->current_dr_role) {
	case USB_DR_MODE_HOST:
		ret = sprintf(buf, "host\n");
		break;
	case USB_DR_MODE_PERIPHERAL:
		ret = sprintf(buf, "device\n");
		break;
	case USB_DR_MODE_OTG:
		ret = sprintf(buf, "otg\n");
		break;
	default:
		ret = sprintf(buf, "UNKNOWN\n");
	}

	return ret;
}

static ssize_t dwc3_mode_store(struct device *device,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct dwc3_siengine	*se1000 = dev_get_drvdata(device);
	struct dwc3		*dwc = se1000->dwc;
	enum usb_dr_mode	new_dr_mode;

	if (dwc->dr_mode != USB_DR_MODE_OTG) {
		dev_err(se1000->dev, "Not support set mode!\n");
		return -EINVAL;
	}

	if (!strncmp(buf, "otg", 3)) {
		new_dr_mode = USB_DR_MODE_OTG;
	} else if (!strncmp(buf, "host", 4)) {
		new_dr_mode = USB_DR_MODE_HOST;
	} else if (!strncmp(buf, "device", 6)) {
		new_dr_mode = USB_DR_MODE_PERIPHERAL;
	} else {
		dev_info(se1000->dev, "illegal dr_mode\n");
		return count;
	}

	if (dwc->current_dr_role == new_dr_mode) {
		dev_info(se1000->dev, "Same with current %s mode\n",
			(new_dr_mode==USB_DR_MODE_HOST)? "host" : (new_dr_mode==USB_DR_MODE_PERIPHERAL)
			? "device" : "otg");
		return count;
	}

	se1000->force_mode = true;

    dwc3_set_mode(dwc, new_dr_mode);
	//schedule_work(&se1000->otg_work);
	//flush_work(&se1000->otg_work);


	dev_info(se1000->dev, "set new mode successfully\n");
	return count;
}

#if defined(CONFIG_USB_DWC3_HOST) || defined(CONFIG_USB_DWC3_DUAL_ROLE)
/**
 * dwc3_siengine_set_test_mode - Enables USB2/USB3 HOST Test Modes
 * @se1000: pointer to our context structure
 * @mode: the mode to set (U2: J, K SE0 NAK, Test_packet,
 * Force Enable; U3: Compliance mode)
 *
 * This function will return 0 on success or -EINVAL if wrong Test
 * Selector is passed.
 */
static int dwc3_siengine_set_test_mode(struct dwc3_siengine *se1000,
				       u32 mode)
{
	struct dwc3	*dwc = se1000->dwc;
	struct usb_hcd	*hcd  = dev_get_drvdata(&dwc->xhci->dev);
	struct xhci_hcd	*xhci = hcd_to_xhci(hcd);
	struct xhci_port	**port_array;
	int		ret, val;
	u32		reg;

	ret = readx_poll_timeout(readl, &hcd->state, val,
				 val != HC_STATE_HALT, 1000,
				 WAIT_FOR_HCD_READY_TIMEOUT);
	if (ret < 0) {
		dev_err(se1000->dev, "Wait for HCD ready timeout\n");
		return -EINVAL;
	}

	switch (mode) {
	case USB_TEST_J:
	case USB_TEST_K:
	case USB_TEST_SE0_NAK:
	case USB_TEST_PACKET:
	case USB_TEST_FORCE_ENABLE:
		//port_array = xhci->usb2_rhub.ports;
		//reg = readl(port_array[0]->addr + PORTPMSC);
		//reg &= ~XHCI_TSTCTRL_MASK;
		//reg |= mode << 28;
		//writel(reg, port_array[0]->addr + PORTPMSC);
		xhci_hub_control(xhci->main_hcd, SetPortFeature, USB_PORT_FEAT_TEST, \
		                 (mode<<8)|1, NULL, 0);
		break;
	case USB_SS_PORT_LS_COMP_MOD:
		//port_array = xhci->usb3_rhub.ports;
		//xhci_set_link_state(xhci, port_array[0], mode);
		xhci_hub_control(xhci->shared_hcd, SetPortFeature, USB_PORT_FEAT_LINK_STATE, \
		                 ((USB_SS_PORT_LS_COMP_MOD<<3)|1 ), NULL, 0);
		reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
		reg |=(1<<30);
		dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg );
		break;
	default:
		return -EINVAL;
	}

	dev_info(se1000->dev, "set USB HOST test mode successfully!\n");

	return 0;
}

static ssize_t host_testmode_show(struct device *device,
				  struct device_attribute *attr, char *buf)
{
	struct dwc3_siengine	*se1000 = dev_get_drvdata(device);
	struct dwc3		*dwc = se1000->dwc;
	struct usb_hcd		*hcd;
	struct xhci_hcd		*xhci;
    struct xhci_port	**port_array;
	u32			reg;
	int			ret;

	if (se1000->dwc->current_dr_role != USB_DR_MODE_HOST) {
		dev_warn(se1000->dev, " not enter host mode!\n");
		return 0;
	}

	hcd  = dev_get_drvdata(&dwc->xhci->dev);
	xhci = hcd_to_xhci(hcd);
	if (hcd->state == HC_STATE_HALT) {
		dev_warn(se1000->dev, "HOST is halted, set test mode first!\n");
		return 0;
	}

	port_array = xhci->usb2_rhub.ports;
	reg = readl(port_array[0]->addr + PORTPMSC);
	printk(" portpmsc vale is %x ", reg);
	reg &= XHCI_TSTCTRL_MASK;
	reg >>= 28;

	switch (reg) {
	case 0:
		ret = sprintf(buf, "U2: no test\n");
		break;
	case USB_TEST_J:
		ret = sprintf(buf, "U2: test_j\n");
		break;
	case USB_TEST_K:
		ret = sprintf(buf, "U2: test_k\n");
		break;
	case USB_TEST_SE0_NAK:
		ret = sprintf(buf, "U2: test_se0_nak\n");
		break;
	case USB_TEST_PACKET:
		ret = sprintf(buf, "U2: test_packet\n");
		break;
	case USB_TEST_FORCE_ENABLE:
		ret = sprintf(buf, "U2: test_force_enable\n");
		break;
	default:
		ret = sprintf(buf, "U2: UNKNOWN %d\n", reg);
	}

	port_array = xhci->usb3_rhub.ports;
	reg = readl(port_array[0]->addr);
	reg &= PORT_PLS_MASK;
	if (reg == USB_SS_PORT_LS_COMP_MOD)
		ret += sprintf(buf + ret, "U3: compliance mode\n");
	else
		ret += sprintf(buf + ret, "U3: UNKNOWN %d\n", reg >> 5);

	return ret;
}

static ssize_t host_testmode_store(struct device *device,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct dwc3_siengine		*se1000 = dev_get_drvdata(device);
	u32				testmode = 0;
	bool				flip = false;
	if (se1000->dwc->current_dr_role != USB_DR_MODE_HOST) {
		dev_warn(se1000->dev, " not enter host mode!\n");
		return count;
	}

	if (!strncmp(buf, "test_j", 6)) {
		testmode = USB_TEST_J;
	} else if (!strncmp(buf, "test_k", 6)) {
		testmode = USB_TEST_K;
	} else if (!strncmp(buf, "test_se0_nak", 12)) {
		testmode = USB_TEST_SE0_NAK;
	} else if (!strncmp(buf, "test_packet", 11)) {
		testmode = USB_TEST_PACKET;
	} else if (!strncmp(buf, "test_force_enable", 17)) {
		testmode = USB_TEST_FORCE_ENABLE;
	} else if (!strncmp(buf, "test_u3", 7)) {
		testmode = USB_SS_PORT_LS_COMP_MOD;
	} else if (!strncmp(buf, "test_flip_u3", 12)) {
		testmode = USB_SS_PORT_LS_COMP_MOD;
		flip = true;
	} else {
		dev_warn(se1000->dev, "Cmd not support! Try test_u3 or test_packet\n");
		return count;
	}

	dwc3_siengine_set_test_mode(se1000, testmode);

	return count;
}

static DEVICE_ATTR_RW(host_testmode);
#endif

static DEVICE_ATTR_RW(dwc3_mode);

static struct attribute *dwc3_siengine_attrs[] = {
	&dev_attr_dwc3_mode.attr,
#if defined(CONFIG_USB_DWC3_HOST) || defined(CONFIG_USB_DWC3_DUAL_ROLE)
	&dev_attr_host_testmode.attr,
#endif
	NULL,
};

static struct attribute_group dwc3_siengine_attr_group = {
	.name = NULL,	/* we want them in the same directory */
	.attrs = dwc3_siengine_attrs,
};

static int dwc3_siengine_device_notifier(struct notifier_block *nb,
					 unsigned long event, void *ptr)
{
	struct dwc3_siengine *se1000 =
		container_of(nb, struct dwc3_siengine, device_nb);

	if (!se1000->suspended)
		schedule_work(&se1000->otg_work);

	return NOTIFY_DONE;
}

static int dwc3_siengine_host_notifier(struct notifier_block *nb,
				       unsigned long event, void *ptr)
{
	struct dwc3_siengine *se1000 =
		container_of(nb, struct dwc3_siengine, host_nb);

	if (!se1000->suspended)
		schedule_work(&se1000->otg_work);

	return NOTIFY_DONE;
}

static void dwc3_siengine_otg_extcon_evt_work(struct work_struct *work)
{
	struct dwc3_siengine	*se1000 =
		container_of(work, struct dwc3_siengine, otg_work);
	struct dwc3		*dwc = se1000->dwc;
	struct extcon_dev	*edev = se1000->edev;
	struct usb_hcd		*hcd;
	struct xhci_hcd		*xhci;
	unsigned long		flags;
	int			ret;
	int			val;
	u32			reg;
	u32			count = 0;

	dev_info(se1000->dev, "dwc3_siengine_otg_extcon_evt_work\n");

	mutex_lock(&se1000->lock);

	if (se1000->force_mode ? dwc->desired_dr_role == USB_DR_MODE_PERIPHERAL :
	    extcon_get_state(edev, EXTCON_USB)) {

		/*
		 * If dr_mode is host only, never to set
		 * the mode to the peripheral mode.
		 */
		if (dwc->dr_mode == USB_DR_MODE_HOST) {
			dev_warn(se1000->dev, "USB peripheral not support!\n");
			goto out;
		}

		/*
		 * Assert otg reset can put the dwc in P2 state, it's
		 * necessary operation prior to phy power on. However,
		 * asserting the otg reset may affect dwc chip operation.
		 * The reset will clear all of the dwc controller registers.
		 * So we need to reinit the dwc controller after deassert
		 * the reset. We use pm runtime to initialize dwc controller.
		 * Also, there are no synchronization primitives, meaning
		 * the dwc3 core code could at least in theory access chip
		 * registers while the reset is asserted, with unknown impact.
		 */
		if (!se1000->skip_suspend) {

			/* Wait until dwc3 core resume from PM suspend */
			while (dwc->dev->power.is_suspended) {
				if (++count > 1000) {
					dev_err(se1000->dev,
						"wait for dwc3 core resume timeout!\n");
						goto out;
				}
				usleep_range(100, 200);
			}

			pm_runtime_get_sync(se1000->dev);
			pm_runtime_get_sync(dwc->dev);
		} else {
			se1000->skip_suspend = false;
		}

		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);

		dev_info(se1000->dev, "USB peripheral connected\n");
	} else if (se1000->force_mode ? dwc->desired_dr_role == USB_DR_MODE_HOST :
		   extcon_get_state(edev, EXTCON_USB_HOST)) {

		if (se1000->skip_suspend) {
			pm_runtime_put(dwc->dev);
			pm_runtime_put(se1000->dev);
			se1000->skip_suspend = false;
		}

		/*
		 * If dr_mode is device only, never to
		 * set the mode to the host mode.
		 */
		if (dwc->dr_mode == USB_DR_MODE_PERIPHERAL) {
			dev_warn(se1000->dev, "USB HOST not support!\n");
			goto out;
		}

		/*
		 * In usb3 phy init, it will access usb3 module, so we need
		 * to resume rockchip dev before phy init to make sure usb3
		 * pd is enabled.
		 */
		pm_runtime_get_sync(se1000->dev);

		/*
		 * Don't abort on errors. If powering on a phy fails,
		 * we still need to init dwc controller and add the
		 * HCDs to avoid a crash when unloading the driver.
		 */
		ret = phy_power_on(dwc->usb2_generic_phy);
		if (ret < 0)
			dev_err(dwc->dev, "Failed to power on usb2 phy\n");

		ret = phy_power_on(dwc->usb3_generic_phy);
		if (ret < 0) {
			phy_power_off(dwc->usb2_generic_phy);
			dev_err(dwc->dev, "Failed to power on usb3 phy\n");
		} else {
			se1000->is_phy_on = true;
		}

		pm_runtime_get_sync(dwc->dev);


		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);

		/*
		 * The following sleep helps to ensure that inserted USB3
		 * Ethernet devices are discovered if already inserted
		 * when booting.
		 */
		usleep_range(10000, 11000);

		dev_info(se1000->dev, "USB HOST connected\n");
	} else if(se1000->force_mode ? dwc->desired_dr_role == USB_DR_MODE_OTG: false) {
		//if(dwc->current_dr_role != dwc->desired_dr_role)
		//	dwc->current_dr_role = DWC3_GCTL_PRTCAP_OTG;

		dwc3_otg_init(dwc);
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_OTG);


		dev_info(se1000->dev, "USB unconnected\n");
	}

	dev_info(se1000->dev, "dwc3_siengine_otg_extcon_evt_work\n");

out:
	mutex_unlock(&se1000->lock);
}

static int dwc3_siengine_get_extcon_dev(struct dwc3_siengine *se1000)
{
	struct device		*dev = se1000->dev;
	struct extcon_dev	*edev;

	if (device_property_read_bool(dev, "extcon")) {
		edev = extcon_get_edev_by_phandle(dev, 0);
		if (IS_ERR(edev)) {
			if (PTR_ERR(edev) != -EPROBE_DEFER)
				dev_err(dev, "couldn't get extcon device\n");
			return PTR_ERR(edev);
		}

		se1000->device_nb.notifier_call =
				dwc3_siengine_device_notifier;
		se1000->host_nb.notifier_call =
				dwc3_siengine_host_notifier;
		se1000->edev = edev;
	}

	return 0;
}


static void dwc3_siengine_async_probe(void *data, async_cookie_t cookie)
{
	struct dwc3_siengine	*se1000 = data;
	struct device		*dev = se1000->dev;
	struct dwc3		*dwc = se1000->dwc;
	int			ret;

	mutex_lock(&se1000->lock);

	if (se1000->edev) {
		ret = devm_extcon_register_notifier(dev, se1000->edev,
						    EXTCON_USB,
						    &se1000->device_nb);
		if (ret < 0) {
			dev_err(dev, "fail to register notifier for USB Dev\n");
			goto err;
		}

		ret = devm_extcon_register_notifier(dev, se1000->edev,
						    EXTCON_USB_HOST,
						    &se1000->host_nb);
		if (ret < 0) {
			dev_err(dev, "fail to register notifier for USB HOST\n");
			goto err;
		}
	}

	ret = sysfs_create_group(&dev->kobj, &dwc3_siengine_attr_group);
	if (ret)
		dev_err(dev, "failed to create sysfs group: %d\n", ret);

err:
	mutex_unlock(&se1000->lock);
}


static int dwc3_se1000_probe(struct platform_device *pdev)
{
	struct dwc3_siengine	*se1000;
	struct device		*dev = &pdev->dev;
	struct device_node	*node = dev->of_node, *child;
	struct platform_device	*child_pdev;

	int			ret;

	se1000 = devm_kzalloc(dev, sizeof(*se1000), GFP_KERNEL);
	if (!se1000)
		return -ENOMEM;

	platform_set_drvdata(pdev, se1000);

	se1000->dev = dev;

	se1000->clk = devm_clk_get(dev, "usb3axi");
	if (IS_ERR(se1000->clk)) {
		dev_err(dev, "couldn't get clock usb3axi\n");
		return -EINVAL;
	}
	ret = clk_prepare_enable(se1000->clk);
	if (ret)
		return ret;

	se1000->usb3pipe_clk = devm_clk_get(dev, "usb3phy");
	if (IS_ERR(se1000->usb3pipe_clk)) {
		se1000->usb3pipe_clk = NULL;
		dev_err(dev, "couldn't get clock usb3phy\n");
		return -EINVAL;
	}

	ret = clk_prepare_enable(se1000->usb3pipe_clk);
	if (ret)
		goto susp_clk_err;

	se1000->usb3utmi_clk = devm_clk_get(dev, "usb3utmi");
	if (IS_ERR(se1000->usb3utmi_clk)) {
		se1000->usb3utmi_clk = NULL;
		dev_err(dev, "couldn't get clock usb3utim\n");
		return -EINVAL;
	}
	ret = clk_prepare_enable(se1000->usb3utmi_clk);
	if (ret)
		goto susp_clk_err;

	se1000->reset = devm_reset_control_get(dev, "por");
	if (IS_ERR(se1000->reset)) {
		se1000->reset = NULL;
		dev_err(dev, "couldn't get por reset\n");
		return -EINVAL;
	}
	ret = reset_control_deassert(se1000->reset);
	if (ret) {
		dev_err(dev, "%s reset deassert failed\n",
			se1000->reset);
		goto err_rst;
	}

	se1000->vdd33 = devm_regulator_get(dev, "vdd33");
	if (IS_ERR(se1000->vdd33)) {
		ret = PTR_ERR(se1000->vdd33);
		goto vdd33_err;
	}
	ret = regulator_enable(se1000->vdd33);
	if (ret) {
		dev_err(dev, "Failed to enable VDD33 supply\n");
		goto vdd33_err;
	}

	se1000->vdd10 = devm_regulator_get(dev, "vdd10");
	if (IS_ERR(se1000->vdd10)) {
		ret = PTR_ERR(se1000->vdd10);
		goto vdd10_err;
	}
	ret = regulator_enable(se1000->vdd10);
	if (ret) {
		dev_err(dev, "Failed to enable VDD10 supply\n");
		goto vdd10_err;
	}


	INIT_WORK(&se1000->otg_work, dwc3_siengine_otg_extcon_evt_work);

	if (node) {

		child = of_get_child_by_name(node, "dwc3");
		if (!child) {
			dev_err(dev, "failed to find dwc3 core node\n");
			ret = -ENODEV;
			goto err1;
		}

		ret = of_platform_populate(node, NULL, NULL, dev);
		if (ret) {
			dev_err(dev, "failed to add dwc3 core\n");
			goto populate_err;
		}

		child_pdev = of_find_device_by_node(child);
		if (!child_pdev) {
			dev_err(dev, "failed to find dwc3 core device\n");
			ret = -ENODEV;
			goto err2;
		}

		se1000->dwc = platform_get_drvdata(child_pdev);
		if (!se1000->dwc) {
			dev_err(dev, "cannot to get drvdata dwc3, deferred probe\n");
			ret = -EPROBE_DEFER;
			goto err2;
		}

	} else {
		dev_err(dev, "no device node, failed to add dwc3 core\n");
		ret = -ENODEV;
		goto populate_err;
	}

	async_schedule(dwc3_siengine_async_probe, se1000);

	return 0;

err2:
	of_platform_depopulate(dev);
err1:
populate_err:
	platform_device_unregister(se1000->usb2_phy);
	platform_device_unregister(se1000->usb3_phy);
phys_err:
	regulator_disable(se1000->vdd10);
vdd10_err:
	regulator_disable(se1000->vdd33);
vdd33_err:
	clk_disable_unprepare(se1000->usb3utmi_clk);
axius_clk_err:
	clk_disable_unprepare(se1000->usb3pipe_clk);
susp_clk_err:
	clk_disable_unprepare(se1000->clk);
err_rst:
	return ret;
}

static int dwc3_se1000_remove(struct platform_device *pdev)
{
	struct dwc3_siengine	*se1000 = platform_get_drvdata(pdev);

	device_for_each_child(&pdev->dev, NULL, dwc3_se1000_remove_child);
	platform_device_unregister(se1000->usb2_phy);
	platform_device_unregister(se1000->usb3_phy);

	clk_disable_unprepare(se1000->usb3utmi_clk);
	clk_disable_unprepare(se1000->usb3pipe_clk);
	clk_disable_unprepare(se1000->clk);

	regulator_disable(se1000->vdd33);
	regulator_disable(se1000->vdd10);

	return 0;
}

static const struct of_device_id se1000_dwc3_match[] = {
	{ .compatible = "siengine,se1000-dwusb31" },
	{},
};
MODULE_DEVICE_TABLE(of, se1000_dwc3_match);

#ifdef CONFIG_PM_SLEEP
static int dwc3_se1000_suspend(struct device *dev)
{
	struct dwc3_siengine *se1000 = dev_get_drvdata(dev);

	clk_disable(se1000->usb3utmi_clk);
	clk_disable(se1000->usb3pipe_clk);
	clk_disable(se1000->clk);

	regulator_disable(se1000->vdd33);
	regulator_disable(se1000->vdd10);
	return 0;
}

static int dwc3_se1000_resume(struct device *dev)
{
	struct dwc3_siengine *se1000 = dev_get_drvdata(dev);
	struct dwc3 *dwc = se1000->dwc;
	int ret;

	ret = regulator_enable(se1000->vdd33);
	if (ret) {
		dev_err(dev, "Failed to enable VDD33 supply\n");
		return ret;
	}

	ret = regulator_enable(se1000->vdd10);
	if (ret) {
		dev_err(dev, "Failed to enable VDD10 supply\n");
		return ret;
	}

	clk_enable(se1000->clk);
	clk_enable(se1000->usb3pipe_clk);
	clk_enable(se1000->usb3utmi_clk);

	dwc->ulpi_ready = 0;
	dwc->phys_ready = 0;
	reset_control_deassert(se1000->reset);

	/* runtime set active to reflect active state. */
	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	return 0;
}

static const struct dev_pm_ops dwc3_se1000_dev_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(dwc3_se1000_suspend, dwc3_se1000_resume)
};

#define DEV_PM_OPS	(&dwc3_se1000_dev_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver dwc3_se1000_driver = {
	.probe		= dwc3_se1000_probe,
	.remove		= dwc3_se1000_remove,
	.driver		= {
		.name	= "siengine-usb3",
		.of_match_table = se1000_dwc3_match,
		.pm	= DEV_PM_OPS,
	},
};

module_platform_driver(dwc3_se1000_driver);

MODULE_AUTHOR("frank <frank@siengine.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 SE1000 Glue Layer");
