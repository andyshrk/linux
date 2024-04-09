#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <asm/io.h>

#define SWITCH_CNT		1

#define SWITCHDEV_IOCIVIMODE (_IO(0XEF, 1))
#define SWITCHDEV_IOCRVCMODE (_IO(0XEF, 2))

#define SWITCHDEV_IOC_MAXNR	3

#define SYSCON
/*misc reg*/
#define MDP_MISC_IVI_REQ	0x214
#define MDP_MISC_RVC_REQ	0x218
#define MDP_MISC_DPU0_VTOTAL	0x21c
#define MDP_MISC_DPU0_HTOTAL	0x220
#define MDP_MISC_DPU0_VFP	0x24c
#define MDP_MISC_DPU0_HFP	0x250
/*dpu reg*/
#define CONFIG_SWITCH0
#define DOU0_BS			0x1E00
#define DOU1_BS			0x3E00
#define BS_ACTIVESIZE		0x0E0
#define BS_HINTERVALS		0x0E4
#define BS_VINTERVALS		0x0E8
#define BS_SYNC			0x0EC
#define DOU0_CONTROL_BS		0x1800
#define DOU1_CONTROL_BS		0x3800
#define DOU_STATUS		0x0B0
#define DOU_STATUS_ACTIVE_BIT	31

enum SWITCH_MODE {
	IVI_MODE = 0,
	RVC_MODE,
};

struct display_parameter{
	u32 htotal;
	u32 hfp;
	u32 vtotal;
	u32 vfp;
};

struct switch_dev {
	dev_t devid;			/*device id	*/
	struct cdev cdev;		/*cdev		*/
	struct class *class;		/*class		*/
	struct device *device;		/*device	*/
	struct platform_device *pdev;
	int major;			/*major devid	*/
	int minor;			/*minor devid	*/
	struct regmap* misc_base;
	struct regmap* dpu_base_ivi;
	struct regmap* dpu_base_rvc;
	u32 flag;			/*dsi/dp switch	*/
	struct dentry *debugfs_root;
	struct mutex lock;
	struct display_parameter dp_ivi;
	struct display_parameter dp_rvc;
	u32 ready;
};

char* switch_name[] = {"switch0", "switch1"};

static inline u32 switch_read_syscon(struct regmap *base, u32 reg)
{
	u32 value;
	regmap_read(base, reg, &value);

	return value;
}

static inline void switch_write_syscon(struct regmap *base, u32 reg, u32 value)
{
	regmap_write(base, reg, value);
}

static int get_disp_param(struct display_parameter *dp, struct regmap *base, u32 dou_bs)
{
	u32 val;
	u32 hdisplay, hfp, hsw, hbp;
	u32 vdisplay, vfp, vsw, vbp;
	struct switch_dev *switchdev = container_of(dp, struct switch_dev, dp_ivi);
	struct device *dev = &switchdev->pdev->dev;

	if ((!dp) || (!base)) {
		dev_err(dev, "invalid parameters\n");
		return -EINVAL;
	}

	memset(dp, 0, sizeof(*dp));

	val = switch_read_syscon(base, dou_bs + BS_ACTIVESIZE);
	hdisplay = val & 0x1fff;
	vdisplay = (val >> 16) & 0x1fff;

	val = switch_read_syscon(base, dou_bs + BS_HINTERVALS);
	hfp = val & 0xfff;
	hbp = (val >> 16) & 0x3ff;

	val = switch_read_syscon(base, dou_bs + BS_VINTERVALS);
	vfp = val & 0x3fff;
	vbp = (val >> 16) & 0xff;

	val = switch_read_syscon(base, dou_bs + BS_SYNC);
	hsw = val & 0x3ff;
	vsw = (val >> 16) & 0xff;

	dp->htotal = hdisplay + hfp + hsw + hbp;
	dp->hfp = hfp;
	dp->vtotal = vdisplay + vfp + vsw + vbp;
	dp->vfp = vfp;

	dev_info(dev, "get display parameters\n");
	dev_info(dev, "hdisplay:%d, hfp:%d, hsw:%d, hbp:%d, vdisplay:%d, vfp:%d, vsw:%d, vbp:%d\n",
		hdisplay, hfp, hsw, hbp, vdisplay, vfp, vsw, vbp);

	return 0;
}

static int disp_param_init(struct switch_dev *switchdev)
{
	u32 dou_bs_ivi = DOU0_BS;
	u32 dou_bs_rvc = (switchdev->flag == 0) ? DOU0_BS : DOU1_BS;
	u32 dou_control_bs_ivi = DOU0_CONTROL_BS;
	u32 dou_control_bs_rvc = (switchdev->flag == 0) ? DOU0_CONTROL_BS : DOU1_CONTROL_BS;
	struct display_parameter *dp_ivi = &switchdev->dp_ivi;
	struct display_parameter *dp_rvc = &switchdev->dp_rvc;
	struct device *dev = &switchdev->pdev->dev;

	if (!(switch_read_syscon(switchdev->dpu_base_ivi, dou_control_bs_ivi + DOU_STATUS) &
		(1 << DOU_STATUS_ACTIVE_BIT))) {
		dev_warn(dev, "DOU_STATUS_ACTIVE of ivi pipe not ready");
		return -EBUSY;
	}
	if (!(switch_read_syscon(switchdev->dpu_base_rvc, dou_control_bs_rvc + DOU_STATUS) &
		(1 << DOU_STATUS_ACTIVE_BIT))) {
		dev_warn(dev, "DOU_STATUS_ACTIVE of ivi pipe not ready");
		return -EBUSY;
	}

	if (get_disp_param(dp_ivi, switchdev->dpu_base_ivi, dou_bs_ivi) < 0) {
		dev_err(dev, "get_disp_param of ivi pipe failed");
		return -EBUSY;
	}

	if (get_disp_param(dp_rvc, switchdev->dpu_base_rvc, dou_bs_rvc) < 0) {
		dev_err(dev, "get_disp_param of rvc pipe failed");
		return -EBUSY;
	}

	if (memcmp(dp_ivi, dp_rvc, sizeof(struct display_parameter)) != 0) {
		dev_warn(dev, "not ready, disp param of ivi and rvc don't match\n");
		return -EBUSY;
	}
	usleep_range(16667*2, 16667*2 + 50);
	dev_info(dev, "Init display parameters complete\n");

	switchdev->ready = 1;
	return 0;
}

static void disp_param_config(struct switch_dev *switchdev)
{
	u32 val;
	struct regmap* misc_base = switchdev->misc_base;
	struct display_parameter *dp = &switchdev->dp_ivi;
	struct device *dev = &switchdev->pdev->dev;

	val = switch_read_syscon(misc_base, MDP_MISC_DPU0_HTOTAL);
	if (val != dp->htotal)
		goto config;

	val = switch_read_syscon(misc_base, MDP_MISC_DPU0_VTOTAL);
	if (val != dp->vtotal)
		goto config;

	val = switch_read_syscon(misc_base, MDP_MISC_DPU0_HFP);
	if (val != dp->hfp)
		goto config;

	val = switch_read_syscon(misc_base, MDP_MISC_DPU0_VFP);
	if (val != dp->vfp)
		goto config;

	dev_info(dev, "display parameters has already been configured\n");
	return;

config:
	switch_write_syscon(misc_base, MDP_MISC_DPU0_HTOTAL, dp->htotal);
	switch_write_syscon(misc_base, MDP_MISC_DPU0_VTOTAL, dp->vtotal);
	switch_write_syscon(misc_base, MDP_MISC_DPU0_HFP, dp->hfp);
	switch_write_syscon(misc_base, MDP_MISC_DPU0_VFP, dp->vfp);

	dev_info(dev, "Config display parameters complete\n");
	return;
}

static bool switch_is_idle(struct switch_dev *switchdev)
{
	struct regmap* misc_base = switchdev->misc_base;
	return !(switch_read_syscon(misc_base, MDP_MISC_IVI_REQ) |
			switch_read_syscon(misc_base, MDP_MISC_RVC_REQ));
}

void ivi_rvc_switch(struct switch_dev *switchdev, u8 switch_mode)
{
	struct device *dev = &switchdev->pdev->dev;
	u8 cur_mode;
	struct regmap* misc_base = switchdev->misc_base;
	u32 flag = switchdev->flag;

	cur_mode = switch_read_syscon(misc_base, MDP_MISC_RVC_REQ);
	cur_mode = cur_mode >> flag;

	if (cur_mode == switch_mode){
		dev_info(dev, "%s has been already set to mode %d\n", switch_name[flag],
			switch_mode);
		return;
	}

	mutex_lock(&switchdev->lock);
	disp_param_config(switchdev);
	if (switch_mode == IVI_MODE){
		switch_write_syscon(misc_base, MDP_MISC_IVI_REQ,
			switch_read_syscon(misc_base, MDP_MISC_IVI_REQ) | (0x1<<flag));
		switch_write_syscon(misc_base, MDP_MISC_RVC_REQ,
			switch_read_syscon(misc_base, MDP_MISC_RVC_REQ) & ~(0x1<<flag));
		dev_info(dev, "%s is switched to IVI mode\n", switch_name[flag]);
	} else if (switch_mode == RVC_MODE){
		switch_write_syscon(misc_base, MDP_MISC_RVC_REQ,
			switch_read_syscon(misc_base, MDP_MISC_RVC_REQ) | (0x1<<flag));
		switch_write_syscon(misc_base, MDP_MISC_IVI_REQ,
			switch_read_syscon(misc_base, MDP_MISC_IVI_REQ) & ~(0x1<<flag));
		dev_info(dev, "%s is switched to RVC mode\n", switch_name[flag]);
	} else {
		dev_info(dev, "Invalid switch mode.\n");
	}
	mutex_unlock(&switchdev->lock);
}

static int switch_open(struct inode *inode, struct file *filp)
{
	struct switch_dev *switchdev = container_of(inode->i_cdev, struct switch_dev, cdev);
	if (!switchdev)
		return -ENODEV;

	get_device(switchdev->device);
	filp->private_data = switchdev;

	return nonseekable_open(inode, filp);
}

static int switch_release(struct inode *inode, struct file *filp)
{
	struct switch_dev *switchdev = container_of(inode->i_cdev, struct switch_dev, cdev);

	put_device(switchdev->device);

	return 0;
}

/*IO operation*/
static long switchdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	struct switch_dev *switchdev = filp->private_data;

	if (_IOC_NR(cmd) > SWITCHDEV_IOC_MAXNR)
		return -EINVAL;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok((void *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok((void *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	if (!switch_is_idle(switchdev)) {
		if (!(switchdev->ready)) {
			if (disp_param_init(switchdev) < 0)
				return -EFAULT;
		}
	}

	switch (cmd) {
	case SWITCHDEV_IOCIVIMODE:
		ivi_rvc_switch(switchdev, IVI_MODE);
		break;
	case SWITCHDEV_IOCRVCMODE:
		ivi_rvc_switch(switchdev, RVC_MODE);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/* file operation structure */
static struct file_operations switch_fops =
{
	.owner = THIS_MODULE,
	.open = switch_open,
	.release = switch_release,
	.unlocked_ioctl = switchdev_ioctl,
};

#ifdef CONFIG_DEBUG_FS
static ssize_t switch_debug_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
	char infor[256];
	struct switch_dev *switchdev = filp->private_data;
	u32 ivi_req, rvc_req, dpu0_vtotal, dpu0_htotal, dpu0_vfp, dpu0_hfp;
	u32 len;

	ivi_req = switch_read_syscon(switchdev->misc_base, MDP_MISC_IVI_REQ);
	rvc_req = switch_read_syscon(switchdev->misc_base, MDP_MISC_RVC_REQ);
	dpu0_vtotal = switch_read_syscon(switchdev->misc_base, MDP_MISC_DPU0_VTOTAL);
	dpu0_htotal = switch_read_syscon(switchdev->misc_base, MDP_MISC_DPU0_HTOTAL);
	dpu0_vfp = switch_read_syscon(switchdev->misc_base, MDP_MISC_DPU0_VFP);
	dpu0_hfp = switch_read_syscon(switchdev->misc_base, MDP_MISC_DPU0_HFP);

	len = snprintf(infor, sizeof(infor), "switch reg info\n\n"
			"MDP_MISC_IVI_REQ:0x%x\n"
			"MDP_MISC_RVC_REQ:0x%x\n"
			"MDP_MISC_DPU0_VTOTAL:0x%x\n"
			"MDP_MISC_DPU0_HTOTAL:0x%x\n"
			"MDP_MISC_DPU0_VFP:0x%x\n"
			"MDP_MISC_DPU0_HFP:0x%x\n",
			ivi_req, rvc_req, dpu0_vtotal,
			dpu0_htotal, dpu0_vfp, dpu0_hfp);
	return simple_read_from_buffer(buf, cnt, offt, infor, len);
}

static struct file_operations switch_debug_fops =
{
	.open = simple_open,
	.read = switch_debug_read,
};
#endif

static int switch_parse_dt(struct switch_dev *switchdev)
{
	int ret;
	struct device_node *np_temp;
	struct device *dev = &switchdev->pdev->dev;
	u32 val;

	if (!dev->of_node){
		dev_err(dev, "mdp_switch pdev->dev.of_node is NULL\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(dev->of_node, "idx", &val);
	if (ret){
		dev_err(dev, "Filed to get switchdev->flag\n");
		return -EINVAL;
	} else {
		dev_info(dev, "switchdev->flag:%d\n", switchdev->flag);
	}

	if (val != 0 && val != 1) {
		dev_err(dev, "Invalid switch device flag\n");
		return -EINVAL;
	}
	switchdev->flag = val;

	np_temp = of_parse_phandle(dev->of_node, "ctl-syscon", 0);
	switchdev->misc_base = syscon_node_to_regmap(np_temp);
	of_node_put(np_temp);

	np_temp = of_parse_phandle(dev->of_node, "ctl-syscon", 1);
	switchdev->dpu_base_ivi = syscon_node_to_regmap(np_temp);
	of_node_put(np_temp);

	np_temp = of_parse_phandle(dev->of_node, "ctl-syscon", 2);
	switchdev->dpu_base_rvc = syscon_node_to_regmap(np_temp);
	of_node_put(np_temp);

	return 0;
}

static int switch_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct switch_dev *switchdev;

	switchdev = devm_kzalloc(dev, sizeof(*switchdev), GFP_KERNEL);
	if (!switchdev)
		return -EINVAL;
	switchdev->pdev = pdev;

	ret = switch_parse_dt(switchdev);
	if (ret < 0)
		goto error1;

	ret = alloc_chrdev_region(&switchdev->devid, 0, SWITCH_CNT, switch_name[switchdev->flag]);
	if (ret)
		goto error1;

	switchdev->major = MAJOR(switchdev->devid);
	switchdev->minor = MINOR(switchdev->devid);

	/*register device*/
	cdev_init(&switchdev->cdev, &switch_fops);
	ret = cdev_add(&switchdev->cdev, switchdev->devid, SWITCH_CNT);
	if (ret)
		goto error_cdev;

	/*3. create class*/
	switchdev->class = class_create(THIS_MODULE, switch_name[switchdev->flag]);
	if (IS_ERR(switchdev->class)) {
		dev_err(dev, KERN_ERR "Error creating switch class\n");
		ret = PTR_ERR(switchdev->class);
		goto error_class1;
	}

	/*4. create device*/
	switchdev->device = device_create(switchdev->class, NULL, switchdev->devid, NULL,
		switch_name[switchdev->flag]);
	if (IS_ERR(switchdev->device)) {
		dev_err(dev, KERN_ERR "Error creating switch device\n");
		ret = -1;
		goto error_class2;
	}

	dev_set_drvdata(dev, switchdev);
	mutex_init(&switchdev->lock);

#ifdef CONFIG_DEBUG_FS
	if (debugfs_initialized()) {
		switchdev->debugfs_root = debugfs_create_dir(switch_name[switchdev->flag], NULL);
		if (IS_ERR(switchdev->debugfs_root))
			switchdev->debugfs_root = NULL;
	}
	if (!switchdev->debugfs_root)
		dev_info(dev, "debugfs_root is NULL\n");
	else
		debugfs_create_file("register", S_IFREG | S_IRUGO | S_IWUSR,
			switchdev->debugfs_root, switchdev, &switch_debug_fops);
#endif
	return 0;

error_class2:
	device_destroy(switchdev->class, switchdev->devid);

error_class1:
	cdev_del(&switchdev->cdev);

error_cdev:
	unregister_chrdev_region(switchdev->devid, SWITCH_CNT);

error1:
	return ret;
}

static int __exit switch_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct switch_dev *switchdev = dev_get_drvdata(dev);
	/*unregister char device driver*/
	cdev_del(&switchdev->cdev);
	unregister_chrdev_region(switchdev->devid, SWITCH_CNT);

	device_destroy(switchdev->class, switchdev->devid);
	class_destroy(switchdev->class);

#ifdef CONFIG_DEBUG_FS
	if (switchdev->debugfs_root) {
		debugfs_remove_recursive(switchdev->debugfs_root);
		switchdev->debugfs_root = NULL;
	}
#endif
	return 0;
}

#ifdef CONFIG_PM
static int switch_runtime_suspend(struct device *dev)
{
	return 0;
}

static int switch_runtime_resume(struct device *dev)
{
	struct switch_dev *switchdev = dev_get_drvdata(dev);

	switchdev->ready = 0;
	return 0;
}

#else
static int switch_runtime_suspend(struct device *dev)
{
	return 0;
}

static int switch_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops switch_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(switch_runtime_suspend,
						switch_runtime_resume)
};

static const struct of_device_id switch_of_match[] = {
	{ .compatible = "siengine,se1000-switch" },
	{/*Sentinel*/}
};

static struct platform_driver switch_driver = {
	.driver	= {
		.name	= "se1000-mdp-switch",
		.pm = &switch_pm_ops,
		.of_match_table = switch_of_match,
	},
	.probe	= switch_probe,
	.remove	= switch_remove,
};

static int __init switch_driver_init(void)
{
	return platform_driver_register(&switch_driver);
}

static void __exit switch_driver_exit(void)
{
	platform_driver_unregister(&switch_driver);
}

module_init(switch_driver_init);
module_exit(switch_driver_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("huimin.tian@siengine.com");
