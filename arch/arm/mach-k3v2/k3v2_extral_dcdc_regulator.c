
#include <mach/system.h>
#include <mach/platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/slab.h>
#include <linux/err.h>

#include <mach/gpio.h>


#define VCC_EXTRAL_DCDC_2V85    0
#define VCC_EXTRAL_DCDC_1V2    	1
#define VCC_EXTRAL_DCDC_MAX		2

typedef struct vcc_extral_dcdc_dev_info{
		struct resource			*res[VCC_EXTRAL_DCDC_MAX];
		struct regulator_dev	*rdev[VCC_EXTRAL_DCDC_MAX];
		unsigned int en_gpio[VCC_EXTRAL_DCDC_MAX];
};

static int vcc_extral_dcdc_2v85_is_enabled(struct regulator_dev *dev)
{
	int  regulator_id = VCC_EXTRAL_DCDC_MAX;
	struct vcc_extral_dcdc_dev_info *di = NULL;
	regulator_id = rdev_get_id(dev);
	di = rdev_get_drvdata(dev);

	if(!di || regulator_id >= VCC_EXTRAL_DCDC_MAX)
		return -EINVAL;

	return gpio_get_value(di->en_gpio[regulator_id]);
}

static int vcc_extral_dcdc_2v85_enable(struct regulator_dev *dev)
{
	int  regulator_id = VCC_EXTRAL_DCDC_MAX;
	struct vcc_extral_dcdc_dev_info *di = NULL;
	regulator_id =  rdev_get_id(dev);
	di = rdev_get_drvdata(dev);

	if(!di || regulator_id >= VCC_EXTRAL_DCDC_MAX)
		return -EINVAL;

	gpio_set_value(di->en_gpio[regulator_id],1);
    return 0;
}

static int vcc_extral_dcdc_2v85_disable(struct regulator_dev *dev)
{
	int  regulator_id = VCC_EXTRAL_DCDC_MAX;
	struct vcc_extral_dcdc_dev_info *di = NULL;
	regulator_id =  rdev_get_id(dev);
	di = rdev_get_drvdata(dev);

	if(!di || regulator_id >= VCC_EXTRAL_DCDC_MAX)
		return -EINVAL;

	gpio_set_value(di->en_gpio[regulator_id],0);
    return 0;
}

struct regulator_ops vcc_extral_dcdc_2v85_ops = {
	.is_enabled = vcc_extral_dcdc_2v85_is_enabled,
	.enable = vcc_extral_dcdc_2v85_enable,
	.disable = vcc_extral_dcdc_2v85_disable,
};

static int vcc_extral_dcdc_1v2_is_enabled(struct regulator_dev *dev)
{
	int  regulator_id = VCC_EXTRAL_DCDC_MAX;
	struct vcc_extral_dcdc_dev_info *di = NULL;
	regulator_id =  rdev_get_id(dev);
	di = rdev_get_drvdata(dev);

	if(!di || regulator_id >= VCC_EXTRAL_DCDC_MAX)
		return -EINVAL;

	return gpio_get_value(di->en_gpio[regulator_id]);
}

static int vcc_extral_dcdc_1v2_enable(struct regulator_dev *dev)
{
	int  regulator_id = VCC_EXTRAL_DCDC_MAX;
	struct vcc_extral_dcdc_dev_info *di = NULL;
	regulator_id =  rdev_get_id(dev);
	di = rdev_get_drvdata(dev);

	if(!di || regulator_id >= VCC_EXTRAL_DCDC_MAX)
		return -EINVAL;

	gpio_set_value(di->en_gpio[regulator_id],1);
    return 0;
}

static int vcc_extral_dcdc_1v2_disable(struct regulator_dev *dev)
{
	int  regulator_id = VCC_EXTRAL_DCDC_MAX;
	struct vcc_extral_dcdc_dev_info *di = NULL;
	regulator_id =  rdev_get_id(dev);
	di = rdev_get_drvdata(dev);

	if(!di || regulator_id >= VCC_EXTRAL_DCDC_MAX)
		return -EINVAL;

	gpio_set_value(di->en_gpio[regulator_id],0);
    return 0;
}

struct regulator_ops vcc_extral_dcdc_1v2_ops = {
	.is_enabled = vcc_extral_dcdc_1v2_is_enabled,
	.enable = vcc_extral_dcdc_1v2_enable,
	.disable = vcc_extral_dcdc_1v2_disable,
};

struct regulator_desc extral_dcdc_regulator_desc[] = {
	{
		.name = "VCC_EXTRAL_DCDC_2V85",
		.id = VCC_EXTRAL_DCDC_2V85,
		.ops = &vcc_extral_dcdc_2v85_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "VCC_EXTRAL_DCDC_1V2",
		.id = VCC_EXTRAL_DCDC_1V2,
		.ops = &vcc_extral_dcdc_1v2_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static __devinit int vcc_extral_dcdc_regulator_probe(struct platform_device *pdev)
{
	int ret = 0, regulator_id = 0, icount = 0;
	struct vcc_extral_dcdc_dev_info* di = NULL;

	/*get initialized data*/
	struct regulator_init_data *regulator_init_data = pdev->dev.platform_data;

	/*alloc dev info data mem*/
	di = kzalloc(sizeof(struct vcc_extral_dcdc_dev_info), GFP_KERNEL);
	if (di == NULL) {
		pr_err("Regulator vcc extral dcdc: kzalloc mem fail, please check!\n");
		return -ENOMEM;
	}

	for (regulator_id = VCC_EXTRAL_DCDC_2V85; regulator_id < VCC_EXTRAL_DCDC_MAX; regulator_id++) {
		di->res[regulator_id] = platform_get_resource(pdev, IORESOURCE_IO, regulator_id);
		if(!di->res[regulator_id])
		{
			ret = -EINVAL;
			goto free_mem;
		}
		else
		{
			di->en_gpio[regulator_id] = di->res[regulator_id]->start;
			ret = gpio_request(di->en_gpio[regulator_id],di->res[regulator_id]->name);
			if(ret)
			{
				for(icount = 0; icount < regulator_id; icount++)
					gpio_free(di->en_gpio[icount]);

				goto free_mem;
			}
			else
				gpio_direction_output(di->en_gpio[regulator_id],0);
		}
	}

	for (regulator_id = VCC_EXTRAL_DCDC_2V85; regulator_id < VCC_EXTRAL_DCDC_MAX; regulator_id++) {
		di->rdev[regulator_id] = regulator_register(&extral_dcdc_regulator_desc[regulator_id], &pdev->dev,
			regulator_init_data + regulator_id, di);
		if (IS_ERR(di->rdev[regulator_id])) {
			goto free_mem;
		}

	}

	platform_set_drvdata(pdev, di);
	return 0;

free_mem:
	kfree(di);
	di = NULL;
	return ret;
}

static __devexit int vcc_extral_dcdc_regulator_remove(struct platform_device *pdev)
{
	int ret = 0, regulator_id = 0;
	struct vcc_extral_dcdc_dev_info* di = NULL;
	di = platform_get_drvdata(pdev);
	if(!di)
		return ret;

	for (regulator_id = VCC_EXTRAL_DCDC_2V85; regulator_id <= VCC_EXTRAL_DCDC_MAX; regulator_id++) {
		if (!IS_ERR(di->rdev[regulator_id])) {
			regulator_unregister(di->rdev[regulator_id]);
		}
		gpio_free(di->en_gpio[regulator_id]);
	}

	kfree(di);
	return 0;
}

#ifdef CONFIG_PM
static int vcc_extral_dcdc_regulator_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}
static int vcc_extral_dcdc_regulator_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static struct platform_driver vcc_extral_dcdc_regulator_driver = {
	.probe = vcc_extral_dcdc_regulator_probe,
	.remove = vcc_extral_dcdc_regulator_remove,
	#ifdef CONFIG_PM
	.suspend = vcc_extral_dcdc_regulator_suspend,
	.resume = vcc_extral_dcdc_regulator_resume,
	#endif
	.driver		= {
		.name	= "vcc-extral-dcdc-regulators",
	},
};
static int __init vcc_extral_dcdc_regulator_init(void)
{
	return platform_driver_register(&vcc_extral_dcdc_regulator_driver);
}
static void __exit vcc_extral_dcdc_regulator_exit(void)
{
	platform_driver_unregister(&vcc_extral_dcdc_regulator_driver);
}

fs_initcall(vcc_extral_dcdc_regulator_init);
module_exit(vcc_extral_dcdc_regulator_exit);
MODULE_LICENSE("GPL");


