/*
 * Copyright 2015-16 Golden Delicious Computers
 *
 * Author: Nikolaus Schaller <hns@goldelico.com>
 *
 * Based on leds-tca6507.c
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * LED driver for the IS31FL319{0,1,3,6,9} to drive 1, 3, 6 or 9 light
 * effect LEDs.
 *
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slab.h>

/* register numbers */
#define IS31FL319X_SHUTDOWN	0x00
#define IS31FL319X_CTRL1	0x01
#define IS31FL319X_CTRL2	0x02
#define IS31FL319X_CONFIG1	0x03
#define IS31FL319X_CONFIG2	0x04
#define IS31FL319X_RAMP_MODE	0x05
#define IS31FL319X_BREATH_MASK	0x06
#define IS31FL319X_PWM1		0x07
#define IS31FL319X_PWM2		0x08
#define IS31FL319X_PWM3		0x09
#define IS31FL319X_PWM4		0x0a
#define IS31FL319X_PWM5		0x0b
#define IS31FL319X_PWM6		0x0c
#define IS31FL319X_PWM7		0x0d
#define IS31FL319X_PWM8		0x0e
#define IS31FL319X_PWM9		0x0f
#define IS31FL319X_DATA_UPDATE	0x10
#define IS31FL319X_T0_1		0x11
#define IS31FL319X_T0_2		0x12
#define IS31FL319X_T0_3		0x13
#define IS31FL319X_T0_4		0x14
#define IS31FL319X_T0_5		0x15
#define IS31FL319X_T0_6		0x16
#define IS31FL319X_T0_7		0x17
#define IS31FL319X_T0_8		0x18
#define IS31FL319X_T0_9		0x19
#define IS31FL319X_T123_1	0x1a
#define IS31FL319X_T123_2	0x1b
#define IS31FL319X_T123_3	0x1c
#define IS31FL319X_T4_1		0x1d
#define IS31FL319X_T4_2		0x1e
#define IS31FL319X_T4_3		0x1f
#define IS31FL319X_T4_4		0x20
#define IS31FL319X_T4_5		0x21
#define IS31FL319X_T4_6		0x22
#define IS31FL319X_T4_7		0x23
#define IS31FL319X_T4_8		0x24
#define IS31FL319X_T4_9		0x25
#define IS31FL319X_TIME_UPDATE	0x26
#define IS31FL319X_RESET	0xff

#define IS31FL319X_REG_CNT	(IS31FL319X_RESET + 1)

#define NUM_LEDS 9	/* max for 3199 chip */

#define LED_MAX_MICROAMP_UPPER_LIMIT ((u32) 40000)
#define LED_MAX_MICROAMP_LOWER_LIMIT ((u32) 5000)
#define LED_MAX_MICROAMP_DEFAULT ((u32) 20000)
#define LED_MAX_MICROAMP_STEP ((u32) 5000)

#define AUDIO_GAIN_DB_MAX ((u32) 21)

/*
 * regmap is used as a cache of chip's register space,
 * to avoid reading back brightness values from chip,
 * which is known to hang.
 */
struct is31fl319x_chip {
	struct i2c_client	*client;
	struct regmap		*regmap;
	struct mutex            lock;
	u32                     audio_gain_db;

	struct is31fl319x_led {
		struct is31fl319x_chip	*chip;
		struct led_classdev	cdev;
		u32                     max_microamp;
		bool                    configured;
	} leds[NUM_LEDS];
};

static const struct i2c_device_id is31fl319x_id[] = {
	{ "is31fl3190", 1 },
	{ "is31fl3191", 1 },
	{ "is31fl3193", 3 },
	{ "is31fl3196", 6 },
	{ "is31fl3199", 9 },
	{ "sn3199", 9 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, is31fl319x_id);

static int is31fl319x_brightness_set(struct led_classdev *cdev,
				   enum led_brightness brightness)
{
	struct is31fl319x_led *led = container_of(cdev, struct is31fl319x_led,
						  cdev);
	struct is31fl319x_chip *is31 = led->chip;
	int chan = led - is31->leds;
	int ret;
	int i;
	u8 ctrl1 = 0, ctrl2 = 0;

	dev_dbg(&is31->client->dev, "%s %d: %d\n", __func__, chan, brightness);

	mutex_lock(&is31->lock);

	/* update PWM register */
	ret = regmap_write(is31->regmap, IS31FL319X_PWM1 + chan, brightness);
	if (ret < 0)
		goto out;

	/* read current brightness of all PWM channels */
	for (i = 0; i < NUM_LEDS; i++) {
		unsigned int pwm_value;
		bool on;

		/*
		 * since neither cdev nor the chip can provide
		 * the current setting, we read from the regmap cache
		 */

		ret = regmap_read(is31->regmap, IS31FL319X_PWM1 + i,
				  &pwm_value);
		dev_dbg(&is31->client->dev, "%s read %d: ret=%d: %d\n",
			__func__, i, ret, pwm_value);
		on = ret >= 0 && pwm_value > LED_OFF;

		if (i < 3)
			ctrl1 |= on << i;	/* 0..2 => bit 0..2 */
		else if (i < 6)
			ctrl1 |= on << (i+1);	/* 3..5 => bit 4..6 */
		else
			ctrl2 |= on << (i-6);	/* 6..8 => bit 0..2 */
	}

	if (ctrl1 > 0 || ctrl2 > 0) {
		dev_dbg(&is31->client->dev, "power up %02x %02x\n",
			ctrl1, ctrl2);
		regmap_write(is31->regmap, IS31FL319X_CTRL1, ctrl1);
		regmap_write(is31->regmap, IS31FL319X_CTRL2, ctrl2);
		/* update PWMs */
		regmap_write(is31->regmap, IS31FL319X_DATA_UPDATE, 0x00);
		/* enable chip from shut down */
		ret = regmap_write(is31->regmap, IS31FL319X_SHUTDOWN, 0x01);
	} else {
		dev_dbg(&is31->client->dev, "power down\n");
		/* shut down (no need to clear CTRL1/2) */
		ret = regmap_write(is31->regmap, IS31FL319X_SHUTDOWN, 0x00);
	}

out:
	mutex_unlock(&is31->lock);

	return ret;
}

static int is31fl319x_parse_child_dt(const struct device *dev,
				     const struct device_node *child,
				     struct is31fl319x_led *led)
{
	struct led_classdev *cdev = &led->cdev;
	int ret;

	if (of_property_read_string(child, "label", &cdev->name))
		cdev->name = child->name;

	cdev->default_trigger = NULL;
	ret = of_property_read_string(child, "linux,default-trigger",
		&cdev->default_trigger);
	if (ret < 0 && ret != -EINVAL)	/* is optional */
		return ret;

	led->max_microamp = LED_MAX_MICROAMP_DEFAULT;
	ret = of_property_read_u32(child, "led-max-microamp",
				   &led->max_microamp);
	if (!ret) {
		led->max_microamp = clamp(led->max_microamp,
					  LED_MAX_MICROAMP_LOWER_LIMIT,
					  LED_MAX_MICROAMP_UPPER_LIMIT);
		led->max_microamp -= led->max_microamp % LED_MAX_MICROAMP_STEP;
	}

	return 0;
}

static int is31fl319x_parse_dt(struct device *dev,
			       struct is31fl319x_chip *is31)
{
	struct device_node *np = dev->of_node, *child;
	struct led_info *is31_leds;
	int count;
	int ret;

	if (!np)
		return -ENODEV;

	count = of_get_child_count(np);

	dev_dbg(dev, "child count %d\n", count);
	if (!count || count > NUM_LEDS)
		return -ENODEV;

	is31_leds = devm_kzalloc(dev, sizeof(struct led_info) * NUM_LEDS,
				 GFP_KERNEL);
	if (!is31_leds)
		return -ENOMEM;

	for_each_child_of_node(np, child) {
		struct is31fl319x_led *led;
		u32 reg;

		ret = of_property_read_u32(child, "reg", &reg);
		if (ret)
			break;

		if (reg < 1 || reg > NUM_LEDS) {
			dev_err(dev, "invalid led reg %u\n", reg);
			ret = -EINVAL;
			break;
		}

		led = &is31->leds[reg - 1];

		if (led->configured) {
			dev_err(dev, "led %u is already configured\n", reg);
			ret = -EINVAL;
			break;
		}

		ret = is31fl319x_parse_child_dt(dev, child, led);
		if (ret) {
			dev_err(dev, "led %u DT parsing failed\n", reg);
			break;
		}

		led->configured = true;
	}
	if (ret) {
		dev_err(dev, "DT child nodes parsing failed, error %d\n", ret);
		return ret;
	}

	is31->audio_gain_db = 0;
	ret = of_property_read_u32(np, "audio-gain-db", &is31->audio_gain_db);
	if (!ret) {
		is31->audio_gain_db = min(is31->audio_gain_db,
					  AUDIO_GAIN_DB_MAX);
		is31->audio_gain_db -= is31->audio_gain_db % 3;
	}

	return 0;
}

static const struct of_device_id of_is31fl319x_leds_match[] = {
	{ .compatible = "issi,is31fl3190", (void *) 1 },
	{ .compatible = "issi,is31fl3191", (void *) 1 },
	{ .compatible = "issi,is31fl3193", (void *) 3 },
	{ .compatible = "issi,is31fl3196", (void *) 6 },
	{ .compatible = "issi,is31fl3199", (void *) 9 },
	{ .compatible = "si-en,sn3199", (void *) 9 },
	{},
};
MODULE_DEVICE_TABLE(of, of_is31fl319x_leds_match);

static bool is31fl319x_readable_reg(struct device *dev, unsigned int reg)
{ /* we have no readable registers */
	return false;
}

static bool is31fl319x_volatile_reg(struct device *dev, unsigned int reg)
{ /* volatile registers are not cached */
	switch (reg) {
	case IS31FL319X_DATA_UPDATE:
	case IS31FL319X_TIME_UPDATE:
	case IS31FL319X_RESET:
		return true;	/* always write-through */
	default:
		return false;
	}
}

static const struct reg_default is31fl319x_reg_defaults[] = {
	{ IS31FL319X_PWM1, 0x00},
	{ IS31FL319X_PWM2, 0x00},
	{ IS31FL319X_PWM3, 0x00},
	{ IS31FL319X_PWM4, 0x00},
	{ IS31FL319X_PWM5, 0x00},
	{ IS31FL319X_PWM6, 0x00},
	{ IS31FL319X_PWM7, 0x00},
	{ IS31FL319X_PWM8, 0x00},
	{ IS31FL319X_PWM9, 0x00},
};

static struct regmap_config regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = IS31FL319X_REG_CNT,
	.cache_type = REGCACHE_FLAT,
	.readable_reg = is31fl319x_readable_reg,
	.volatile_reg = is31fl319x_volatile_reg,
	.reg_defaults = is31fl319x_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(is31fl319x_reg_defaults),
};

static int is31fl319x_microamp_to_cs(u32 microamp)
{
	switch (microamp) {
	default:
		WARN(1, "Invalid microamp setting");
	case 20000: return 0;
	case 15000: return 1;
	case 10000: return 2;
	case 5000:  return 3;
	case 40000: return 4;
	case 35000: return 5;
	case 30000: return 6;
	case 25000: return 7;
	}
}

static int is31fl319x_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct is31fl319x_chip *is31;
	struct i2c_adapter *adapter;
	int err;
	int i = 0;
	u32 aggregated_led_microamp = LED_MAX_MICROAMP_UPPER_LIMIT;

	adapter = to_i2c_adapter(client->dev.parent);

	dev_dbg(&client->dev, "probe IS31FL319x for num_leds = %d\n",
		(int) id->driver_data);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	is31 = devm_kzalloc(&client->dev, sizeof(*is31), GFP_KERNEL);
	if (!is31)
		return -ENOMEM;

	mutex_init(&is31->lock);

	err = is31fl319x_parse_dt(&client->dev, is31);
	if (err) {
		dev_err(&client->dev, "DT parsing error %d\n", err);
		return err;
	}

	is31->client = client;
	is31->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(is31->regmap)) {
		dev_err(&client->dev, "failed to allocate register map\n");
		return PTR_ERR(is31->regmap);
	}

	i2c_set_clientdata(client, is31);

	/* check for write-reply from chip (we can't read any registers) */
	err = regmap_write(is31->regmap, IS31FL319X_RESET, 0x00);
	if (err < 0) {
		dev_err(&client->dev, "no response from chip write: err = %d\n",
			err);
		return -EIO;	/* does not answer */
	}

	regmap_write(is31->regmap, IS31FL319X_CTRL1, 0x00);
	regmap_write(is31->regmap, IS31FL319X_CTRL2, 0x00);

	/*
	 * Kernel conventions require per-LED led-max-microamp property.
	 * But the chip does not allow to limit individual LEDs.
	 * So we take minimum from all subnodes.
	 */
	for (i = 0; i < NUM_LEDS; i++)
		if (is31->leds[i].configured &&
		    is31->leds[i].max_microamp < aggregated_led_microamp)
			aggregated_led_microamp = is31->leds[i].max_microamp;

	regmap_write(is31->regmap, IS31FL319X_CONFIG2,
		     is31fl319x_microamp_to_cs(aggregated_led_microamp) << 4 |
		     is31->audio_gain_db / 3);

	for (i = 0; i < NUM_LEDS; i++) {
		struct is31fl319x_led *led = &is31->leds[i];

		if (!led->configured)
			continue;

		led->chip = is31;
		led->cdev.brightness_set_blocking = is31fl319x_brightness_set;

		err = devm_led_classdev_register(&client->dev, &led->cdev);
		if (err < 0)
			return err;
	}

	return 0;
}

static struct i2c_driver is31fl319x_driver = {
	.driver   = {
		.name    = "leds-is31fl319x",
		.of_match_table = of_match_ptr(of_is31fl319x_leds_match),
	},
	.probe    = is31fl319x_probe,
	.id_table = is31fl319x_id,
};

module_i2c_driver(is31fl319x_driver);

MODULE_AUTHOR("H. Nikolaus Schaller <hns@goldelico.com>");
MODULE_AUTHOR("Andrey Utkin <andrey_utkin@fastmail.com>");
MODULE_DESCRIPTION("IS31FL319X LED driver");
MODULE_LICENSE("GPL v2");
