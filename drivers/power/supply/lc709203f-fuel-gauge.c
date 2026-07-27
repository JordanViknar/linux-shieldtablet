// SPDX-License-Identifier: GPL-2.0-only
/*
 * Linux power_supply driver for the onsemi (ON Semiconductor) LC709203F
 * battery fuel gauge, connected over I2C.
 *
 * Originally made by JordanViknar <jordanviknar@gmail.com> for linux-tn8
 * (Linux for the NVIDIA SHIELD Tablet)
 * Some elements of this driver were influenced by the downstream kernel
 * for the tablet.
 *
 * ... and yes I was helped out by Claude quite a bit.
 * Give me a break, this is my first Linux driver ever.
 *
 * Ported from a Zephyr RTOS driver:
 *   Copyright (c) 2025 Philipp Steiner <philipp.steiner1987@gmail.com>
 *   (Apache-2.0)
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "lc709203f-fuel-gauge.h"

/*
 * Basic register accessors.
 *
 * The LC709203F protects every SMBus word transaction with a PEC
 * (Packet Error Checking) byte -- a CRC-8 (poly 0x07, init 0) computed
 * over the address/command/data bytes of the transaction. This is
 * standard SMBus PEC, so rather than hand-rolling the CRC like the
 * Zephyr driver does, we just set I2C_CLIENT_PEC on the client in
 * probe() and let the I2C core's smbus emulation generate/verify it
 * for us on every call below.
 */
static int lc709203f_read_word(struct lc709203f_chip *chip, u8 reg)
{
	int ret = i2c_smbus_read_word_data(chip->client, reg);

	if (ret < 0)
		dev_dbg(&chip->client->dev, "read reg 0x%02x failed: %d\n", reg, ret);

	return ret;
}

static int lc709203f_write_word(struct lc709203f_chip *chip, u8 reg, u16 value)
{
	int ret = i2c_smbus_write_word_data(chip->client, reg, value);

	if (ret < 0)
		dev_err(&chip->client->dev, "write reg 0x%02x failed: %d\n", reg, ret);

	return ret;
}

/*
 * The LC709203F has no way to sense charge current itself -- that's
 * why boards route a shunt/ADC current sense through "io-channels"
 * instead. When that channel exists we use its sign to tell charging
 * from discharging; without it we fall back to the standard
 * power_supply_am_i_supplied() trick used by e.g. generic-adc-battery.
 */
static int lc709203f_get_status(struct power_supply *psy, struct lc709203f_chip *chip,
				 int soc, union power_supply_propval *val)
{
	int ret, current_ua;

	if (chip->current_channel) {
		ret = iio_read_channel_processed(chip->current_channel, &current_ua);
		if (ret < 0) {
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			return 0;
		}
		current_ua *= 1000; /* mA -> uA */

		if (current_ua > LC709203F_CHARGE_CURRENT_THRESH_UA)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (current_ua < -LC709203F_CHARGE_CURRENT_THRESH_UA)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (soc >= 100)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

		return 0;
	}

	/* No current sense available -- infer from whether we're supplied at all */
	ret = power_supply_am_i_supplied(psy);
	if (ret < 0)
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
	else if (!ret)
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (soc >= 100)
		val->intval = POWER_SUPPLY_STATUS_FULL;
	else
		val->intval = POWER_SUPPLY_STATUS_CHARGING;

	return 0;
}

static int lc709203f_get_property(struct power_supply *psy, enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct lc709203f_chip *chip = power_supply_get_drvdata(psy);
	int ret, raw;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_HEALTH:
		/* The chip doesn't measure pack wear/impedance, only charge. */
		val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "LC709203F";
		return 0;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "onsemi";
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		raw = lc709203f_read_word(chip, LC709203F_REG_CELL_VOLTAGE);
		if (raw < 0)
			return raw;
		val->intval = raw * 1000; /* mV -> uV */
		return 0;
	case POWER_SUPPLY_PROP_CAPACITY:
		raw = lc709203f_read_word(chip, LC709203F_REG_RSOC);
		if (raw < 0)
			return raw;
		val->intval = raw; /* chip already reports 0-100 directly, no rescaling */
		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		if (!chip->has_thermistor)
			return -ENODATA;
		raw = lc709203f_read_word(chip, LC709203F_REG_CELL_TEMPERATURE);
		if (raw < 0)
			return raw;
		val->intval = raw - 2732; /* deci-Kelvin -> deci-Celsius */
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!chip->current_channel)
			return -ENODATA;
		ret = iio_read_channel_processed(chip->current_channel, &val->intval);
		if (ret < 0)
			return ret;
		val->intval *= 1000; /* mA -> uA */
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		raw = lc709203f_read_word(chip, LC709203F_REG_RSOC);
		if (raw < 0)
			return raw;
		return lc709203f_get_status(psy, chip, raw, val);
	default:
		return -EINVAL;
	}
}

/*
 * IRQ handler: called (threaded, no hard-IRQ half) when the battery
 * alarm fires, i.e. RSOC or cell voltage dropped below the configured
 * threshold. We don't need to read back which condition tripped it --
 * just tell the power_supply core to re-poll and let userspace see
 * the new state.
 */
static irqreturn_t lc709203f_irq_handler(int irq, void *data)
{
	struct lc709203f_chip *chip = data;

	dev_warn(&chip->client->dev, "battery alarm triggered (low RSOC or low cell voltage)\n");
	power_supply_changed(chip->psy);
	return IRQ_HANDLED;
}

/*
 * Polling work function: called periodically to update the power supply state.
 */
static void lc709203f_poll_work(struct work_struct *work)
{
	struct lc709203f_chip *chip = container_of(to_delayed_work(work),
						    struct lc709203f_chip, poll_work);

	power_supply_changed(chip->psy);
	schedule_delayed_work(&chip->poll_work, LC709203F_POLL_INTERVAL);
}

/*
 * Called when the external power source is connected or disconnected,
 * so we refresh state (and thus STATUS) immediately instead of waiting
 * up to LC709203F_POLL_INTERVAL for the next scheduled poll.
 */
static void lc709203f_external_power_changed(struct power_supply *psy)
{
	struct lc709203f_chip *chip = power_supply_get_drvdata(psy);

	mod_delayed_work(system_wq, &chip->poll_work, 0);
}

/*
 * All of these are the "onsemi,*" properties used across the
 * onsemi,lc709203f downstream Linux tree lineage. Your node sets
 * thermistor-beta and alert-low-rsoc/voltage; initial-rsoc,
 * appli-adjustment and battery-profile are supported too (same
 * convention, other boards in that lineage use them) but stay at
 * safe no-op defaults if you don't add them.
 */
static void lc709203f_parse_dt(struct i2c_client *client, struct lc709203f_chip *chip)
{
	struct device_node *np = client->dev.of_node;

	of_property_read_u32(np, "onsemi,alert-low-rsoc", &chip->alert_low_rsoc);
	of_property_read_u32(np, "onsemi,alert-low-voltage", &chip->alert_low_voltage);

	chip->has_thermistor = !of_property_read_u32(np, "onsemi,thermistor-beta",
						      &chip->thermistor_beta);

	of_property_read_u32(np, "onsemi,initial-rsoc", &chip->initial_rsoc);
	of_property_read_u32(np, "onsemi,appli-adjustment", &chip->appli_adjustment);
	of_property_read_u32(np, "onsemi,battery-profile", &chip->battery_profile);
}

/*
 * Hardware init sequence. Order matters here: APA (pack size),
 * battery profile and thermistor config all feed the chip's internal
 * RSOC model, so they need to be in place *before* we trigger a
 * quickstart RSOC recalculation -- otherwise the recalculation runs
 * against default/incorrect pack parameters. The Zephyr driver this
 * was ported from does the quickstart write last for the same reason;
 * keep that ordering here.
 */
static int lc709203f_hw_init(struct lc709203f_chip *chip)
{
	int ret;

	ret = lc709203f_write_word(chip, LC709203F_REG_ALARM_LOW_RSOC, chip->alert_low_rsoc);
	if (ret < 0)
		return ret;

	ret = lc709203f_write_word(chip, LC709203F_REG_ALARM_LOW_VOLTAGE, chip->alert_low_voltage);
	if (ret < 0)
		return ret;

	if (chip->appli_adjustment) {
		ret = lc709203f_write_word(chip, LC709203F_REG_APA, chip->appli_adjustment);
		if (ret < 0)
			return ret;
	}

	ret = lc709203f_write_word(chip, LC709203F_REG_BAT_PROFILE, chip->battery_profile);
	if (ret < 0)
		return ret;

	if (chip->has_thermistor) {
		ret = lc709203f_write_word(chip, LC709203F_REG_STATUS_BIT,
					    LC709203F_TEMP_MODE_THERMISTOR);
		if (ret < 0)
			return ret;

		ret = lc709203f_write_word(chip, LC709203F_REG_THERMISTOR_B, chip->thermistor_beta);
		if (ret < 0)
			return ret;
	}

	/*
	 * Quickstart: chip->initial_rsoc is just a "do it or don't" flag
	 * from devicetree (onsemi,initial-rsoc). The value actually
	 * written must be the fixed LC709203F_INIT_RSOC_VAL magic word --
	 * anything else is not a recognized command and won't trigger a
	 * recalculation. Left unset (0) by default.
	 */
	if (chip->initial_rsoc) {
		ret = lc709203f_write_word(chip, LC709203F_REG_INITIAL_RSOC,
					    LC709203F_INIT_RSOC_VAL);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static const struct power_supply_desc lc709203f_psy_desc = {
	.name = "lc709203f-gauge",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = lc709203f_get_property,
	/* .properties and .num_properties are set in probe(), based on optional features */
};

/*
 * Probe function: initializes the LC709203F fuel gauge
 */
static int lc709203f_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = { };
	struct power_supply_desc *psy_desc;
	struct lc709203f_chip *chip;
	enum power_supply_property *props;
	int nprops, ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	i2c_set_clientdata(client, chip);

	/*
	 * Word reads/writes are protected by an SMBus PEC byte (CRC-8,
	 * polynomial 0x07) -- the same CRC-8 the Zephyr driver computed
	 * by hand. Setting I2C_CLIENT_PEC lets the I2C core generate and
	 * verify it for us via the plain smbus word calls below.
	 */
	client->flags |= I2C_CLIENT_PEC;

	/* Sanity-check that something is actually on the bus at this address. */
	ret = lc709203f_read_word(chip, LC709203F_REG_NUM_PARAM);
	if (ret < 0)
		return dev_err_probe(dev, ret, "device did not respond on I2C\n");

	lc709203f_parse_dt(client, chip);

	ret = lc709203f_hw_init(chip);
	if (ret < 0)
		return dev_err_probe(dev, ret, "hardware init failed\n");

	/* Optional battery-current IIO channel; absence is not an error. */
	chip->current_channel = devm_iio_channel_get(dev, "battery-current");
	if (IS_ERR(chip->current_channel)) {
		ret = PTR_ERR(chip->current_channel);
		chip->current_channel = NULL;
		if (ret != -ENODEV)
			return dev_err_probe(dev, ret, "failed to get battery-current IIO channel\n");
	}

	/* Build the properties array: base set, plus whichever optional features we found. */
	nprops = ARRAY_SIZE(lc709203f_props_base) + (chip->has_thermistor ? 1 : 0) +
		 (chip->current_channel ? 1 : 0);
	props = devm_kcalloc(dev, nprops, sizeof(*props), GFP_KERNEL);
	if (!props)
		return -ENOMEM;

	memcpy(props, lc709203f_props_base, sizeof(lc709203f_props_base));
	nprops = ARRAY_SIZE(lc709203f_props_base);
	if (chip->has_thermistor)
		props[nprops++] = POWER_SUPPLY_PROP_TEMP;
	if (chip->current_channel)
		props[nprops++] = POWER_SUPPLY_PROP_CURRENT_NOW;

	psy_desc = devm_kmemdup(dev, &lc709203f_psy_desc, sizeof(*psy_desc), GFP_KERNEL);
	if (!psy_desc)
		return -ENOMEM;
	psy_desc->properties = props;
	psy_desc->num_properties = nprops;
	psy_desc->external_power_changed = lc709203f_external_power_changed;

	psy_cfg.drv_data = chip;
	psy_cfg.fwnode = dev_fwnode(dev);

	chip->psy = devm_power_supply_register(dev, psy_desc, &psy_cfg);
	if (IS_ERR(chip->psy))
		return dev_err_probe(dev, PTR_ERR(chip->psy), "failed to register power supply\n");

	INIT_DELAYED_WORK(&chip->poll_work, lc709203f_poll_work);
	schedule_delayed_work(&chip->poll_work, LC709203F_POLL_INTERVAL);

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL, lc709203f_irq_handler,
						 IRQF_ONESHOT, dev_name(dev), chip);
		if (ret < 0)
			dev_warn(dev, "failed to request alarm IRQ: %d (alerts disabled)\n", ret);
		else
			device_set_wakeup_capable(dev, true);
	}

	dev_info(dev, "LC709203F fuel gauge registered (%s thermistor, %s current sense)\n",
		 chip->has_thermistor ? "with" : "without",
		 chip->current_channel ? "with" : "without");

	return 0;
}

/*
 * Remove function: cancels the polling work
 */
static void lc709203f_remove(struct i2c_client *client)
{
	struct lc709203f_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&chip->poll_work);
}

/*
 * Power management functions
 */
static int __maybe_unused lc709203f_suspend(struct device *dev)
{
	struct lc709203f_chip *chip = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&chip->poll_work);
	if (chip->client->irq > 0 && device_may_wakeup(dev))
		enable_irq_wake(chip->client->irq);

	return 0;
}

static int __maybe_unused lc709203f_resume(struct device *dev)
{
	struct lc709203f_chip *chip = dev_get_drvdata(dev);

	if (chip->client->irq > 0 && device_may_wakeup(dev))
		disable_irq_wake(chip->client->irq);

	/* State may well have changed while we were suspended -- let userspace know. */
	power_supply_changed(chip->psy);
	schedule_delayed_work(&chip->poll_work, LC709203F_POLL_INTERVAL);

	return 0;
}

static SIMPLE_DEV_PM_OPS(lc709203f_pm_ops, lc709203f_suspend, lc709203f_resume);

/* Device tree bindings */
static const struct of_device_id lc709203f_of_match[] = {
	{ .compatible = "onsemi,lc709203f" },
	{ }
};
MODULE_DEVICE_TABLE(of, lc709203f_of_match);

/* I2C device IDs */
static const struct i2c_device_id lc709203f_id[] = {
	{ "lc709203f", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lc709203f_id);

/* Driver structure */
static struct i2c_driver lc709203f_driver = {
	.driver = {
		.name = "lc709203f",
		.of_match_table = lc709203f_of_match,
		.pm = &lc709203f_pm_ops,
	},
	.probe = lc709203f_probe,
	.remove = lc709203f_remove,
	.id_table = lc709203f_id,
};
module_i2c_driver(lc709203f_driver);

MODULE_AUTHOR("JordanViknar <jordanviknar@gmail.com>");
MODULE_AUTHOR("Philipp Steiner <philipp.steiner1987@gmail.com>"); /* original Zephyr driver */
MODULE_DESCRIPTION("onsemi LC709203F battery fuel gauge driver");
MODULE_LICENSE("GPL");