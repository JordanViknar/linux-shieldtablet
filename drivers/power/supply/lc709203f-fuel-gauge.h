// SPDX-License-Identifier: GPL-2.0-only

#ifndef LC709203F_FUEL_GAUGE_H
#define LC709203F_FUEL_GAUGE_H

/* Register map */
// Check https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/fuel_gauge/lc709203f/lc709203f.h
#define LC709203F_REG_BEFORE_RSOC		0x04	/* Initialize before RSOC */
#define LC709203F_REG_THERMISTOR_B		0x06	/* Read/write thermistor B */
#define LC709203F_REG_INITIAL_RSOC		0x07	/* Initialize RSOC calculation */
#define LC709203F_REG_CELL_TEMPERATURE		0x08	/* Read/write cell temperature */
#define LC709203F_REG_CELL_VOLTAGE		0x09	/* Read batt voltage */
#define LC709203F_REG_CURRENT_DIRECTION	0x0A	/* Read/write current direction */
#define LC709203F_REG_APA			0x0B	/* Adjustment Pack Application */
#define LC709203F_REG_APT			0x0C	/* Read/write Adjustment Pack Thermistor */
#define LC709203F_REG_RSOC			0x0D	/* Read state of charge; 1% 0-100 scale */
#define LC709203F_REG_ITE			0x0F	/* Read batt indicator to empty */
#define LC709203F_REG_IC_VERSION		0x11	/* Read IC version */
#define LC709203F_REG_BAT_PROFILE		0x12	/* Set the battery profile */
#define LC709203F_REG_ALARM_LOW_RSOC		0x13	/* Alarm on percent threshold */
#define LC709203F_REG_ALARM_LOW_VOLTAGE	0x14	/* Alarm on voltage threshold */
#define LC709203F_REG_IC_POWER_MODE		0x15	/* Sets sleep/power mode */
#define LC709203F_REG_STATUS_BIT		0x16	/* Temperature obtaining method */
#define LC709203F_REG_NUM_PARAM		0x1A	/* Batt profile code */

/* Battery temperature source, written to LC709203F_REG_STATUS_BIT */
#define LC709203F_TEMP_MODE_I2C		0x0000
#define LC709203F_TEMP_MODE_THERMISTOR		0x0001

/*
 * Magic "quickstart" value for LC709203F_REG_INITIAL_RSOC. Writing this
 * exact word (not an arbitrary value) tells the chip to throw away its
 * current RSOC estimate and recompute it from the present cell voltage.
 * Matches LC709203F_INIT_RSOC_VAL in the Zephyr driver this was ported
 * from. Only do this after APA/battery-profile/thermistor setup, since
 * the recalculation uses those parameters.
 */
#define LC709203F_INIT_RSOC_VAL		0xAA55

#define LC709203F_POLL_INTERVAL		(30 * HZ)
#define LC709203F_CHARGE_CURRENT_THRESH_UA	50000

/*
 * Adjustment Pack Application (APA) codes referenced by the
 * optional "onsemi,appli-adjustment" property (not present in every
 * board's node -- see lc709203f_hw_init()):
 *   100mAh  -> 0x08     1000mAh -> 0x19
 *   200mAh  -> 0x0B     2000mAh -> 0x2D
 *   500mAh  -> 0x10     3000mAh -> 0x36
 */

struct lc709203f_chip {
	struct i2c_client *client;
	struct power_supply *psy;
	struct delayed_work poll_work;
	struct iio_channel *current_channel;

	u32 alert_low_rsoc;
	u32 alert_low_voltage;
	u32 thermistor_beta;
	u32 initial_rsoc;
	u32 appli_adjustment;
	u32 battery_profile;
	bool has_thermistor;
};

static enum power_supply_property lc709203f_props_base[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

#endif