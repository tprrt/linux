// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Aaeon MCU driver
 *
 * Copyright (C) 2026 Bootlin
 * Author: Jérémie Dautheribes <jeremie.dautheribes@bootlin.com>
 * Author: Thomas Perrot <thomas.perrot@bootlin.com>
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/mfd/aaeon-mcu.h>
#include <linux/mfd/core.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

struct aaeon_mcu {
	struct i2c_client *client;
	u8 *cmd;      /* DMA-safe 3-byte write buffer [opcode, arg, value] */
	u8 *response; /* DMA-safe 1-byte read buffer for MCU acknowledgment */
};

static const struct mfd_cell aaeon_mcu_devs[] = {
	MFD_CELL_BASIC("aaeon-mcu-wdt", NULL, NULL, 0, 0),
	MFD_CELL_BASIC("aaeon-mcu-gpio", NULL, NULL, 0, 0),
};

/* Number of bytes in a MCU command: [opcode, arg, value] */
#define AAEON_MCU_CMD_LEN      3

/*
 * Custom regmap bus for the Aaeon MCU I2C protocol.
 *
 * The MCU uses a fixed 3-byte command format [opcode, arg, value] followed
 * by a 1-byte response. It requires a STOP condition between the command
 * write and the response read, so two separate transfers are issued. The
 * I2C segment lock is held across both so that no other transfer on the
 * same I2C segment can land between the command and its response. The
 * regmap lock serialises concurrent accesses from the GPIO and watchdog
 * child drivers.
 *
 * Register addresses are encoded as a 16-bit big-endian value where the
 * high byte is the opcode and the low byte is the argument, matching the
 * wire layout produced by regmap for reg_bits=16.
 */

/*
 * Send the command held in mcu->cmd and read the 1-byte response into
 * mcu->response. Both buffers are heap-allocated and thus DMA-safe.
 */
static int aaeon_mcu_xfer(struct aaeon_mcu *mcu)
{
	struct i2c_adapter *adap = mcu->client->adapter;
	struct i2c_msg msg = {
		.addr	= mcu->client->addr,
		.flags	= I2C_M_DMA_SAFE,
		.buf	= mcu->cmd,
		.len	= AAEON_MCU_CMD_LEN,
	};
	int ret;

	i2c_lock_bus(adap, I2C_LOCK_SEGMENT);

	ret = __i2c_transfer(adap, &msg, 1);
	if (ret == 1) {
		msg.flags = I2C_M_RD | I2C_M_DMA_SAFE;
		msg.buf   = mcu->response;
		msg.len   = 1;
		ret = __i2c_transfer(adap, &msg, 1);
	}

	i2c_unlock_bus(adap, I2C_LOCK_SEGMENT);

	if (ret < 0)
		return ret;

	return ret == 1 ? 0 : -EIO;
}

static int aaeon_mcu_regmap_write(void *context, const void *data, size_t count)
{
	struct aaeon_mcu *mcu = context;

	/*
	 * use_single_write guarantees one register per call: a 2-byte
	 * address followed by a 1-byte value.
	 */
	if (count != AAEON_MCU_CMD_LEN)
		return -EINVAL;

	memcpy(mcu->cmd, data, count);

	/* The MCU always sends a response byte after each command; discard it. */
	return aaeon_mcu_xfer(mcu);
}

static int aaeon_mcu_regmap_read(void *context, const void *reg_buf,
				 size_t reg_size, void *val_buf, size_t val_size)
{
	struct aaeon_mcu *mcu = context;
	int ret;

	/*
	 * reg_buf holds the 2-byte big-endian register address [opcode, arg].
	 * Append a trailing 0x00 to form the full 3-byte MCU command.
	 */
	mcu->cmd[0] = ((u8 *)reg_buf)[0];
	mcu->cmd[1] = ((u8 *)reg_buf)[1];
	mcu->cmd[2] = 0x00;

	ret = aaeon_mcu_xfer(mcu);
	if (ret)
		return ret;

	/*
	 * val_buf is owned by the regmap caller and is not necessarily
	 * DMA-safe, so go through the driver's response buffer.
	 * use_single_read guarantees val_size == 1.
	 */
	*(u8 *)val_buf = *mcu->response;

	return 0;
}

static const struct regmap_bus aaeon_mcu_regmap_bus = {
	.write = aaeon_mcu_regmap_write,
	.read  = aaeon_mcu_regmap_read,
};

static bool aaeon_mcu_volatile_reg(struct device *dev, unsigned int reg)
{
	/*
	 * GPIO input registers are driven by external signals and can change
	 * at any time without CPU involvement, always read from hardware.
	 *
	 * The watchdog status register reflects hardware state and can change
	 * autonomously.
	 *
	 * All other registers are written by the driver and their values are
	 * stable, so they can be safely cached.
	 */
	if ((reg >> 8) == AAEON_MCU_READ_GPIO_OPCODE)
		return true;
	if (reg == AAEON_MCU_REG(AAEON_MCU_CONTROL_WDT_OPCODE, 0x02))
		return true;
	return false;
}

static bool aaeon_mcu_readable_reg(struct device *dev, unsigned int reg)
{
	/*
	 * Every opcode but the GPIO input read and the watchdog status is a
	 * command: the frame sent for a read is identical to the one used to
	 * write it, so reading such a register would execute the command.
	 * Only the registers reflecting hardware state may be read.
	 */
	return aaeon_mcu_volatile_reg(dev, reg);
}

static const struct regmap_config aaeon_mcu_regmap_config = {
	.reg_bits          = 16,
	.val_bits          = 8,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.max_register      = AAEON_MCU_MAX_REGISTER,
	.readable_reg      = aaeon_mcu_readable_reg,
	.volatile_reg      = aaeon_mcu_volatile_reg,
	.cache_type        = REGCACHE_MAPLE,
	.use_single_write  = true,
	.use_single_read   = true,
};

static int aaeon_mcu_probe(struct i2c_client *client)
{
	struct aaeon_mcu *ddata;
	struct regmap *regmap;

	ddata = devm_kzalloc(&client->dev, sizeof(*ddata), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	ddata->client = client;

	ddata->cmd = devm_kcalloc(&client->dev, AAEON_MCU_CMD_LEN, sizeof(*ddata->cmd),
				  GFP_KERNEL);
	if (!ddata->cmd)
		return -ENOMEM;

	ddata->response = devm_kzalloc(&client->dev, sizeof(*ddata->response), GFP_KERNEL);
	if (!ddata->response)
		return -ENOMEM;

	regmap = devm_regmap_init(&client->dev, &aaeon_mcu_regmap_bus,
				  ddata, &aaeon_mcu_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(&client->dev, PTR_ERR(regmap),
				     "failed to initialize regmap\n");

	return devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO,
				    aaeon_mcu_devs, ARRAY_SIZE(aaeon_mcu_devs),
				    NULL, 0, NULL);
}

static const struct of_device_id aaeon_mcu_of_match[] = {
	{ .compatible = "aaeon,srg-imx8p-mcu" },
	{},
};
MODULE_DEVICE_TABLE(of, aaeon_mcu_of_match);

static struct i2c_driver aaeon_mcu_driver = {
	.driver = {
		.name = "aaeon-mcu",
		.of_match_table = aaeon_mcu_of_match,
	},
	.probe = aaeon_mcu_probe,
};
module_i2c_driver(aaeon_mcu_driver);

MODULE_DESCRIPTION("Aaeon MCU Driver");
MODULE_AUTHOR("Jérémie Dautheribes <jeremie.dautheribes@bootlin.com>");
MODULE_LICENSE("GPL");
