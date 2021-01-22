/*
 * Copyright (C) 2011 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#define NAME			"pmic-bd71815"
#define IMX_GPIO_NR(b,p)  ( ((b - 1) * 32) + p )

#define PMIC_LDO4       IMX_GPIO_NR(1, 6)
#define PMIC_LDO5VSEL   IMX_GPIO_NR(4, 22)
#define AUDIO_DSP_RESET IMX_GPIO_NR(4, 23)
#define WIFI_REG_ON     IMX_GPIO_NR(1, 0)
#define WIFI_AP_READY   IMX_GPIO_NR(1, 2)
#define BT_REG_ON       IMX_GPIO_NR(1, 3)
#define BT_DEV_WAKE     IMX_GPIO_NR(1, 5)
#define SIERRA_PWR_ON   IMX_GPIO_NR(4, 17)
#define SIERRA_RESET    IMX_GPIO_NR(4, 18)
#define CELL_BATT_EN    IMX_GPIO_NR(4, 21)


#define PMIC_LDO4_VOLT_REG_ADDR 0x17
#define PMIC_LDO4_VOLT_REG_VALUE 0x08 //For 3.2V
#define PMIC_LDO5_VOLT_H_REG_ADDR 0x18
#define PMIC_LDO5_VOLT_H_REG_VALUE 0x30
#define PMIC_LDO_MODE1_REG_ADDR 0x10
#define PMIC_LDO_MODE3_REG_ADDR 0x12
#define PMIC_LDO_MODE3_REG_VALUE 0xFF


struct pmic_data {
	struct i2c_client *client;
	struct device *dev;
	char name[30];
};


static int pmic_i2c_read(struct pmic_data *pmic, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = pmic->client->addr,
			.flags = pmic->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = pmic->client->addr,
			.flags = pmic->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	
	return i2c_transfer(pmic->client->adapter, msgs, 2);
}

/* Reads and matches the device id, and returns 0 on success */
static int pmic_check_i2c(struct pmic_data *pmic)
{
	int error;
	char recvbuf;
	
	if (pmic == NULL)
		return -EINVAL;

	error = pmic_i2c_read(pmic, 0x00, &recvbuf , 1);
        if (error != 2) {
                dev_err(pmic->dev, "%s: device id read failed = %d\n",__func__,error);
		return -EIO; 
	}
	else {
		if (recvbuf != 0x41) {
			dev_err(pmic->dev, "%s: device id mismatch = 0x%x", __func__, recvbuf);
			return -EIO;
		}	
	}

	return 0;
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t pmic_set_i2ctest(struct device *dev, struct device_attribute *attr,
                                                const char *buf, size_t count)
{
        struct i2c_client *client = to_i2c_client(dev);
        struct pmic_data *pmic = i2c_get_clientdata(client);
        int error;
        unsigned int input;
        char recvbuf;
        u8 reg_addr;

        error = kstrtouint(buf, 10, &input);
        if (error < 0)
                return error;

        dev_info(dev," Got input = 0x%x\n", input);
	dev_info(dev," pmic name = %s\n", pmic->name);
        reg_addr = (u8)(input & 0xFF);
        dev_info(dev, "Reading from register address = 0x%x", reg_addr);

        error = pmic_i2c_read(pmic, reg_addr, &recvbuf , 1);
        if (error != 2)
                dev_err(dev, "1: recv failed with error = %d\n", error);

        dev_info(dev, "1: recvbuf = 0x%x\n", recvbuf);

        return count;
}
/* Sysfs to enable /disable wifi gpio controls */
static ssize_t pmic_set_wlan_enable(struct device *dev, struct device_attribute *attr,
                                                const char *buf, size_t count)
{

        int error;
        unsigned int input;


        error = kstrtouint(buf, 10, &input);
        if (error < 0)
                return error;

        if (input == 0) {
		dev_info(dev, "%s: Turning off wlan", __func__);
		gpio_set_value(WIFI_REG_ON , 0);
		gpio_set_value(WIFI_AP_READY , 0);
	
	}
	else {
		dev_info(dev, "%s: Turning on wlan", __func__);
                gpio_set_value(WIFI_REG_ON , 1);
                gpio_set_value(WIFI_AP_READY , 1);
        }

	return count;
}

/* Sysfs to enable /disable dsp gpio controls */
static ssize_t pmic_set_dsp_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int error;
	unsigned int input;

	error = kstrtouint(buf, 10, &input);
	if (error < 0)
		return error;

	if (input == 0) {
		dev_info(dev, "%s: Turning off dsp", __func__);
		gpio_set_value(AUDIO_DSP_RESET , 0);
	}else {
		dev_info(dev, "%s: Turning on dsp", __func__);
		gpio_set_value(AUDIO_DSP_RESET , 1);
	}
	return count;
}

/* Sysfs to enable /disable cellular battery control */
static ssize_t pmic_set_cellbatt_enable(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int error;
	unsigned int input;

	error = kstrtouint(buf, 10, &input);
	if (error < 0)
		return error;

	if (input == 0) {
		dev_info(dev, "%s: Turning off cell batt", __func__);
		gpio_set_value(CELL_BATT_EN , 0);
	}
	else {
		dev_info(dev, "%s: Turning on cell batt", __func__);
		gpio_set_value(CELL_BATT_EN , 1);
	}
	return count;
}

/* Configure pmic to 3.2V and turn on LDO4 */
static int pmic_ldo_configure(struct pmic_data *pmic)
{
	int error;
	char recvbuf;

	/* Configure LDO4 voltage */	
	// Configure it to be set at 3.2V
        recvbuf = PMIC_LDO4_VOLT_REG_VALUE;
        error = i2c_smbus_write_byte_data(pmic->client, PMIC_LDO4_VOLT_REG_ADDR, recvbuf);
        if (error){
                dev_err(pmic->dev, "Error writing to PMIC_LDO4_VOLT_REG_ADDR = 0x%d", error);
		goto exit_ldo;
	}
	//Read it back
        recvbuf = 0;
        pmic_i2c_read(pmic, PMIC_LDO4_VOLT_REG_ADDR, &recvbuf, 1);
	if ( recvbuf != PMIC_LDO4_VOLT_REG_VALUE ) {
	        dev_info(pmic->dev, "Error: PMIC_LDO4_VOLT_REG value was expected \
			       0x%x but read 0x%x", PMIC_LDO4_VOLT_REG_VALUE, recvbuf);
		error = -EIO;
		goto exit_ldo;
	}
	/* Set LDO4_reg_mode bit in LDO_MODE1 register */
	recvbuf = 0;
        error = pmic_i2c_read(pmic, PMIC_LDO_MODE1_REG_ADDR, &recvbuf, 1);
	if (error != 2) {
		dev_info(pmic->dev, "Error reading PMIC_LDO_MODE1_REG_ADDR = %d\n", error);
		goto exit_ldo;
	}

	recvbuf = recvbuf & 0xF7; // clear bit 3
	error = i2c_smbus_write_byte_data(pmic->client, PMIC_LDO_MODE1_REG_ADDR, recvbuf);
        if (error){
                dev_err(pmic->dev, "Error writing 0x%x to PMIC_LDO_MODE1_REG_ADDR = 0x%d\n", recvbuf, error);
                goto exit_ldo;
        }
	//Read it back
        recvbuf = 0;
        pmic_i2c_read(pmic, PMIC_LDO_MODE1_REG_ADDR, &recvbuf, 1);
	if (recvbuf & 0x08) {
		dev_err(pmic->dev, "Register 0x%x read 0x%x, expected bit 3 to be cleared\n",
				PMIC_LDO_MODE1_REG_ADDR, recvbuf);
	}
        //Turn on LDO4
        gpio_set_value(PMIC_LDO4 , 1);

	/* Configure LDO5 voltage */
        recvbuf = PMIC_LDO5_VOLT_H_REG_VALUE;
	//Write to LDO5 to configure for 3.2V	
        error = i2c_smbus_write_byte_data(pmic->client, PMIC_LDO5_VOLT_H_REG_ADDR, recvbuf);
        if (error ) {
                dev_err(pmic->dev, "Error writing to PMIC_LDO5_VOLT_H_REG_ADDR = 0x%d", error);
		goto exit_ldo;
	}
        //Read it back
        recvbuf = 0;
        pmic_i2c_read(pmic, PMIC_LDO5_VOLT_H_REG_ADDR, &recvbuf, 1);
        if ( recvbuf != PMIC_LDO5_VOLT_H_REG_VALUE ) {
                dev_info(pmic->dev, "Error: PMIC_LDO5_VOLT_H_REG_ADDR value was expected \
                               0x%x but read 0x%x", PMIC_LDO5_VOLT_H_REG_VALUE, recvbuf);
		error = -EIO;
		goto exit_ldo;
        }
	//set LDO5VSEL to high
        gpio_set_value(PMIC_LDO5VSEL , 1);


	//Turn on LDO4 and LDO5
	//Write 0xFF to 0x12h
        recvbuf = PMIC_LDO_MODE3_REG_VALUE;
        error = i2c_smbus_write_byte_data(pmic->client, PMIC_LDO_MODE3_REG_ADDR, recvbuf);
        if (error ) {
                dev_err(pmic->dev, "Error writing to PMIC_LDO_MODE3_REG_ADDR = 0x%d", error);
                goto exit_ldo;
        }
        //Read it back
        recvbuf = 0;
        pmic_i2c_read(pmic, PMIC_LDO_MODE3_REG_ADDR, &recvbuf, 1);
        if ( recvbuf != PMIC_LDO_MODE3_REG_VALUE ) {
                dev_info(pmic->dev, "Error: PMIC_LDO_MODE3_REG_ADDR value was expected \
                               0x%x but read 0x%x", PMIC_LDO_MODE3_REG_VALUE, recvbuf);
                error = -EIO;
                goto exit_ldo;
        }

	dev_info(pmic->dev, "pmic ldo configured successfully\n");
	return 0;

exit_ldo:
	dev_err(pmic->dev, "PMIC LDO configuration failed\n");
	return error;	
}

//Sysfs to configure pmic
static ssize_t pmic_set_configure(struct device *dev, struct device_attribute *attr,
                                                const char *buf, size_t count)
{
	int error;
        struct i2c_client *client = to_i2c_client(dev);
        struct pmic_data *pmic = i2c_get_clientdata(client);

	
	error = pmic_ldo_configure(pmic);
	dev_info(dev, "pmic ldo configure returned = %d", error);
        return count;
}

//Sysfs to enable sierra
static ssize_t pmic_set_sierra_enable(struct device *dev, struct device_attribute *attr,
                                                const char *buf, size_t count)
{
        int error;
        unsigned int input;


        error = kstrtouint(buf, 10, &input);
        if (error < 0)
                return error;

        if (input == 0) {
                dev_info(dev, "%s: Turning off sierra --- TODO", __func__);
		//TODO VIDI
        }
        else {
                dev_info(dev, "%s: Sierra power on", __func__);
	        // Toggle sierra_power_on
	        gpio_set_value(SIERRA_PWR_ON , 0);
        	mdelay(100);
        	gpio_set_value(SIERRA_PWR_ON , 1);
        }

        return count;

}



static ssize_t pmic_set_gpiotest(struct device *dev, struct device_attribute *attr,
                                                const char *buf, size_t count)
{
        int error;
        unsigned int input;

        error = kstrtouint(buf, 10, &input);
        if (error < 0)
                return error;

	if(input == 0) {
		dev_info(dev, "Making cellbatt low \n");
		gpio_set_value(CELL_BATT_EN, 0);
	}
	if(input == 1) {
		dev_info(dev, "Making cellbat high\n");
		gpio_set_value(CELL_BATT_EN, 1);
	}

	return count;
}


static ssize_t pmic_set_test(struct device *dev, struct device_attribute *attr,
                                                const char *buf, size_t count)
{

        struct i2c_client *client = to_i2c_client(dev);
        struct pmic_data *pmic = i2c_get_clientdata(client);
        int error;
	unsigned int input;
	char recvbuf;


        error = kstrtouint(buf, 10, &input);
        if (error < 0)
                return error;

	dev_info(dev,"Configuring pmic = 0x%x\n", input);
	dev_info(dev," pmic name = %s\n", pmic->name);
	dev_info(dev,"Got input = 0x%x\n", input);
	pmic_i2c_read(pmic, 0x17, &recvbuf, 1);
	dev_info(dev, "Read 0x17 = 0x%x", recvbuf);
	recvbuf = 0x08; 
	dev_info(dev, "Write to 0x17  = 0x%x", recvbuf);
	error = i2c_smbus_write_byte_data(client, 0x17, recvbuf);
	if (error )
		dev_err(dev, "Error writing to 0x17 = 0x%d", error);
	//REad it back
	recvbuf = 0;
        pmic_i2c_read(pmic, 0x17, &recvbuf, 1);
        dev_info(dev, "Read  back 0x17 = 0x%x", recvbuf);
	//Set LDO4 high
        gpio_set_value(PMIC_LDO4 , 1);


	dev_info(dev, "PMIC_LDO4 ping is set to %d", gpio_get_value(PMIC_LDO4));

        pmic_i2c_read(pmic, 0x18, &recvbuf, 1);
        dev_info(dev, "Read 0x18 = 0x%x", recvbuf);
        recvbuf = 0x30;
        dev_info(dev, "Write to 0x18  = 0x%x", recvbuf);
        error = i2c_smbus_write_byte_data(client, 0x18, recvbuf);
        if (error )
                dev_err(dev, "Error writing to 0x18 = 0x%d", error);
        //REad it back
        recvbuf = 0;
        pmic_i2c_read(pmic, 0x18, &recvbuf, 1);
        dev_info(dev, "Read  back 0x18 = 0x%x", recvbuf);

	//set LDO5VSEL to high
	gpio_set_value(PMIC_LDO5VSEL , 1);



	// Toggle sierra_power_on
	gpio_set_value(SIERRA_PWR_ON , 0);
	mdelay(100);
	gpio_set_value(SIERRA_PWR_ON , 1);

	dev_info(dev, "PMIC_LDO5VSEL ping is set to %d", gpio_get_value(PMIC_LDO5VSEL));

        return count;

}


static ssize_t pmic_get_battery_level(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pmic_data *pmic = i2c_get_clientdata(client);
	int error;
	uint8_t msb, lsb;
	int batt_level = 0;

	error = pmic_i2c_read(pmic, 0x5d, &msb, 1);
	if (error != 2)
		dev_err(dev, "Error reading 0x5d = 0x%d, err = %d", msb, error);

	error = pmic_i2c_read(pmic, 0x5f, &lsb, 1);
	if (error != 2)
		dev_err(dev, "Error reading 0x5f = 0x%d, err = %d", lsb, error);

	msb = msb & 0x1F;
	batt_level = (msb << 8 ) | (lsb);

	dev_info(dev, "\nBattery_level = %d\n", batt_level);
	return sprintf(buf, "%d\n", batt_level);
}

static ssize_t pmic_register_dump(struct device *dev,
                        struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pmic_data *pmic = i2c_get_clientdata(client);
	int error;
	uint8_t val;
	uint8_t addr;

	for(addr=0x00; addr<=0xFF; ++addr)
	{
		if(addr % 0x0F == 0)
		{
			dev_info(dev, "\n0x%02X\t", addr);
		}
		error = pmic_i2c_read(pmic, addr, &val, 1);
		if (error != 2)
			dev_err(dev, "Error reading 0x%02x = 0x%d, err = %d", addr, val, error);
		dev_info(dev, "0x%02X ", val);
	}

	dev_info(dev, "\nDone\n");
	return sprintf(buf, "%d\n", 0);
}

static DEVICE_ATTR(wlan_enable, S_IRUGO|S_IWUSR, NULL, pmic_set_wlan_enable);
static DEVICE_ATTR(test, S_IRUGO|S_IWUSR, NULL, pmic_set_test);
static DEVICE_ATTR(i2ctest, S_IRUGO|S_IWUSR, NULL, pmic_set_i2ctest);
static DEVICE_ATTR(pmic_configure, S_IRUGO|S_IWUSR, NULL, pmic_set_configure);
static DEVICE_ATTR(sierra_enable, S_IRUGO|S_IWUSR, NULL, pmic_set_sierra_enable);
static DEVICE_ATTR(gpiotest, S_IRUGO|S_IWUSR, NULL, pmic_set_gpiotest);
static DEVICE_ATTR(dsp_enable, S_IRUGO|S_IWUSR, NULL, pmic_set_dsp_enable);
static DEVICE_ATTR(cellbatt_enable, S_IRUGO|S_IWUSR, NULL, pmic_set_cellbatt_enable);
static DEVICE_ATTR(battery_level, S_IRUGO|S_IWUSR, pmic_get_battery_level, NULL);
static DEVICE_ATTR(register_dump, S_IRUGO|S_IWUSR, pmic_register_dump, NULL);


static struct attribute *pmic_attributes[] = {
	&dev_attr_i2ctest.attr,
	&dev_attr_test.attr,
	&dev_attr_wlan_enable.attr,
	&dev_attr_pmic_configure.attr,
	&dev_attr_sierra_enable.attr,
	&dev_attr_gpiotest.attr,
	&dev_attr_dsp_enable.attr,
	&dev_attr_cellbatt_enable.attr,
	&dev_attr_battery_level.attr,
	&dev_attr_register_dump.attr,
	NULL
};

static struct attribute_group pmic_attribute_group = {
	.attrs = pmic_attributes
};

static int pmic_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct pmic_data *pmic;
	int err;


	dev_info(&client->dev,  "8/28   --- 1 %s\n", __func__);

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}


	pmic = kzalloc(sizeof(*pmic), GFP_KERNEL);
	if (!pmic) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	pmic->client = client;
	pmic->dev = &client->dev;
	strcpy(pmic->name , NAME);
	i2c_set_clientdata(client, pmic);

	/* Check i2c connectivity */
	err = pmic_check_i2c(pmic);
	if (err) {
		dev_err(&client->dev, "PMIC i2c communication failed: %d\n", err);
		goto err_free_mem;
	}

	/* Create sysfs group */
        err = sysfs_create_group(&client->dev.kobj, &pmic_attribute_group);
	if (err) {
		dev_err(&client->dev, "sysfs create failed: %d\n", err);
		goto err_free_mem;
	}

	/* Configure PMIC gpios */
	/* GPIO1-06 to be make as output */
	err = gpio_request(PMIC_LDO4, "LDO4 enable");
        if (err) {
		dev_err(&client->dev, "gpio_request for LDO4 enable failed %d\n", err);
                goto err_free_sysfs;
        }
	err = gpio_direction_output(PMIC_LDO4, 0);
        if (err) {
                dev_err(&client->dev, "Failed to configure LDO4 enable as output %d\n", err);
                goto err_free_sysfs;
        }
	/* CSI_DATA01( gpio4_22) to be configured as output */
        err = gpio_request(PMIC_LDO5VSEL, "LDO5VSEL");
        if (err) {
                dev_err(&client->dev, "gpio_request for LDO5VSEL enable failed %d\n", err);
                goto err_free_sysfs;
        }
        err = gpio_direction_output(PMIC_LDO5VSEL, 0);
        if (err) {
		dev_err(&client->dev, "Failed to configure LDO5VSEL enable as output %d\n", err);
		goto err_free_sysfs;
	}

	//Configure pmic
	err = pmic_ldo_configure(pmic);
	if (err) {
		dev_err(&client->dev, "Failed to configure PMIC LDO %d\n", err);
		goto err_free_sysfs;
	}

        /* CSI_DATA02( gpio4_23) to be configured as output and left high */
        err = gpio_request(AUDIO_DSP_RESET, "DSPRESET");
        if (err) {
                dev_err(&client->dev, "gpio_request for DSP_RESET enable failed %d\n", err);
                goto err_free_sysfs;
        }
        err = gpio_direction_output(AUDIO_DSP_RESET, 1);
        if (err) {
                dev_err(&client->dev, "Failed to configure DSP_RESET enable as output %d\n", err);
                goto err_free_sysfs;
        }


        /* CSI_DATA02( gpio4_23) to be configured as output and left high */
        err = gpio_request(WIFI_REG_ON, "WIFI_REG_ON");
        if (err) {
		dev_err(&client->dev, "gpio_request for WIFI_REG_ON enable failed %d\n", err);
		goto err_free_sysfs;
        }
        err = gpio_direction_output(WIFI_REG_ON, 1);
        if (err) {
		dev_err(&client->dev, "Failed to configure WIFI_REG_ON enable as output %d\n", err);
		goto err_free_sysfs;
	}

        /* CSI_DATA02( gpio4_23) to be configured as output and left high */
        err = gpio_request(WIFI_AP_READY, "WIFI_AP_READY");
        if (err) {
		dev_err(&client->dev, "gpio_request for WIFI_AP_READY enable failed %d\n", err);
		goto err_free_sysfs;
        }
        err = gpio_direction_output(WIFI_AP_READY, 1);
        if (err) {
		dev_err(&client->dev, "Failed to configure WIFI_AP_READY enable as output %d\n", err);
		goto err_free_sysfs;
        }


        /* SIERRA_PWR_ON to be configured as output */
        err = gpio_request(SIERRA_PWR_ON, "SIERRA_PWR_ON");
        if (err) {
		dev_err(&client->dev, "gpio_request for SIERRA_PWR_ON enable failed %d\n", err);
		goto err_free_sysfs;
        }
        err = gpio_direction_output(SIERRA_PWR_ON, 0);
        if (err) {
		dev_err(&client->dev, "Failed to configure SIERRA_PWR_ON enable as output %d\n", err);
		goto err_free_sysfs;
        }
        /* SIERRA_RESET to be configured as output and left high */
        err = gpio_request(SIERRA_RESET, "SIERRA_RESET");
        if (err) {
		dev_err(&client->dev, "gpio_request for SIERRA_RESET enable failed %d\n", err);
		goto err_free_sysfs;
        }
        err = gpio_direction_output(SIERRA_RESET, 1);
        if (err) {
		dev_err(&client->dev, "Failed to configure SIERRA_RESET enable as output %d\n", err);
		goto err_free_sysfs;
        }

        /* CELL_BATT_EN to be configured as output and left low */
        err = gpio_request(CELL_BATT_EN, "CELL_BATT_EN");
        if (err) {
		dev_err(&client->dev, "gpio_request for CELL_BATT_EN enable failed %d\n", err);
		goto err_free_sysfs;
        }
        err = gpio_direction_output(CELL_BATT_EN, 1);
        if (err) {
		dev_err(&client->dev, "Failed to configure CELL_BATT_EN enable as output %d\n", err);
		goto err_free_sysfs;
        }

#if 1 //For now BT is not enabled in the project 
        err = gpio_request(BT_REG_ON, "BT_REG_ON");
        if (err) {
                        dev_err(&client->dev, "gpio_request for BT_REG_ON enable failed %d\n", err);
                        goto err_free_mem;
        }
        err = gpio_direction_output(BT_REG_ON, 0);
        if (err) {
                        dev_err(&client->dev, "Failed to configure BT_REG_ON enable as output %d\n", err);
                        goto err_free_mem;
        }
        err = gpio_request(BT_DEV_WAKE, "BT_DEV_WAKE");
        if (err) {
                        dev_err(&client->dev, "gpio_request for BT_DEV_WAKE enable failed %d\n", err);
                        goto err_free_mem;
        }
        err = gpio_direction_output(BT_DEV_WAKE, 0);
        if (err) {
                        dev_err(&client->dev, "Failed to configure BT_DEV_WAKE enable as output %d\n", err);
                        goto err_free_mem;
        }
#endif

	dev_info (&client->dev, "PMIC probe successful\n");
	return 0;

err_free_sysfs:
	sysfs_remove_group(&client->dev.kobj, &pmic_attribute_group);
err_free_mem:
	kfree(pmic);
	dev_err(&client->dev, "PMIC probe failed\n");
	return err;
}

static int pmic_remove(struct i2c_client *client)
{
	struct pmic_data *pmic = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &pmic_attribute_group);
	kfree(pmic);

	return 0;
}

static const struct i2c_device_id pmic_id[] = {
	{ NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, pmic_id);

static struct i2c_driver pmic_driver = {
	.driver = {
		.name	= NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= pmic_probe,
	.remove		= pmic_remove,
	.id_table	= pmic_id,
};

module_i2c_driver(pmic_driver);

MODULE_DESCRIPTION("PMIC driver");
MODULE_AUTHOR("viditha <viditha@mpconsultants.org>");
MODULE_LICENSE("GPL");
