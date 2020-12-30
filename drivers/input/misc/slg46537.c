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
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <linux/init.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/gpio.h> // Required for the GPIO functions
#include <linux/time.h>
#include <linux/ktime.h>

#define NAME "slg46537"

#define EV_VOLUME_UP REL_X
#define EV_VOLUME_DOWN REL_Y
#define EV_ANGLE_CHANGE REL_Z
#define EV_MAKE_CALL REL_RX
#define EV_END_CALL REL_RY
#define EV_BATTERY_TIMER REL_RZ
#define EV_POWER_DOWN REL_HWHEEL

#define SLG_VOLUME_UP_IO 0x08
#define SLG_VOLUME_DOWN_IO 0x10
#define SLG_CALL_IO 0x04
#define SLG_POWER_IO 0x10

#define SLG_IN_CALL_STATE 0x10
#define SLG_END_CALL_STATE 0x20

static int log_enabled = 0;
struct slg_data
{
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct device *dev;
	struct delayed_work battery_monitor_work;
	struct delayed_work power_keypress_work;
	struct delayed_work call_keypress_work;
	int battery_monitor_enable;
	int incall;
};

struct slg_data *dummy_slg;


static int slg_i2c_read(struct slg_data *slg, u8 addr, u8 *data, int len)
{
	int err;

	struct i2c_msg msgs[] = {
		{
			.addr = slg->client->addr,
			.flags = slg->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = slg->client->addr,
			.flags = slg->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	err = i2c_transfer(slg->client->adapter, msgs, 2);

	if (err != 2)
	{
		dev_err(slg->dev, "i2c error reading register 0x%x, err = %d \n", addr, err);
	}
	return err;
}

/* Reads and matches the oscillator register, and returns 0 on success */
static int slg_check_i2c(struct slg_data *slg)
{
	int error;
	char recvbuf;

	if (slg == NULL)
		return -EINVAL;

	error = slg_i2c_read(slg, 0xA7, &recvbuf, 1);
	if (error != 2)
	{
		dev_err(slg->dev, "%s: i2c check failed = %d\n", __func__, error);
		return -EIO;
	}
	else
	{
		if (recvbuf != 0x98)
		{
			dev_err(slg->dev, "%s: i2c check failed with result mismatch = 0x%x", __func__, recvbuf);
			return -EIO;
		}
	}

	return 0;
}

static irqreturn_t slg_isr(int irq, void *data)
{
	struct slg_data *slg = data;
	int err;
	char recvbuf[2] = {0x00};
	int dummyvalue = 2;
	char savef0;

	mdelay(120);
	err = slg_i2c_read(slg, 0xF0, &recvbuf[0], 1);
	if (err != 2)
	{
		dev_err(&slg->client->dev, "%s failed", __func__);
		goto exit_irq;
	}
	if (log_enabled)
		dev_info(&slg->client->dev, "SLG: 0xF0 read 0x%x \n", recvbuf[0]);

	if ((recvbuf[0] & SLG_VOLUME_UP_IO) && (recvbuf[0] & SLG_VOLUME_DOWN_IO))
	{
		if (log_enabled)
			dev_info(&slg->client->dev, "Sending decrease angle event\n");
		input_report_rel(slg->input_dev, EV_ANGLE_CHANGE, dummyvalue);
	}
	else if (recvbuf[0] & SLG_VOLUME_UP_IO)
	{
		if (log_enabled)
			dev_info(&slg->client->dev, " Sending volume up event\n");
		input_report_rel(slg->input_dev, EV_VOLUME_UP, dummyvalue);
	}
	else if (recvbuf[0] & SLG_VOLUME_DOWN_IO)
	{
		if (log_enabled)
			dev_info(&slg->client->dev, " Sending volume down event\n");
		input_report_rel(slg->input_dev, EV_VOLUME_DOWN, dummyvalue);
	}
	else
	{
		savef0 = recvbuf[0];
		recvbuf[0] = recvbuf[1] = 0x00;
		err = slg_i2c_read(slg, 0xF5, recvbuf, 2);
		if (err < 0)
		{
			dev_err(&slg->client->dev, "%s: failed \n", __func__);
			goto exit_irq;
		}
		if (log_enabled)
			dev_info(&slg->client->dev, "SLG: 0xF5 and 0xF6 read = 0x%x, 0x%x", recvbuf[0], recvbuf[1]);

		//check for power button press and hold, if press and hold for 5 secs turn off device.
		if ((recvbuf[1] & SLG_POWER_IO))
		{
			if (log_enabled)
				dev_info(&slg->client->dev, "Power io pressed, starting a delay for 5 secs\n");
			cancel_delayed_work_sync(&slg->power_keypress_work);
			//schedule a delay of 5 seconds -- for now setting to 2sec since silego shuts off at 5
			schedule_delayed_work(&slg->power_keypress_work, msecs_to_jiffies(2000)); //TBD VIDI
		}
		
		// else if ((recvbuf[1] & SLG_CALL_IO) && (recvbuf[0] & SLG_IN_CALL_STATE))
		// {
		// 	if(slg->incall  == 0) {
		// 		if (log_enabled)
		// 			dev_info(&slg->client->dev, " Sending start call event\n");
		// 		input_report_rel(slg->input_dev, EV_MAKE_CALL, dummyvalue);
		// 		slg->incall = 1;
		// 	}

		// }
		// else if((recvbuf[1] & SLG_CALL_IO) && (recvbuf[0] & SLG_END_CALL_STATE))
		// {
		// 	if (log_enabled)
		// 		dev_info(&slg->client->dev, "Sending end call event\n");
		//                 input_report_rel(slg->input_dev, EV_END_CALL, dummyvalue);
		// 	slg->incall = 0;
		// }
		// else
		// {
		// 	dev_info(&slg->client->dev, "%s: Unknown interrupt: registers F0, F5 and F6 read x%x, 0x%x, 0x%x",
		// 				    __func__, savef0, recvbuf[0], recvbuf[1]);
		// 	goto exit_irq;
		// }
	}
	input_sync(slg->input_dev);

exit_irq:
	return IRQ_HANDLED;
}

static void slg_delayed_keypress_work(struct work_struct *work)
{
	struct slg_data *slg = container_of(work,
										struct slg_data,
										power_keypress_work.work);
	char recvbuf = 0x00;
	int err;

	err = slg_i2c_read(slg, 0xF6, &recvbuf, 1);
	if (err < 0)
	{
		dev_err(&slg->client->dev, "%s: failed \n", __func__);
		goto exit_keydelay;
	}
	//check if power button is still pressed
	if ((recvbuf & SLG_POWER_IO))
	{
		if (log_enabled)
			dev_info(&slg->client->dev, "Power key is still pressed, sending power key event\n");
		input_report_rel(slg->input_dev, EV_POWER_DOWN, 2);
		input_sync(slg->input_dev);
	}

exit_keydelay:
	cancel_delayed_work_sync(&slg->power_keypress_work);
}

// ckr 

static void slg_delayed_call_keypress_work(struct work_struct *work)
{
	struct slg_data *slg = container_of(work,
										struct slg_data,
										call_keypress_work.work);
	
	//check if power button is still pressed
	printk(KERN_INFO "2 sec timer for call button  \n");


}

static void slg_delayed_battery_work(struct work_struct *work)
{
	struct slg_data *slg = container_of(work,
										struct slg_data,
										battery_monitor_work.work);

	input_report_rel(slg->input_dev, EV_BATTERY_TIMER, 2);
	input_sync(slg->input_dev);
	schedule_delayed_work(&slg->battery_monitor_work,
						  msecs_to_jiffies(600000));
}

/* Allow users to read any register on silego via i2c */
static ssize_t slg_set_i2ctest(struct device *dev, struct device_attribute *attr,
							   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct slg_data *slg = i2c_get_clientdata(client);
	int error;
	unsigned int input;
	char recvbuf;
	u8 reg_addr;

	error = kstrtouint(buf, 10, &input);
	if (error < 0)
		return error;

	reg_addr = (u8)(input & 0xFF);

	error = slg_i2c_read(slg, reg_addr, &recvbuf, 1);
	if (error != 2)
		dev_err(dev, "%s: i2c read failed reading 0x%x with %d\n", __func__, reg_addr, error);

	dev_info(dev, "Register address 0x%x reads 0x%x\n", reg_addr, recvbuf);

	return count;
}

static ssize_t slg_set_shutdown(struct device *dev, struct device_attribute *attr,
								const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct slg_data *slg = i2c_get_clientdata(client);
	int error;

	dev_info(dev, "\nShutdown request recieved\n");
	error = i2c_smbus_write_byte_data(slg->client, 0xCF, 0x40);
	if (error)
		dev_err(dev, "%s: Failed to write 0x40 to 0xCF with %d\n", __func__, error);
	return count;
}

/* Allow users to enable/disable battery_monitoring feature */
static ssize_t slg_set_batt_monitor_enable(struct device *dev, struct device_attribute *attr,
										   const char *buf, size_t count)
{
	unsigned int input;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct slg_data *slg = i2c_get_clientdata(client);

	error = kstrtouint(buf, 10, &input);
	if (error < 0)
		return error;

	slg->battery_monitor_enable = input;
	if (input)
	{
		dev_info(dev, "Enabling battery monitoring feature\n");
		cancel_delayed_work_sync(&slg->battery_monitor_work);
		//First read neesds to be immediately after enabling and so scheduling it for 1 sec
		schedule_delayed_work(&slg->battery_monitor_work, msecs_to_jiffies(1000));
	}
	else
	{
		dev_info(dev, "Disabling battery monitoring feature\n");
		cancel_delayed_work_sync(&slg->battery_monitor_work);
	}
	return count;
}

static ssize_t slg_get_batt_monitor_enable(struct device *dev,
										   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct slg_data *slg = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", slg->battery_monitor_enable);
}

/* Allow users to read any register on silego via i2c */
static ssize_t slg_set_logen(struct device *dev, struct device_attribute *attr,
							 const char *buf, size_t count)
{
	unsigned int input;
	int error;

	error = kstrtouint(buf, 10, &input);
	if (error < 0)
		return error;

	log_enabled = input;
	return count;
}

static ssize_t slg_get_logen(struct device *dev,
							 struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", log_enabled);
}

/* When the other end disconnects the call, this function is to be called to update state machine in silego */
static ssize_t slg_set_endcall_state(struct device *dev, struct device_attribute *attr,
									 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct slg_data *slg = i2c_get_clientdata(client);
	int error;
	unsigned int input;
	char recvbuf;

	error = kstrtouint(buf, 10, &input);
	if (error < 0)
		return error;

	/* Set bit 1 of the 0xF4 register */
	error = slg_i2c_read(slg, 0xF4, &recvbuf, 1);
	if (error != 2)
		dev_err(dev, "%s: i2c read failed reading 0xF4 with %d\n", __func__, error);

	recvbuf = recvbuf | 0x02;

	error = i2c_smbus_write_byte_data(slg->client, 0xF4, recvbuf);
	if (error)
		dev_err(dev, "%s - 1: Failed to write 0x%x to 0xF4 with %d\n", __func__, recvbuf, error);

	mdelay(100);

	/* Clear bit 1 of 0xF4 register */
	recvbuf = recvbuf & ~(0x02);

	error = i2c_smbus_write_byte_data(slg->client, 0xF4, recvbuf);
	if (error)
		dev_err(dev, "%s - 2: Failed to write 0x%x to 0xF4 with %d\n", __func__, recvbuf, error);

	slg->incall = 0;
	dummy_slg->incall = 0;
	dev_info(dev, "%s: update silego state to end call\n");

	return count;
}

static DEVICE_ATTR(i2ctest, S_IRUGO | S_IWUSR, NULL, slg_set_i2ctest);
static DEVICE_ATTR(logen, S_IRUGO | S_IWUSR, slg_get_logen, slg_set_logen);
static DEVICE_ATTR(shutdown, S_IRUGO | S_IWUSR, NULL, slg_set_shutdown);
static DEVICE_ATTR(battery_monitor_en, S_IRUGO | S_IWUSR, slg_get_batt_monitor_enable, slg_set_batt_monitor_enable);
static DEVICE_ATTR(endcall, S_IRUGO | S_IWUSR, NULL, slg_set_endcall_state);

static struct attribute *slg_attributes[] = {
	&dev_attr_i2ctest.attr,
	&dev_attr_logen.attr,
	&dev_attr_shutdown.attr,
	&dev_attr_battery_monitor_en.attr,
	&dev_attr_endcall.attr,
	NULL};

static struct attribute_group slg_attribute_group = {
	.attrs = slg_attributes};

static int slg_probe(struct i2c_client *client,
					 const struct i2c_device_id *id)
{
	struct slg_data *slg;
	int err;

	dev_info(&client->dev, "8/29 - 5 %s\n", __func__);

	if (!i2c_check_functionality(client->adapter,
								 I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA))
	{
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	slg = kzalloc(sizeof(*slg), GFP_KERNEL);
	if (!slg)
	{
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	slg->client = client;
	slg->dev = &client->dev;
	slg->incall = 0;
	i2c_set_clientdata(client, slg);

	/* Check i2c connectivity */
	err = slg_check_i2c(slg);
	if (err)
	{
		dev_err(&client->dev, "SLG i2c communication failed: %d\n", err);
		goto err_free_mem;
	}

	/* Register irq */
	if (client->irq)
	{
		err = devm_request_threaded_irq(&client->dev, client->irq, NULL, slg_isr,
										IRQF_TRIGGER_RISING | IRQF_ONESHOT,
										"slg-irq", slg);
		if (err)
		{
			dev_err(&client->dev, "%s: request irq failed: %d\n", __func__, err);
			goto err_free_mem;
		}
	}

	/* Configure input device */
	slg->input_dev = devm_input_allocate_device(&client->dev);
	if (!slg->input_dev)
	{
		dev_dbg(&client->dev, "unable to allocate input device\n");
		goto err_free_irq;
	}

	__set_bit(EV_REL, slg->input_dev->evbit);
	__set_bit(REL_X, slg->input_dev->relbit);
	__set_bit(REL_Y, slg->input_dev->relbit);
	__set_bit(REL_Z, slg->input_dev->relbit);
	__set_bit(REL_RX, slg->input_dev->relbit);
	__set_bit(REL_RY, slg->input_dev->relbit);
	__set_bit(REL_RZ, slg->input_dev->relbit);
	__set_bit(REL_HWHEEL, slg->input_dev->relbit);
	slg->input_dev->name = "slg46537";
	slg->input_dev->phys = "slg46537/input0";

	err = input_register_device(slg->input_dev);
	if (err)
	{
		dev_err(&client->dev, "%s: failed to register input device: %d\n",
				__func__, err);
		goto err_free_irq;
	}

	/* Regsiter sysfs */
	err = sysfs_create_group(&client->dev.kobj, &slg_attribute_group);
	if (err)
	{
		dev_err(&client->dev, "%s: sysfs create failed: %d\n", __func__, err);
		goto err_destroy_input;
	}

	/* Create work queue for battery monitoring */
	INIT_DELAYED_WORK(&slg->battery_monitor_work, slg_delayed_battery_work);
	/* Create work queue for powerkey press tracking */
	INIT_DELAYED_WORK(&slg->power_keypress_work, slg_delayed_keypress_work);
/* Create work queue for call button press tracking */
	INIT_DELAYED_WORK(&slg->call_keypress_work, slg_delayed_call_keypress_work);

	dev_info(&client->dev, "Silego probed successfully\n");
	dummy_slg = kzalloc(sizeof(*dummy_slg), GFP_KERNEL);
	dummy_slg->client = client;
	dummy_slg->dev = &client->dev;
	dummy_slg->incall = 0;
	dummy_slg->input_dev = slg->input_dev;
	// i2c_set_clientdata(client, dummy_slg);

	return 0;

err_destroy_input:
	input_unregister_device(slg->input_dev);
err_free_irq:
	free_irq(client->irq, slg);
err_free_mem:
	kfree(slg);
	dev_err(&client->dev, "Silego probe failed\n");
	return err;
}

static int slg_remove(struct i2c_client *client)
{
	struct slg_data *slg = i2c_get_clientdata(client);

	if (client->irq)
	{
		input_unregister_device(slg->input_dev);
		sysfs_remove_group(&client->dev.kobj, &slg_attribute_group);
		free_irq(client->irq, slg);
	}
	kfree(slg);

	return 0;
}

static const struct i2c_device_id slg_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, slg_id);

static struct i2c_driver slg_driver = {
	.driver = {
		.name = NAME,
		.owner = THIS_MODULE,
	},
	.probe = slg_probe,
	.remove = slg_remove,
	.id_table = slg_id,
};

module_i2c_driver(slg_driver);

MODULE_DESCRIPTION("Silego driver");
MODULE_AUTHOR("viditha <viditha@mpconsultants.org>");
MODULE_LICENSE("GPL");

static unsigned int gpioButton = 121;  ///< hard coding the button gpio for this example to P4_21 (GPIO121)
static unsigned int irqNumber;		   ///< Used to share the IRQ number within this file
static unsigned int numberPresses = 0; ///< For information, store the number of button presses
static unsigned int button_current_state = 0;
static unsigned int button_previous_state = 0;

long get_epoch_time(void);

static unsigned long release_time = 0;
static unsigned long press_time = 0;
static unsigned long total_time = 0;


/// Function prototype for the custom IRQ handler function -- see below for the implementation
static irq_handler_t ebbgpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs);

/** @brief The LKM initialization function
 *  The static keyword restricts the visibility of the function to within this C file. The __init
 *  macro means that for a built-in driver (not a LKM) the function is only used at initialization
 *  time and that it can be discarded and its memory freed up after that point. In this example this
 *  function sets up the GPIOs and the IRQ
 *  @return returns 0 if successful
 */
static int __init ebbgpio_init(void)
{
	int result = 0;
	printk(KERN_INFO "GPIO_TEST: Initializing the GPIO_TEST LKM\n");

	gpio_request(gpioButton, "sysfs");	// Set up the gpioButton
	gpio_direction_input(gpioButton);	// Set the button GPIO to be an input
	gpio_set_debounce(gpioButton, 200); // Debounce the button with a delay of 200ms
	gpio_export(gpioButton, false);		// Causes gpio121 to appear in /sys/class/gpio
	// the bool argument prevents the direction from being changed
	// Perform a quick test to see that the button is working as expected on LKM load
	printk(KERN_INFO "GPIO_TEST: The button state is currently: %d\n", gpio_get_value(gpioButton));

	// GPIO numbers and IRQ numbers are not the same! This function performs the mapping for us
	irqNumber = gpio_to_irq(gpioButton);
	printk(KERN_INFO "GPIO_TEST: The button is mapped to IRQ: %d\n", irqNumber);

	// This next call requests an interrupt line
	result = request_irq(irqNumber,									 // The interrupt number requested
						 (irq_handler_t)ebbgpio_irq_handler,		 // The pointer to the handler function below
						 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, // Interrupt on rising edge not falling edge(button press not release)
						 "ebb_gpio_handler",						 // Used in /proc/interrupts to identify the owner
						 NULL);										 // The *dev_id for shared interrupt lines, NULL is okay

	printk(KERN_INFO "GPIO_TEST: The interrupt request result is: %d\n", result);
	return result;
}

/** @brief The LKM cleanup function
 *  Similar to the initialization function, it is static. The __exit macro notifies that if this
 *  code is used for a built-in driver (not a LKM) that this function is not required. Used to release the
 *  GPIOs and display cleanup messages.
 */
static void __exit ebbgpio_exit(void)
{
	printk(KERN_INFO "GPIO_TEST: The button state is currently: %d\n", gpio_get_value(gpioButton));
	printk(KERN_INFO "GPIO_TEST: The button was pressed and releeased %d times\n", numberPresses);

	free_irq(irqNumber, NULL); // Free the IRQ number, no *dev_id required in this case
	gpio_unexport(gpioButton); // Unexport the Button GPIO
	gpio_free(gpioButton);	   // Free the Button GPIO
	printk(KERN_INFO "GPIO_TEST: Goodbye from the LKM!\n");
}

/** @brief The GPIO IRQ Handler function
 *  This function is a custom interrupt handler that is attached to the GPIO above. The same interrupt
 *  handler cannot be invoked concurrently as the interrupt line is masked out until the function is complete.
 *  This function is static as it should not be invoked directly from outside of this file.
 *  @param irq    the IRQ number that is associated with the GPIO -- useful for logging.
 *  @param dev_id the *dev_id that is provided -- can be used to identify which device caused the interrupt
 *  Not used in this example as NULL is passed.
 *  @param regs   h/w specific register values -- only really ever used for debugging.
 *  return returns IRQ_HANDLED if successful -- should return IRQ_NONE otherwise.
 */

long get_epoch_time(void)
{
	struct timespec ts;
	getnstimeofday(&ts);
	return ts.tv_sec;
}

static irq_handler_t ebbgpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs)
{
	int dummyvalue = 2;

	button_current_state = gpio_get_value(gpioButton);

	if (button_current_state == 1 && button_previous_state != button_current_state)
	{
		press_time = get_epoch_time();
	}
	else if (button_current_state == 0 && button_previous_state != button_current_state)
	{
		release_time = get_epoch_time();
	
		total_time = release_time - press_time;

		cancel_delayed_work_sync(&dummy_slg->call_keypress_work);
		schedule_delayed_work(&dummy_slg->call_keypress_work, msecs_to_jiffies(2000));
		
		if (dummy_slg->incall == 0)
		{
			printk(KERN_INFO "valid button is pressed. make call event should trigger \n");

			if (log_enabled)
				dev_info(&dummy_slg->client->dev, " Sending start call event\n");
			input_report_rel(dummy_slg->input_dev, EV_MAKE_CALL, dummyvalue);
			dummy_slg->incall = 1;
		}
		else if (dummy_slg->incall == 1)
		{
			printk(KERN_INFO "valid button is pressed. end call event should trigger as call is already started \n");

			if (log_enabled)
				dev_info(&dummy_slg->client->dev, "Sending end call event\n");
			input_report_rel(dummy_slg->input_dev, EV_END_CALL, dummyvalue);
			dummy_slg->incall = 0;
		}
	}
	input_sync(dummy_slg->input_dev);
	button_previous_state = button_current_state;

	numberPresses++;				   // Global counter, will be outputted when the module is unloaded
	return (irq_handler_t)IRQ_HANDLED; // Announce that the IRQ has been handled correctly
}

/// This next calls are  mandatory -- they identify the initialization function
/// and the cleanup function (as above).
module_init(ebbgpio_init);
module_exit(ebbgpio_exit);


