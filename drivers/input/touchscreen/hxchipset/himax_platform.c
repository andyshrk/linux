// SPDX-License-Identifier: GPL-2.0
/*  Himax Android Driver Sample Code for QCT platform
 *
 *  Copyright (C) 2021 Himax Corporation.
 *
 *  This software is licensed under the terms of the GNU General Public
 *  License version 2,  as published by the Free Software Foundation,  and
 *  may be copied,  distributed,  and modified under those terms.
 *
 *  This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include "himax_common.h"

#if (HX_RST_PIN_FUNC == 0x01)
void himax_tp_ext_rstn_reset(struct himax_ts_data *ts)
{
	if (ts->pdata->tp_ext_rstn >= 0) {
		gpio_direction_output(ts->pdata->tp_ext_rstn, 0);
		msleep(20);
		gpio_direction_output(ts->pdata->tp_ext_rstn, 1);
		msleep(50);
	}
}
//EXPORT_SYMBOL(himax_tp_ext_rstn_reset);
#endif
int himax_dev_set(struct himax_ts_data *ts)
{
	int ret = 0;

	ts->input_dev = input_allocate_device();

	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		E("%s: Failed to allocate input device-input_dev\n", __func__);
		return ret;
	}

	ts->input_dev->name = "himax-touchscreen";

	if(ts->pdata->phys_port)
		 ts->input_dev->phys = ts->pdata->phys_port;

	return ret;
}
int himax_input_register_device(struct input_dev *input_dev)
{
	return input_register_device(input_dev);
}

int himax_parse_dt(struct himax_device *hdev, struct himax_i2c_platform_data *pdata)
{
	struct device_node *dt = hdev->client->dev.of_node;
	u32 data = 0;
	uint32_t coords[4] = { 0 };
	int rc, coords_size = 0;
	int ret = 0;
	struct property *prop;

	prop = of_find_property(dt, "himax,panel-coords", NULL);
	if (prop) {
		coords_size = prop->length / sizeof(u32);
		if (coords_size != 4)
			D(" %s:Invalid panel coords size %d\n", __func__,
			  coords_size);
	}
	ret = of_property_read_u32_array(dt, "himax,panel-coords", coords,
					 coords_size);
	if (ret == 0) {
		pdata->abs_x_min = coords[0];
		pdata->abs_x_max = (coords[1] - 1);
		pdata->abs_y_min = coords[2];
		pdata->abs_y_max = (coords[3] - 1);
		I(" DT:panel-coords = %d, %d, %d, %d\n", pdata->abs_x_min,
		  pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);
	}
	prop = of_find_property(dt, "himax,display-coords", NULL);
	if (prop) {
		coords_size = prop->length / sizeof(u32);
		if (coords_size != 4)
			D(" %s:Invalid display coords size %d\n", __func__,
			  coords_size);
	}
	rc = of_property_read_u32_array(dt, "himax,display-coords", coords,
					coords_size);
	if (rc && (rc != -EINVAL)) {
		D(" %s:Fail to read display-coords %d\n", __func__, rc);
		return rc;
	}
	pdata->screenWidth = coords[1];
	pdata->screenHeight = coords[3];
	I(" DT:display-coords = (%d, %d)\n", pdata->screenWidth,
	  pdata->screenHeight);

	pdata->TSIX = of_get_named_gpio(dt, "himax,TSIX", 0);

	if (!gpio_is_valid(pdata->TSIX))
		I(" DT:TSIX value is not valid\n");

	pdata->fail_det = of_get_named_gpio(dt, "himax,FAIL-DET", 0);

	if (!gpio_is_valid(pdata->fail_det))
		I(" DT:FAIL-DET value is not valid\n");

	pdata->tp_ext_rstn = of_get_named_gpio(dt, "himax,TP-EXT-RSTN", 0);
	if (!gpio_is_valid(pdata->tp_ext_rstn))
		I(" DT:TP-EXT-RSTN value is not valid\n");

	pdata->PON = of_get_named_gpio(dt, "himax,PON", 0);

	if (!gpio_is_valid(pdata->PON))
		I(" DT:PON value is not valid\n");

	pdata->RESX = of_get_named_gpio(dt, "himax,RESX", 0);

	if (!gpio_is_valid(pdata->RESX))
		I(" DT:RESX value is not valid\n");

	I(" DT:PON=%d, RESX=%d\n", pdata->PON, pdata->RESX);
	I(" DT:TSIX=%d, TP-EXT-RSTN=%d\n", pdata->TSIX, pdata->tp_ext_rstn);
	I(" DT:FAIL-DET=%d", pdata->fail_det);

	if (of_property_read_u32(dt, "report_type", &data) == 0) {
		pdata->protocol_type = data;
		I(" DT:protocol_type=%d\n", pdata->protocol_type);
	}
	if (of_property_read_string(dt, "phys-port", &pdata->phys_port))
		I("DT: get phys-port fail in dts\n");
	return 0;
}
//EXPORT_SYMBOL(himax_parse_dt);

int himax_bus_read(struct himax_device *hdev, uint8_t command, uint8_t *data,
			uint32_t length, uint8_t toRetry)
{
	int retry;
	int ret = 0;
	struct i2c_client *client = hdev->client;
	struct i2c_msg msg[] = { {
					 .addr = client->addr,
					 .flags = 0,
					 .len = 1,
					 .buf = &command,
				},
				{
					 .addr = client->addr,
					 .flags = I2C_M_RD,
					 .len = length,
					 .buf = hdev->gp_rw_buf,
				} };
	mutex_lock(&hdev->private_ts->rw_lock);

	for (retry = 0; retry < toRetry; retry++) {
		ret = i2c_transfer(client->adapter, msg, 2);
		if (ret == 2) {
			memcpy(data, hdev->gp_rw_buf, length);
			break;
		}
		/*msleep(20);*/
	}

	if (retry == toRetry) {
		E("%s: i2c_read_block retry over %d\n", __func__, toRetry);
		mutex_unlock(&hdev->private_ts->rw_lock);
#if (HX_RST_PIN_FUNC == 0x01)
		himax_tp_ext_rstn_reset(hdev->private_ts);
#endif
		return -EIO;
	}

	mutex_unlock(&hdev->private_ts->rw_lock);
	return 0;
}
//EXPORT_SYMBOL(himax_bus_read);

int himax_bus_write(struct himax_device *hdev, uint8_t command, uint8_t *data,
			uint32_t length, uint8_t toRetry)
{
	int retry;
	int ret = 0;
	struct i2c_client *client = hdev->client;
	struct i2c_msg msg[] = { {
		.addr = client->addr,
		.flags = 0,
		.len = length + 1,
		.buf = hdev->gp_rw_buf,
	} };

	mutex_lock(&hdev->private_ts->rw_lock);
	hdev->gp_rw_buf[0] = command;
	if (data != NULL)
		memcpy(hdev->gp_rw_buf + 1, data, length);

	for (retry = 0; retry < toRetry; retry++) {
		ret = i2c_transfer(client->adapter, msg, 1);
		if (ret == 1)
			break;
		msleep(20);
	}

	if (retry == toRetry) {
		E("%s: i2c_write_block retry over %d\n", __func__, toRetry);
		mutex_unlock(&hdev->private_ts->rw_lock);
#if (HX_RST_PIN_FUNC == 0x01)
		himax_tp_ext_rstn_reset(hdev->private_ts);
#endif
		return -EIO;
	}

	mutex_unlock(&hdev->private_ts->rw_lock);
	return 0;
}
//EXPORT_SYMBOL(himax_bus_write);

int himax_bus_read_slave(struct himax_device *hdev, uint8_t device,
			uint8_t command, uint8_t *data, uint32_t length, uint8_t toRetry)
{
	int retry;
	int ret = 0;
	struct i2c_client *client = hdev->client;
	struct i2c_msg msg[] = { {
					 .addr = (client->addr+device),
					 .flags = 0,
					 .len = 1,
					 .buf = &command,
				},
				{
					 .addr = (client->addr+device),
					 .flags = I2C_M_RD,
					 .len = length,
					 .buf = hdev->gp_rw_buf,
				} };
	mutex_lock(&hdev->private_ts->rw_lock);

	for (retry = 0; retry < toRetry; retry++) {
		ret = i2c_transfer(client->adapter, msg, 2);
		if (ret == 2) {
			memcpy(data, hdev->gp_rw_buf, length);
			break;
		}
		/*msleep(20);*/
	}

	if (retry == toRetry) {
		if (toRetry == HIMAX_I2C_RETRY_TIMES) {
			E("%s: i2c_read_block retry over %d\n", __func__, toRetry);
			mutex_unlock(&hdev->private_ts->rw_lock);
#if (HX_RST_PIN_FUNC == 0x01)
			himax_tp_ext_rstn_reset(hdev->private_ts);
#endif
		} else {
			mutex_unlock(&hdev->private_ts->rw_lock);
		}
		return -EIO;
	}

	mutex_unlock(&hdev->private_ts->rw_lock);
	return 0;
}
//EXPORT_SYMBOL(himax_bus_read_slave);

int himax_bus_write_slave(struct himax_device *hdev, uint8_t device,
			uint8_t command, uint8_t *data, uint32_t length, uint8_t toRetry)
{
	int retry;
	int ret = 0;
	struct i2c_client *client = hdev->client;
	struct i2c_msg msg[] = { {
		.addr = (client->addr+device),
		.flags = 0,
		.len = length + 1,
		.buf = hdev->gp_rw_buf,
	} };

	mutex_lock(&hdev->private_ts->rw_lock);
	hdev->gp_rw_buf[0] = command;
	if (data != NULL)
		memcpy(hdev->gp_rw_buf + 1, data, length);

	for (retry = 0; retry < toRetry; retry++) {
		ret = i2c_transfer(client->adapter, msg, 1);
		if (ret == 1)
			break;
		/*msleep(20);*/
	}

	if (retry == toRetry) {
		E("%s: i2c_write_block retry over %d\n", __func__, toRetry);
		mutex_unlock(&hdev->private_ts->rw_lock);
#if (HX_RST_PIN_FUNC == 0x01)
		himax_tp_ext_rstn_reset(hdev->private_ts);
#endif
		return -EIO;
	}

	mutex_unlock(&hdev->private_ts->rw_lock);
	return 0;
}
//EXPORT_SYMBOL(himax_bus_write_slave);

void himax_int_enable(struct himax_device *hdev, int enable)
{
	//struct himax_ts_data *ts = private_ts;
	struct himax_ts_data *ts = hdev->private_ts;
	unsigned long irqflags = 0;
	int irqnum = hdev->client->irq;

	if (enable == 1)
		himax_mcu_clear_event_stack(hdev);

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	I("%s: Entering!\n", __func__);
	if (enable == 1 && atomic_read(&ts->irq_state) == 0) {
		atomic_set(&ts->irq_state, 1);
		enable_irq(irqnum);
		ts->irq_enabled = 1;
	} else if (enable == 0 && atomic_read(&ts->irq_state) == 1) {
		atomic_set(&ts->irq_state, 0);
		disable_irq_nosync(irqnum);
		ts->irq_enabled = 0;
	}

	I("enable = %d\n", enable);
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}
//EXPORT_SYMBOL(himax_int_enable);


void himax_gpio_set(int pinnum, uint8_t value)
{
	gpio_direction_output(pinnum, value);
}
//EXPORT_SYMBOL(himax_gpio_set);


uint8_t himax_int_gpio_read(int pinnum)
{
	return gpio_get_value(pinnum);
}

int himax_gpio_power_config(struct himax_device *hdev,
			struct himax_i2c_platform_data *pdata)
{
	int error = 0;
	struct i2c_client *client = hdev->client;
#if (HX_RST_PIN_FUNC == 0x01)
	if (pdata->tp_ext_rstn >= 0) {
		error = gpio_request(pdata->tp_ext_rstn, "tp_ext_rstn");

		if (error < 0) {
			E("%s: request tp-reset pin failed\n", __func__);
			goto err_tp_ext_rstn_req;
		}

		error = gpio_direction_output(pdata->tp_ext_rstn, 0);

		if (error) {
			E("unable to set direction for tp-reset [%d]\n",
			  pdata->tp_ext_rstn);
			goto err_tp_ext_rstn_dir;
		}

		if (gpio_get_value(pdata->tp_ext_rstn) == 1) {
			E("unable control TP_EXT_RSTN, please check it\n");
			pdata->g_customer_control_tp_reset = 1;
		} else {
			pdata->g_customer_control_tp_reset = 0;
		}
	}
#elif (HX_RST_PIN_FUNC == 0x02)
	/* Need Customer pull TP reset pin low */
#endif

	if (pdata->RESX >= 0) {
		error = gpio_request(pdata->RESX, "RESX");

		if (error < 0) {
			E("%s: request RESX pin failed\n", __func__);
			goto err_RESX_req;
		}

		error = gpio_direction_output(pdata->RESX, 0);
		if (error) {
			E("unable to set direction for RESX [%d]\n",
			  pdata->RESX);
			goto err_RESX_dir;
		}
	}

	if (gpio_is_valid(pdata->PON)) {
		error = gpio_request(pdata->PON, "PON");

		if (error) {
			E("unable to request PON [%d]\n", pdata->PON);
			goto err_PON_req;
		}

		error = gpio_direction_output(pdata->PON, 0);

		if (error) {
			E("unable to set direction for PON [%d]\n", pdata->PON);
			goto err_PON_dir;
		}
	}

	if (gpio_is_valid(pdata->TSIX)) {
		/* configure touchscreen irq gpio */
		error = gpio_request(pdata->TSIX, "TSIX");

		if (error) {
			E("unable to request TSIX [%d]\n", pdata->TSIX);
			goto err_TSIX_req;
		}

		error = gpio_direction_input(pdata->TSIX);

		if (error) {
			E("unable to set direction for TSIX [%d]\n",
			  pdata->TSIX);
			goto err_TSIX_set_input;
		}

		client->irq = gpio_to_irq(pdata->TSIX);
		if (client->irq < 0)
			E("fail to request IRQ: %d\n", client->irq);

		hdev->private_ts->hx_irq = client->irq;
	} else {
		E("TSIX gpio not provided\n");
		goto err_TSIX_req;
	}

	if (gpio_is_valid(pdata->fail_det)) {
		/* configure touchscreen fail_det gpio */
		error = gpio_request(pdata->fail_det, "FAIL-DET");

		if (error)
			E("unable to request FAIL-DET [%d]\n", pdata->fail_det);

		error = gpio_direction_input(pdata->fail_det);

		if (error)
			E("unable to set direction for FAIL-DET [%d]\n",
			  pdata->fail_det);

		hdev->hx_fail_det_irq = gpio_to_irq(pdata->fail_det);

	} else {
		I("FAIL-DET not provided\n");
	}

	usleep_range(6000, 6100);

	if (pdata->RESX >= 0) {
		error = gpio_direction_output(pdata->RESX, 1);

		if (error) {
			E("RESX unable to set direction for RESX [%d]\n",
			  pdata->RESX);
			goto err_lcm_reset_set_high;
		}
	}
	usleep_range(1000, 1100);

#if (HX_RST_PIN_FUNC == 0x01)

	if (pdata->tp_ext_rstn >= 0) {
		error = gpio_direction_output(pdata->tp_ext_rstn, 1);

		if (error) {
			E("unable to set direction for tp_ext_rstn [%d]\n",
			  pdata->tp_ext_rstn);
			goto err_tp_ext_rstn_set_high;
		}
	}
#elif (HX_RST_PIN_FUNC == 0x02)
	/* Need Customer pull TP reset pin high */
#endif

	msleep(95);

	if (gpio_is_valid(pdata->PON)) {
		error = gpio_direction_output(pdata->PON, 1);

		if (error) {
			E("PON unable to set direction for PON [%d]\n",
			  pdata->PON);
			goto err_PON_set_high;
		}
	}

	return error;

err_PON_set_high:
#if (HX_RST_PIN_FUNC == 0x01)
err_tp_ext_rstn_set_high:
#endif
err_lcm_reset_set_high:
err_TSIX_set_input:
	if (gpio_is_valid(pdata->TSIX))
		gpio_free(pdata->TSIX);
err_TSIX_req:
err_PON_dir:
	if (gpio_is_valid(pdata->PON))
		gpio_free(pdata->PON);
err_PON_req:
err_RESX_dir:
	if (gpio_is_valid(pdata->RESX))
		gpio_free(pdata->RESX);
err_RESX_req:

#if (HX_RST_PIN_FUNC == 0x01)
err_tp_ext_rstn_dir:
	if (pdata->tp_ext_rstn >= 0)
		gpio_free(pdata->tp_ext_rstn);
err_tp_ext_rstn_req:
#endif
	return error;
}

void himax_gpio_power_deconfig(struct himax_i2c_platform_data *pdata)
{
	int error = 0;

	if (gpio_is_valid(pdata->PON)) {
		error = gpio_direction_output(pdata->PON, 0);

		if (error)
			E("unable to set direction for PON [%d]\n", pdata->PON);
	}

#if (HX_RST_PIN_FUNC == 0x01)
	if (pdata->tp_ext_rstn >= 0) {
		error = gpio_direction_output(pdata->tp_ext_rstn, 0);

		if (error) {
			E("unable to set direction for tp-reset [%d]\n",
			  pdata->tp_ext_rstn);
		}
	}
#elif (HX_RST_PIN_FUNC == 0x02)
	/* Need Customer pull TP reset pin low */
#endif

	if (pdata->RESX >= 0) {
		error = gpio_direction_output(pdata->RESX, 0);
		if (error) {
			E("unable to set direction for RESX [%d]\n",
			  pdata->RESX);
		}
	}

	if (gpio_is_valid(pdata->TSIX))
		gpio_free(pdata->TSIX);
	if (gpio_is_valid(pdata->fail_det))
		gpio_free(pdata->fail_det);
#if (HX_RST_PIN_FUNC == 0x01)
	if (gpio_is_valid(pdata->tp_ext_rstn))
		gpio_free(pdata->tp_ext_rstn);
#endif
	if (gpio_is_valid(pdata->PON))
		gpio_free(pdata->PON);
	if (gpio_is_valid(pdata->RESX))
		gpio_free(pdata->RESX);
}

static void himax_ts_isr_func(struct himax_device *hdev)
{
	himax_ts_work(hdev);
}

irqreturn_t himax_ts_thread(int irq, void *ptr)
{
	himax_ts_isr_func((struct himax_device *)ptr);

	return IRQ_HANDLED;
}

static void himax_ts_work_func(struct work_struct *work)
{
	struct himax_ts_data *ts =
		container_of(work, struct himax_ts_data, work);
	struct himax_device *hdev =
		container_of(&ts, struct himax_device, private_ts);

	himax_ts_work(hdev);
}

irqreturn_t himax_fail_det_thread(int irq, void *ptr)
{
	himax_fail_det_work((struct himax_device *)ptr);

	return IRQ_HANDLED;
}

int himax_int_register_trigger(struct himax_device *hdev)
{
	int ret = 0;
	struct himax_ic_data *ic_data = hdev->ic_data;
	struct i2c_client *client = hdev->client;

	if (ic_data->HX_INT_IS_EDGE) {
		I("%s edge trigger falling\n", __func__);
		ret = request_threaded_irq(client->irq, NULL, himax_ts_thread,
					   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					   client->name, hdev);
	}

	else {
		I("%s level trigger low\n", __func__);
		ret = request_threaded_irq(client->irq, NULL, himax_ts_thread,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					   client->name, hdev);
	}

	return ret;
}

int himax_fail_det_int_register_trigger(struct himax_device *hdev)
{
	int ret = 0;

	char *hx_fail_det_name = "hx_fail_det";

	I("%s level trigger high\n", __func__);
	ret = request_threaded_irq(hdev->hx_fail_det_irq, NULL, himax_fail_det_thread,
				   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				   hx_fail_det_name, (void *)hdev);

	return ret;
}

int himax_fail_det_register_interrupt(struct himax_device *hdev)
{
	int ret = 0;

	if (hdev->hx_fail_det_irq) {
		ret = himax_fail_det_int_register_trigger(hdev);

		if (ret == 0)
			I("%s: fail_det enabled at gpio: %d\n", __func__,
			  hdev->hx_fail_det_irq);
		else
			E("%s: request fail_det failed\n", __func__);

	} else {
		I("%s: hx_fail_det is empty.\n", __func__);
	}

	return ret;
}

int himax_ts_register_interrupt(struct himax_device *hdev)
{
	struct himax_ts_data *ts = hdev->private_ts;
	struct i2c_client *client = hdev->client;
	int ret = 0;

	ts->irq_enabled = 0;

	/* Work functon */
	if (client->irq && ts->hx_irq) { /*INT mode*/
		ts->use_irq = 1;
		ret = himax_int_register_trigger(hdev);

		if (ret == 0) {
			ts->irq_enabled = 1;
			atomic_set(&ts->irq_state, 1);
			I("%s: irq enabled at gpio: %d\n", __func__,
			  client->irq);
		} else {
			ts->use_irq = 0;
			E("%s: request_irq failed\n", __func__);
		}
	} else {
		I("%s: client->irq is empty, use polling mode.\n", __func__);
	}

	/*if use polling mode need to disable HX_ESD_RECOVERY function*/
	if (!ts->use_irq) {
		ts->himax_wq = create_singlethread_workqueue("himax_touch");
		INIT_WORK(&ts->work, himax_ts_work_func);
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = himax_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		I("%s: polling mode enabled\n", __func__);
	}

	return ret;
}

int himax_ts_unregister_interrupt(struct himax_device *hdev)
{
	struct himax_ts_data *ts = hdev->private_ts;
	int ret = 0;

	I("%s: entered.\n", __func__);

	/* Work functon */
	if (ts->hx_irq && ts->use_irq) { /*INT mode*/

		free_irq(ts->hx_irq, ts);
		I("%s: irq disabled at qpio: %d\n", __func__,
		  ts->hx_irq);
	}
	if (hdev->hx_fail_det_irq) {
		free_irq(hdev->hx_fail_det_irq, NULL);
		I("%s: irq disabled at qpio: %d\n", __func__, hdev->hx_fail_det_irq);
	}
	/*if use polling mode need to disable HX_ESD_RECOVERY function*/
	if (!ts->use_irq) {
		hrtimer_cancel(&ts->timer);
		cancel_work_sync(&ts->work);
		if (ts->himax_wq != NULL)
			destroy_workqueue(ts->himax_wq);
		I("%s: polling mode destroyed", __func__);
	}

	return ret;
}

static int himax_common_suspend(struct device *dev)
{
	struct himax_device *hdev = dev_get_drvdata(dev);

	I("%s: enter\n", __func__);
	himax_chip_common_suspend(hdev);
	return 0;
}

static int himax_common_resume(struct device *dev)
{
	struct himax_device *hdev = dev_get_drvdata(dev);

	I("%s: enter\n", __func__);
	himax_chip_common_resume(hdev);
	return 0;
}

int himax_chip_common_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	int ret = 0;
	struct himax_device *hdev;

	hdev = kzalloc(sizeof(struct himax_device), GFP_KERNEL);
	if (hdev == NULL) {
		E("%s: allocate himax_device failed\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_dev_failed;
	}

	hdev->gp_rw_buf = kcalloc(BUS_RW_MAX_LEN, sizeof(uint8_t), GFP_KERNEL);
	if (hdev->gp_rw_buf == NULL) {
		E("Allocate I2C RW Buffer failed\n");
		ret = -ENODEV;
		goto err_alloc_rw_buf_failed;
	}

	/* Check I2C functionality */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		E("%s: i2c check functionality error\n", __func__);
		ret = -ENODEV;
		goto err_i2c_functionality_failed;
	}

	hdev->private_ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
	if (hdev->private_ts == NULL) {
		E("%s: allocate himax_ts_data failed\n", __func__);
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	i2c_set_clientdata(client, hdev);
	hdev->client = client;
	hdev->dev = &client->dev;
	mutex_init(&hdev->private_ts->rw_lock);

	I("%s: %d-bit I2C address: 0x%02hx\n", __func__,
	  client->flags & I2C_CLIENT_TEN ? 10 : 7, client->addr);

	hdev->private_ts->initialized = false;
	ret = himax_chip_common_init(hdev);
	if (ret < 0)
		goto err_common_init_failed;

	return ret;

err_common_init_failed:
	kfree(hdev->private_ts);
err_alloc_data_failed:
err_i2c_functionality_failed:
	kfree(hdev->gp_rw_buf);
err_alloc_rw_buf_failed:
	kfree(hdev);
err_alloc_dev_failed:

	return ret;
}

int himax_chip_common_remove(struct i2c_client *client)
{
	struct himax_device *hdev = i2c_get_clientdata(client);

	if (hdev->hx_chip_inited)
		himax_chip_common_deinit(hdev);

	kfree(hdev->gp_rw_buf);
	kfree(hdev);

	return 0;
}

static const struct i2c_device_id himax_common_ts_id[] = {
	{ HIMAX_common_NAME, 0 },
	{},
};

static const struct dev_pm_ops himax_common_pm_ops = {
	.suspend = himax_common_suspend,
	.resume = himax_common_resume,
};

#if defined(CONFIG_OF)
static const struct of_device_id himax_match_table[] = {
	{ .compatible = "himax,hxcommon" },
	{},
};
#else
#define himax_match_table NULL
#endif

static struct i2c_driver himax_common_driver = {
	.id_table	= himax_common_ts_id,
	.probe		= himax_chip_common_probe,
	.remove		= himax_chip_common_remove,
	.driver		= {
		.name = HIMAX_common_NAME,
		.owner = THIS_MODULE,
		.of_match_table = himax_match_table,
#if defined(CONFIG_PM)
		.pm				= &himax_common_pm_ops,
#endif
	},
};

static int __init himax_common_init(void)
{
	I("Himax common touch panel driver init\n");
	D("Himax check double loading\n");
	i2c_add_driver(&himax_common_driver);

	return 0;
}

static void __exit himax_common_exit(void)
{
	i2c_del_driver(&himax_common_driver);
}

module_init(himax_common_init);
module_exit(himax_common_exit);

MODULE_DESCRIPTION("Himax_common driver");
MODULE_LICENSE("GPL");
