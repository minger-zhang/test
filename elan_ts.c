/*
 * Elan Microelectronics touch panels with I2C interface
 *
 * Copyright (C) 2014 Elan Microelectronics Corporation.
 * Scott Liu <scott.liu@emc.com.tw>
 *
 * This code is partly based on hid-multitouch.c:
 *
 *  Copyright (c) 2010-2012 Stephane Chatty <chatty@enac.fr>
 *  Copyright (c) 2010-2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 *  Copyright (c) 2010-2012 Ecole Nationale de l'Aviation Civile, France
 *
 *
 * This code is partly based on i2c-hid.c:
 *
 * Copyright (c) 2012 Benjamin Tissoires <benjamin.tissoires@gmail.com>
 * Copyright (c) 2012 Ecole Nationale de l'Aviation Civile, France
 * Copyright (c) 2012 Red Hat, Inc

 */

/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/errno.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <linux/firmware.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
    #include <linux/pm.h>
    #include <linux/earlysuspend.h>
#endif
#include <linux/miscdevice.h>
#include "elan_ts.h"
#include <linux/fs.h>
/*#define SWAP_X_Y_RESOLUTION*/
#ifdef ELAN_HID_PEN
static unsigned int gPrint_point;
#endif
struct elan_i2c_platform_data {
	int intr_gpio;
	int rst_gpio;
	int elan_irq;
};

struct elan_ts_data {
	struct elan_i2c_platform_data *pdata;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	bool power_enabled;
    /*Firmware Information*/
	int fw_ver;
	int fw_id;
	int fw_bcd;
	int bc_ver;
	int x_resolution;
	int y_resolution;
	int x_trace;
	int y_trace;
	int hand_osr;
	int pen_osr;
#ifdef ELAN_HID_IIC
	int pen_x_res;
	int pen_y_res;
#endif
/*queue or thread handler interrupt*/
	struct task_struct *work_thread;
	int recover;
	int power_lock;
/*For Firmare Update*/
	struct miscdevice firmware;
	struct wake_lock wakelock;
	struct proc_dir_entry *p;
};

static struct workqueue_struct *init_elan_ic_wq;
static struct delayed_work init_work;
static unsigned long delay = 2*HZ;

static struct elan_ts_data *private_ts;

#define FT_VTG_MIN_UV		2850000
#define FT_VTG_MAX_UV		2850000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000

/*galobal function*/
static int __hello_packet_handler(struct i2c_client *client);
static int __fw_packet_handler(struct i2c_client *client);
static int elan_ts_rough_calibrate(struct i2c_client *client);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ts_early_suspend(struct early_suspend *h);
static void elan_ts_late_resume(struct early_suspend *h);
#endif

#if defined IAP_PORTION
static void check_update_flage(struct elan_ts_data *ts);
#endif

int PageNum;
int tpd_flag;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static void elan_reset(void)
{
	dev_err(&private_ts->client->dev,
			"[elan]:%s enter\n", __func__);
	gpio_set_value(private_ts->pdata->rst_gpio, 1);
	mdelay(10);
	gpio_set_value(private_ts->pdata->rst_gpio, 0);
	mdelay(20);
	gpio_set_value(private_ts->pdata->rst_gpio, 1);
	mdelay(10);
}

static void elan_switch_irq(int on)
{
	dev_dbg(&private_ts->client->dev,
			"[elan] %s enter, irq = %d, on = %d\n",
			__func__, private_ts->pdata->elan_irq, on);
	if (on)
		enable_irq(private_ts->pdata->elan_irq);
	else
		disable_irq(private_ts->pdata->elan_irq);
}

static int elan_ts_poll(void)
{
	int status = 0, retry = 20;

	do {
		status = gpio_get_value(private_ts->pdata->intr_gpio);
		dev_dbg(&private_ts->client->dev,
				"%s: status = %d\n", __func__, status);
		if (status == 0)
			break;
		retry--;
		mdelay(20);
	} while (status == 1 && retry > 0);

	dev_dbg(&private_ts->client->dev,
			"%s: poll interrupt status %s\n",\
			__func__, status == 1 ? "high" : "low");

	return status == 0 ? 0 : -ETIMEDOUT;
}

static int elan_i2c_send(const struct i2c_client *client,
		const char *buf, int count)
{
	int ret;
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;


	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (char *)buf;

	ret = i2c_transfer(adap, &msg, 1);

    /*
     * If everything went ok (i.e. 1 msg transmitted), return #bytes
     * transmitted, else error code.
     */
	return (ret == 1) ? count : ret;
}

static int elan_i2c_recv(const struct i2c_client *client, char *buf, int count)
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = buf;

	ret = i2c_transfer(adap, &msg, 1);

	/*
     * If everything went ok (i.e. 1 msg received), return #bytes received,
     * else error code.
     */

	return (ret == 1) ? count : ret;
}

static int elan_ts_send_cmd(struct i2c_client *client,
		uint8_t *cmd, size_t size)
{
#ifdef ELAN_HID_IIC
	dev_dbg(&client->dev,
			"dump cmd: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x,\n",
			cmd[0], cmd[1], cmd[2], cmd[3],
			cmd[4], cmd[5], cmd[6], cmd[7]);
#else
	dev_dbg(&client->dev,
			"dump cmd: %02x:%02x:%02x:%02x\n",
			cmd[0], cmd[1], cmd[2], cmd[3]);
#endif
	if (elan_i2c_send(client, cmd, size) != size) {
		dev_err(&client->dev,
				"[elan error]%s: elan_ts_send_cmd failed\n",
				__func__);
		return -EINVAL;
	} else
		dev_err(&client->dev,
				"[elan] elan_ts_send_cmd ok");

	return size;
}

#ifndef ELAN_HID_IIC
static int elan_ts_get_data(struct i2c_client *client, uint8_t *cmd,
		uint8_t *buf, size_t size)
{
	int rc;

	if (buf == NULL)
		return -EINVAL;

	if (elan_ts_send_cmd(client, cmd, size) != size)
		return -EINVAL;

	mdelay(2);

	rc = elan_ts_poll();
	if (rc < 0)
		return -EINVAL;
	else{
		rc = elan_i2c_recv(client, buf, size);
		if (buf[0] != CMD_S_PKT || rc != size) {
			dev_err(&client->dev,
					"[elan error]%s: cmd respone error\n",\
					__func__);
			return -EINVAL;
		}
	}

	return 0;
}
#endif

#ifdef ELAN_HID_IIC
static int __hello_packet_handler(struct i2c_client *client)
{
	return 0;
}
#else
static int __hello_packet_handler(struct i2c_client *client)
{
	struct elan_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	uint8_t buf_recv[8] = { 0 };

	rc = elan_ts_poll();
	if (rc != 0)
		dev_err(&client->dev,
				"[elan error] %s: Int poll 55 55 55 55 failed!\n",
				__func__);

	rc = elan_i2c_recv(client, buf_recv, sizeof(buf_recv));
	if (rc != sizeof(buf_recv))
		dev_err(&client->dev,
				"[elan error] __hello_packet_handler recv error,retry\n");

	dev_info(&client->dev,
			"%s: hello packet %2x:%2x:%2x:%2x\n", __func__,
			buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);

	if (buf_recv[0] == 0x55
		&& buf_recv[1] == 0x55
		&& buf_recv[2] == 0x80
		&& buf_recv[3] == 0x80) {

		dev_dbg(&client->dev,
				"%s: boot code packet %2x:%2X:%2x:%2x\n",
				__func__, buf_recv[4], buf_recv[5],
				buf_recv[6], buf_recv[7]);
		ts->recover = 0x80;
		rc = ts->recover;
	} else if (buf_recv[0] == 0x55
			&& buf_recv[1] == 0x55
			&& buf_recv[2] == 0x55
			&& buf_recv[3] == 0x55) {

		dev_dbg(&client->dev,
				"__hello_packet_handler recv ok\n");
		ts->recover = 0x0;
		rc = ts->recover;
	}

	return rc;
}
#endif

int elan_read_fw_from_sdcard(char *file_path)
{
	mm_segment_t oldfs;
	struct file *firmware_fp;
	int ret = 0;
	int retry = 0;
	int  file_len;

	if (file_fw_data_out != NULL)
		kfree(file_fw_data_out);

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	for (retry = 0; retry < 5; retry++) {
		firmware_fp = filp_open(file_path, O_RDONLY, 0);
		if (IS_ERR(firmware_fp)) {
			dev_err(&private_ts->client->dev,
					"[elan] retry to open user bin filen\n");
			mdelay(1000);
		} else
			break;
	}

	if (retry >= 5) {
		dev_err(&private_ts->client->dev,
				"[elan error] open %s file error!\n",\
				file_path);
		ret = -1;
		goto out_read_fw_from_user2;
	} else
		dev_dbg(&private_ts->client->dev,
				"[elan] open %s file sucess!\n", file_path);

	file_len = firmware_fp->f_path.dentry->d_inode->i_size;
	if (file_len == 0) {
		dev_dbg(&private_ts->client->dev,
				"elan file len error !!!!!!\n");
		ret = -2;
		goto out_read_fw_from_user1;
	} else
		PageNum = file_len / 132;

	dev_dbg(&private_ts->client->dev,
			"[elan] user bin data len=%d\n", file_len);

	file_fw_data_out = kzalloc(file_len, GFP_KERNEL);
	if (file_fw_data == NULL) {
		dev_err(&private_ts->client->dev,
				"[elan error] mallco file_fw_data error\n");
		ret = -3;
		goto out_read_fw_from_user1;
	}

	ret = firmware_fp->f_op->read(firmware_fp, file_fw_data_out,
			file_len, &firmware_fp->f_pos);
	if (ret != file_len) {
		dev_err(&private_ts->client->dev,
				"[elan error] read BIN file size error,ret = %d!\n",
				ret);
		ret = -4;
	}

	ret = 0;

out_read_fw_from_user1:
	filp_close(firmware_fp, NULL);
out_read_fw_from_user2:
	set_fs(oldfs);

	return ret;
}

int WritePage(struct i2c_client *client, uint8_t * szPage, int byte, int which)
{
	int len = 0;

	len = elan_i2c_send(client, szPage,  byte);
	if (len != byte) {
		dev_err(&client->dev,
				"[elan] ERROR: write the %d th page error,len=%d\n",
				which, len);
		return -1;
	}

	return 0;
}

/*every page write to recv 2 bytes ack */
int GetAckData(struct i2c_client *client)
{
	int len = 0;

#ifdef ELAN_HID_IIC
	uint8_t ack_buf[67] = { 0 };
	len = elan_ts_poll();
	len = elan_i2c_recv(client, ack_buf, 67);
	dev_dbg(&client->dev,
			"[elan] %s: %x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n",
			__func__ , ack_buf[0], ack_buf[1],
			ack_buf[2], ack_buf[3], ack_buf[4],
			ack_buf[5], ack_buf[6], ack_buf[7],
			ack_buf[8], ack_buf[9], ack_buf[10], ack_buf[11]);
#else
	uint8_t ack_buf[2] = { 0 };
	len = elan_i2c_recv(client, ack_buf, 2);
	if (len != 2) {
		dev_err(&client->dev,
				"[elan] ERROR: GetAckData. len=%d\r\n", len);
		return -1;
	}
#endif

#ifdef ELAN_HID_IIC
	if (ack_buf[4] == 0xaa && ack_buf[5] == 0xaa)
		return ACK_OK;
	else if (ack_buf[4] == 0x55 && ack_buf[5] == 0x55)
		return ACK_REWRITE;
	else
		return ACK_Fail;
#else
	if (ack_buf[0] == 0xaa && ack_buf[1] == 0xaa)
		return ACK_OK;
	else if (ack_buf[0] == 0x55 && ack_buf[1] == 0x55)
		return ACK_REWRITE;
	else
		return ACK_Fail;
#endif		

	return 0;
}

/* Check Master & Slave is "55 aa 33 cc" */
int CheckIapMode(struct i2c_client *client)
{
	char buff[4] = { 0 };
	int rc = 0;

	rc = elan_i2c_recv(client, buff, 4);
	if (rc != 4) {
		dev_err(&client->dev,
				"[elan] ERROR: CheckIapMode. len=%d\r\n", rc);
		return -1;
	} else
		dev_err(&client->dev,
				"Mode= 0x%x 0x%x 0x%x 0x%x\r\n",
				buff[0], buff[1], buff[2], buff[3]);

	return 0;
}

int check_2k_recover_mode(struct i2c_client *client)
{
	uint8_t buf_recv[8] = { 0 };
	int rc = 0;
	int retry_cnt = 0;

retry_to_check_mode:
	elan_reset();
	rc = elan_ts_poll();
	if (rc != 0)
		dev_err(&client->dev,
				"[elan error] %s: Int poll 55 55 55 55 failed!\n",
				__func__);

	rc = elan_i2c_recv(client, buf_recv, sizeof(buf_recv));
	if (rc != sizeof(buf_recv))
		dev_err(&client->dev,
				"[elan error] __hello_packet_handler recv error,retry\n");

	dev_dbg(&client->dev,
			"%s: hello packet %2x:%2X:%2x:%2x\n", __func__,
			buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);

	if (buf_recv[0] == 0x55
			&& buf_recv[1] == 0x55
			&& buf_recv[2] == 0x80
			&& buf_recv[3] == 0x80) {

		dev_dbg(&client->dev,
				"%s: boot code packet %2x:%2X:%2x:%2x\n",
				__func__, buf_recv[4], buf_recv[5],
				buf_recv[6], buf_recv[7]);
		mdelay(5);

		rc = elan_i2c_recv(client, buf_recv, sizeof(buf_recv));
		dev_dbg(&client->dev,
				"%s: enter mode check %2x:%2X:%2x:%2x\n",
				__func__, buf_recv[0], buf_recv[0],
				buf_recv[0], buf_recv[0]);
		mdelay(5);

		return 0x80;
	} else if (buf_recv[0] == 0x55
			&& buf_recv[1] == 0x55
			&& buf_recv[2] == 0x55
			&& buf_recv[3] == 0x55) {

		dev_dbg(&client->dev, " __hello_packet_handler recv ok\n");
		return 0;
	} else {
		retry_cnt++;
		if (retry_cnt < 3)
			goto retry_to_check_mode;
	}

	return -1;
}

#ifdef ELAN_HID_IIC
int SendEndCmd(struct i2c_client *client)
{
	int len = 0;
	uint8_t send_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x1A};

	len = elan_ts_send_cmd(private_ts->client, send_cmd, sizeof(send_cmd));
	if (len != sizeof(send_cmd)) {
		dev_err(&client->dev,
				"[elan] ERROR: Send Cmd fail!len=%d\r\n", len);
		return -1;
	} else
		dev_dbg(&client->dev,
				"[elan] check send cmd = [%x, %x, %x, %x, %x, %x]\n",
				send_cmd[0], send_cmd[1], send_cmd[2],
				send_cmd[3], send_cmd[4], send_cmd[5]);
	return 0;
}

int HID_EnterISPMode(struct i2c_client *client)
{
	int len = 0;
	int i;
	uint8_t flash_key[37]  = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00,
		0x04, 0x54, 0xc0, 0xe1, 0x5a};
	uint8_t isp_cmd[37]    = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00,
		0x04, 0x54, 0x00, 0x12, 0x34};
	uint8_t check_addr[37] = {0x04, 0x00, 0x23, 0x00, 0x03,
		0x00, 0x01, 0x10};
	uint8_t buf[67] = {0};

	len = elan_ts_send_cmd(private_ts->client, flash_key,
			sizeof(flash_key));
	if (len != sizeof(flash_key)) {
		dev_err(&client->dev,
				"[elan] ERROR: Flash key fail!len=%d\r\n",
				len);
		return -1;
	} else
		dev_dbg(&client->dev,
				"[elan] FLASH key cmd = [%2x, %2x, %2x, %2x]\n",
				flash_key[7], flash_key[8],
				flash_key[9], flash_key[10]);
	mdelay(20);

	len = elan_ts_send_cmd(private_ts->client, isp_cmd,
			sizeof(isp_cmd));
	if (len != 37) {
		dev_err(&client->dev,
				"[elan] ERROR: EnterISPMode fail!len=%d\r\n",
				len);
		return -1;
	} else
		dev_dbg(&client->dev,
				"[elan] IAPMode data cmd = [%2x, %2x, %2x, %2x]\n",
				isp_cmd[7], isp_cmd[8],
				isp_cmd[9], isp_cmd[10]);

	mdelay(20);
	len = elan_ts_send_cmd(private_ts->client, check_addr,
			sizeof(check_addr));
	if (len != sizeof(check_addr)) {
		dev_err(&client->dev,
				"[elan] ERROR: Check Address fail!len=%d\r\n",
				len);
		return -1;
	} else
		dev_dbg(&client->dev,
				"[elan] Check Address cmd = [%2x, %2x, %2x, %2x]\n",
				check_addr[7], check_addr[8],
				check_addr[9], check_addr[10]);

	mdelay(20);
	len = elan_i2c_recv(private_ts->client, buf, sizeof(buf));
	if (len != sizeof(buf)) {
		dev_err(&client->dev,
				"[elan] ERROR: Check Address Read Data error.len=%d\n",
				len);
		return -1;
	} else {
		dev_dbg(&client->dev, "[Check Addr]: ");
		for (i = 0; i < (37+3)/8; i++)
			dev_dbg(&client->dev,
					"%02x %02x %02x %02x %02x %02x %02x %02x",
					buf[i*8+0], buf[i*8+1], buf[i*8+2],
					buf[i*8+3], buf[i*8+4], buf[i*8+5],
					buf[i*8+6], buf[i*8+7]);
	}

	return 0;
}

int HID_RecoveryISP(struct i2c_client *client)
{
	int len = 0;
	int i;
	uint8_t flash_key[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00,
		0x04, 0x54, 0xc0, 0xe1, 0x5a};
	uint8_t check_addr[37] = {0x04, 0x00, 0x23,
		0x00, 0x03, 0x00, 0x01, 0x10};
	uint8_t buf[67] = {0};

	len = elan_ts_send_cmd(private_ts->client,
			flash_key, sizeof(flash_key));
	if (len != sizeof(flash_key)) {
		dev_err(&client->dev,
				"[elan] ERROR: Flash key fail!len=%d\n",
				len);
		return -1;
	} else
		dev_dbg(&client->dev,
				"[elan] FLASH key cmd = [%2x:%2x:%2x:%2x]\n",
				flash_key[7], flash_key[8],
				flash_key[9], flash_key[10]);

	mdelay(40);

	len = elan_ts_send_cmd(private_ts->client,
			check_addr, sizeof(check_addr));
	if (len != sizeof(check_addr)) {
		dev_err(&client->dev,
				"[elan] ERROR: Check Address fail!len=%d\n",
				len);
		return -1;
	} else
		dev_dbg(&client->dev,
				"[elan] Check Address cmd = [%2x:%2x:%2x:%2x]\n",
				check_addr[7], check_addr[8],
				check_addr[9], check_addr[10]);

	mdelay(20);
	len = elan_i2c_recv(private_ts->client, buf, sizeof(buf));
	if (len != sizeof(buf)) {
		dev_dbg(&client->dev,
				"[elan] ERROR: Check Address Read Data error.len=%d\n",
				len);
		return -1;
	} else {
		dev_dbg(&client->dev, "[elan][Check Addr]: ");
		for (i = 0; i < (37+3)/8; i++) {
			dev_dbg(&client->dev,
					"%02x %02x %02x %02x %02x %02x %02x %02x",
					 buf[i*8+0], buf[i*8+1], buf[i*8+2],
					 buf[i*8+3], buf[i*8+4], buf[i*8+5],
					 buf[i*8+6], buf[i*8+7]);
		}
	}

	return 0;
}

int CheckISPstatus(struct i2c_client *client)
{
	int len = 0;
	int i;
	uint8_t checkstatus[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x18};
	uint8_t buf[67] = {0};

	len = elan_ts_send_cmd(private_ts->client,
			checkstatus, sizeof(checkstatus));
	if (len != sizeof(checkstatus)) {
		dev_err(&client->dev,
				"[elan] ERROR: Flash key fail!len=%d\n",
				len);
		return -1;
	} else
		dev_dbg(&client->dev,
				"[elan] check status cmd = [%x:%x:%x:%x:%x:%x]\n",
				checkstatus[0], checkstatus[1], checkstatus[2],
				checkstatus[3], checkstatus[4], checkstatus[5]);

	mdelay(10);
	len = elan_i2c_recv(private_ts->client, buf, sizeof(buf));
	if (len != sizeof(buf)) {
		dev_err(&client->dev,
				"[elan] ERROR: Check Address Read Data error.len=%d\n",
				len);
		return -1;
	} else {
		dev_dbg(&client->dev, "[elan][Check status]: ");
		for (i = 0; i < (37+3)/8; i++) {
			dev_dbg(&client->dev,
					"%02x %02x %02x %02x %02x %02x %02x %02x",
					buf[i*8+0], buf[i*8+1], buf[i*8+2],
					buf[i*8+3], buf[i*8+4], buf[i*8+5],
					buf[i*8+6], buf[i*8+7]);
		}
		if (buf[6] == 0xa6)
			return 0xa6; /* return recovery mode*/
	}

	return 0;
}

static int FW_Update(struct i2c_client *client, int source)
{
	struct elan_ts_data *ts = i2c_get_clientdata(client);
	int res = 0;
	int iPage = 0;
	int j = 0;
	int write_times = 142;
	uint8_t data;
	int byte_count;
	const u8 *szBuff;
	const u8 *fw_data;
	const struct firmware *p_fw_entry;
	uint8_t write_buf[37] = {0x04, 0x00, 0x23, 0x00, 0x03,
		0x21, 0x00, 0x00, 0x1c};
	uint8_t cmd_iap_write[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x22};
	uint8_t cmd_idel[37] = {0x04, 0x00, 0x23, 0x00, 0x03,
		0x00, 0x04, 0x54, 0x2c, 0x03, 0x01};
	int curIndex = 0;
	int offset = 0;
	int retry_num = 0;
	int rc, fw_size;
	char *fw_name;
	char fw_local_path[50];

	dev_dbg(&ts->client->dev, "[elan] enter %s.\n", __func__);

	if (source == 1) {
		fw_name = kasprintf(GFP_KERNEL, "elants_i2c_%04x.bin",
				ts->fw_id);
		if (!fw_name)
			return -ENOMEM;
		dev_dbg(&ts->client->dev,
				"request_firmware name = %s\n", fw_name);
		rc = request_firmware(&p_fw_entry, fw_name, &client->dev);
		if (rc != 0) {
			dev_err(&ts->client->dev,
					"rc=%d, request_firmware fail\n", rc);
			return -1;
		} else
			dev_dbg(&ts->client->dev,
					"Firmware Size=%zu\n",
					p_fw_entry->size);
		fw_data = p_fw_entry->data;
		fw_size = p_fw_entry->size;
		PageNum = fw_size/132;
	} else if (source == 2) {
		fw_name = kasprintf(GFP_KERNEL, "elants_i2c_%04x.bin",
				ts->fw_id);
		sprintf(fw_local_path, "%s%s", "/sdcard/", fw_name);
		dev_dbg(&ts->client->dev, "[elan] Update Firmware from %s\n",
				fw_local_path);
		rc = elan_read_fw_from_sdcard(fw_local_path);
		if (rc != 0) {
			dev_err(&ts->client->dev, "read fw fail\n");
			return -1;
		}
		fw_data = file_fw_data_out;
	} else {
		PageNum = sizeof(file_fw_data)/132;
		fw_data = file_fw_data;
	}

	elan_switch_irq(0);

IAP_RESTART:
	data = ELAN_7BITS_ADDR;/*dummy byte*/
	dev_dbg(&ts->client->dev,
			"[ELAN] %s: address data=0x%x \r\n",
			__func__, data);

	elan_reset();
	msleep(200);
	res = CheckISPstatus(ts->client);
	dev_dbg(&ts->client->dev,
			"[elan] res: 0x%x\n", res);
	if (res == 0xa6) { /* 0xa6 recovery mode  */
		elan_reset();
		mdelay(200);
		dev_dbg(&ts->client->dev,
				"[elan hid iap] Recovery mode\n");
		res = HID_RecoveryISP(ts->client);
	} else {
		dev_dbg(&ts->client->dev, "[elan hid iap] Normal mode\n");
		res = HID_EnterISPMode(ts->client);/*enter HID ISP mode*/
	}

	mdelay(50);

	for (iPage = 1; iPage <= PageNum; iPage += 30) {
		offset = 0;
		if (iPage == 451)
			write_times = 137;
		else
			write_times = 142;  /*30*132=3960, 3960=141*28+12*/

		mdelay(5);
		for (byte_count = 1; byte_count <= write_times; byte_count++) {
			mdelay(1);
			if (byte_count != write_times) {
				/* header + data = 9+28=37 bytes*/
				szBuff = fw_data + curIndex;
				/*payload length = 0x1c => 28??*/
				write_buf[8] = 28;
				write_buf[7] = offset & 0x00ff;/*===>0?*/
				write_buf[6] = offset >> 8;/*==0?*/
				offset += 28;
				curIndex =  curIndex + 28;
				for (j = 0; j < 28; j++)
					write_buf[j+9] = szBuff[j];
				res = WritePage(ts->client, write_buf,
						37, iPage);
			} else if ((iPage == 451)
					&& (byte_count == write_times)) {
				/*the final page, header + data = 9+20=29 bytes,
				 *the rest of bytes are the previous page's data
				 */
				dev_dbg(&ts->client->dev, "[elan] Final Page...\n");
				szBuff = fw_data + curIndex;
				/*payload length = 0x14 => 20*/
				write_buf[8] = 20;
				write_buf[7] = offset & 0x00ff;
				write_buf[6] = offset >> 8;
				curIndex =  curIndex + 20;
				for (j = 0; j < 20; j++)
					write_buf[j+9] = szBuff[j];
				res = WritePage(ts->client, write_buf,
						37, iPage);
			} else {
				/*last run of this 30 page,
				 *header + data = 9+12=21 bytes,
				 *the rest of bytes are the previous page's data
				 */
				szBuff = fw_data + curIndex;
				/*payload length = 0x0c => 12*/
				write_buf[8] = 12;
				write_buf[7] = offset & 0x00ff;
				write_buf[6] = offset >> 8;
				curIndex =  curIndex + 12;
				for (j = 0; j < 12; j++)
					write_buf[j+9] = szBuff[j];
				res = WritePage(ts->client, write_buf,
						37, iPage);
			}
		} /*end of for(byte_count=1;byte_count<=17;byte_count++)*/

		mdelay(200);
		res = WritePage(ts->client, cmd_iap_write,
				37, iPage);
		mdelay(200);
		dev_dbg(&ts->client->dev, "[iap] iPage=%d :", iPage);
		res = GetAckData(ts->client);
		if (res < 0) {
			if (retry_num < 1) {
				dev_err(&ts->client->dev, "[elan] Update Firmware retry_num=%d fail!!\n",
						retry_num);
				retry_num++;
				goto IAP_RESTART;
			} else {
				dev_err(&ts->client->dev,
						"[elan] Update Firmware retry_num=%d!!\n",
						retry_num);
			}
		}

		mdelay(10);
	} /*end of for(iPage = 1; iPage <= PageNum; iPage++)*/

	res = SendEndCmd(ts->client);
	mdelay(200);
	elan_reset();
	mdelay(200);
	elan_ts_send_cmd(ts->client, cmd_idel, 37);
	dev_dbg(&ts->client->dev, "[elan] Update Firmware successfully!\n");
	elan_switch_irq(1);

	return 0;
}

#else

static int FW_Update(struct i2c_client *client, int source)
{
	struct elan_ts_data *ts = i2c_get_clientdata(client);
	int res = 0;
	int iPage = 0, rewriteCnt = 0;
	uint8_t data;
	int restartCnt = 0; /*For IAP_RESTART*/
	u8 *szBuff = NULL;
	int curIndex = 0;
	int rc, fw_size;
	uint8_t isp_cmd[] = {0x45, 0x49, 0x41, 0x50};
	u8 *fw_data;
	const struct firmware *p_fw_entry;
	const int PAGERETRY = 10;
	int retry_cnt;
	char fw_local_path[50];
	char *fw_name;
  int NEW_FW_VERSION = 0;
	int New_FW_ID = 0;
	int byte_count;
	
	if (source == 1) {
		fw_name = kasprintf(GFP_KERNEL, "elants_i2c.bin");
		if (fw_name == NULL)
			return -ENOMEM;
		dev_err(&client->dev, "request_firmware name = %s\n", fw_name);
		rc = request_firmware(&p_fw_entry, fw_name, &client->dev);
		if (rc != 0) {
			dev_err(&client->dev,
					"rc=%d, request_firmware fail\n", rc);
			return -1;
		} else
			dev_dbg(&client->dev, "Firmware Size=%zu\n",
					p_fw_entry->size);
		fw_data = (u8 *) p_fw_entry->data;
		fw_size = p_fw_entry->size;
		PageNum = fw_size/132;
		if(ts->recover != 0x80){
		New_FW_ID  = fw_data[0xE2CF]<<8  | fw_data[0xE2CE];
		NEW_FW_VERSION = fw_data[0xDEC3]<<8  | fw_data[0xDEC2];
		dev_err(&ts->client->dev, "[elan] FW_ID=0x%x, New_FW_ID=0x%x\n",
				ts->fw_id, New_FW_ID);
		dev_err(&ts->client->dev, "[elan] FW_VERSION=0x%x,New_FW_VER=0x%x\n",
				ts->fw_ver, NEW_FW_VERSION);
			if ((ts->fw_ver&0xff) >= (NEW_FW_VERSION&0xff)) {
				dev_dbg(&ts->client->dev, "[elan] fw version is newest!!\n");
				goto no_update_elan_fw;
			}
		} else 
			dev_err(&ts->client->dev, "[elan] enter recovery mode");
	} else if (source == 2) {
		fw_data = NULL;
		fw_size = 0;
		fw_name = kasprintf(GFP_KERNEL, "elants_i2c_%04x.bin",
				ts->fw_id);
		sprintf(fw_local_path, "%s%s", "/sdcard/", fw_name);
		dev_dbg(&client->dev, "[elan] Update Firmware from %s\n",\
				fw_local_path);
		rc = elan_read_fw_from_sdcard(fw_local_path);
		if (rc != 0) {
			dev_err(&client->dev, "read fw fail\n");
			return -1;
		}

		fw_data = file_fw_data_out;
	} else {
		PageNum = sizeof(file_fw_data) / 132;
		fw_data = file_fw_data;
	}

IAP_RESTART:
	data = ELAN_7BITS_ADDR;/*dummy byte*/
	dev_dbg(&client->dev,
			"[ELAN] %s: address data=0x%x \r\n",
			__func__, data);

	elan_reset();
	res = elan_ts_send_cmd(client, isp_cmd, sizeof(isp_cmd));
	mdelay(5);
	res = CheckIapMode(client);

    /*send dummy byte*/
	res = elan_i2c_send(client, &data,  sizeof(data));
	if (res != sizeof(data)) {
		dev_err(&client->dev,
				"[elan] dummy error code = %d\n", res);
		return -1;
	} else {
		dev_dbg(&client->dev,
				"[elan] send Dummy byte sucess data:%x", data);
	}

    /* Start IAP*/
	for (iPage = 1; iPage <= PageNum; iPage++) {
#if 1
PAGE_REWRITE:
        //write every page
        for(byte_count=1;byte_count<=5;byte_count++)
        {
            if(byte_count!=5)
            {
                szBuff = fw_data + curIndex;
                curIndex =  curIndex + 32;
                res = WritePage(client,szBuff, 32,iPage);
            }
            else{
                szBuff = fw_data + curIndex;
                curIndex =  curIndex + 4;
                res = WritePage(client,szBuff, 4,iPage);
            }
        } // end of for(byte_count=1;byte_count<=17;byte_count++)
#else        
		szBuff = fw_data + curIndex;
		curIndex =  curIndex + PageSize;

PAGE_REWRITE:
		res = WritePage(client, szBuff, PageSize, iPage);
#endif		
		if (iPage == PageNum || iPage == 1)
			mdelay(600);
		else
			mdelay(50);

		res = GetAckData(private_ts->client);
		if (ACK_OK != res) {
			mdelay(50);
			dev_err(&client->dev,
					"[ELAN] ERROR: GetAckData fail! res=%d\n",
					res);
			if (res == ACK_REWRITE) {
				rewriteCnt = rewriteCnt + 1;
				if (rewriteCnt == PAGERETRY) {
					dev_err(&client->dev,
							"[ELAN] ID 0x%02x %dth page ReWrite%d times fails!\n",
							data, iPage, PAGERETRY);
				   return E_FD;
				} else {
					dev_dbg(&client->dev,
							"[ELAN] ---%d---page ReWrite %d times!\n",
							iPage, rewriteCnt);
					goto PAGE_REWRITE;
				}
			} else {
				restartCnt = restartCnt + 1;
				if (restartCnt >= 5) {
					dev_err(&client->dev,
							"[ELAN] ID 0x%02x ReStart %d times fails!\n",
							data, IAPRESTART);
					return E_FD;
				} else {
					dev_err(&client->dev,
							"[ELAN] ===%d=== page ReStart %d times!\n",
							iPage, restartCnt);
					goto IAP_RESTART;
				}
			}
		} else {
			rewriteCnt = 0;
			dev_dbg(&client->dev,
					"[elan]---%d--- page flash ok\n",
					iPage);
		}
		mdelay(10);
	} /*end of for(iPage = 1; iPage <= PageNum; iPage++)*/
no_update_elan_fw:
	elan_reset();
	res = __hello_packet_handler(client);
	mdelay(500);
	for (retry_cnt = 0; retry_cnt < 3; retry_cnt++) {
		rc = __fw_packet_handler(private_ts->client);
		if (rc < 0)
			dev_err(&client->dev,
					"[elan error] %s, fw_packet_handler fail,rc = %d\n",
					__func__, rc);
		else
			break;
	}

	return 0;
}
#endif

#ifdef SYS_DEFAULT_ATTR
static ssize_t store_disable_irq(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	disable_irq(private_ts->pdata->elan_irq);
	dev_info(&private_ts->client->dev, "Disable IRQ.\n");
	return count;
}
static DEVICE_ATTR(disable_irq, S_IWUSR, NULL, store_disable_irq);

static ssize_t store_enable_irq(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	enable_irq(private_ts->pdata->elan_irq);
	dev_info(&private_ts->client->dev, "Enable IRQ.\n");
	return count;
}
static DEVICE_ATTR(enable_irq, S_IWUSR, NULL, store_enable_irq);

static ssize_t store_reset(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	elan_reset();
	dev_info(&private_ts->client->dev,
			"Reset Touch Screen Controller!\n");
	return count;
}
static DEVICE_ATTR(reset, S_IWUSR, NULL, store_reset);

static ssize_t store_calibrate(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;

	ret = elan_ts_rough_calibrate(client);
	if (ret == 0)
		dev_info(&client->dev, "ELAN CALIBRATE Success\n");
	else
		dev_err(&client->dev, "ELAN CALIBRATE Fail\n");
	return count;
}
static DEVICE_ATTR(calibrate, S_IWUSR, NULL, store_calibrate);


static ssize_t show_gpio_int(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",
			gpio_get_value(private_ts->pdata->intr_gpio));
}
static DEVICE_ATTR(gpio_int, S_IRUGO, show_gpio_int, NULL);


static ssize_t store_update_fw_from_system(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	struct elan_ts_data *ts = private_ts;
	disable_irq(private_ts->pdata->elan_irq);
	ret = FW_Update(ts->client, 1);
	enable_irq(private_ts->pdata->elan_irq);
	if (ret == 0)
		dev_info(&ts->client->dev, "Update Touch Screen Success!\n");
	else
		dev_err(&ts->client->dev, "Update Touch Screen Fail!\n");
	return count;
}
static DEVICE_ATTR(update_fw_system, S_IWUSR, NULL,
		store_update_fw_from_system);


static ssize_t store_update_fw_from_sdcard(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	struct elan_ts_data *ts = private_ts;
	disable_irq(private_ts->pdata->elan_irq);
	ret = FW_Update(ts->client, 2);
	enable_irq(private_ts->pdata->elan_irq);
	if (ret == 0)
		dev_info(&ts->client->dev, "Update Touch Screen Success!\n");
	else
		dev_err(&ts->client->dev, "Update Touch Screen Fail!\n");
	return count;
}
static DEVICE_ATTR(update_fw_sdcard, S_IWUSR, NULL,
		store_update_fw_from_sdcard);

static ssize_t show_fw_info(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct elan_ts_data *ts = private_ts;
	return sprintf(buf, "FW VER = 0x%x, FW ID = 0x%x, BC VER = 0x%x\n", ts->fw_ver, ts->fw_id, ts->fw_bcd);
}
static ssize_t store_fw_info(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret;
	struct elan_ts_data *ts = private_ts;
	disable_irq(ts->pdata->elan_irq);
	ret = __fw_packet_handler(ts->client);
	enable_irq(ts->pdata->elan_irq);
	return count;
}
static DEVICE_ATTR(fw_info, S_IWUSR | S_IRUSR, show_fw_info,
		store_fw_info);

static struct attribute *elan_default_attributes[] = {
	&dev_attr_gpio_int.attr,
	&dev_attr_reset.attr,
	&dev_attr_enable_irq.attr,
	&dev_attr_disable_irq.attr,
	&dev_attr_calibrate.attr,
	&dev_attr_update_fw_sdcard.attr,
	&dev_attr_update_fw_system.attr,
	&dev_attr_fw_info.attr,
	NULL
};

static struct attribute_group elan_default_attribute_group = {
	.name = "elan_ktf",
	.attrs = elan_default_attributes,
};
#endif

#ifdef ELAN_IAP_DEV
int elan_iap_open(struct inode *inode, struct file *filp)
{
	dev_info(&private_ts->client->dev, "%s enter", __func__);
	if (private_ts == NULL)
		dev_err(&private_ts->client->dev,
				"[elan error]private_ts is NULL~~~");

	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp)
{
	dev_info(&private_ts->client->dev, "%s enter", __func__);
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff,
		size_t count, loff_t *offp)
{
	int ret;
	char *tmp;
	struct i2c_client *client = private_ts->client;

	dev_info(&client->dev, "%s enter", __func__);
	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	if (copy_from_user(tmp, buff, count))
		return -EFAULT;

	ret = elan_i2c_send(client, tmp, count);
	if (ret != count)
		dev_err(&client->dev,
				"[elan error]elan elan_i2c_send fail,ret=%d\n",
				ret);

	kfree(tmp);

	return ret;
}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp)
{
	char *tmp;
	int ret;
	long rc;
	struct i2c_client *client = private_ts->client;

	dev_info(&client->dev, "%s enter", __func__);

	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	ret = elan_i2c_recv(client, tmp, count);
	if (ret != count)
		dev_err(&client->dev,
				"[elan error]elan elan_i2c_recv fail,ret=%d\n",
				ret);

	if (ret == count)
		rc = copy_to_user(buff, tmp, count);

	kfree(tmp);
	return ret;
}

static long elan_iap_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	int __user *ip = (int __user *)arg;

	dev_info(&private_ts->client->dev,
			"%s enter, cmd value %x\n", __func__, cmd);

	switch (cmd) {
	case IOCTL_I2C_SLAVE:
		dev_info(&private_ts->client->dev,
				"pre addr is %X\n",  private_ts->client->addr);
		private_ts->client->addr = (int __user)arg;
		dev_info(&private_ts->client->dev,
				 "new addr is %X\n",  private_ts->client->addr);
		break;
	case IOCTL_RESET:
		elan_reset();
		break;
	case IOCTL_IAP_MODE_LOCK:
		if (private_ts->power_lock == 0) {
			private_ts->power_lock = 1;
			elan_switch_irq(0);
		}
		break;
	case IOCTL_IAP_MODE_UNLOCK:
		if (private_ts->power_lock == 1) {
			private_ts->power_lock = 0;
			elan_switch_irq(1);
		}
		break;
	case IOCTL_CHECK_RECOVERY_MODE:
		return private_ts->recover;
		break;
	case IOCTL_ROUGH_CALIBRATE:
		return elan_ts_rough_calibrate(private_ts->client);
	case IOCTL_I2C_INT:
		put_user(gpio_get_value(private_ts->pdata->intr_gpio), ip);
		break;
	default:
		break;
	}
	return 0;
}

const struct file_operations elan_touch_fops = {
	.open = elan_iap_open,
	.write = elan_iap_write,
	.read = elan_iap_read,
	.release =  elan_iap_release,
	.unlocked_ioctl = elan_iap_ioctl,
	.compat_ioctl = elan_iap_ioctl,
};
#endif

static void elan_touch_node_init(void)
{
	struct elan_ts_data *ts = private_ts;

#ifdef SYS_DEFAULT_ATTR
	if (sysfs_create_group(&ts->client->dev.kobj,
				&elan_default_attribute_group) < 0)
		dev_err(&ts->client->dev, "sysfs create group error\n");
#endif

#ifdef ELAN_IAP_DEV
	ts->firmware.minor = MISC_DYNAMIC_MINOR;
	ts->firmware.name = "elan-iap";
	ts->firmware.fops = &elan_touch_fops;
	ts->firmware.mode = S_IFREG|S_IRWXUGO;

	if (misc_register(&ts->firmware) < 0)
		dev_err(&ts->client->dev, "misc_register failed!!\n");

	ts->p = proc_create("elan-iap", 0644, NULL, &elan_touch_fops);
	if (ts->p == NULL)
		dev_err(&ts->client->dev, "[elan error] proc_create failed!!\n");
	else
		dev_info(&ts->client->dev, "proc_create ok!!\n");
#endif

	return;
}

#ifdef ELAN_HID_IIC
static int elan_ktf_ts_get_data(struct i2c_client *client,
		uint8_t *cmd, uint8_t *buf,
		size_t w_size, size_t r_size)
{
	int rc;

	if (buf == NULL)
		return -EINVAL;
#if 1
	if ((elan_ts_send_cmd(client, cmd, w_size)) != w_size) {
		dev_err(&client->dev, "[elan]%s: elan_ts_send_cmd failed\n",
				__func__);
		return -EINVAL;
	}
#else
	if ((i2c_master_send(client, cmd, w_size)) != w_size) {
		dev_err(&client->dev, "[elan]%s: i2c_master_send failed\n",\
				__func__);
		return -EINVAL;
	}
#endif
	rc = elan_ts_poll();
	if (rc < 0)
		dev_err(&client->dev, "%s: poll is high\n", __func__);

	if (r_size <= 0)
		r_size = w_size;

	if (elan_i2c_recv(client, buf, r_size) != r_size)
		return -EINVAL;

	return 0;
}

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t buf_recv[67] = {0};

	/*Firmware Version*/
	uint8_t cmd_fw_ver[37]  =   {0x04, 0x00, 0x23, 0x00, 0x03, 0x00,
		0x04, 0x53, 0x00, 0x00, 0x01};
	/*firmware ID*/
	uint8_t cmd_id[37]      =   {0x04, 0x00, 0x23, 0x00, 0x03, 0x00,
		0x04, 0x53, 0xf0, 0x00, 0x01};
	/*BootCode Version*/
	uint8_t cmd_bc[37]      =   {0x04, 0x00, 0x23, 0x00, 0x03, 0x00,
		0x04, 0x53, 0x10, 0x00, 0x01};
	/*Tp Osr*/
	uint8_t cmd_osr[37]	    =	{0x04, 0x00, 0x23, 0x00, 0x03, 0x00,
		0x04, 0x53, 0xD6, 0x00, 0x01};
	/*Tp Resolution */
	uint8_t cmd_res[37]     =   {0x04, 0x00, 0x23, 0x00, 0x03, 0x00,
		0x04, 0x5B, 0x00, 0x00, 0x00, 0x00, 0x00};
	/*Pen Xmax*/
	uint8_t pen_xmax[37]    =   {0x04, 0x00, 0x23, 0x00, 0x03, 0x00,
		0x04, 0x53, 0x60, 0x00, 0x01};
	/*Pen Ymax*/
	uint8_t pen_ymax[37]    =   {0x04, 0x00, 0x23, 0x00, 0x03, 0x00,
		0x04, 0x53, 0x61, 0x00, 0x01};

    /*Get FW VERSION*/
	rc = elan_ktf_ts_get_data(client, cmd_fw_ver, buf_recv,
			sizeof(cmd_fw_ver), sizeof(buf_recv));
	if (rc < 0)
		dev_err(&ts->client->dev, "Get Firmware version error\n");

	dev_dbg(&ts->client->dev,
			"[elan] %s: (Firmware version) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n",
			__func__, buf_recv[0], buf_recv[1], buf_recv[2],
			buf_recv[3], buf_recv[4], buf_recv[5],
			buf_recv[6], buf_recv[7]);

	major = ((buf_recv[5] & 0x0f) << 4) | ((buf_recv[6] & 0xf0) >> 4);
	minor = ((buf_recv[6] & 0x0f) << 4) | ((buf_recv[7] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;

    /*Get FW ID*/
	rc = elan_ktf_ts_get_data(client, cmd_id, buf_recv,
			sizeof(cmd_id), sizeof(buf_recv));
	if (rc < 0)
		dev_err(&ts->client->dev, "Get Firmware ID error\n");

	dev_dbg(&ts->client->dev,
			"[elan] %s: (Firmware ID) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n",
			__func__, buf_recv[0], buf_recv[1], buf_recv[2],
			buf_recv[3], buf_recv[4], buf_recv[5],
			buf_recv[6], buf_recv[7]);

	major = ((buf_recv[5] & 0x0f) << 4) | ((buf_recv[6] & 0xf0) >> 4);
	minor = ((buf_recv[6] & 0x0f) << 4) | ((buf_recv[7] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;

    /*Get BOOTCODE VERSION*/
	rc = elan_ktf_ts_get_data(client, cmd_bc, buf_recv,
			sizeof(cmd_bc), sizeof(buf_recv));
	if (rc < 0)
		dev_err(&ts->client->dev, "Get Bootcode version error\n");

	dev_dbg(&ts->client->dev,
			"[elan] %s: (Bootcode version) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n",
			__func__, buf_recv[0], buf_recv[1], buf_recv[2],
			buf_recv[3], buf_recv[4], buf_recv[5],
			buf_recv[6], buf_recv[7]);

	major = ((buf_recv[5] & 0x0f) << 4) | ((buf_recv[6] & 0xf0) >> 4);
	minor = ((buf_recv[6] & 0x0f) << 4) | ((buf_recv[7] & 0xf0) >> 4);
	ts->bc_ver = major << 8 | minor;

	/*Get Tp Osr*/
	rc = elan_ktf_ts_get_data(client, cmd_osr, buf_recv,
			sizeof(cmd_osr), sizeof(buf_recv));
	if (rc < 0)
		dev_err(&ts->client->dev, "Get Tp Osr error\n");

	dev_dbg(&ts->client->dev,
			"[elan] %s: (Tp Resolution) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n",
				__func__, buf_recv[0], buf_recv[1], buf_recv[2],
				buf_recv[3], buf_recv[4], buf_recv[5],
				buf_recv[6], buf_recv[7]);

	ts->hand_osr = buf_recv[7];

    /*Get TP RESOLUTION*/
	rc = elan_ktf_ts_get_data(client, cmd_res, buf_recv,
			sizeof(cmd_res), sizeof(buf_recv));
	if (rc < 0)
		dev_err(&ts->client->dev, "Get  Tp Resolution error\n");

	dev_dbg(&ts->client->dev,
			"[elan] %s: (Tp Resolution) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n",
			__func__, buf_recv[0], buf_recv[1], buf_recv[2],
			buf_recv[3], buf_recv[4], buf_recv[5],
			buf_recv[6], buf_recv[7]);

	if ((((ts->fw_ver)&0xff00) >> 8) == 0x56) {
		ts->x_trace  = buf_recv[6];
		ts->y_trace  = buf_recv[7];
	} else {
		ts->x_trace = buf_recv[6] + buf_recv[10];
		ts->y_trace = buf_recv[7] + buf_recv[11];
	}
	dev_dbg(&ts->client->dev, "[elan] %s: x= %d, y=%d\n", __func__, ts->x_trace,ts->y_trace);
	ts->x_resolution = (ts->x_trace - 1) * ts->hand_osr;
	ts->y_resolution = (ts->y_trace - 1) * ts->hand_osr;

    /*Get Pen X_Res*/
	rc = elan_ktf_ts_get_data(client, pen_xmax, buf_recv,
			sizeof(pen_xmax), sizeof(buf_recv));
	if (rc < 0)
		dev_err(&ts->client->dev,
				"Get Pen X Resolution error\n");

	dev_info(&ts->client->dev,
			"[elan] %s: (Pen x Resolution) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n",
			__func__, buf_recv[0], buf_recv[1], buf_recv[2],
			buf_recv[3] , buf_recv[4], buf_recv[5],
			buf_recv[6], buf_recv[7]);
	ts->pen_x_res = (buf_recv[7]<<8)|buf_recv[6];

    /*Pen Y_Res*/
	rc = elan_ktf_ts_get_data(client, pen_ymax, buf_recv,
			sizeof(pen_ymax), sizeof(buf_recv));
	if (rc < 0)
		dev_err(&ts->client->dev, "Get Pen Y Resolution error\n");

	dev_info(&ts->client->dev,
			"[elan] %s: (Pen Y Resolution) %2x:%2x:%2x:%2x:%2x:%2x:%2x:%2x\n\n",
			__func__, buf_recv[0], buf_recv[1], buf_recv[2],
			buf_recv[3] , buf_recv[4], buf_recv[5],
			buf_recv[6], buf_recv[7]);
	ts->pen_y_res = (buf_recv[7]<<8)|buf_recv[6];

	dev_info(&ts->client->dev,
			"[elan] %s: Firmware version: 0x%4.4x\n",
			__func__, ts->fw_ver);
	dev_info(&ts->client->dev,
			"[elan] %s: Firmware ID: 0x%4.4x\n",
			__func__, ts->fw_id);
	dev_info(&ts->client->dev,
			"[elan] %s: Bootcode Version: 0x%4.4x\n",
			__func__, ts->bc_ver);
	dev_info(&ts->client->dev,
			"[elan] %s: TP Osr: 0x%4.4x\n",
			__func__, ts->hand_osr);
	dev_info(&ts->client->dev,
			"[elan] %s: penx resolution: %d, pen y resolution: %d\n",
			__func__, ts->pen_x_res, ts->pen_y_res);

	return 0;
}

#else

static int __fw_packet_handler(struct i2c_client *client)
{
	struct elan_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;

	/*Firmware Version*/
	uint8_t cmd[]           = {0x53, 0x00, 0x00, 0x01};
	/*Firmware ID*/
	uint8_t cmd_id[]        = {0x53, 0xf0, 0x00, 0x01};
	/*BootCode Version*/
	uint8_t cmd_bc[]        = {0x53, 0x10, 0x00, 0x01};

#ifdef TWO_LAYER
	/*Tp Resolution*/
	int x, y;
	uint8_t cmd_getinfo[] = {0x5B, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t adcinfo_buf[17] = {0};
#else
	/*Get X Resolution*/
	uint8_t cmd_x[]         = {0x53, 0x60, 0x00, 0x00};
	/*Get Y Resolution*/
	uint8_t cmd_y[]         = {0x53, 0x63, 0x00, 0x00};
#endif
	uint8_t buf_recv[4]     = {0};

    /*Get Firmware version*/
	rc = elan_ts_get_data(client, cmd, buf_recv, sizeof(buf_recv));
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_ver = major << 8 | minor;

    /*Get Firmware ID*/
	rc = elan_ts_get_data(client, cmd_id, buf_recv, sizeof(buf_recv));
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_id = major << 8 | minor;

#ifndef TWO_LAYER
    /*Get X Resolution*/
	rc = elan_ts_get_data(client, cmd_x, buf_recv, sizeof(buf_recv));
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);

#ifdef SWAP_X_Y_RESOLUTION
	ts->y_resolution = minor;
#else
	ts->x_resolution = minor;
#endif

	/*Get Y Resolution*/
	rc = elan_ts_get_data(client, cmd_y, buf_recv, sizeof(buf_recv));
	if (rc < 0)
		return rc;
	minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
#ifdef SWAP_X_Y_RESOLUTION
	ts->x_resolution = minor;
#else
	ts->y_resolution = minor;
#endif

#else
	elan_i2c_send(client, cmd_getinfo, sizeof(cmd_getinfo));
	mdelay(10);
	elan_i2c_recv(client, adcinfo_buf, 17);
	x = adcinfo_buf[2]+adcinfo_buf[6];
	y = adcinfo_buf[3]+adcinfo_buf[7];

	dev_dbg(&ts->client->dev,
			"[elan] %s: x= %d, y=%d\n", __func__, x, y);
	ts->x_resolution = (x - 1) * 64;
	ts->y_resolution = (y - 1) * 64;
#endif
    /*Firmware BC*/
	rc = elan_ts_get_data(client, cmd_bc, buf_recv, sizeof(buf_recv));
	if (rc < 0)
		return rc;
	major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	ts->fw_bcd = major << 8 | minor;

	dev_info(&ts->client->dev, "%s: firmware version: 0x%4.4x\n",
			__func__, ts->fw_ver);
	dev_info(&ts->client->dev,
			"%s: firmware ID: 0x%4.4x\n", __func__, ts->fw_id);
	dev_info(&ts->client->dev,
			"%s: firmware BC: 0x%4.4x\n", __func__, ts->fw_bcd);
	dev_info(&ts->client->dev,
			"%s: x resolution: %d, y resolution: %d\n", __func__,
			ts->x_resolution, ts->y_resolution);

	return 0;
}
#endif

#ifdef ELAN_HID_IIC
static int elan_ts_rough_calibrate(struct i2c_client *client)
{
	uint8_t flash_key[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04,
		CMD_W_PKT, 0xc0, 0xe1, 0x5a};
	uint8_t cal_cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04,
		CMD_W_PKT, 0x29, 0x00, 0x01};

	dev_info(&client->dev, "[elan] %s: Flash Key cmd\n", __func__);
	if ((elan_ts_send_cmd(client, flash_key, sizeof(flash_key)))
			!= sizeof(flash_key)) {
		dev_err(&client->dev,
				"[elan] %s: i2c_master_send failed\n",
				__func__);
		return -EINVAL;
	}

	dev_dbg(&client->dev,
			"[elan] %s: Calibration cmd: %02x, %02x, %02x, %02x\n",
			__func__, cal_cmd[7], cal_cmd[8],
			cal_cmd[9], cal_cmd[10]);

	if ((elan_ts_send_cmd(client, cal_cmd, sizeof(cal_cmd)))
			!= sizeof(cal_cmd)) {
		dev_err(&client->dev, "[elan] %s: i2c_master_send failed\n",\
				__func__);
		return -EINVAL;
	}

	return 0;
}

#else

static int elan_ts_rough_calibrate(struct i2c_client *client)
{
	uint8_t flash_key[] = {CMD_W_PKT, 0xc0, 0xe1, 0x5a};
	uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};

	dev_info(&client->dev, "[elan] %s: enter\n", __func__);
	dev_info(&client->dev,
			"[elan] dump flash_key : %02x, %02x, %02x, %02x\n",
			flash_key[0], flash_key[1], flash_key[2], flash_key[3]);

	if ((i2c_master_send(client, flash_key, sizeof(flash_key)))
			!= sizeof(flash_key)) {
		dev_err(&client->dev,
				"[elan] %s: i2c_master_send failed\n",
				__func__);
		return -EINVAL;
	}

	dev_info(&client->dev,
			"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
			cmd[0], cmd[1], cmd[2], cmd[3]);

	if ((i2c_master_send(client, cmd, sizeof(cmd)))
			!= sizeof(cmd)) {
		dev_err(&client->dev,
				"[elan] %s: i2c_master_send failed\n",
				__func__);
		return -EINVAL;
	}

	return 0;
}
#endif

#if defined IAP_PORTION
static void check_update_flage(struct elan_ts_data *ts)
{

/*	int NEW_FW_VERSION = 0;
	int New_FW_ID = 0;*/
	int rc = 0;
	int retry_cnt = 0;

#ifdef ELAN_HID_IIC
	rc = CheckISPstatus(ts->client);
	if (rc == 0xa6)
		goto update_elan_fw;
#else
	if (ts->recover == 0x80) {
		dev_dbg(&ts->client->dev, "[elan] ***fw is miss, force update!!!!***\n");
		goto update_elan_fw;
	}
#endif
/*
#ifdef ELAN_HID_IIC
	New_FW_ID           = file_fw_data[0xD5EB] << 8 | file_fw_data[0xD5EA];
	NEW_FW_VERSION      = file_fw_data[0xD1DF] << 8 | file_fw_data[0xD1DE];
#else

#ifdef ELAN_2K_XX
	New_FW_ID = file_fw_data[0x7BD3]<<8 | file_fw_data[0x7BD2];
	NEW_FW_VERSION = file_fw_data[0x7BD1]<<8 | file_fw_data[0x7BD0];
#endif

#ifdef ELAN_3K_XX
	New_FW_ID  = file_fw_data[0xE2CF]<<8  | file_fw_data[0xE2CE];
	NEW_FW_VERSION = file_fw_data[0xDEC3]<<8  | file_fw_data[0xDEC2];
#endif
#endif
	dev_info(&ts->client->dev, "[elan] FW_ID=0x%x, New_FW_ID=0x%x\n",
			ts->fw_id, New_FW_ID);
	dev_info(&ts->client->dev, "[elan] FW_VERSION=0x%x,New_FW_VER=0x%x\n",
			ts->fw_ver, NEW_FW_VERSION);

	if ((ts->fw_id&0xff) != (New_FW_ID&0xff)) {
		printk(KERN_INFO "[elan] ***fw id is different,can not update !***\n");
		goto no_update_elan_fw;
	} else
		printk(KERN_INFO "[elan] fw id is same !\n");

	if ((ts->fw_ver&0xff) >= (NEW_FW_VERSION&0xff)) {
		dev_dbg(&ts->client->dev, "[elan] fw version is newest!!\n");
		goto no_update_elan_fw;
	}
*/
update_elan_fw:

	FW_Update(ts->client, 1);
	mdelay(500);
	elan_switch_irq(0);

	for (retry_cnt = 0; retry_cnt < 3; retry_cnt++) {
		rc = __fw_packet_handler(private_ts->client);
		if (rc < 0)
			dev_err(&ts->client->dev,
					"[elan error] %s,fw_packet_handler fail,rc = %d\n",
					__func__, rc);
		else
			break;
	}

	elan_switch_irq(1);
	mdelay(200);
	elan_ts_rough_calibrate(ts->client);

/*no_update_elan_fw:*/
	dev_info(&ts->client->dev,
			"[elan] %s, fw check end..............\n",
			__func__);

	return;
}
#endif

static int elan_request_input_dev(struct elan_ts_data *ts)
{
	int err = 0;
	int i = 0;
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		err = -ENOMEM;
		dev_err(&ts->client->dev,
				"[elan error] Failed to allocate input device\n");
		return err;
	}

	ts->input_dev->evbit[0] = BIT(EV_KEY)|BIT_MASK(EV_REP);

    /*key setting*/
	for (i = 0; i < ARRAY_SIZE(key_value); i++)
		__set_bit(key_value[i], ts->input_dev->keybit);

	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN)|
		BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->absbit[0] = BIT(ABS_X) |
		BIT(ABS_Y) | BIT(ABS_PRESSURE);
	__set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

#ifdef ELAN_ICS_SLOT_REPORT
	input_mt_init_slots(ts->input_dev, FINGERS_NUM);
#else
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
#endif
	dev_dbg(&ts->client->dev,
			"[elan] %s: x resolution: %d, y resolution: %d\n",
			__func__, ts->x_resolution, ts->y_resolution);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			0, 1473, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			0, 2368, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 511, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 511, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	ts->input_dev->name = ELAN_TS_NAME;
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 2013;

	err = input_register_device(ts->input_dev);
	if (err) {
		input_free_device(ts->input_dev);
		dev_err(&ts->client->dev,
				"[elan error]%s: unable to register %s input device\n",
				__func__, ts->input_dev->name);
		return err;
	}

	return 0;
}

#ifdef ELAN_ESD_CHECK
static void elan_touch_esd_func(struct work_struct *work)
{
	int res;
	struct i2c_client *client = private_ts->client;
	dev_info(&private_ts->client->dev, "esd %s: enter.......", __func__);

	if (private_ts->power_lock == 1)
		goto elan_esd_check_out;

	if (atomic_read(&elan_cmd_response) == 0)
		dev_err(&private_ts->client->dev,
				"[elan esd] %s:  response 0x78 failed\n",
				__func__);
	else {
		dev_dbg(&private_ts->client->dev,
				"esd %s: response ok!!!", __func__);
		goto elan_esd_check_out;
	}

	elan_reset();
elan_esd_check_out:
	atomic_set(&elan_cmd_response, 0);
	queue_delayed_work(esd_wq, &esd_work, delay);
	dev_info(&private_ts->client->dev,
			"[elan esd] %s: out.......", __func__);
	return;
}
#endif

#ifdef ELAN_HID_IIC
static inline int elan_ts_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[6]);
	*x <<= 8;
	*x |= data[5];

	*y = (data[10]);
	*y <<= 8;
	*y |= data[9];

	return 0;
}

static inline int elan_ts_parse_pen(uint8_t *data, uint16_t *x,
		uint16_t *y, uint16_t *p)
{

	*x = *y = *p = 0;
	*x = data[5];
	*x <<= 8;
	*x |= data[4];

	*y = data[7];
	*y <<= 8;
	*y |= data[6];

	*p = data[9];
	*p <<= 8;
	*p |= data[8];

	return 0;
}

#else

static inline int elan_ts_parse_xy(uint8_t *data, uint16_t *x, uint16_t *y)
{
	*x = *y = 0;

	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];

	return 0;
}
#endif
static void elan_ts_touch_down(struct elan_ts_data *ts, s32 id,
		s32 x, s32 y, s32 w)
{
	if (ts->input_dev == NULL) {
		dev_err(&ts->client->dev, "input dev is null\n");
		return;
	}

#ifdef ROTATE_X
	x = ts->x_resolution-x;
#endif

#ifdef ROTATE_Y
	y = ts->y_resolution-y;
#endif

#ifdef ELAN_ICS_SLOT_REPORT
	input_mt_slot(ts->input_dev, id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_PRESSURE, w);
#else
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_PRESSURE, w);
	input_mt_sync(ts->input_dev);
#endif

	dev_info(&ts->client->dev,
			"Touch ID:%d, X:%d, Y:%d, W:%d down", id, x, y, w);
}

static void elan_ts_touch_up(struct elan_ts_data *ts, s32 id, s32 x, s32 y)
{
	if (ts->input_dev == NULL) {
		dev_err(&ts->client->dev, "input dev is null\n");
		return;
	}

#ifdef ELAN_ICS_SLOT_REPORT
	input_mt_slot(ts->input_dev, id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
	dev_dbg(&ts->client->dev, "Touch id[%2d] release!", id);
#else
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_mt_sync(ts->input_dev);
	dev_dbg(&ts->client->dev, "Touch all release!");
#endif
}

static void elan_ts_report_key(struct elan_ts_data *ts, uint8_t button_data)
{
	static unsigned int key_value;
	static unsigned int x, y;

	switch (button_data) {
	case ELAN_KEY_BACK:
		key_value = KEY_BACK;
		input_report_key(ts->input_dev, key_value, 1);
		break;
	case ELAN_KEY_HOME:
		key_value = KEY_HOME;
		input_report_key(ts->input_dev, key_value, 1);
		break;
	case ELAN_KEY_MENU:
		key_value = KEY_MENU;
		input_report_key(ts->input_dev, key_value, 1);
		break;
	default:
		if (key_value != 0) {
			input_report_key(ts->input_dev, key_value, 0);
			key_value = 0;
		} else
			elan_ts_touch_up(ts, 0, x, y);
		break;
	}

}

static void elan_ts_handler_event(struct elan_ts_data *ts, uint8_t *buf)
{
	if (buf[0] == 0x55) {
		if (buf[2] == 0x55)
			ts->recover = 0;
		else if (buf[2] == 0x80)
			ts->recover = 0x80;
	}
#ifdef ELAN_ESD_CHECK
	else if (buf[0] == 0x78)
		atomic_set(&elan_cmd_response, 1);
#endif
}

static int elan_ts_recv_data(struct elan_ts_data *ts, uint8_t *buf)
{
	int rc;

#ifdef ELAN_HID_IIC
#ifdef TEN_FINGERS
	uint8_t buf1[PACKET_SIZE] = { 0 };
#endif
#endif

#ifdef PRINT_INT_INFO
	int i = 0;
#endif
	rc = elan_i2c_recv(ts->client, buf, PACKET_SIZE);
	if (PACKET_SIZE != rc) {
		dev_err(&ts->client->dev, "[elan error] elan_ts_recv_data\n");
		return -1;
	}
#ifdef ELAN_HID_IIC
	if (FINGERS_PKT == buf[2]) {
		buf[1] = buf[62];
		if ((buf[2] == FINGERS_PKT) && (buf[62] > 5)) {
			rc = elan_i2c_recv(ts->client, buf1, PACKET_SIZE);
			if (rc < 0) {
				dev_err(&ts->client->dev, "[elan error] elan_ts_recv_data next packet error\n");
				return -1;
			}
			for (i = 3; i < 67; i++)
				buf[55+i] = buf1[i];
		}
	}
#endif
#ifdef PRINT_INT_INFO
#ifdef ELAN_HID_IIC
	if ((buf[1] > 5) && (FINGERS_PKT == buf[2])) {
		for (i = 0; i < (PACKET_SIZE * 2 + 4) / 8; i++) {
			dev_dbg(&ts->client->dev, "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
					buf[i*8+0], buf[i*8+1], buf[i*8+2],
					buf[i*8+3], buf[i*8+4], buf[i*8+5],
					buf[i*8+6], buf[i*8+7]);
		}
	}
#ifdef ELAN_HID_PEN
	else if (PEN_PKT == buf[2]) {
		for (i = 0; i < (PACKET_SIZE + 5) / 8; i++) {
			dev_dbg(&ts->client->dev,
					"%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
					buf[i*8+0], buf[i*8+1], buf[i*8+2],
					buf[i*8+3], buf[i*8+4], buf[i*8+5],
					buf[i*8+6], buf[i*8+7]);
		}
	}
#endif
	else
#endif
	{
		for (i = 0; i < (PACKET_SIZE + 7) / 8; i++) {
			dev_dbg(&ts->client->dev,
					"%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
					buf[i*8+0], buf[i*8+1], buf[i*8+2],
					buf[i*8+3], buf[i*8+4], buf[i*8+5],
					buf[i*8+6], buf[i*8+7]);
		}
	}
#endif
#ifdef ELAN_HID_IIC
	if (FINGERS_PKT != buf[2]
#else
	if (FINGERS_PKT != buf[0]
#endif	
#ifdef ELAN_HID_PEN
			&& PEN_PKT != buf[2]
#endif
			){
#ifndef PRINT_INT_INFO
		dev_dbg(&ts->client->dev,
				"other event packet:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
				buf[0], buf[1], buf[2], buf[3],
				buf[4], buf[5], buf[6], buf[7]);
#endif
		elan_ts_handler_event(ts, buf);
		return -1;
	}
#ifdef ELAN_ESD_CHECK
	atomic_set(&elan_cmd_response, 1);
#endif
	return 0;
}

#ifdef ELAN_HID_PEN
static void elan_ts_report_pen(struct elan_ts_data *ts, uint8_t *buf)
{
	int pen_hover = 0;
	int pen_down = 0;
	int pen_key = 0;
	uint16_t p = 0;
	uint16_t x = 0;
	uint16_t y = 0;

	pen_hover = buf[3] & 0x1;
	pen_down = buf[3] & 0x03;
	pen_key = buf[3];

	if (pen_key) {
		dev_dbg(&ts->client->dev,
				"[elan] report pen key %02x\n", pen_key);
		/*elan_ts_report_key(ts,pen_key);*/
	}

	if (pen_hover) {
		elan_ts_parse_pen(&buf[0], &x, &y, &p);
		y = y * ts->y_resolution / ((ts->y_trace-1) * 260);
		x = x * ts->x_resolution / ((ts->x_trace-1) * 260);
		dev_dbg(&ts->client->dev,
				"[elan] pen id--------=%d x=%d y=%d\n",
				p, x, y);
		if (pen_down == 0x01) {  /* report hover function  */
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
			input_report_abs(ts->input_dev, ABS_MT_DISTANCE, 15);
			dev_dbg(&ts->client->dev, "[elan pen] Hover DISTANCE=15\n");
		} else {
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, p);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, p);
			dev_dbg(&ts->client->dev,
					"[elan pen] PEN PRESSURE=%d\n", p);
		}

		input_report_key(ts->input_dev, BTN_TOUCH, 1);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
		input_mt_sync(ts->input_dev);
	} else
		input_mt_sync(ts->input_dev);

	if (unlikely(gPrint_point)) {
		dev_dbg(&ts->client->dev,
				"[elan pen] %x:%x:%x:%x:%x:%x:%x:%x\n",
				buf[0], buf[1], buf[2], buf[3],
				buf[4], buf[5], buf[6], buf[7]);
		dev_dbg(&ts->client->dev,
				"[elan] x=%d y=%d p=%d\n", x, y, p);
	}

	if (pen_down == 0) {
		/*All Finger Up*/
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_mt_sync(ts->input_dev);
	}
}
#endif

static void elan_ts_report_data(struct elan_ts_data *ts, uint8_t *buf)
{
	uint16_t fbits = 0;
#ifdef ELAN_ICS_SLOT_REPORT
	static uint16_t pre_fbits;
	uint16_t fbits_tmp = 0;
#else
	int reported = 0;
#endif
	uint8_t idx;
	int finger_num;
	int num = 0;
	uint16_t x = 0;
	uint16_t y = 0;
	int position = 0;
	uint8_t button_byte = 0;
	finger_num = FINGERS_NUM;

#ifdef TWO_FINGERS
	num = buf[7] & 0x03;
	fbits = buf[7] & 0x03;
	idx = 1;
	button_byte = buf[PACKET_SIZE - 1];
#endif

#ifdef FIVE_FINGERS
	num = buf[1] & 0x07;
	fbits = buf[1] >> 3;
	idx = 2;
	button_byte = buf[PACKET_SIZE - 1];
#endif

#ifdef TEN_FINGERS
  #ifdef ELAN_HID_IIC
	num = buf[1];
	idx = 3;
	button_byte = buf[63];
  #else
	fbits = buf[2] & 0x30;
	fbits = (fbits << 4) | buf[1];
	num = buf[2] & 0x0f;
	idx = 3;
	button_byte = buf[PACKET_SIZE-1];
 #endif
#endif

#ifdef ELAN_HID_PEN
	if (PEN_PKT == buf[2])
		elan_ts_report_pen(ts, buf);
	else
#endif
	{
#ifdef ELAN_ICS_SLOT_REPORT
		fbits_tmp = fbits;
		if (fbits || pre_fbits) {
			for (position = 0; position < finger_num; position++) {
				if (fbits & 0x01) {
#ifdef SWAP_X_Y_RESOLUTION
					elan_ts_parse_xy(&buf[idx], &y, &x);
#else
					elan_ts_parse_xy(&buf[idx], &x, &y);
#endif
					elan_ts_touch_down(ts, position, x,
							y, 255);
				} else if (pre_fbits & 0x01)
					elan_ts_touch_up(ts, position, x, y);
				fbits >>= 1;
				pre_fbits >>= 1;
				idx += 3;
			}
		} else
			elan_ts_report_key(ts, button_byte);
		pre_fbits = fbits_tmp;
#else
		if (num == 0)
			elan_ts_report_key(ts, button_byte);
		else{
			dev_dbg(&ts->client->dev, "[elan] %d fingers", num);
			for (position = 0; (position < finger_num)
					&& (reported < num); position++) {
#ifdef ELAN_HID_IIC
				if ((buf[idx] & 0x03) == 0x00) {
					num--;
					input_mt_sync(ts->input_dev);
				/*	reported++;*/
				} else {
					fbits = (buf[idx] & 0xfc) >> 2;
					elan_ts_parse_xy(&buf[idx], &x, &y);
					dev_dbg(&ts->client->dev, "[elan] finger id=%d x=%d y=%d\n",
							fbits, x, y);
					elan_ts_touch_down(ts, fbits, x, y, 8);
					/*reported++;*/
				}
				reported++;
				idx += 11;
#else
				if ((fbits & 0x01)) {
#ifdef SWAP_X_Y_RESOLUTION
					elan_ts_parse_xy(&buf[idx], &y, &x);
#else
					elan_ts_parse_xy(&buf[idx], &x, &y);
#endif
				/*	y = ts->y_resolution-y;*/
					elan_ts_touch_down(ts, position, x,
							y, 8);
					reported++;
				}
				fbits = fbits >> 1;
				idx += 3;
#endif
			}
		}
#endif
	}
	input_sync(ts->input_dev);
	return;
}

#if 0
static irqreturn_t elan_ts_irq_handler(int irq, void *dev_id)
{
	int rc = 0;
#ifdef ELAN_HID_IIC
#ifdef TEN_FINGERS
	uint8_t buf[PACKET_SIZE * 2] = { 0 };
#else
	uint8_t buf[PACKET_SIZE] = { 0 };
#endif
#else
	uint8_t buf[PACKET_SIZE] = { 0 };
#endif

	rc = elan_ts_recv_data(private_ts, buf);
	if (rc < 0 || private_ts->input_dev == NULL)
		return IRQ_HANDLED;

	elan_ts_report_data(private_ts, buf);

	return IRQ_HANDLED;
}
#else
static int touch_event_handler(void *unused)
{
	/*   uint8_t buf[PACKET_SIZE] = { 0 };*/
#ifdef ELAN_HID_IIC
#ifdef TEN_FINGERS
	uint8_t buf[PACKET_SIZE * 2] = { 0 };
#else
	uint8_t buf[PACKET_SIZE] = { 0 };
#endif
#else
	uint8_t buf[PACKET_SIZE] = { 0 };
#endif
	int rc = 0;
	struct elan_ts_data *ts = private_ts;
	struct sched_param param = { .sched_priority = 4};
	sched_setscheduler(current, SCHED_RR, &param);

	do {
		dev_dbg(&ts->client->dev, "[elan] %s work!\n", __func__);
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);
		rc = elan_ts_recv_data(ts, buf);
		if (rc < 0)
			continue;
		elan_ts_report_data(ts, buf);
	} while (!kthread_should_stop());

	return 0;
}

static irqreturn_t elan_ts_irq_handler(int irq, void *dev_id)
{
	dev_dbg(&private_ts->client->dev, "[elan] enter %s\n", __func__);
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
}


#endif
static int elan_ts_register_interrupt(struct elan_ts_data *ts)
{
	int err = 0;

	err = request_threaded_irq(ts->pdata->elan_irq,
			NULL, elan_ts_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			ELAN_TS_NAME, ts);

	if (err < 0)
		dev_err(&ts->client->dev,
				"[elan error] %s: request_irq %d failed,err = %d\n",
				__func__, ts->client->irq, err);
	return err;
}

static void elan_ic_init_work(struct work_struct *work)
{
	int rc = 0;
	int retry_cnt = 0;

	if (private_ts->recover == 0x00) {
		printk("[elan] normal +++++");
		for (retry_cnt = 0; retry_cnt < 3; retry_cnt++) {
			rc = __fw_packet_handler(private_ts->client);
			if (rc < 0)
				dev_err(&private_ts->client->dev,
						" %s, fw_packet_handler fail, rc = %d\n",
						__func__, rc);
			else
				break;
		}
		if (retry_cnt >= 3)
			return;
	} else {
		printk("[elan] run here +++++++++++++++\n");
		printk("[elan] auto update fw in recovery mode");
	}
#if defined IAP_PORTION
	check_update_flage(private_ts);
#endif
	/*init input devices*/
	rc = elan_request_input_dev(private_ts);
	if (rc < 0)
		dev_err(&private_ts->client->dev,
				"[elan error]: %s elan_request_input_dev\n",
				__func__);

#ifdef ELAN_ESD_CHECK
	INIT_DELAYED_WORK(&esd_work, elan_touch_esd_func);
	esd_wq = create_singlethread_workqueue("esd_wq");
	if (!esd_wq)
		return;
	queue_delayed_work(esd_wq, &esd_work, delay);
#endif

	rc = elan_ts_register_interrupt(private_ts);
	if (rc < 0) {
		dev_err(&private_ts->client->dev,
				"[elan error]: %s elan_ts_register_interrupt\n",
				__func__);
		return;
	}
}

static int elan_ts_setup(struct i2c_client *client)
{
	int rc = 0;
#ifdef SENSOR_OPTION
	/*Get Firware ID*/
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01};
	uint8_t buf_recv[4] = {0};
	int major, minor;
	struct elan_ktf_ts_data *ts = i2c_get_clientdata(client);
#endif

	elan_reset();
	mdelay(200);
	rc = __hello_packet_handler(client);
	if (rc < 0)
		dev_err(&client->dev, "[elan error] %s, hello_packet_handler fail,rc = %d\n",
				__func__, rc);

#ifdef SENSOR_OPTION
	if (0x80 == rc) {
		dev_err(&client->dev, "[elan error] %s, Enter Recovery Mode\n",
				__func__);
		rc = elan_ts_get_data(client, cmd, buf_recv, 4);
		if (rc < 0)
			return rc;

		major = ((buf_recv[1] & 0x0f) << 4)|
			((buf_recv[2] & 0xf0) >> 4);
		minor = ((buf_recv[2] & 0x0f) << 4)|
			((buf_recv[3] & 0xf0) >> 4);
		ts->fw_id = major << 8 | minor;
	}
#endif

	return rc;
}

#ifdef CONFIG_OF
int elan_ts_parse_dt(struct device *dev, struct elan_i2c_platform_data *pdata)
{
    /*int rc;*/
    /*struct device_node *np = dev->of_node;*/
	struct elan_ts_data *ts = private_ts;

	dev_info(&ts->client->dev, "[elan] Get device tree property\n");

	/*check dts file elan define name */
	/*
	rc = of_property_read_string(np, "elan,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
	dev_err(dev, "Unable to read name\n");
	return rc;
	}
	 */
    /*get int_gpio & rst_gpio from dts*/
	ts->pdata->intr_gpio = of_get_named_gpio_flags(ts->client->dev.of_node,
			"elan,irq-gpio", 0, NULL);
	if (!gpio_is_valid(ts->pdata->intr_gpio)) {
		dev_err(&ts->client->dev, "[elan] ts->pdata->intr_gpio invalid\n");
		return ts->pdata->intr_gpio;
	}

	ts->pdata->rst_gpio = of_get_named_gpio_flags(ts->client->dev.of_node,
			"elan,reset-gpio", 0, NULL);
	if (!gpio_is_valid(ts->pdata->rst_gpio)) {
		dev_err(&ts->client->dev, "[elan] ts->pdata->rst_gpio invalid\n");
		return ts->pdata->rst_gpio;
	}

	dev_info(&ts->client->dev, "[elan] ts->intr_gpio = %d, ts->rst_gpio = %d\n",
			ts->pdata->intr_gpio, ts->pdata->rst_gpio);

	ts->pdata->elan_irq = gpio_to_irq(ts->pdata->intr_gpio);

	return 0;
}
#endif

static int elan_gpio_init(struct i2c_client *client,
		struct elan_i2c_platform_data *pdata)
{
	int rc = 0;
	struct elan_ts_data *ts = i2c_get_clientdata(client);

	rc = gpio_request(ts->pdata->rst_gpio, "tp_reset");
	if (rc < 0) {
		gpio_free(ts->pdata->rst_gpio);
		pr_err("%s: request rst_gpio pin failed\n", __func__);
		return rc;
	}

	gpio_direction_output(ts->pdata->rst_gpio, 1);

    /*set int pin input*/
	rc = gpio_request(ts->pdata->intr_gpio, "tp_irq");
	if (rc < 0) {
		gpio_free(ts->pdata->intr_gpio);
		pr_err("%s: request intr_gpio pin failed\n", __func__);
		return rc;
	}

	gpio_direction_input(ts->pdata->intr_gpio);

	return rc;
}

static int elan_ts_power_on(struct elan_ts_data *data, bool on)
{
	int rc = 0;
	int err = 0;

	if (!on && data->power_enabled) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator vdd disable failed rc=%d\n",
					rc);
			return rc;
		}

		rc = regulator_disable(data->vcc_i2c);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator vcc_i2c disable failed rc=%d\n",
					rc);
			err = regulator_enable(data->vdd);
			return rc;
		}
		data->power_enabled = on;
	} else if (on && !data->power_enabled) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator vdd enable failed rc=%d\n",
					rc);
			return rc;
		}

		rc = regulator_enable(data->vcc_i2c);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator vcc_i2c enable failed rc=%d\n",
					rc);
			regulator_disable(data->vdd);
			return rc;
		}
		data->power_enabled = on;
	} else {
		dev_info(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->power_enabled);
	}

	return rc;
}

static int elan_ts_power_init(struct elan_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
				"Regulator get failed vdd rc=%d\n",
				rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator set_vtg failed vdd rc=%d\n",
					rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
				"Regulator get failed vcc_i2c rc=%d\n",
				rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
					"Regulator set_vtg failed vcc_i2c rc=%d\n",
					rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int elan_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	struct elan_i2c_platform_data *pdata;
	struct elan_ts_data *ts;

	dev_info(&client->dev, "[elan] enter probe....\n");

    /*check i2c bus support fuction*/
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_dbg(&client->dev, "[elan] %s: i2c check functionality error\n",
				__func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	/*kzalloc struct elan_ts_data memory */
	ts = kzalloc(sizeof(struct elan_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		dev_dbg(&client->dev, "[elan] %s: allocate elan_ts_data failed\n",
				__func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	private_ts = ts;

    /*two way to get gpio define dts or user config*/
#ifdef CONFIG_OF
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct elan_i2c_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev,
					"Failed to allocate elan_ts_i2c_platform_data memory\n");
			err =  -ENOMEM;
			goto err_alloc_pdata_failed;
		}
	}

	ts->pdata = pdata;
	err = elan_ts_parse_dt(&client->dev, pdata);
	if (err < 0) {
		err =  -EINVAL;
		goto err_parse_dt_failed;
	}
#else
	pdata = client->dev.platform_data;
	ts->pdata = pdata;
	if (pdata) {
		ts->pdata->rst_gpio   =    pdata->rst_gpio;
		ts->pdata->intr_gpio  =    pdata->intr_gpio;
		ts->pdata->elan_irq   =    gpio_to_irq(pdata->intr_gpio);
		dev_dbg(&client->dev, "[elan]:rst_gpio = %d, intr_gpio = %d\n",
				pdata->rst_gpio, pdata->intr_gpio);
	} else
		dev_err(&client->dev,
				"[elan] get pdata faile,please check board file setting\n");

	dev_dbg(&client->dev,
			"[elan] rst_gpio = %d, intr_gpio = %d\n",
			ts->pdata->rst_gpio, ts->pdata->intr_gpio);
#endif

    dev_err(&client->dev, "[elan] power setting....\n");
    err = elan_ts_power_init(ts, true);
    if (err) {
	dev_err(&client->dev, "power init failed");
	err = -EINVAL;
	goto err_alloc_data_failed;
    }
    err = elan_ts_power_on(ts, true);
    if (err) {
        dev_err(&client->dev, "power on failed");
	    goto pwr_deinit;
    }
   private_ts = ts;
   /*init gpio setring rst output / int input */
	err = elan_gpio_init(client, pdata);
	if (err < 0) {
		dev_err(&client->dev, "init fail check forward step");
		err = -EINVAL;
		goto err_gpio_init_failed;
	}
   /*reset & get hello pakect may need recv calibrate packect */
	err = elan_ts_setup(client);
	if (err < 0) {
		dev_err(&client->dev, "[elan] No Elan chip inside err = %d\n",err);
		err =  -EINVAL;
		goto err_no_elan_chip;
	}
   /*creat dev node for file operation & use sysfs to send cmd & recv packet*/
	elan_touch_node_init();

   /*read fw msg & register input_dev & register intterupt*/
	INIT_DELAYED_WORK(&init_work, elan_ic_init_work);
	init_elan_ic_wq = create_singlethread_workqueue("init_elan_ic_wq");
	if (!init_elan_ic_wq) {
		err = -EINVAL;
		goto err_ic_init_failed;
	}
	queue_delayed_work(init_elan_ic_wq, &init_work, delay);

	ts->work_thread = kthread_run(touch_event_handler, 0, ELAN_TS_NAME);
	if (IS_ERR(ts->work_thread)) {
		err = PTR_ERR(ts->work_thread);
		dev_err(&client->dev,
				"[elan error] failed to create kernel thread: %d\n",
				err);
		goto err_creat_thread_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = elan_ts_early_suspend;
    ts->early_suspend.resume = elan_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

	return 0;
err_creat_thread_failed:
err_ic_init_failed:
err_no_elan_chip:
	if (gpio_is_valid(ts->pdata->rst_gpio))
		gpio_free(ts->pdata->rst_gpio);
	if (gpio_is_valid(ts->pdata->intr_gpio))
		gpio_free(ts->pdata->intr_gpio);
err_gpio_init_failed:
pwr_deinit:
	elan_ts_power_init(ts, false);
#ifdef CONFIG_OF	
err_parse_dt_failed:
	/*devm_kfree(pdata);*/
#endif
err_alloc_pdata_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return err;

}

static int elan_ts_remove(struct i2c_client *client)
{
	struct elan_ts_data *ts = i2c_get_clientdata(client);
	dev_info(&client->dev, "%s: enter\n", __func__);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int elan_ts_set_power_state(struct i2c_client *client, int state)
{
	int size = 0;
#ifdef ELAN_HID_IIC
	uint8_t cmd[37] = {0x04, 0x00, 0x23, 0x00, 0x03, 0x00, 0x04,
		CMD_W_PKT, 0x50, 0x00, 0x01};
	cmd[8] |= (state << 3);
#else
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};
	cmd[1] |= (state << 3);
#endif
	size = sizeof(cmd);

	if (elan_ts_send_cmd(client, cmd, size) != size)
		return -EINVAL;

	return 0;
}

static int __maybe_unused elan_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ts_data *ts = i2c_get_clientdata(client);
	int rc = 0;
#ifdef ELAN_ICS_SLOT_REPORT
	int i = 0;
#endif
	dev_info(&client->dev, "%s: enter\n", __func__);
	if (ts->power_lock == 0) {
		elan_switch_irq(0);
		rc = elan_ts_set_power_state(ts->client, PWR_STATE_DEEP_SLEEP);
	}

#ifdef ELAN_ESD_CHECK
	cancel_delayed_work_sync(&esd_work);
#endif

#ifdef ELAN_ICS_SLOT_REPORT
	for (i = 0; i < FINGERS_NUM; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#else
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_mt_sync(ts->input_dev);
#endif
	input_sync(ts->input_dev);

	return 0;
}

static int __maybe_unused elan_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct elan_ts_data *ts = i2c_get_clientdata(client);
#ifdef ELAN_ICS_SLOT_REPORT
	int i = 0;
#endif
	dev_info(&client->dev, "[elan] %s: enter\n", __func__);
	if (ts->power_lock == 0) {
		dev_dbg(&client->dev, "[elan] reset gpio to resum tp\n");
		elan_reset();
		elan_switch_irq(1);
	}
#ifdef ELAN_ESD_CHECK
	queue_delayed_work(esd_wq, &esd_work, delay);
#endif
 #ifdef ELAN_ICS_SLOT_REPORT
	for (i = 0; i < FINGERS_NUM; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
#else
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_mt_sync(ts->input_dev);
#endif
	input_sync(ts->input_dev);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ts_early_suspend(struct early_suspend *h)
{
	struct elan_ts_data *ts;
	ts = container_of(h, struct elan_ts_data, early_suspend);
	elan_ts_suspend(&ts->client->dev);
}

static void elan_ts_late_resume(struct early_suspend *h)
{
	struct elan_ts_data *ts;
	ts = container_of(h, struct elan_ts_data, early_suspend);
	elan_ts_resume(&ts->client->dev);
}
#endif


static const struct i2c_device_id elan_ts_id[] = {
	{ELAN_TS_NAME, 0},
	{ }
};

#ifdef CONFIG_OF
static const struct of_device_id elants_of_match[] = {
        { .compatible = "elan,ektf" },
        { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, elants_of_match);
#endif
static SIMPLE_DEV_PM_OPS(elants_i2c_pm_ops,
		elan_ts_suspend, elan_ts_resume);

static struct i2c_driver elan_ts_driver = {
	.probe			= elan_ts_probe,
	.remove			= elan_ts_remove,
	.id_table		= elan_ts_id,
	.driver			= {
		.pm			= &elants_i2c_pm_ops,
		.name		= ELAN_TS_NAME,
		.owner		= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table	= of_match_ptr(elants_of_match),
#endif
	},
};


static int __init elan_ts_init(void)
{
	pr_info("[elan] %s driver version 0x0005:support 10 finger\n",
			__func__);
	return i2c_add_driver(&elan_ts_driver);
}
module_init(elan_ts_init);

static void __exit elan_ts_exit(void)
{
	i2c_del_driver(&elan_ts_driver);
	return;
}
module_exit(elan_ts_exit);

MODULE_DESCRIPTION("ELAN Touchscreen Driver");
MODULE_LICENSE("GPL v2");
