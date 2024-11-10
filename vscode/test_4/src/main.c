/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>
#include <zephyr/bluetooth/services/ias.h>
#include "cts.h"


static int print_samples;
static int lsm6dsl_trig_cnt;

static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;
#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
static struct sensor_value magn_x_out, magn_y_out, magn_z_out;
#endif
#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
static struct sensor_value press_out, temp_out;
#endif

#ifdef CONFIG_LSM6DSL_TRIGGER
static void lsm6dsl_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trig)
{
	static struct sensor_value accel_x, accel_y, accel_z;
	static struct sensor_value gyro_x, gyro_y, gyro_z;
#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
	static struct sensor_value magn_x, magn_y, magn_z;
#endif
#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
	static struct sensor_value press, temp;
#endif
	lsm6dsl_trig_cnt++;

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_MAGN_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_X, &magn_x);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_Y, &magn_y);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_Z, &magn_z);
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_PRESS);
	sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_AMBIENT_TEMP);
	sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
#endif

	if (print_samples) {
		print_samples = 0;

		accel_x_out = accel_x;
		accel_y_out = accel_y;
		accel_z_out = accel_z;

		gyro_x_out = gyro_x;
		gyro_y_out = gyro_y;
		gyro_z_out = gyro_z;

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
		magn_x_out = magn_x;
		magn_y_out = magn_y;
		magn_z_out = magn_z;
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
		press_out = press;
		temp_out = temp;
#endif
	}

}
#endif


static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(
    0x12, 0x34, 0x56, 0x78, 0x90, 0xab, 0xcd, 0xef,
    0x12, 0x34, 0x56, 0x78, 0x90, 0xab, 0xcd, 0xef);

static struct bt_uuid_128 x1_char_uuid = BT_UUID_INIT_128(
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x45);

static struct bt_uuid_128 x2_char_uuid = BT_UUID_INIT_128(
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x46);

static struct bt_uuid_128 x3_char_uuid = BT_UUID_INIT_128(
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x47);

static struct bt_uuid_128 y1_char_uuid = BT_UUID_INIT_128(
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x48);

static struct bt_uuid_128 y2_char_uuid = BT_UUID_INIT_128(
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x49);

static struct bt_uuid_128 y3_char_uuid = BT_UUID_INIT_128(
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x50);

static uint8_t x1_value[4];  // 4 bytes to store float value of x1
static uint8_t x2_value[4];  // 4 bytes to store float value of x2
static uint8_t x3_value[4];  // 4 bytes to store float value of x3
static uint8_t y1_value[4];  // 4 bytes to store float value of y1
static uint8_t y2_value[4];  // 4 bytes to store float value of y2
static uint8_t y3_value[4];  // 4 bytes to store float value of y3

// Read handler for x1_accel_x
static ssize_t read_x1(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, x1_value, sizeof(x1_value));
}

// Read handler for x2_accel_y
static ssize_t read_x2(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, x2_value, sizeof(x2_value));
}

// Read handler for x3_accel_z
static ssize_t read_x3(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, x3_value, sizeof(x3_value));
}

// Read handler for y1_gyro_x
static ssize_t read_y1(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, y1_value, sizeof(y1_value));
}

// Read handler for y2_gyro_y
static ssize_t read_y2(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, y2_value, sizeof(y2_value));
}

// Read handler for y3_gyro_z
static ssize_t read_y3(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, y3_value, sizeof(y3_value));
}


BT_GATT_SERVICE_DEFINE(custom_service,
    BT_GATT_PRIMARY_SERVICE(&service_uuid),
    BT_GATT_CHARACTERISTIC(&x1_char_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,  // Allow read and notify
        BT_GATT_PERM_READ,                       // Permission to read
        read_x1, NULL, x1_value),               // Provide the read handler for x1_accel_x
    BT_GATT_CHARACTERISTIC(&x2_char_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,  // Allow read and notify
        BT_GATT_PERM_READ,                       // Permission to read
        read_x2, NULL, x2_value),               // Provide the read handler for x2_accel_y
	BT_GATT_CHARACTERISTIC(&x3_char_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,  // Allow read and notify
        BT_GATT_PERM_READ,                       // Permission to read
        read_x3, NULL, x3_value),               // Provide the read handler for x3_accel_z
	BT_GATT_CHARACTERISTIC(&y1_char_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,  // Allow read and notify
        BT_GATT_PERM_READ,                       // Permission to read
        read_y1, NULL, y1_value),               // Provide the read handler for y1_gyro_x
	BT_GATT_CHARACTERISTIC(&y2_char_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,  // Allow read and notify
        BT_GATT_PERM_READ,                       // Permission to read
        read_y2, NULL, y2_value),               // Provide the read handler for y2_gyro_y
	BT_GATT_CHARACTERISTIC(&y3_char_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,  // Allow read and notify
        BT_GATT_PERM_READ,                       // Permission to read
        read_y3, NULL, y3_value),               // Provide the read handler for y3_gyro_z
);

void main(void) 
{
	//==================================================================
	double counter = 0;
	int cnt = 0;
	char out_str[64];
	struct sensor_value odr_attr;
	const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

	if (!device_is_ready(lsm6dsl_dev)) {
		printk("sensor: device not ready.\n");
		return 0;
	}

    int err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

	// Start advertising
    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, 0x12, 0x34, 0x56, 0x78, 0x90, 0xab, 0xcd, 0xef,
                      0x12, 0x34, 0x56, 0x78, 0x90, 0xab, 0xcd, 0xef),
    };

	struct bt_le_adv_param adv_params = {
        .options = BT_LE_ADV_OPT_CONNECTABLE,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    };

    err = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising start failed (err %d)\n", err);
        return;
    }
    printk("Advertising started\n");

	odr_attr.val1 = 104;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
		return 0;
	}

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for gyro.\n");
		return 0;
	}

#ifdef CONFIG_LSM6DSL_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
		printk("Could not set sensor type and channel\n");
		return 0;
	}
#endif

	if (sensor_sample_fetch(lsm6dsl_dev) < 0) {
		printk("Sensor sample update error\n");
		return 0;
	}
	//===============================================================================================
	float x1 = 0, x2 = 0, x3 = 0, y1 = 0, y2 = 0, y3 = 0;
    while (1) 
	{
		printk("\0033\014");
		printf("LSM6DSL sensor samples:\n\n");

		sprintf(out_str, "accel x:%f ms/2 y:%f ms/2 z:%f ms/2",
							  sensor_value_to_double(&accel_x_out),
							  sensor_value_to_double(&accel_y_out),
							  sensor_value_to_double(&accel_z_out));
		printk("%s\n", out_str);

		sprintf(out_str, "gyro x:%f dps y:%f dps z:%f dps",
							   sensor_value_to_double(&gyro_x_out),
							   sensor_value_to_double(&gyro_y_out),
							   sensor_value_to_double(&gyro_z_out));
		printk("%s\n", out_str);
		x1 = sensor_value_to_double(&accel_x_out); 
		x2 = sensor_value_to_double(&accel_y_out);
		x3 = sensor_value_to_double(&accel_z_out);
		y1 = sensor_value_to_double(&gyro_x_out);
		y2 = sensor_value_to_double(&gyro_y_out);
		y3 = sensor_value_to_double(&gyro_z_out);

		// Copy float x1 and x2 values to byte arrays
        memcpy(x1_value, &x1, sizeof(x1));
        memcpy(x2_value, &x2, sizeof(x2));
		memcpy(x3_value, &x3, sizeof(x3));
        memcpy(y1_value, &y1, sizeof(y1));
		memcpy(y2_value, &y2, sizeof(y2));
        memcpy(y3_value, &y3, sizeof(y3));

		// Send notifications
        bt_gatt_notify(NULL, &custom_service.attrs[1], x1_value, sizeof(x1_value));
        bt_gatt_notify(NULL, &custom_service.attrs[2], x2_value, sizeof(x2_value));
		bt_gatt_notify(NULL, &custom_service.attrs[3], x3_value, sizeof(x3_value));
        bt_gatt_notify(NULL, &custom_service.attrs[4], y1_value, sizeof(y1_value));
		bt_gatt_notify(NULL, &custom_service.attrs[5], y2_value, sizeof(y2_value));
        bt_gatt_notify(NULL, &custom_service.attrs[6], y3_value, sizeof(y3_value));

		printk("Sending x1: %f, x2: %f, x3: %f, y1: %f, y2: %f, y3: %f\n", x1, x2, x3, y1, y2, y3);


#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
		sprintf(out_str, "magn x:%f gauss y:%f gauss z:%f gauss",
							   sensor_value_to_double(&magn_x_out),
							   sensor_value_to_double(&magn_y_out),
							   sensor_value_to_double(&magn_z_out));
		printk("%s\n", out_str);
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
		sprintf(out_str, "press: %f kPa - temp: %f deg",
			sensor_value_to_double(&press_out), sensor_value_to_double(&temp_out));
		printk("%s\n", out_str);
#endif

		printk("loop:%d trig_cnt:%d\n\n", ++cnt, lsm6dsl_trig_cnt);

		print_samples = 1;
		k_sleep(K_MSEC(2000));

    }
}
























/*=========================================================================================================================================

static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(
    0x12, 0x34, 0x56, 0x78, 0x90, 0xab, 0xcd, 0xef,
    0x12, 0x34, 0x56, 0x78, 0x90, 0xab, 0xcd, 0xef);

static struct bt_uuid_128 x1_char_uuid = BT_UUID_INIT_128(
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x45);

static struct bt_uuid_128 x2_char_uuid = BT_UUID_INIT_128(
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x46);

static struct bt_uuid_128 x3_char_uuid = BT_UUID_INIT_128(
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x47);

static struct bt_uuid_128 y1_char_uuid = BT_UUID_INIT_128(
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x48);

static struct bt_uuid_128 y2_char_uuid = BT_UUID_INIT_128(
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x49);

static struct bt_uuid_128 y3_char_uuid = BT_UUID_INIT_128(
    0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10,
    0x01, 0x23, 0x45, 0x67, 0x89, 0x01, 0x23, 0x50);

static uint8_t x1_value[4];  // 4 bytes to store float value of x1
static uint8_t x2_value[4];  // 4 bytes to store float value of x2
static uint8_t x3_value[4];  // 4 bytes to store float value of x3
static uint8_t y1_value[4];  // 4 bytes to store float value of y1
static uint8_t y2_value[4];  // 4 bytes to store float value of y2
static uint8_t y3_value[4];  // 4 bytes to store float value of y3

// Read handler for x1
static ssize_t read_x1(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, x1_value, sizeof(x1_value));
}

// Read handler for x2
static ssize_t read_x2(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, x2_value, sizeof(x2_value));
}

// Read handler for x3
static ssize_t read_x3(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, x3_value, sizeof(x3_value));
}

// Read handler for y1
static ssize_t read_y1(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, y1_value, sizeof(y1_value));
}

// Read handler for y2
static ssize_t read_y2(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, y2_value, sizeof(y2_value));
}

// Read handler for y3
static ssize_t read_y3(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                       void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset, y3_value, sizeof(y3_value));
}


BT_GATT_SERVICE_DEFINE(custom_service,
    BT_GATT_PRIMARY_SERVICE(&service_uuid),
    BT_GATT_CHARACTERISTIC(&x1_char_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,  // Allow read and notify
        BT_GATT_PERM_READ,                       // Permission to read
        read_x1, NULL, x1_value),               // Provide the read handler for x1_accel_x
    BT_GATT_CHARACTERISTIC(&x2_char_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,  // Allow read and notify
        BT_GATT_PERM_READ,                       // Permission to read
        read_x2, NULL, x2_value),               // Provide the read handler for x2_accel_y
	BT_GATT_CHARACTERISTIC(&x3_char_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,  // Allow read and notify
        BT_GATT_PERM_READ,                       // Permission to read
        read_x3, NULL, x3_value),               // Provide the read handler for x3_accel_z
	BT_GATT_CHARACTERISTIC(&y1_char_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,  // Allow read and notify
        BT_GATT_PERM_READ,                       // Permission to read
        read_y1, NULL, y1_value),               // Provide the read handler for y1_gyro_x
	BT_GATT_CHARACTERISTIC(&y2_char_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,  // Allow read and notify
        BT_GATT_PERM_READ,                       // Permission to read
        read_y2, NULL, y2_value),               // Provide the read handler for y2_gyro_y
	BT_GATT_CHARACTERISTIC(&y3_char_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,  // Allow read and notify
        BT_GATT_PERM_READ,                       // Permission to read
        read_y3, NULL, y3_value),               // Provide the read handler for y3_gyro_z
);

void main(void)
{
    int err;
    float x1 = 3, x2 = 5, x3 = 7, y1 = 9, y2 = 11, y3 = 13;  // Use float instead of double

    // Initialize Bluetooth
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    // Start advertising
    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_UUID128_ALL, 0x12, 0x34, 0x56, 0x78, 0x90, 0xab, 0xcd, 0xef,
                      0x12, 0x34, 0x56, 0x78, 0x90, 0xab, 0xcd, 0xef),
    };

    struct bt_le_adv_param adv_params = {
        .options = BT_LE_ADV_OPT_CONNECTABLE,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    };

    err = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("Advertising start failed (err %d)\n", err);
        return;
    }
    printk("Advertising started\n");

    while (1) {
		
        // Copy float x1 and x2 values to byte arrays
        memcpy(x1_value, &x1, sizeof(x1));
        memcpy(x2_value, &x2, sizeof(x2));
		memcpy(x3_value, &x3, sizeof(x3));
        memcpy(y1_value, &y1, sizeof(y1));
		memcpy(y2_value, &y2, sizeof(y2));
        memcpy(y3_value, &y3, sizeof(y3));

        // Send notifications
        bt_gatt_notify(NULL, &custom_service.attrs[1], x1_value, sizeof(x1_value));
        bt_gatt_notify(NULL, &custom_service.attrs[2], x2_value, sizeof(x2_value));
		bt_gatt_notify(NULL, &custom_service.attrs[3], x3_value, sizeof(x3_value));
        bt_gatt_notify(NULL, &custom_service.attrs[4], y1_value, sizeof(y1_value));
		bt_gatt_notify(NULL, &custom_service.attrs[5], y2_value, sizeof(y2_value));
        bt_gatt_notify(NULL, &custom_service.attrs[6], y3_value, sizeof(y3_value));

        printk("Sending x1: %f, x2: %f, x3: %f, y1: %f, y2: %f, y3: %f\n", x1, x2, x3, y1, y2, y3);
        k_sleep(K_SECONDS(1));  // Send data every second
		x1 = x1+1;
		x2 = x2+1;
		x3 = x3+1;
		y1 = y1+1;
		y2 = y2+1;
		y3 = y3+1;
    }
}
==================================================================================================================================*/