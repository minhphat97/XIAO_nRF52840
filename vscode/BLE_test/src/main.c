#include <zephyr.h>
#include <device.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/gatt.h>
#include <bluetooth/uuid.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(main);

// Define the UUIDs for the BLE service and characteristic
static const struct bt_uuid_128 imu_service_uuid = BT_UUID_INIT_128(
    0x00000001, 0x0000, 0x1000, 0x8000, 0x00805F9B34FB);
static const struct bt_uuid_128 imu_data_uuid = BT_UUID_INIT_128(
    0x00000002, 0x0000, 0x1000, 0x8000, 0x00805F9B34FB);

static struct bt_gatt_service imu_service;
static struct bt_gatt_attr imu_data_attr;

// Global variables to store IMU data
static struct sensor_value accel[3];  // Acceleration data (x, y, z)
static struct sensor_value gyro[3];    // Gyroscope data (x, y, z)

// Function to read IMU data
static ssize_t read_imu_data(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              void *buf, uint16_t len, uint16_t offset) {
    // Format the IMU data to send as a string
    char data[64];
    snprintf(data, sizeof(data), "ax: %d, ay: %d, az: %d, gx: %d, gy: %d, gz: %d\n",
             accel[0].val1, accel[1].val1, accel[2].val1,
             gyro[0].val1, gyro[1].val1, gyro[2].val1);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, data, strlen(data));
}

// Function to initialize BLE
static void init_ble(void) {
    int err;

    // Initialize Bluetooth
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");

    // Initialize the GATT service
    imu_service.attrs = &imu_data_attr;
    imu_service.uuid = &imu_service_uuid;

    // Register the GATT service
    bt_gatt_service_register(&imu_service);

    LOG_INF("BLE service registered");

    // Start advertising
    struct bt_le_adv_param adv_params = {
        .options = BT_LE_ADV_OPT_CONNECTABLE,
        .interval_min = BT_GAP_ADV_INT_MIN,
        .interval_max = BT_GAP_ADV_INT_MAX,
    };

    err = bt_le_adv_start(&adv_params, NULL, 0, NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
    } else {
        LOG_INF("BLE advertising started");
    }
}

void main(void) {
    const struct device *imu_dev = device_get_binding(DT_LABEL(DT_NODELABEL(lsm6ds3)));
    if (!imu_dev) {
        LOG_ERR("Failed to get IMU device");
        return;
    }

    // Initialize BLE
    init_ble();

    // Start reading IMU data in the main loop
    while (1) {
        // Fetch sensor data
        if (sensor_sample_fetch(imu_dev) < 0) {
            LOG_ERR("Failed to fetch sensor data");
            continue;
        }

        // Read acceleration and gyroscope data
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_X, &accel[0]);
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Y, &accel[1]);
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_Z, &accel[2]);
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_X, &gyro[0]);
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Y, &gyro[1]);
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_Z, &gyro[2]);

        // Wait before the next sample
        k_sleep(K_MSEC(1000));
    }
}
