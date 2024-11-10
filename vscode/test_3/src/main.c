#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/printk.h>
#include <string.h>

// Define custom UUIDs for the Bluetooth service and characteristic
static struct bt_uuid_128 custom_service_uuid = BT_UUID_INIT_128(
    0x23, 0x19, 0xA0, 0x62, 0xE6, 0xA0, 0x45, 0x12,
    0xB4, 0xED, 0x99, 0xB0, 0x03, 0x00, 0x00, 0x00);

static struct bt_uuid_128 custom_char_uuid = BT_UUID_INIT_128(
    0x23, 0x19, 0xA0, 0x62, 0xE6, 0xA0, 0x45, 0x12,
    0xB4, 0xED, 0x99, 0xB0, 0x04, 0x00, 0x00, 0x00);

// Buffer to hold x1, x2, and x3 values (3 floats at 4 bytes each)
static uint8_t data_buffer[12];

// Callback for reading the characteristic
static ssize_t read_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, data_buffer, sizeof(data_buffer));
}

// Define the GATT service and characteristic for sending x1, x2, x3 values
BT_GATT_SERVICE_DEFINE(custom_svc,
    BT_GATT_PRIMARY_SERVICE(&custom_service_uuid),
    BT_GATT_CHARACTERISTIC(&custom_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ, read_data, NULL, NULL),
);

// Function to simulate or retrieve x1, x2, and x3 values
void update_x_values(float *x1, float *x2, float *x3) {
    // Replace these with actual sensor readings if available
    *x1 = 1.23f; // Example value for x1
    *x2 = 4.56f; // Example value for x2
    *x3 = 7.89f; // Example value for x3
}

void main(void) {
    int err;

    // Initialize Bluetooth
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("Bluetooth initialized\n");

    // Set up Bluetooth advertising parameters
    struct bt_le_adv_param adv_params = BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_CONNECTABLE, BT_GAP_ADV_SLOW_INT_MIN, BT_GAP_ADV_SLOW_INT_MAX, NULL);

    // Start advertising
    err = bt_le_adv_start(&adv_params, NULL, 0, NULL, 0);
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }
    printk("Advertising successfully started\n");

    // Main loop to update and send x1, x2, x3 values
    while (1) {
        float x1, x2, x3;

        // Update x1, x2, and x3 values
        update_x_values(&x1, &x2, &x3);

        // Copy x1, x2, and x3 values into the data buffer
        memcpy(&data_buffer[0], &x1, sizeof(float));
        memcpy(&data_buffer[4], &x2, sizeof(float));
        memcpy(&data_buffer[8], &x3, sizeof(float));

        // Send the updated data via Bluetooth notification
        bt_gatt_notify(NULL, &custom_svc.attrs[1], data_buffer, sizeof(data_buffer));

        // Wait before sending the next update
        k_sleep(K_MSEC(100)); // Adjust as needed
    }
}