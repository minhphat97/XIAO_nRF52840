#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/bas.h>

LOG_MODULE_REGISTER(ble_test);
volatile bool ble_ready = false;

static const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
        BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_BAS_VAL))
};

void bt_ready(int err)
{
        if (err)
        {
                LOG_ERR("bt enable rreturn %d", err);
        }
        LOG_INF("bt_ready!\n");
        ble_ready = true;
}

int init_ble(void)
{
        LOG_INF("Init BLE");
        int err;

        err = bt_enable(bt_ready);
        if(err)
        {
                LOG_ERR("bt_enable failed (err %d)", err);
                return err;
        }
        return 0;

}

void main(void)
{

        init_ble();
        while(!ble_ready)
        {
                LOG_INF("BLE stack not ready yet");
                k_msleep(100);
        }
        LOG_INF("BLE stack ready");

        int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
        
        if (err)
        {
                printk("Advertising failed to start (err %d)\n", err);
                return;
        }

        printk("Hello World! %s\n", CONFIG_BOARD);
}
