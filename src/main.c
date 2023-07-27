/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/logging/log.h>

#include "imu.h"

LOG_MODULE_REGISTER(MAIN, LOG_LEVEL_DBG);

#define ACCL_SERVICE_UUID_VAL		BT_UUID_128_ENCODE(0xADAF0200, 0xC332, 0x42A8, 0x93BD, 0x25E905756CB8)
#define MEASURE_PERIOD_UUID_VAL		BT_UUID_128_ENCODE(0xADAF0001, 0xC332, 0x42A8, 0x93BD, 0x25E905756CB8)
#define ACCELERATION_UUID_VAL		BT_UUID_128_ENCODE(0xADAF0201, 0xC332, 0x42A8, 0x93BD, 0x25E905756CB8)

#define RSSI_SERVICE_UUID_VAL		BT_UUID_128_ENCODE(0xADAFF000, 0xC332, 0x42A8, 0x93BD, 0x25E905756CB8)
#define RSSI_CHRC_UUID_VAL			BT_UUID_128_ENCODE(0xADAFF001, 0xC332, 0x42A8, 0x93BD, 0x25E905756CB8)

struct bt_conn *default_conn;
bool accl_notifications_enabled = false, rssi_notifications_enabled = false;

float last_accl_values[3];
int32_t measure_period_ms = 1000;
int8_t last_rssi = 127;

static ssize_t accl_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	LOG_DBG("Accl read");
	return bt_gatt_attr_read(conn, attr, buf, len, offset, last_accl_values, sizeof(last_accl_values));
}

static ssize_t measure_period_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	LOG_DBG("Measure period read");
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &measure_period_ms, sizeof(measure_period_ms));
}

static ssize_t measure_period_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if(len != sizeof(measure_period_ms)){
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	memcpy(&measure_period_ms, buf, sizeof(measure_period_ms));
	LOG_DBG("Measure period write, value %d", measure_period_ms);
	return len;
}

static ssize_t rssi_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	LOG_DBG("RSSI read");
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &last_rssi, sizeof(last_rssi));
}

static void accl_ccc_changed_cb(const struct bt_gatt_attr *attr, uint16_t value)
{
	if(value & BT_GATT_CCC_NOTIFY){
		LOG_INF("Accl Notifications enabled");
		accl_notifications_enabled = true;
	}
	else{
		accl_notifications_enabled = false;
	}
}

static void rssi_ccc_changed_cb(const struct bt_gatt_attr *attr, uint16_t value)
{
	if(value & BT_GATT_CCC_NOTIFY){
		LOG_INF("RSSI Notifications enabled");
		rssi_notifications_enabled = true;
	}
	else{
		rssi_notifications_enabled = false;
	}	
}

BT_GATT_SERVICE_DEFINE(
	accl_service,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(ACCL_SERVICE_UUID_VAL)),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(ACCELERATION_UUID_VAL), 
							BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
							BT_GATT_PERM_READ, accl_read_cb, NULL, NULL),
	BT_GATT_CCC(accl_ccc_changed_cb, BT_GATT_PERM_WRITE | BT_GATT_PERM_READ),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(MEASURE_PERIOD_UUID_VAL), 
							BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
							BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, measure_period_read_cb, measure_period_write_cb, &measure_period_ms),
);

BT_GATT_SERVICE_DEFINE(
	rssi_service,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(RSSI_SERVICE_UUID_VAL)),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(RSSI_CHRC_UUID_VAL), 
							BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
							BT_GATT_PERM_READ, rssi_read_cb, NULL, &last_rssi),
	BT_GATT_CCC(rssi_ccc_changed_cb, BT_GATT_PERM_WRITE | BT_GATT_PERM_READ),
);

struct bt_data adv_data[] = {
	BT_DATA(BT_DATA_UUID128_ALL, BT_UUID_DECLARE_128(ACCL_SERVICE_UUID_VAL), BT_UUID_SIZE_128),
};

static int start_advertisement()
{
	int err = 0;

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
	if(err){
		printk("Adv start fail(err %d)", err);
		return err;
	}

	printk("Advertisement started!");

	return 0;
}

static void get_rssi(int8_t *rssi)
{
	uint16_t conn_handle = 0;
	int err = 0;

	struct net_buf *buf, *resp = NULL;
	struct bt_hci_cp_read_rssi *cp;
	struct bt_hci_rp_read_rssi *rp;

	if(default_conn == NULL){
		printk("No BLE connection");
	}
	else{
		err = bt_hci_get_conn_handle(default_conn, &conn_handle);
		if(err){
			printk("Conn handle get fail (err %d)", err);
			return;
		}

		printk("Conn handle: %d", conn_handle);

		buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(*cp));
		if(buf == NULL){
			printk("Unable to allocate command buffer");
			return;
		}

		cp = net_buf_add(buf, sizeof(*cp));
		cp->handle = sys_cpu_to_le16(conn_handle);

		err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &resp);
		if(err){
			printk("Read RSSI fail");
			return;
		}

		rp = (void *)resp->data;
		*rssi = rp->rssi;

		net_buf_unref(resp);
	}
}

static void ble_connected(struct bt_conn *conn, uint8_t err)
{
	if(err){
		LOG_WRN("Connection error %d", err);
		bt_conn_unref(conn);
		default_conn = NULL;
		return;
	}

	LOG_INF("BLE connected");
	default_conn = bt_conn_ref(conn);
}

static void ble_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("BLE disconnected, reason %d", reason);
	bt_conn_unref(default_conn);
	default_conn = NULL;

	start_advertisement();
}

static struct bt_conn_cb conn_cbs = {
	.connected = ble_connected,
	.disconnected = ble_disconnected
};

void main(void)
{
	if(bt_enable(NULL) != 0){
		return;
	}

	bt_conn_cb_register(&conn_cbs);

	if(start_advertisement() != 0){
		return;
	}

	if(imu_init(12.5) != 0){
		// return;
	}

	while(1){
		k_sleep(K_MSEC(measure_period_ms));

		if(default_conn){
			get_rssi(&last_rssi);
			if(fetch_imu_accl(&last_accl_values[0], &last_accl_values[1], &last_accl_values[2]) == 0){
				if(accl_notifications_enabled){
					if(bt_gatt_notify(default_conn, &accl_service.attrs[1], last_accl_values, sizeof(last_accl_values)) != 0){
						LOG_WRN("Accl notification fail");
					}
				}

				if(rssi_notifications_enabled){
					if(bt_gatt_notify(default_conn, &rssi_service.attrs[1], &last_rssi, sizeof(last_rssi)) != 0){
						LOG_WRN("rssi notification fail");
					}					
				}
			}
		}
	}
}
