/*
 * ble_server.h
 *
 *  Created on: Sep 10, 2017
 *      Author: iron
 */

#ifndef MAIN_BLE_SERVER_H_
#define MAIN_BLE_SERVER_H_

#include "bt.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "sdkconfig.h"

#define BLE_DEVICE_NAME				"BLE-Skateboard"
#define GATTS_TAG					"BLE-Server"
#define SERVICE_UUID  	 			0x00AA
#define CHAR_UUID					0xFF01

int speed;

esp_err_t BLE_Server_Init();
void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

#endif /* MAIN_BLE_SERVER_H_ */
