#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "ble_server.h"
#include "bta_api.h"

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static uint8_t service_uuid128[ESP_UUID_LEN_128] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xAB, 0xCD, 0x00, 0x00
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

uint8_t char_str[] = {0x11,0x22,0x33};
esp_attr_value_t char_val =
{
    .attr_max_len = 0x40,
    .attr_len     = sizeof(char_str),
    .attr_value   = char_str,
};

struct gatts_profile_inst gatt_prof = {
		.gatts_if = ESP_GATT_IF_NONE
};

esp_err_t BLE_Server_Init()
{
	esp_err_t ret;
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s initialize controller failed\n", __func__);
		return ret;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s enable controller failed\n", __func__);
		return ret;
	}
	ret = esp_bluedroid_init();
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s init bluetooth failed\n", __func__);
		return ret;
	}
	ret = esp_bluedroid_enable();
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed\n", __func__);
		return ret;
	}

	speed = 0;
	esp_ble_gatts_register_callback(gatts_event_handler);
	esp_ble_gap_register_callback(gap_event_handler);
	esp_ble_gatts_app_register(0);
	ESP_LOGI(GATTS_TAG,"speed init %d\n", speed);

	return ESP_OK;
}

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

	ESP_LOGI(GATTS_TAG,"%s--------event: %d\n", __func__, event);
	switch (event) {
	    case ESP_GATTS_REG_EVT:
	        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
	        gatt_prof.service_id.is_primary = true;
	        gatt_prof.service_id.id.inst_id = 0x00;
	        gatt_prof.service_id.id.uuid.len = ESP_UUID_LEN_16;
	        gatt_prof.service_id.id.uuid.uuid.uuid16 = SERVICE_UUID;

	        esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
	        esp_ble_gap_config_adv_data(&adv_data);
	        esp_ble_gatts_create_service(gatts_if, &gatt_prof.service_id, 4);
	        break;
	    case ESP_GATTS_READ_EVT: {
	        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
	        esp_gatt_rsp_t rsp;
	        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
	        rsp.attr_value.handle = param->read.handle;
	        rsp.attr_value.len = 4;
	        rsp.attr_value.value[0] = 0xde;
	        rsp.attr_value.value[1] = 0xed;
	        rsp.attr_value.value[2] = 0xbe;
	        rsp.attr_value.value[3] = 0xef;
	        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
	                                    ESP_GATT_OK, &rsp);
	        break;
	    }
	    case ESP_GATTS_WRITE_EVT: {
	        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
	        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value %08x(%d)\n", param->write.len, *(uint32_t *)param->write.value, speed);
	        esp_gatt_rsp_t rsp;
	        if (*(char *)param->write.value == 0xaa)
	        	speed = 1;
	        else if (*(char *)param->write.value == 0xbb)
	        	speed = -1;
	        ESP_LOGI(GATTS_TAG, "Speed: %d\n", speed);
	        //example_write_event_env(gatts_if, &a_prepare_write_env, param);
	        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
	        	                                    ESP_GATT_OK, &rsp);
	        break;
	    }
	    case ESP_GATTS_EXEC_WRITE_EVT:
	        ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
	        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
	        //example_exec_write_event_env(&a_prepare_write_env, param);
	        break;
	    case ESP_GATTS_MTU_EVT:
	    case ESP_GATTS_CONF_EVT:
	    case ESP_GATTS_UNREG_EVT:
	        break;
	    case ESP_GATTS_CREATE_EVT:
	        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
	        gatt_prof.service_handle = param->create.service_handle;
	        gatt_prof.char_uuid.len = ESP_UUID_LEN_16;
	        gatt_prof.char_uuid.uuid.uuid16 = CHAR_UUID;

	        esp_ble_gatts_start_service(gatt_prof.service_handle);

	        esp_ble_gatts_add_char(gatt_prof.service_handle, &gatt_prof.char_uuid,
	                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
	                               ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
	                               &char_val, NULL);
	        break;
	    case ESP_GATTS_ADD_INCL_SRVC_EVT:
	        break;
	    case ESP_GATTS_ADD_CHAR_EVT: {
	        uint16_t length = 0;
	        const uint8_t *prf_char;

	        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
	                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
	        gatt_prof.char_handle = param->add_char.attr_handle;
	        gatt_prof.descr_uuid.len = ESP_UUID_LEN_16;
	        gatt_prof.descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
	        esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);

	        ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x\n", length);
	        for(int i = 0; i < length; i++){
	            ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
	        }
	        esp_ble_gatts_add_char_descr(gatt_prof.service_handle, &gatt_prof.descr_uuid,
	                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
	        break;
	    }
	    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
	        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
	                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
	        break;
	    case ESP_GATTS_DELETE_EVT:
	        break;
	    case ESP_GATTS_START_EVT:
	        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
	                 param->start.status, param->start.service_handle);
	        break;
	    case ESP_GATTS_STOP_EVT:
	        break;
	    case ESP_GATTS_CONNECT_EVT:
	        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:, is_conn %d\n",
	                 param->connect.conn_id,
	                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
	                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5],
	                 param->connect.is_connected);
	        gatt_prof.conn_id = param->connect.conn_id;
	        break;
	    case ESP_GATTS_DISCONNECT_EVT:
	        esp_ble_gap_start_advertising(&adv_params);
	        break;
	    case ESP_GATTS_OPEN_EVT:
	    case ESP_GATTS_CANCEL_OPEN_EVT:
	    case ESP_GATTS_CLOSE_EVT:
	    case ESP_GATTS_LISTEN_EVT:
	    case ESP_GATTS_CONGEST_EVT:
	    default:
	        break;
	    }
}

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	switch (event) {
	    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
	        esp_ble_gap_start_advertising(&adv_params);
	        break;
	    default:
	        break;
	}
}

