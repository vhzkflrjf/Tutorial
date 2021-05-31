/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_bps_main main.c
 * @{
 * @ingroup ble_sdk_app_bps
 * @brief Blood Pressure Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Blood pressure service.
 * This file also contains the code for initializing and using the Battery Service and the Device
 * Information Service. Furthermore, it demonstrates the use of the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "ble_nus.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_uart.h"

//user_lib include
#include "smd_boards_01.h"
#include "mlx90632.h"
#include "stc3115_Driver.h"
#include "twi_i2c.h"
#include "t_uart.h"
#include "ICM20948.h"
#include "interface.h"
#include "vc_app.h"

/**************Hz Test mode Flag*****************/
#define IMU_TEST                        0
#define PPG_TEST                        0
#define BATTERY_GAUSE_TEST              0
#define TEMPERATURE_TEST                0
#define TEST_CNT                        20
/************************************************/

//#include "max32664.h"

/******** max32664 sunsor hub library *********/
//#include "../../../user_lib/MAX32664C/SHComm/SHComm.h"
//#include "../../../user_lib/MAX32664C/simplest/algoConfigAPI.h"
/************************************************/
// add pairing action
#define ADD_PAIRING_ACTION
#ifdef ADD_PAIRING_ACTION
#define STATIC_PASSKEY "111111"
#endif

// martin add poweron in charging
//#define EN_POWER_ON_IN_CHARGING
// martin add poweroff directly in connceted status
#define DISABLE_DISCONNECT_BY_BUTTON

// martin
#define DEVICE_NAME                     "SMD_VitalCare"                            /**< Name of device. Will be included in the advertising data. */
//#define DEVICE_NAME                     "SMD_NeckCare"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "SMDsolutions"                   /**< Manufacturer. Will be passed to Device Information Service. */
// martin
#define MODEL_NUM                       "VITAL_CARE_V1.00"                        /**< Model number. Will be passed to Device Information Service. */
//#define MODEL_NUM                       "NECK_CARE_V1.00"                        /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 0x1122334455                            /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   0x667788                                /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64//64//1600             // 1600: 1 sec, 8000: 5 sec                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */

#define APP_ADV_DURATION                0//180//0                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define NUM_SIM_MEAS_VALUES             4                                       /**< Number of simulated measurements to cycle through. */

#define SIM_MEAS_1_SYSTOLIC             117                                     /**< Simulated measurement value for systolic pressure. */
#define SIM_MEAS_1_DIASTOLIC            76                                      /**< Simulated measurement value for diastolic pressure. */
#define SIM_MEAS_1_MEAN_AP              103                                     /**< Simulated measurement value for mean arterial pressure. */
#define SIM_MEAS_1_PULSE_RATE           60                                      /**< Simulated measurement value for pulse rate. */

#define SIM_MEAS_2_SYSTOLIC             121                                     /**< Simulated measurement value for systolic pressure. */
#define SIM_MEAS_2_DIASTOLIC            81                                      /**< Simulated measurement value for diastolic pressure. */
#define SIM_MEAS_2_MEAN_AP              106                                     /**< Simulated measurement value for mean arterial pressure. */
#define SIM_MEAS_2_PULSE_RATE           72                                      /**< Simulated measurement value for pulse rate. */

#define SIM_MEAS_3_SYSTOLIC             138                                     /**< Simulated measurement value for systolic pressure. */
#define SIM_MEAS_3_DIASTOLIC            88                                      /**< Simulated measurement value for diastolic pressure. */
#define SIM_MEAS_3_MEAN_AP              120                                     /**< Simulated measurement value for mean arterial pressure. */
#define SIM_MEAS_3_PULSE_RATE           105                                     /**< Simulated measurement value for pulse rate. */

#define SIM_MEAS_4_SYSTOLIC             145                                     /**< Simulated measurement value for systolic pressure. */
#define SIM_MEAS_4_DIASTOLIC            100                                     /**< Simulated measurement value for diastolic pressure. */
#define SIM_MEAS_4_MEAN_AP              131                                     /**< Simulated measurement value for mean arterial pressure. */
#define SIM_MEAS_4_PULSE_RATE           125                                     /**< Simulated measurement value for pulse rate. */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000)                   /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                      /**< Minimum battery level as returned by the simulated measurement function. */
#define MAX_BATTERY_LEVEL               100                                     /**< Maximum battery level as returned by the simulated measurement function. */
#define BATTERY_LEVEL_INCREMENT         1                                       /**< Value by which the battery level is incremented/decremented for each call to the simulated measurement function. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(50, UNIT_1_25_MS)       /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of indication) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
// add pairing action
#ifdef ADD_PAIRING_ACTION
#define SEC_PARAM_MITM                  1 // 0 _HYUN                                       /**< Man In The Middle protection not required. */
#else
#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
#endif
#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
// add pairing action
#ifdef ADD_PAIRING_ACTION
#define SEC_PARAM_IO_CAPABILITIES      BLE_GAP_IO_CAPS_DISPLAY_ONLY // _HYUN     BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#else
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#endif
#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */
// add pairing action
#ifdef ADD_PAIRING_ACTION
#define PASSKEY_TXT                     "Passkey:"                                  /**< Message to be displayed together with the pass-key. */
#define PASSKEY_TXT_LENGTH              8                                           /**< Length of message to be displayed together with the pass-key. */
#define PASSKEY_LENGTH                  6                                           /**< Length of pass-key received by the stack for display. */
#endif

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define CONN_TX_POWER    0
#define ADV_TX_POWER       0

/*************** PLATFORM ***************************/
#define POLL_PERIOD_25MS   (1)
#define POLL_PERIOD_1000MS (25)

/***************APP CONFIGURATION******************/

//#define MEASURE_CONT_WHRM_CONT_WSPO2
//#define AUTHENTICATE_TO_SENSORHUB
//#define MEASURE_CONT_WHRM_ONESHOT_WSPO2
//#define MEASURE_CONT_HRM
//#define MEASURE_WHRM_WSPO2_EXTENDED_REPORT
//#define GET_RAW_GREEN_IR_RED_PPG
//#define BOOTLOADER_SEQUENCE


NUS_RX_DATA                       nus_rx_data;

APP_TIMER_DEF(m_battery_timer_id);                                      /**< Battery timer. */
BLE_BAS_DEF(m_bas);                                                     /**< Structure used to identify the battery service. */
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                       /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                 /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                     /**< Advertising module instance. */

// add pairing action
#ifdef ADD_PAIRING_ACTION
static pm_peer_id_t m_peer_to_be_deleted = PM_PEER_ID_INVALID;  // _HYUN
#endif

uint16_t             m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */   //static 
static bool                 m_bps_meas_ind_conf_pending = false;        /**< Flag to keep track of when an indication confirmation is pending. */

static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static ble_uuid_t m_adv_uuids[] =                                       /**< Universally unique service identifiers. */
{
    {BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,    BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}//BLE_UUID_TYPE_VENDOR_BEGIN}
};

// add pairing action
#ifdef ADD_PAIRING_ACTION
static void set_static_passkey()
{
    static ble_opt_t    m_static_pin_option;
    uint8_t passkey[] = STATIC_PASSKEY;
    m_static_pin_option.gap_opt.passkey.p_passkey = &passkey[0];
    uint32_t err_code = sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &m_static_pin_option);
    printf("err_code: %d\r\n", err_code);
    APP_ERROR_CHECK(err_code);
}
#endif

static uint8_t nrf_power_off_sec = 4;

extern IntergratedRxBuffer RxBuffer;
extern IntergratedRxBuffer_int RxBuffer_int;

STC3115_ConfigData_TypeDef STC3115_ConfigData;
STC3115_BatteryData_TypeDef STC3115_BatteryData;

static void advertising_start(void);//(bool erase_bonds);
static void blood_pressure_measurement_send(void);
//void ReadIMUSensor(void);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;
    bool       is_indication_enabled;

    pm_handler_on_pm_evt(p_evt);
// add pairing action
#ifdef ADD_PAIRING_ACTION
    pm_handler_disconnect_on_sec_failure(p_evt);  // _HYUN
#endif
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
// add pairing action
#ifdef ADD_PAIRING_ACTION
        {
            pm_conn_sec_status_t conn_sec_status;

            // Check if the link is authenticated (meaning at least MITM).
            err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
            APP_ERROR_CHECK(err_code);

            if (conn_sec_status.mitm_protected)
            {
                NRF_LOG_INFO("Link secured. Role: %d. conn_handle: %d, Procedure: %d",
                             ble_conn_state_role(p_evt->conn_handle),
                             p_evt->conn_handle,
                             p_evt->params.conn_sec_succeeded.procedure);
            }
            else
            {
                // The peer did not use MITM, disconnect.
                NRF_LOG_INFO("Collector did not use MITM, disconnecting");
                err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
                APP_ERROR_CHECK(err_code);
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
        }
#else
            // Send a single blood pressure measurement if indication is enabled.
            // NOTE: For this to work, make sure ble_bps_on_ble_evt() is called before
            // ble_bondmngr_on_ble_evt() in ble_evt_dispatch().
            
            if (is_indication_enabled)
            {
            
            }
#endif
		break;

// add pairing action
#ifdef ADD_PAIRING_ACTION
        case PM_EVT_CONN_SEC_FAILED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

// martin add for reparing
// https://devzone.nordicsemi.com/f/nordic-q-a/51274/bonding-doesn-t-work-after-deleting-bond-info-on-nrf-connect
        case PM_EVT_CONN_SEC_CONFIG_REQ:
          {
              // Reject pairing request from an already bonded peer.              
              pm_conn_sec_config_t conn_sec_config = {.allow_repairing = true};
              pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
          } 
        break;
#endif

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start();//(false);
            break;

        default:
            break;
    }
}

/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t  battery_level;
    unsigned int bat_calc;

    GasGauge_Task(&STC3115_ConfigData, &STC3115_BatteryData);
    
    battery_level = (uint8_t)(STC3115_BatteryData.SOC/10);

    mySYS->bat_level = battery_level;

//    printf("power: %d, voltage: %d\r\n", battery_level, STC3115_BatteryData.Voltage);
      printf("power: %d, voltage: %d, current: %d\r\n", mySYS->bat_level, STC3115_BatteryData.Voltage,STC3115_BatteryData.Current);
        
    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }

    if(STC3115_BatteryData.Voltage<3200)//||(battery_level==0))
    {
        nrf_gpio_pin_write(LED_BLUE, true);
        nrf_gpio_pin_write(CHRG_DONE, true);
        nrf_gpio_pin_write(CHARGING, false);
        PowerOff();
        while(1)
        {
          nrf_gpio_pin_write(LED_BLUE, true);
//                nrf_gpio_pin_write(CHARGING, false);
        }
    }
}



/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
void battery_level_meas_timeout_handler(void * p_context)  //statick
{
    int chrg, pwr_on;
    static int count=0;

    UNUSED_PARAMETER(p_context);
    if(++count>10)
    {
        count = 0;
        battery_level_update();
    }
    
#ifdef EN_POWER_ON_IN_CHARGING
#else
    pwr_on = nrf_gpio_pin_read(CHRG_POWERON);
    chrg = nrf_gpio_pin_read(CHRG);

    if(pwr_on==0&&chrg==1) // charge complete
    {
        NVIC_SystemReset();
    }
    else if(pwr_on==0&&chrg==0)
    {
        NVIC_SystemReset();
    }
#endif
}

/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void IMU_update(void)
{
    ret_code_t err_code;
    //ReadIMUSensor();
    for(int i; i < 15; i++)
    {
      do
      {
        //err_code = app_uart_put((uint8_t)(RxBuffer.acc_data_1[0]));
        if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
        {
            NRF_LOG_ERROR("Failed receiving Temp message. Error 0x%x. ", err_code);
            APP_ERROR_CHECK(err_code);
        }
      } while (err_code == NRF_ERROR_BUSY);
    }

    //do
    //{
    //    uint16_t length = 1;
    //    err_code = ble_nus_data_send(&m_nus, &temperature, &length, m_conn_handle);
    //    if ((err_code != NRF_ERROR_INVALID_STATE) &&
    //        (err_code != NRF_ERROR_RESOURCES) &&
    //        (err_code != NRF_ERROR_NOT_FOUND))
    //    {
    //        APP_ERROR_CHECK(err_code);
    //    }
    //} while (err_code == NRF_ERROR_RESOURCES);
}

/**@brief Function for handling the temperature measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void IMU_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    IMU_update();
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    ble_gap_addr_t          addr;
    char buf[20];

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_addr_get(&addr);
    APP_ERROR_CHECK(err_code);

    sprintf(buf, "%s_%02X%02X", DEVICE_NAME, addr.addr[1], addr.addr[0]);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) buf,
                                          strlen(buf));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for simulating and sending one Blood Pressure Measurement.
 */
static void something_you_want(void)
{
    //something_you_want
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    NUS_RX_DATA         * rx_data         = &nus_rx_data;

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        if(p_evt->params.rx_data.length <= sizeof(NUS_RX_DATA))
        {
            memcpy(rx_data, p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
            command_handler(rx_data, rx_data->len);

        }
    }
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Blood Pressure, Battery, and Device Information services.
 */
static void services_init(void)
{
    uint32_t           err_code;    
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    ble_dis_sys_id_t   sys_id;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
    mySYS->m_nus = &m_nus;

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.

    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;


    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str, MODEL_NUM);

    sys_id.manufacturer_id            = MANUFACTURER_ID;
    sys_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id                 = &sys_id;

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameter events.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail configuration parameter, but instead we use the
 *                event handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t connection_params_init;

    memset(&connection_params_init, 0, sizeof(connection_params_init));

    connection_params_init.p_conn_params                  = NULL;
    connection_params_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    connection_params_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    connection_params_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    connection_params_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    connection_params_init.disconnect_on_fail             = false;
    connection_params_init.evt_handler                    = on_conn_params_evt;
    connection_params_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&connection_params_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);

    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).    
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();

            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

// add pairing action
#ifdef ADD_PAIRING_ACTION
    pm_handler_secure_on_connection(p_ble_evt);
#endif

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);

            err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, CONN_TX_POWER);
            APP_ERROR_CHECK(err_code);

#if 0
            err_code = app_timer_start(m_imu_timer_id, APP_TIMER_TICKS(500), NULL);   //2hz require
            APP_ERROR_CHECK(err_code);
#endif
            break;

        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected");
            m_conn_handle               = BLE_CONN_HANDLE_INVALID;
            m_bps_meas_ind_conf_pending = false;

            mySYS->is_imu = 0;
#if 1 // martin add for sensor hub
            mySYS->sh_mode = 0;          
            mySYS->sh_mode_old = 0;
            stop_hub_event_poll();
            stop_measure_whrm_wspo2();
#endif
// add pairing action
#ifdef ADD_PAIRING_ACTION
//_HYUN
             if (m_peer_to_be_deleted != PM_PEER_ID_INVALID)
            {
                err_code = pm_peer_delete(m_peer_to_be_deleted);
                APP_ERROR_CHECK(err_code);
                NRF_LOG_DEBUG("Collector's bond deleted");
                m_peer_to_be_deleted = PM_PEER_ID_INVALID;
            }
 // end _HYUN
#endif
#if 0
            err_code = app_timer_stop(m_imu_timer_id);
            APP_ERROR_CHECK(err_code);
#endif
           } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);

            mySYS->is_imu = 0;
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);

            mySYS->is_imu = 0;
            break;
// add pairing action
#ifdef ADD_PAIRING_ACTION                    
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;

        case BLE_GAP_EVT_PASSKEY_DISPLAY:
        {
            char passkey[PASSKEY_LENGTH + 1];
            memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, PASSKEY_LENGTH);
            passkey[PASSKEY_LENGTH] = 0;

            NRF_LOG_INFO("Passkey: %s", nrf_log_push(passkey));
        } break;
        
        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

        case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

         case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

void PowerOff(void)
{
    nrf_gpio_cfg_output(PWR_ON_OFF);     
    nrf_gpio_pin_write(PWR_ON_OFF, false);   //keep Board PWR_OFF
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
          // martin add for power off
            nrf_gpio_pin_write(LED_BLUE, true);
            nrf_gpio_pin_write(CHRG_DONE, true);
            nrf_gpio_pin_write(CHARGING, false);
            PowerOff();
            while(1)
            {
              nrf_gpio_pin_write(LED_BLUE, true);
//                nrf_gpio_pin_write(CHARGING, false);
            }
            break;

        case BSP_EVENT_DISCONNECT:
#ifdef DISABLE_DISCONNECT_BY_BUTTON
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
// martin key test
              err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
              if (err_code != NRF_ERROR_INVALID_STATE)
              {
                  APP_ERROR_CHECK(err_code);
              }

              mySYS->is_imu = 0;
              mySYS->sh_mode = 0;          
              mySYS->sh_mode_old = 0;
              stop_hub_event_poll();
              stop_measure_whrm_wspo2();
              nrf_gpio_pin_write(LED_BLUE, true);
              nrf_gpio_pin_write(CHRG_DONE, true);
              nrf_gpio_pin_write(CHARGING, false);
              PowerOff();
              while(1)
              {
                nrf_gpio_pin_write(LED_BLUE, true);
//                nrf_gpio_pin_write(CHARGING, false);
              }
            }
#else
// martin ; need to check below codes...
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
//            nrf_gpio_pin_write(CHRG_DONE, false);
             //nrf_gpio_pin_write(CHRG_DONE, true);
             mySYS->is_imu = 0;
            sleep_mode_enter();
            //PowerOff();
#endif
            break;

// add pairing action
#ifdef ADD_PAIRING_ACTION                    
        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
#endif
        
        case BSP_EVENT_KEY_0:
#ifdef DISABLE_DISCONNECT_BY_BUTTON
#else
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
// martin key test
              err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
              if (err_code != NRF_ERROR_INVALID_STATE)
              {
                  APP_ERROR_CHECK(err_code);
              }

              mySYS->is_imu = 0;
              mySYS->sh_mode = 0;          
              mySYS->sh_mode_old = 0;
              stop_hub_event_poll();
              stop_measure_whrm_wspo2();
            }
#endif
            break;

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc             = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = false;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, ADV_TX_POWER);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)//(bool erase_bonds)
{
    //if (erase_bonds == true)
    //{
    //    delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
   // }
   // else{
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
   // }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool     erase_bonds;
    int chrg;
    volatile int pwron, pwr_on;
    int i=0;

    // leds off
    nrf_gpio_cfg_output(CHARGING);
    nrf_gpio_cfg_output(CHRG_DONE);
    nrf_gpio_cfg_output(LED_BLUE);
    nrf_gpio_pin_write(CHARGING, true);
    nrf_gpio_pin_write(CHRG_DONE, true);
    nrf_gpio_pin_write(LED_BLUE, true);

// check charger status
//    nrf_gpio_cfg_input(CHRG, NRF_GPIO_PIN_PULLUP);//NRF_GPIO_PIN_PULLUP);
//    nrf_delay_ms(100); // need delay in release mode. why???
//    nrf_gpio_cfg_input(CHRG_POWERON, NRF_GPIO_PIN_PULLUP);//NRF_GPIO_PIN_PULLUP);
//    nrf_delay_ms(100); // need delay in release mode. why???

    twi_init();
    GasGauge_Reset();
    GasGauge_Initialization(&STC3115_ConfigData, &STC3115_BatteryData); //battery gauge
//    GasGauge_Reset();

    while(1)
    {
        // check charger status
        nrf_gpio_cfg_input(CHRG, NRF_GPIO_PIN_PULLUP);//NRF_GPIO_PIN_NOPULL);//NRF_GPIO_PIN_PULLUP);//NRF_GPIO_PIN_PULLUP);
        nrf_delay_ms(100); // need delay in release mode. why???
        nrf_gpio_cfg_input(CHRG_POWERON, NRF_GPIO_PIN_PULLUP);//NRF_GPIO_PIN_NOPULL);//NRF_GPIO_PIN_PULLUP);//NRF_GPIO_PIN_PULLUP);
        nrf_delay_ms(100); // need delay in release mode. why???

        pwron = nrf_gpio_pin_read(CHRG_POWERON);
        nrf_delay_ms(100); // need delay in release mode. why???
        chrg = nrf_gpio_pin_read(CHRG);
        nrf_delay_ms(100); // need delay in release mode. why???

        if(pwron==0&&chrg==1)                  // charge complete
        {
            nrf_gpio_pin_write(CHARGING, true);//false);
            nrf_gpio_pin_write(CHRG_DONE, false);//true);
            nrf_gpio_pin_write(LED_BLUE, true);
            printf("charging complete!!!\r\n");
        }
        else if(pwron==0&&chrg==0)             // charging
        {
            nrf_gpio_pin_write(CHARGING, false);//true);
            nrf_gpio_pin_write(CHRG_DONE, true);//false);
            nrf_gpio_pin_write(LED_BLUE, true);
            printf("charging!!!\r\n");
        }
        else // 11
        {
            nrf_gpio_pin_write(CHARGING, true);
            nrf_gpio_pin_write(CHRG_DONE, true);
            nrf_gpio_pin_write(LED_BLUE, true);
            for(int pwr_cnt=0; pwr_cnt<5; pwr_cnt++)
            {
                GasGauge_Task(&STC3115_ConfigData, &STC3115_BatteryData);
                mySYS->bat_level = (uint8_t)(STC3115_BatteryData.SOC/10);
                printf("power: %d, voltage: %d, current: %d\r\n", mySYS->bat_level, STC3115_BatteryData.Voltage,STC3115_BatteryData.Current);
                nrf_delay_ms(500);
            }
            nrf_gpio_pin_write(LED_BLUE, false);
            //nrf_delay_ms(500);
            break;
        }

        GasGauge_Task(&STC3115_ConfigData, &STC3115_BatteryData);
        mySYS->bat_level = (uint8_t)(STC3115_BatteryData.SOC/10);
        printf("power: %d, voltage: %d, current: %d\r\n", mySYS->bat_level, STC3115_BatteryData.Voltage,STC3115_BatteryData.Current);
        nrf_delay_ms(100); // need delay in release mode. why???
    }
    nrf_gpio_cfg_output(PWR_ON_OFF);      
    nrf_gpio_pin_write(PWR_ON_OFF, true);   //keep Board PWR_ON
//    nrf_delay_ms(500);

    // Initialize.
    log_init();
    uart_init();
    printf("Vital Care F/W Start\r\n");

//    twi_init();
    spi_init();

//    GasGauge_Initialization(&STC3115_ConfigData, &STC3115_BatteryData); //battery gauge
//    GasGauge_Reset();
    for(int i=0; i<2; i++)
    {
        GasGauge_Task(&STC3115_ConfigData, &STC3115_BatteryData);
        mySYS->bat_level = (uint8_t)(STC3115_BatteryData.SOC/10);
        printf("power: %d, voltage: %d, current: %d\r\n", mySYS->bat_level, STC3115_BatteryData.Voltage,STC3115_BatteryData.Current);
        nrf_delay_ms(500);
    }

    if(STC3115_BatteryData.Voltage<3200)//||(mySYS->bat_level==0))
    {
        printf("Low battery power off!!!\r\n");
        nrf_gpio_pin_write(LED_BLUE, true);
        nrf_gpio_pin_write(CHRG_DONE, true);
        nrf_gpio_pin_write(CHARGING, false);
        PowerOff();
        while(1)
        {
          nrf_gpio_pin_write(LED_BLUE, true);
//                nrf_gpio_pin_write(CHARGING, false);
        }
    }

    mlx90632_init();      //temperature sensor
    
    ICM_PowerOn();
    printf("ICM_PowerOn\r\n");

#if 1 // martin add for sensor hub
    sh_hard_reset(0x00);
//    sh_init_hwcomm_interface();
    sh_init_hubinterface();
//    sh_hold_reset();
    nrf_gpio_cfg_output(EN_5V_PIN);
    nrf_gpio_pin_write(EN_5V_PIN, true);   //Turn on GREEN
    printf("MAX32664C PowerOn\r\n");
#endif

    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
// add pairing action
#ifdef ADD_PAIRING_ACTION
    set_static_passkey();
#endif
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();

    conn_params_init();
    peer_manager_init();
    // Start execution.
    NRF_LOG_INFO("Measurement started.");
    //application_timers_start(); 

    //advertising_start(erase_bonds);
    advertising_start();
    app_init();   // 시스텝 변수 초기화
    printf("app_init\r\n");

    // Enter main loop.
    for (;;)
    {
        app_main();  // Vital Care 메인 동작
        idle_state_handle();
    }
}
/**
 * @}
 */
