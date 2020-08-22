/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "sdk_common.h"
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_util_platform.h"
#include "nrf_drv_adc.h"


#define CENTRAL_LINK_COUNT              0                                 /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           0                                 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                 /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(10000, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define DEVICE_NAME											"luXb"

#define APP_BEACON_INFO_LENGTH          0x17                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                              /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xB3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x004C                            /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x00, 0x01                      /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x00, 0x08
/**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x43, 0x79, 0x50, 0x68, \
                                        0x79, 0x4D, 0x65, 0x64, \
                                        0x69, 0x61, 0x42, 0x65, \
                                        0x61, 0x63, 0x6f, 0x6e            /**< Proprietary UUID for Beacon. */
#define LUXB_FW_VERSION									0x00

#define DEAD_BEEF                       0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_TIMER_PRESCALER             0                                 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                 /**< Size of timer operation queues. */

static ble_gap_adv_params_t m_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};

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

static uint32_t timestamp = 0;
static nrf_adc_value_t       adc_buffer[2]; /**< ADC buffer. */
static nrf_drv_adc_channel_t m_channel_1_config = {{{ \
																									.resolution = NRF_ADC_CONFIG_RES_8BIT,               \
																									.input      = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD, \
																									.reference  = NRF_ADC_CONFIG_REF_VBG,                 \
																									.ain        = NRF_ADC_CONFIG_INPUT_5 }} , NULL};

static nrf_drv_adc_channel_t m_channel_2_config = {{{ \
																									.resolution = NRF_ADC_CONFIG_RES_8BIT,               \
																									.input      = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD, \
																									.reference  = NRF_ADC_CONFIG_REF_VBG,                 \
																									.ain        = NRF_ADC_CONFIG_INPUT_6 }} , NULL};

void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
		if (p_event->type == NRF_DRV_ADC_EVT_DONE){
#ifdef DEBUG
		NRF_LOG_INFO("NRF_DRV_ADC_EVT_DONE triggered\r\n");
#endif
		NRF_LOG_INFO("Current SC sample value 1: %d\r\n", (adc_buffer[0]*3.6*1000/256));
		NRF_LOG_INFO("Current BAT sample value 2: %d\r\n", (adc_buffer[1]*3.6*1000/256));
//		adc_buffer[0] = adc_buffer[0]*3.6*1000/1024;
//		adc_buffer[1] = adc_buffer[1]*3.6*1000/1024;
    }else if (p_event->type == NRF_DRV_ADC_EVT_SAMPLE){
#ifdef DEBUG
			NRF_LOG_INFO("NRF_DRV_ADC_EVT_SAMPLE triggered\r\n");
#endif
		}
}																								
/**
 * @brief ADC initialization.
 */
static void adc_config(void)
{
#ifdef DEBUG
		NRF_LOG_INFO("running: adc_config\r\n");
#endif
    ret_code_t ret_code;
    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;

    ret_code = nrf_drv_adc_init(&config, adc_event_handler);
    APP_ERROR_CHECK(ret_code);

    nrf_drv_adc_channel_enable(&m_channel_1_config);
		nrf_drv_adc_channel_enable(&m_channel_2_config);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */

static ble_advdata_t ibeacon_advdata;
static ble_advdata_t luXbeacon_advdata;

static void ibeacon_init(void)
{
    uint32_t      err_code;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&ibeacon_advdata, 0, sizeof(ibeacon_advdata));

    ibeacon_advdata.name_type             = BLE_ADVDATA_NO_NAME;
    ibeacon_advdata.flags                 = flags;
    ibeacon_advdata.p_manuf_specific_data = &manuf_specific_data;
	
    err_code = ble_advdata_set(&ibeacon_advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}

static uint8_t ibeacon_info[6] = 
{
    APP_DEVICE_TYPE,
		LUXB_FW_VERSION,
		APP_MAJOR_VALUE,
		APP_MINOR_VALUE	
};

static void luXbeacon_init(void)
{
#ifdef DEBUG
		NRF_LOG_INFO("running: luXbeacon_init\r\n");
#endif
		uint32_t      err_code;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = 0x3117;
	
		uint8_t luXbeacon_info[12];
		
		for(int i = 0; i < 6 ; i++){
			luXbeacon_info[i] = ibeacon_info[i];
		}
		for(int i = 6; i < 10 ; i++){
			luXbeacon_info[i] = timestamp >> (9-i)*8 & 0xff;
		}
		for(int i = 10; i < 11 ; i++){
			luXbeacon_info[i] = adc_buffer[0] & 0xff;
		}
		for(int i = 11; i < 12 ; i++){
			luXbeacon_info[i] = adc_buffer[1] & 0xff;
		}
		
#ifdef DEBUG		
		NRF_LOG_INFO("luXbeacon_info: %02x\r\n",luXbeacon_info[0]);
		NRF_LOG_INFO("luXbeacon_info: %02x\r\n",luXbeacon_info[1]);
		NRF_LOG_INFO("luXbeacon_info: %02x\r\n",luXbeacon_info[2]);
		NRF_LOG_INFO("luXbeacon_info: %02x\r\n",luXbeacon_info[3]);
		NRF_LOG_INFO("luXbeacon_info: %02x\r\n",luXbeacon_info[4]);
		NRF_LOG_INFO("luXbeacon_info: %02x\r\n",luXbeacon_info[5]);
		NRF_LOG_INFO("luXbeacon_info: %02x\r\n",luXbeacon_info[6]);
		NRF_LOG_INFO("luXbeacon_info: %02x\r\n",luXbeacon_info[7]);
		NRF_LOG_INFO("luXbeacon_info: %02x\r\n",luXbeacon_info[8]);
		NRF_LOG_INFO("luXbeacon_info: %02x\r\n",luXbeacon_info[9]);
		NRF_LOG_INFO("luXbeacon_info: %02x\r\n",luXbeacon_info[10]);
		NRF_LOG_INFO("luXbeacon_info: %02x\r\n",luXbeacon_info[11]);
#endif
		
    manuf_specific_data.data.p_data = (uint8_t *) luXbeacon_info;
    manuf_specific_data.data.size   = 12;
		
    // Build and set advertising data.
    memset(&luXbeacon_advdata, 0, sizeof(luXbeacon_advdata));
		
    luXbeacon_advdata.name_type             = BLE_ADVDATA_FULL_NAME;
    luXbeacon_advdata.flags                 = flags;
    luXbeacon_advdata.p_manuf_specific_data = &manuf_specific_data;

		err_code = ble_advdata_set(&luXbeacon_advdata, NULL);
    APP_ERROR_CHECK(err_code);
    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
#ifdef DEBUG
		NRF_LOG_INFO("running: advertising_start\r\n");
#endif
    uint32_t err_code;
    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

//    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//    APP_ERROR_CHECK(err_code);
}


APP_TIMER_DEF(m_battery_timer_id); 
APP_TIMER_DEF(m_adv_timer_id); 
#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER) 
#define	SWITCH_MEAS_INTERVAL			APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)

int count = 0;

static void adv_timeout_handler(void * p_context)
{   
#ifdef DEBUG
	NRF_LOG_INFO("running: adv_timeout_handler\r\n");
#endif
	count++;
	uint32_t err_code;
	err_code = sd_ble_gap_adv_stop();
	APP_ERROR_CHECK(err_code);
	if(count == 20){
		luXbeacon_init();
		count = 0;
	}else{
		ibeacon_init();
	}
	advertising_start();
}

static void battery_level_meas_timeout_handler(void * p_context){
#ifdef DEBUG
		NRF_LOG_INFO("running: battery_level_meas_timeout_handler\r\n");
#endif
	ret_code_t ret_code = nrf_drv_adc_buffer_convert(adc_buffer,2);
	APP_ERROR_CHECK(ret_code);
	nrf_drv_adc_sample();	
	timestamp++;
}

static void timers_init(void)
{
#ifdef DEBUG
		NRF_LOG_INFO("running: timers_init\r\n");
#endif
    uint32_t err_code;
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&m_adv_timer_id,
																APP_TIMER_MODE_REPEATED,
																adv_timeout_handler);
		APP_ERROR_CHECK(err_code);
}

static void application_timers_start(void)
{
#ifdef DEBUG
		NRF_LOG_INFO("running: application_timers_start\r\n");
#endif
    uint32_t err_code;
	
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(m_adv_timer_id, SWITCH_MEAS_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
#ifdef DEBUG
		NRF_LOG_INFO("running: ble_stack_init\r\n");
#endif
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
}

static void gap_params_init(void)
{
#ifdef DEBUG
		NRF_LOG_INFO("running: gap_params_init\r\n");
#endif
		uint32_t err_code;
		ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
		APP_ERROR_CHECK(err_code);
}

/**@brief Function for doing power management.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    // Initialize.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
		timers_init();
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
		
    //err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
    //APP_ERROR_CHECK(err_code);
    ble_stack_init();
		gap_params_init();
		//ibeacon_init();
		luXbeacon_init();
		adc_config();
		err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE );
		APP_ERROR_CHECK(err_code);
		err_code = sd_ble_gap_tx_power_set(-20);
		APP_ERROR_CHECK(err_code);
	application_timers_start();	
	
		
    // Start execution.
    advertising_start();

    // Enter main loop.
    for (;; )
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
