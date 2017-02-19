
/** 
 * @brief iBeacon Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
//#include <stdbool.h>  
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
//#include "ble_hrs.h"
#include "ble_dis.h"
#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#endif // BLE_DFU_APP_SUPPORT
#include "ble_conn_params.h"
#include "boards.h"
//#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "bsp_btn_ble.h"
#include "pstorage.h"
#include "nrf_gpio.h" 
#ifdef NRF52
 #include "nrf_drv_saadc.h"
 #include "nrf_saadc.h"
 #include "nrf_drv_ppi.h"
 #include "ble_nus.h"
 #include "nrf_drv_timer.h"
#endif
#include "nrf_drv_config.h"

#include "app_util_platform.h"
//#include "nrf_adc.h"
//#include "nrf_drv_config.h"
//#include <stdbool.h>
//#include <stddef.h>

#include "nrf.h"

#define NRF_ADC_CONFIG { NRF_ADC_CONFIG_RES_8BIT,               \
                                 NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD, \
                                 NRF_ADC_CONFIG_REF_VBG }

#define BATTERY_MIN 119
#define BATTERY_MAX 146
//**********************************************************************
#ifdef NRF52																 
  #define SAADC_SAMPLES_IN_BUFFER         1                                        //Number of SAADC samples in RAM before returning a SAADC event. 
																                                                   //For low power SAADC set this constant to 1. 
																                                                   //Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
  #define SAADC_SAMPLE_RATE_DIVIDER				0x2000;
  static ble_nus_t                        m_nus;  															 
  static const nrf_drv_timer_t   m_timer = NRF_DRV_TIMER_INSTANCE(3);
  static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
  static nrf_ppi_channel_t       m_ppi_channel;
  static uint32_t                m_adc_evt_counter;
  static nrf_saadc_value_t       m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
  uint8_t value[SAADC_SAMPLES_IN_BUFFER*2];																 
#endif
//#include "ble_myservice.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define CENTRAL_LINK_COUNT               0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                      "solar"                                     /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 1600                                      /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
//设定发射时间间隔 1600=1s
#define APP_ADV_TIMEOUT_IN_SECONDS       0                                          /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */

//100秒测一个电量

#define MIN_BATTERY_LEVEL                81                                         /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                100                                        /**< Maximum simulated 7battery level. */
#define BATTERY_LEVEL_INCREMENT          1                                          /**< Increment between each simulated battery level measurement. */

//#define HEART_RATE_MEAS_INTERVAL         APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) /**< Heart rate measurement interval (ticks). */
//#define MIN_HEART_RATE                   140                                        /**< Minimum heart rate as returned by the simulated measurement function. */
//#define MAX_HEART_RATE                   300                                        /**< Maximum heart rate as returned by the simulated measurement function. */
//#define HEART_RATE_INCREMENT             10                                         /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

//#define RR_INTERVAL_INTERVAL             APP_TIMER_TICKS(300, APP_TIMER_PRESCALER)  /**< RR interval interval (ticks). */
//#define MIN_RR_INTERVAL                  100                                        /**< Minimum RR interval as returned by the simulated measurement function. */
//#define MAX_RR_INTERVAL                  500                                        /**< Maximum RR interval as returned by the simulated measurement function. */
//#define RR_INTERVAL_INCREMENT            1                                          /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

//#define SENSOR_CONTACT_DETECTED_INTERVAL APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(400, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.4 seconds).400 */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(650, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.65 second).650 */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     6                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#ifdef BLE_DFU_APP_SUPPORT
#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */
#endif // BLE_DFU_APP_SUPPORT

#define VERSION                          0xA1,0x01

/**************************************************************/
/*BEACON定义 20160514*/
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x17                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                              /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x004c                            /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                        /**< Major value used to identify Beacons. */ 
#define APP_MINOR_VALUE                 0x03, 0x04                        /**< Minor value used to identify Beacons. */ 
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */
																				
																				
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

/*BEACON定义结束 20160514     */
/***********************************************************/

static uint8_t data_srv[17]=
{
	0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x00,0x00
};


/**************************************************************/
/*eddystone定义开始 20160628*/

// Eddystone common data
#define APP_EDDYSTONE_UUID              0xFEAA                            /**< UUID for Eddystone beacons according to specification. */
#define APP_EDDYSTONE_RSSI              0xEE                              /**< 0xEE = -18 dB is the approximate signal strength at 0 m. */

// Eddystone UID data
#define APP_EDDYSTONE_UID_FRAME_TYPE    0x00                              /**< UID frame type is fixed at 0x00. */
#define APP_EDDYSTONE_UID_RFU           0x00, 0x00                        /**< Reserved for future use according to specification. */
#define APP_EDDYSTONE_UID_ID            0x01, 0x02, 0x03, 0x04, \
                                        0x05, 0x06                        /**< Mock values for 6-byte Eddystone UID ID instance.  */
#define APP_EDDYSTONE_UID_NAMESPACE     0xAA, 0xAA, 0xBB, 0xBB, \
                                        0xCC, 0xCC, 0xDD, 0xDD, \
                                        0xEE, 0xEE                        /**< Mock values for 10-byte Eddystone UID ID namespace. */

// Eddystone URL data
#define APP_EDDYSTONE_URL_FRAME_TYPE    0x10                              /**< URL Frame type is fixed at 0x10. */
#define APP_EDDYSTONE_URL_SCHEME        0x00                              /**< 0x00 = "http://www" URL prefix scheme according to specification. */
#define APP_EDDYSTONE_URL_URL           0x6e, 0x6f, 0x72, 0x64, \
                                        0x69, 0x63, 0x73, 0x65, \
                                        0x6d,0x69, 0x00                   /**< "nordicsemi.com". Last byte suffix 0x00 = ".com" according to specification. */
// Eddystone TLM data
#define APP_EDDYSTONE_TLM_FRAME_TYPE    0x20                              /**< TLM frame type is fixed at 0x20. */
#define APP_EDDYSTONE_TLM_VERSION       0x00                              /**< TLM version might change in the future to accommodate other data according to specification. */
#define APP_EDDYSTONE_TLM_BATTERY       0x00, 0xF0                        /**< Mock value. Battery voltage in 1 mV per bit. */
#define APP_EDDYSTONE_TLM_TEMPERATURE   0x0F, 0x00                        /**< Mock value. Temperature [C]. Signed 8.8 fixed-point notation. */
#define APP_EDDYSTONE_TLM_ADV_COUNT     0x00, 0x00, 0x00, 0x00            /**< Running count of advertisements of all types since power-up or reboot. */
#define APP_EDDYSTONE_TLM_SEC_COUNT     0x00, 0x00, 0x00, 0x00            /**< Running count in 0.1 s resolution since power-up or reboot. */


static uint8_t eddystone_url_data[] =   /**< Information advertised by the Eddystone URL frame type. */
{
    APP_EDDYSTONE_URL_FRAME_TYPE,   // Eddystone URL frame type.
    APP_EDDYSTONE_RSSI,             // RSSI value at 0 m.
    APP_EDDYSTONE_URL_SCHEME,       // Scheme or prefix for URL ("http", "http://www", etc.)
    APP_EDDYSTONE_URL_URL           // URL with a maximum length of 17 bytes. Last byte is suffix (".com", ".org", etc.)
};

/** @snippet [Eddystone UID data] */
static uint8_t eddystone_uid_data[] =   /**< Information advertised by the Eddystone UID frame type. */
{
    APP_EDDYSTONE_UID_FRAME_TYPE,   // Eddystone UID frame type.
    APP_EDDYSTONE_RSSI,             // RSSI value at 0 m.
    APP_EDDYSTONE_UID_NAMESPACE,    // 10-byte namespace value. Similar to Beacon Major.
    APP_EDDYSTONE_UID_ID,           // 6-byte ID value. Similar to Beacon Minor.
    APP_EDDYSTONE_UID_RFU           // Reserved for future use.
};
/** @snippet [Eddystone UID data] */

static uint8_t eddystone_tlm_data[] =   /**< Information advertised by the Eddystone TLM frame type. */
{
    APP_EDDYSTONE_TLM_FRAME_TYPE,   // Eddystone TLM frame type.
    APP_EDDYSTONE_TLM_VERSION,      // Eddystone TLM version.
    APP_EDDYSTONE_TLM_BATTERY,      // Battery voltage in mV/bit.
    APP_EDDYSTONE_TLM_TEMPERATURE,  // Temperature [C].
    APP_EDDYSTONE_TLM_ADV_COUNT,    // Number of advertisements since power-up or reboot.
    APP_EDDYSTONE_TLM_SEC_COUNT     // Time since power-up or reboot. 0.1 s increments.
};



/*eddystone定义结束 20160628*/
/***********************************************************/

/*
****************************************************************
*/
#ifdef NRF51
  void nrf_adc_init(void){
	

    NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << 0)  //adc?????10?
                    | (2 << 2) //adc???????1/3
                    | (0 << 5) //????1.2?????
                    | (ADC_CONFIG_PSEL_AnalogInput5 << 8);//??AIN5 -p0.04???
    //??adc END??????
//  NRF_ADC->INTENSET = 0x01;

    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled; 

  }
  void nrf_adc_start(){

    NRF_ADC->TASKS_START = 0X01;
  }
#endif
/*
**********************************************************************
*/
	
uint8_t adv_mode=0;           //定义广播模式 0：ibeacon    3：eddystone-uid   4:eddystone-url 
uint8_t adv_mode_flag=0;      //轮播 0：ibeacon<-> eddystone url    1：ibeacon<->eddystone uid
/******20160614*************************************************/
/**/
#define NAME_SIZE 32
uint8_t device_name[NAME_SIZE];
uint8_t uuid_major_minor[20];
uint8_t appkey[16];
uint8_t char7[20];
uint8_t temp[20];
pstorage_handle_t my_name_addr;
pstorage_handle_t block_id;//flash
pstorage_handle_t dest_block_id;
pstorage_handle_t base_handle;
pstorage_handle_t block_handle1;     //devicename
pstorage_handle_t block_handle2;     //uuid_major_minor
pstorage_handle_t block_handle3;     //appkey
pstorage_handle_t block_handle4;     //char7
pstorage_handle_t block_handle5;     //char7
pstorage_handle_t block_handle6;     //char7
pstorage_handle_t block_handle7;     //char7

int adv_interval;

static void my_cb(pstorage_handle_t * handle,uint8_t op_code,uint32_t result,uint8_t * p_data,uint32_t data_len)
{
	switch(op_code)
	{
		case PSTORAGE_UPDATE_OP_CODE:
			if(result==NRF_SUCCESS)
			{
				//
			}
			break;
	}
}

void beacon_flash_init(void)
{
	uint32_t err_code;
	pstorage_module_param_t param;
	param.block_count=10;
	param.block_size=200;
	param.cb=my_cb;
	err_code=pstorage_init();
	err_code=pstorage_register(&param,&base_handle);
	//APP_ERROR_HANDLER(err_code);
	
	err_code=pstorage_block_identifier_get(&base_handle,1,&block_handle1);
	err_code=pstorage_load(device_name,&block_handle1,NAME_SIZE,0);
	
	err_code=pstorage_block_identifier_get(&base_handle,2,&block_handle2);
	err_code=pstorage_load(uuid_major_minor,&block_handle2,NAME_SIZE,0);
		
	err_code=pstorage_block_identifier_get(&base_handle,3,&block_handle3);
	err_code=pstorage_load(char7,&block_handle3,NAME_SIZE,0);
	APP_ERROR_CHECK(err_code);
	//err_code=pstorage_block_identifier_get(&base_handle,3,&block_handle3);
	//err_code=pstorage_load(temp,&block_handle3,NAME_SIZE,0);	
	//err_code=pstorage_block_identifier_get(&base_handle,4,&block_handle4);
	//err_code=pstorage_load(temp,&block_handle4,NAME_SIZE,0);	
	//err_code=pstorage_block_identifier_get(&base_handle,2,&block_handle2);
	//err_code=pstorage_load(uuid_major_minor,&block_handle2,NAME_SIZE,0);	
	//err_code=pstorage_block_identifier_get(&base_handle,6,&block_handle6);
	//err_code=pstorage_load(appkey,&block_handle6,NAME_SIZE,0);	
	//err_code=pstorage_block_identifier_get(&base_handle,7,&block_handle7);
	//err_code=pstorage_load(char7,&block_handle7,NAME_SIZE,0);

}

static void name_change(ble_evt_t * p_ble_evt)
{	
	 ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;  
	 
	 if(p_ble_evt->header.evt_id == BLE_GATTS_EVT_WRITE)
	 {
		 switch (p_evt_write->uuid.uuid)
		 {
			 case BLE_UUID_GAP_CHARACTERISTIC_DEVICE_NAME:
				 device_name[0] = 0xaa;
         device_name[1] = p_evt_write->len;
         memcpy(device_name+2, p_evt_write->data, p_evt_write->len);		
         pstorage_update(&block_handle1, device_name, NAME_SIZE, 0 );			
			 break; 
       case 0x8681:           		  
       break;
       case 0x8682:
           //sd_ble_gap_tx_power_set(4);
       break ;
			 case 0x8683:
          //memcpy(uuid_major_minor, p_evt_write->data, p_evt_write->len);			
          //pstorage_update(&block_handle2, uuid_major_minor, NAME_SIZE, 0);
			    //sd_ble_gap_tx_power_set(-40);
       break ;
			 case 0x8684:           
       break ;
			 case 0x8685:          //写入uuid，major，minor
					memcpy(uuid_major_minor, p_evt_write->data, p_evt_write->len);			
          pstorage_update(&block_handle2, uuid_major_minor, NAME_SIZE, 0 );          		   
       break;
			 
			 
			 case 0x8686:
					//memcpy(uuid_major_minor, p_evt_write->data, p_evt_write->len);			
          //pstorage_update(&block_handle6, appkey, NAME_SIZE, 0 );
           
       break ;
			 
			  case 0x8687:
					memcpy(char7, p_evt_write->data, p_evt_write->len);			
          pstorage_update(&block_handle3, char7, NAME_SIZE, 0 );
           
       break ;
			 
				
       default:
            // No implementation needed.
       break;
		 }	 
		}
}


/**/
/*********************************************************/

static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_bas_t                         m_bas;                                     /**< Structure used to identify the battery service. */
//static ble_hrs_t                         m_hrs;                                     /**< Structure used to identify the heart rate service. */
//static bool                              m_rr_interval_enabled = true;              /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

//static sensorsim_cfg_t                   m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
//static sensorsim_state_t                 m_battery_sim_state;                       /**< Battery Level sensor simulator state. */
//static sensorsim_cfg_t                   m_heart_rate_sim_cfg;                      /**< Heart Rate sensor simulator configuration. */
//static sensorsim_state_t                 m_heart_rate_sim_state;                    /**< Heart Rate sensor simulator state. */
//static sensorsim_cfg_t                   m_rr_interval_sim_cfg;                     /**< RR Interval sensor simulator configuration. */
//static sensorsim_state_t                 m_rr_interval_sim_state;                   /**< RR Interval sensor simulator state. */


APP_TIMER_DEF(m_adv_timer_id); 
APP_TIMER_DEF(m_battery_timer_id);                                                    /**< Battery timer. */
//APP_TIMER_DEF(m_heart_rate_timer_id);                                               /**< Heart rate measurement timer. */
//APP_TIMER_DEF(m_rr_interval_timer_id);                                              /**< RR interval timer. */                 /**< RR interval timer. */
//APP_TIMER_DEF(m_sensor_contact_timer_id);                                           /**< Sensor contact detected timer. */

static dm_application_instance_t         m_app_handle;                                /**< Application identifier allocated by device manager */


//static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE},
//                                   {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
//                                   {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */
#ifdef BLE_DFU_APP_SUPPORT
static ble_dfu_t                         m_dfus;                                    /**< Structure used to identify the DFU service. */
#endif // BLE_DFU_APP_SUPPORT


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
uint16_t adc_value=0x0400;
uint8_t  battery_level;
static uint8_t battery_levelx[2];
float valuex;
uint8_t jsq=0;
uint16_t temp1;
static void battery_level_update(void)
{
    uint32_t err_code;
    //uint8_t  battery_level;
    //battery_level = (uint8_t)(&m_battery_sim_state, &m_battery_sim_cfg);
    //battery_level=((float)adc_value-582)/341*100;          //这个值还需要修正;
	  #ifdef NRF52
	    temp1=adc_value;
	    temp1 = temp1*3.6/1024*1000;      //电压值，单位mv
	    battery_level=battery_level_in_percent(temp1);   //获取电量百分比		
      	
   	  //battery_level=(((float)adc_value*1.2f)-580)*3/1.3f*100;  //这个值还需要修正;
	  #endif
	  #ifdef NRF51
	    nrf_adc_init();
	    nrf_adc_start();
	    while(NRF_ADC->BUSY&1)
			valuex=(NRF_ADC->RESULT)*1.0; 
	    temp1=valuex;		  
		  battery_levelx[0]  = (temp1>>8)&0xff;
      battery_levelx[1]  = (temp1>>0)&0xff;
			temp1 = temp1*3.6/1024*1000;
			jsq=jsq+1;
      battery_level=battery_level_in_percent(temp1);   //获取电量百分比			
		#endif
		if(adv_mode==6)
		{			
			if (temp1>=3000)
			{
				if(char7[4]!=0xff)
				{
				  adv_interval=char7[4]*160;
				}
				else
				{
		      adv_interval=1600;
				}
			}
			
			if (temp1<3000 && temp1>=2800)
			{
		    adv_interval=4800;  //2秒
			}
			
			if (temp1<2800)
			{
		    adv_interval=9600;  //4秒
			}
			
		}
    err_code = ble_bas_battery_level_update(&m_bas, battery_level);  //将电量传送给电池服务
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
}



/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
//static void heart_rate_meas_timeout_handler(void * p_context)
//{
//    static uint32_t cnt = 0;
//    uint32_t        err_code;
//    uint16_t        heart_rate;

//    UNUSED_PARAMETER(p_context);

//    heart_rate = (uint16_t)sensorsim_measure(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

//    cnt++;
//    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
//    if ((err_code != NRF_SUCCESS) &&
//        (err_code != NRF_ERROR_INVALID_STATE) &&
//        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
//        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//        )
//    {
//        APP_ERROR_HANDLER(err_code);
//    }

//    // Disable RR Interval recording every third heart rate measurement.
//    // NOTE: An application will normally not do this. It is done here just for testing generation
//    //       of messages without RR Interval measurements.
//    m_rr_interval_enabled = ((cnt % 3) != 0);
//}


/**@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
//static void rr_interval_timeout_handler(void * p_context)
//{
//    UNUSED_PARAMETER(p_context);

//    if (m_rr_interval_enabled)
//    {
//        uint16_t rr_interval;

//        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
//                                                      &m_rr_interval_sim_cfg);
//        //ble_hrs_rr_interval_add(&m_hrs, rr_interval);
//    }
//}


/**@brief Function for handling the Sensor Contact Detected timer timeout.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
//static void sensor_contact_detected_timeout_handler(void * p_context)
//{
//    static bool sensor_contact_detected = false;

//    UNUSED_PARAMETER(p_context);

//    sensor_contact_detected = !sensor_contact_detected;
//    //ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
//}





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

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	
	  /**/
	  /**************************************/
	  if(device_name[0]==0xaa)
		{
			 err_code = sd_ble_gap_device_name_set(&sec_mode,
                                    (const uint8_t *) device_name+2,
                                       device_name[1]);

		}else
		{
			
			 err_code = sd_ble_gap_device_name_set(&sec_mode,
                                       (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
		}
	
	  /***************************************/
	  /**/
	   
	
	

    APP_ERROR_CHECK(err_code);

    //err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    //APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


#ifdef BLE_DFU_APP_SUPPORT
/**@brief Function for stopping advertising.
 */
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for loading application-specific context after establishing a secure connection.
 *
 * @details This function will load the application context and check if the ATT table is marked as
 *          changed. If the ATT table is marked as changed, a Service Changed Indication
 *          is sent to the peer if the Service Changed CCCD is set to indicate.
 *
 * @param[in] p_handle The Device Manager handle that identifies the connection for which the context
 *                     should be loaded.
 */
static void app_context_load(dm_handle_t const * p_handle)
{
    uint32_t                 err_code;
    static uint32_t          context_data;
    dm_application_context_t context;

    context.len    = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
            err_code = sd_ble_gatts_service_changed(m_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_PACKETS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */
static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(500);
}
/** @snippet [DFU BLE Reset prepare] */
#endif // BLE_DFU_APP_SUPPORT

/************************************ 
my service 20160516
增加自定义服务特征值
***********************************/
typedef struct MyServiceTag{
    uint16_t conn_handle; // ???? ??????,???????
    uint16_t service_handle; // ???????
    ble_gatts_char_handles_t handle; //??????
}MyService;


MyService my_service;

static void MyChar(int16_t myuuid,uint8_t characteristic_value[],int8_t len,int r_flag,int w_flag){
	
	  ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_md_t attr_md;

	
	  memset(&cccd_md, 0, sizeof(cccd_md));
	  memset(&char_md, 0, sizeof(char_md)); 
	  memset(&attr_md, 0, sizeof(attr_md)); 
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md = &cccd_md;
    char_md.char_props.notify = 0;
    char_md.char_props.write = w_flag;
		char_md.char_props.read = r_flag;
    char_md.p_char_pf = NULL;
    char_md.p_char_user_desc = NULL;
    char_md.p_sccd_md = NULL;
    char_md.p_user_desc_md = NULL;
 
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.vlen = 1;

    ble_uuid_t val_uuid;
    val_uuid.type = BLE_UUID_TYPE_BLE;
    val_uuid.uuid = myuuid;

    attr_char_value.p_uuid = &val_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len=len;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = 20;
   
		// attr_char_value.init_len=sizeof();      //特征值长度
   
		//attr.init_offs = 0;
    attr_char_value.p_value =characteristic_value;             //特征值赋值
		
		//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    sd_ble_gatts_characteristic_add(my_service.service_handle, &char_md,&attr_char_value,&my_service.handle);
	
}


/******************************************
**********************************************/


///////////////////////////////////////////////////


void service_write_handle(ble_evt_t * p_ble_evt){
    //uint16_t data_len;
    uint8_t data;
    //data_len = sizeof(uint8_t);
    data = p_ble_evt ->evt.gatts_evt.params.write.data[0]; 
	  
	  data++;
	 
    //ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;  
    //判断哪个特征uuid被改写了，并获取值    20160526
    //if(p_ble_evt->header.evt_id == BLE_GATTS_EVT_WRITE)
    //{ 
    //}
}

void my_server_handler(ble_evt_t *p_ble_evt){
    switch (p_ble_evt ->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            my_service.conn_handle =p_ble_evt ->evt.gap_evt.conn_handle;
        break;
        case BLE_GATTS_EVT_WRITE:
					 //flash_page_erase(addr-1);
           service_write_handle(p_ble_evt);							
        break ;
        default:
            // No implementation needed.
        break;
    }
}

///////////////////////////////////////////////////

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{		
    uint32_t       err_code;
    //ble_hrs_init_t hrs_init;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;
    //uint8_t        body_sensor_location;

    //Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    //Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
		
		
		
	/***********************************************
	                 自定义服务
	******************************************/
	
	  //ble_uuid_t service_uuid;
		 //uint8_t y[]={0xff};
		 ble_uuid_t service_uuid;
		 service_uuid.type = BLE_UUID_TYPE_BLE;
	//***********************************************************	  
    service_uuid.type = BLE_UUID_TYPE_BLE;
    service_uuid.uuid = 0x8680;
    sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,&service_uuid,&my_service.service_handle);
	 
   /***********************************************
	  特征值1~8
	  ***********************************************/

	  uint8_t v[]={0xff,0x02,0x02,0x02,0x04,0x09,0x07,0x04,0x02};
		uint8_t t3[]={0x03};
		uint8_t t8[]={0x43,0x2b,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
		uint8_t t9[]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
		MyChar(0x8681,t3,20,1,1);
		MyChar(0x8682,t3,20,1,1);
		MyChar(0x8683,v,20,1,1);
		MyChar(0x8684,v,20,1,1);
		MyChar(0x8685,uuid_major_minor,20,1,1);
		MyChar(0x8686,t9,16,1,1);
		MyChar(0x8687,char7,6,1,1);
		MyChar(0x8688,t8,19,1,1);
		
	/*********************************************
	***********************************************/
		

#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [DFU BLE Service initialization] */
    ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);
    /** @snippet [DFU BLE Service initialization] */
#endif // BLE_DFU_APP_SUPPORT
}






/**@brief Function for initializing the sensor simulators.
 */
//static void sensor_simulator_init(void)
//{
	
	  
//    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
//    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
//    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
//    m_battery_sim_cfg.start_at_max = true;

//    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

////    m_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
////    m_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
////    m_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
////    m_heart_rate_sim_cfg.start_at_max = false;

////    sensorsim_init(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

//    m_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
//    m_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
//    m_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
//    m_rr_interval_sim_cfg.start_at_max = false;

//    sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);
//}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
	
	  err_code = app_timer_start(m_adv_timer_id, adv_interval, NULL);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
	  //err_code = app_timer_start(m_battery_timer_id, adv_interval, NULL);
    APP_ERROR_CHECK(err_code);

    //err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
    //APP_ERROR_CHECK(err_code);

    //err_code = app_timer_start(m_rr_interval_timer_id, RR_INTERVAL_INTERVAL, NULL);
    //APP_ERROR_CHECK(err_code);

    //err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
    //APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
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
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    //cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
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
   // uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);   //广播闪灯
            //APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:					
				    //flash_word_write(addr-1, (uint32_t)0x45);
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);   //连接亮灯
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
				    //flash_wr(0x88888888,0x01);
				    // flash_page_erase(addr);
            //patwr=0x34;
            //	   	        
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
				    //advertising_init();
				    //flash_wr(0x77777777,0x01);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	  name_change(p_ble_evt);
	  if ( p_ble_evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED )
			 {      
	         NVIC_SystemReset();
       }

	  
    dm_ble_evt_handler(p_ble_evt);
    //ble_hrs_on_ble_evt(&m_hrs, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [Propagating BLE Stack events to DFU Service] */
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    /** @snippet [Propagating BLE Stack events to DFU Service] */
#endif // BLE_DFU_APP_SUPPORT
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
	
	  my_server_handler(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
	
	  //my_server_handler(p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.	
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

#ifdef BLE_DFU_APP_SUPPORT
    ble_enable_params.gatts_enable_params.service_changed = 1;
#endif // BLE_DFU_APP_SUPPORT
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}






static ble_gap_adv_params_t m_adv_params;  
ble_gap_addr_t  mac_addr;
/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init()
{
    uint32_t      err_code;
    ble_advdata_t advdata;
	  uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    ble_advdata_manuf_data_t manuf_specific_data;	
	  ble_uuid_t    adv_uuids[] = {{APP_EDDYSTONE_UUID, BLE_UUID_TYPE_BLE}};
		
		
    uint8_array_t eddystone_data_array;                             // Array for Service Data structure.
/** @snippet [Eddystone data array] */
    eddystone_data_array.p_data = (uint8_t *) eddystone_url_data;   // Pointer to the data to advertise.
    eddystone_data_array.size = sizeof(eddystone_url_data);         // Size of the data to advertise.
/** @snippet [Eddystone data array] */
    ble_advdata_service_data_t service_data;                        // Structure to hold Service Data.
    service_data.service_uuid = APP_EDDYSTONE_UUID;                 // Eddystone UUID to allow discoverability on iOS devices.
    service_data.data = eddystone_data_array;                       // Array for service advertisement data.
	
		
		if (adv_mode==0||adv_mode==1||adv_mode==2||adv_mode==6)
			{	  
					manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;  
					manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;  
					manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;
					memset(&advdata, 0, sizeof(advdata));
					advdata.name_type             = BLE_ADVDATA_NO_NAME;
					advdata.flags                 = flags;
					advdata.p_manuf_specific_data = &manuf_specific_data;
			}
			
			
		if(adv_mode==3)
			{
					manuf_specific_data.data.p_data = (uint8_t *) eddystone_url_data;;
					manuf_specific_data.data.size   = sizeof(eddystone_url_data);
					memset(&advdata, 0, sizeof(advdata));
					advdata.name_type               = BLE_ADVDATA_NO_NAME;
					advdata.flags                   = flags;
					advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
					advdata.uuids_complete.p_uuids  = adv_uuids;
					advdata.p_service_data_array    = &service_data;                // Pointer to Service Data structure.
					advdata.service_data_count      = 1;				 
			}
					
			if(adv_mode==4)
			{
				manuf_specific_data.data.p_data = (uint8_t *) eddystone_uid_data;;
				manuf_specific_data.data.size   = sizeof(eddystone_uid_data);
				memset(&advdata, 0, sizeof(advdata));
				advdata.name_type               = BLE_ADVDATA_NO_NAME;
				advdata.flags                   = flags;
				advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
				advdata.uuids_complete.p_uuids  = adv_uuids;
				advdata.p_service_data_array    = &service_data;                // Pointer to Service Data structure.
				advdata.service_data_count      = 1;				 
				
			}
		
    //Build advertising data struct to pass into @ref ble_advertising_init.	
    //advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    //advdata.include_appearance      = true;
    //advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    //advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    //advdata.uuids_complete.p_uuids  = m_adv_uuids;	
    //uint32_t err_code;
    //uint8_t  battery_level;
 
		
		
	//scanrsp 定义 20160514
	  ble_advdata_t scanrsp;
    ble_advdata_service_data_t  srv_data;
    uint8_array_t               data_array;
   			
    data_array.p_data = (uint8_t *)data_srv;          //why is (uint8_t *)？ 20160615
    data_array.size = sizeof(data_srv);
    srv_data.service_uuid = BLE_UUID_DEVICE_INFORMATION_SERVICE;
    srv_data.data = data_array;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.p_service_data_array    = &srv_data;
    scanrsp.service_data_count      = 1;
	  scanrsp.name_type               = BLE_ADVDATA_FULL_NAME;
		
    //scanrsp 定义结束  20160514

    // err_code = ble_advdata_set(&advdata, &scanrsp);
    //APP_ERROR_CHECK(err_code);
		
    //*******************2016.6.20
        
    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));
		
    // m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
	  //BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    //m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    // m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    //m_adv_params.timeout     = 0;
    
		//********************2016.6.20				
	  			
	  
    
    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = adv_interval;//APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


uint8_t tx_power;

static void my_value_init(void)
{
	
	int8_t i;
	 for(i=0;i<=19;i++)  
		{
	     m_beacon_info[i+2]=uuid_major_minor[i];
	  }
		m_beacon_info[22]=char7[2];
		
		if(char7[4]!=0xff)
		{
		adv_interval=char7[4]*160;
		//adv_interval=APP_ADV_INTERVAL;	
		adv_mode=char7[5];
		
		}
		else
		{
		adv_interval=APP_ADV_INTERVAL;
		adv_mode=0x00;
		}
		
		if(char7[5]!=0xff)
		{
		//adv_interval=char7[4]*160;
		adv_mode=char7[5];
		}
		else
		{
		//adv_interval=APP_ADV_INTERVAL;
		adv_mode=0x00;
		}
		//adv_mode=0x00;
		tx_power=char7[3];
		//APP_ADV_INTERVAL=800;	
		
		//ble_gap_addr_t  mac_addr;
    sd_ble_gap_address_get(&mac_addr);
		
		//sensor_simulator_init();
    //battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
		//const uint8_t address1[] = {0xe7,0xAA,0xAA,0xAA,0xAA,0xAA}; 
		//ble.setAddress(Gap::ADDR_TYPE_PUBLIC, address1);
		
		
		//*******************
		//****修改MacAddress
		//mac_addr.addr[1]=0xff;		
		//sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &mac_addr);

    //********************
		
		for(i=0;i<6;i++)
		{
		  data_srv[i]=mac_addr.addr[i];
		}
		
		for(i=6;i<10;i++)
		{
		  data_srv[i]=uuid_major_minor[i+10];
		}
		
		data_srv[10]=battery_level;
		data_srv[11]=jsq;
		data_srv[12]=0xA1;
		data_srv[13]=0x01;
		data_srv[16]=adv_mode;
		//data_srv[16]=0x0a;
		
		
		
		switch(adv_mode)
		{
			case 0:
			case 3:
			case 4:
			case 6:
				adv_mode_flag=0x00;
			break;
			case 1:
				adv_mode_flag=0x01;
			  adv_mode=0x00;
			break;
			case 2:
				adv_mode_flag=0x02;
			  adv_mode=0x00;
			break;
			case 5:
				adv_mode_flag=3;
			break;
		  default:
				adv_mode_flag=0;
			break;		
		}


}

#define minus40dbm   0x50
#define minus30dbm   0x40
#define minus20dbm   0x30
#define minus16dbm   0x20
#define minus12dbm   0x10
#define minus8dbm    0x00
#define minus4dbm    0x01
#define plus0dbm     0x02
#define plus4dbm     0x03

static void TX_POWER(uint8_t tx)
{
  switch(tx)
	{
		case minus40dbm:
	    sd_ble_gap_tx_power_set(-40);
		break;
		case minus30dbm:
	    sd_ble_gap_tx_power_set(-30);
		break;
		case minus20dbm:
	    sd_ble_gap_tx_power_set(-20);
		break;
		case minus16dbm:
	    sd_ble_gap_tx_power_set(-16);
		break;
		case minus12dbm:
	    sd_ble_gap_tx_power_set(-12);
		break;
		case minus8dbm:
	    sd_ble_gap_tx_power_set(-8);
		break;
		case minus4dbm:
	    sd_ble_gap_tx_power_set(-4);
		break;
		case plus0dbm:
			sd_ble_gap_tx_power_set(0);
		break;
		case plus4dbm:
			sd_ble_gap_tx_power_set(4);
		break;
		default:
			sd_ble_gap_tx_power_set(0);
    break;
	}


}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 * 电量更新
 */
//uint16_t adc_value=0x1234;
//uint8_t value[SAADC_SAMPLES_IN_BUFFER*2];


static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
		sd_ble_gap_adv_stop();   

	  data_srv[10]=battery_level;
	  data_srv[11]=jsq;
	    //uint8_t per=((float)adc_value-482)/341*100;  //这个值还需要修正
	 #ifdef  NRF52
	  data_srv[14]=value[1];
	  data_srv[15]=value[0];
	 #endif
	 #ifdef  NRF51
	  data_srv[14]=battery_levelx[0];
	  data_srv[15]=battery_levelx[1];
	 #endif
	
	    //data_srv[10]=per; 
  	advertising_init();      //????????????? 
  	ble_advertising_start(BLE_ADV_MODE_FAST);
	
}

/*
轮播模式使用
*/
static void adv_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	if(adv_mode_flag==0x01)
	{		
   	sd_ble_gap_adv_stop();   //??? 
	  switch (adv_mode)
		{
			case 0x00:
				adv_mode=0x04;
			break;
			case 0x04:
				adv_mode=0x00;
			break;		
		}
	  advertising_init();      //????????????? 
	  ble_advertising_start(BLE_ADV_MODE_FAST);
	}
		
		if(adv_mode_flag==0x02)
	{		
   	sd_ble_gap_adv_stop();   //??? 
	  switch (adv_mode)
		{
			case 0x00:
				adv_mode=0x03;
			break;
			case 0x03:
				adv_mode=0x00;
			break;		
		}
	  advertising_init();      //????????????? 
	  ble_advertising_start(BLE_ADV_MODE_FAST);
	}

	}

 /**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
	  err_code = app_timer_create(&m_adv_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                adv_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);

    //err_code = app_timer_create(&m_heart_rate_timer_id,
    //                            APP_TIMER_MODE_REPEATED,
    //                            heart_rate_meas_timeout_handler);
    //APP_ERROR_CHECK(err_code);

    //err_code = app_timer_create(&m_rr_interval_timer_id,
     //                           APP_TIMER_MODE_REPEATED,
     //                           rr_interval_timeout_handler);
    //APP_ERROR_CHECK(err_code);

    //err_code = app_timer_create(&m_sensor_contact_timer_id,
    //                            APP_TIMER_MODE_REPEATED,
    //                           sensor_contact_detected_timeout_handler);
    //APP_ERROR_CHECK(err_code);
}


#ifdef NRF52
void timer_handler(nrf_timer_event_t event_type, void* p_context)
{

}

void saadc_sampling_event_init(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_timer_init(&m_timer, NULL, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event */
    uint32_t ticks = SAADC_SAMPLE_RATE_DIVIDER;
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_event_addr = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_event_addr);
    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}



void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
				//uint16_t adc_value;
				//uint8_t value[SAADC_SAMPLES_IN_BUFFER*2];
				uint8_t bytes_to_send;
     
				// set buffers
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
						
				//print samples on hardware UART and parse data for BLE transmission
        //printf("ADC event number: %d\r\n",(int)m_adc_evt_counter);
        for (int i = 0; i < SAADC_SAMPLES_IN_BUFFER; i++)
        {
            //printf("%d\r\n", p_event->data.done.p_buffer[i]);

						adc_value = p_event->data.done.p_buffer[i];
						value[i*2] = adc_value;
						value[(i*2)+1] = adc_value >> 8;
				}
				
				// Send data over BLE via NUS service. Makes sure not to send more than 20 bytes.
				if((SAADC_SAMPLES_IN_BUFFER*2) <= 20) 
				{
						bytes_to_send = (SAADC_SAMPLES_IN_BUFFER*2);
				}
				else 
				{
						bytes_to_send = 20;
				}
				err_code = ble_nus_string_send(&m_nus, value, bytes_to_send);
				if (err_code != NRF_ERROR_INVALID_STATE) 
				{
						APP_ERROR_CHECK(err_code);
				}
						
        m_adc_evt_counter++;
    }
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
             NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAADC_SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

#endif

///**@brief Function for application main entry.
// *///
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
	  //int8_t i;
	  beacon_flash_init();
	   
    app_trace_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    device_manager_init(erase_bonds);
    gap_params_init();
	
	  	#ifdef NRF51
	  battery_level_update();
	#endif 
	  my_value_init();
    	
    services_init();
    //sensor_simulator_init();
    conn_params_init();

    //APP_ERROR_CHECK(err_code);
		advertising_init();
		#ifdef NRF52
	  saadc_sampling_event_init();
    saadc_init();
    saadc_sampling_event_enable();
		#endif
	 
    TX_POWER(tx_power);
    // Start execution.
    application_timers_start();			
	  //sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);  //kkkkkkkkkkkkkkkkkkkkkkkkkk
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
		//advertising_start();
    APP_ERROR_CHECK(err_code);
		
    //sd_ble_gap_tx_power_set(-8);
		
    // Enter main loop.
    for (;;)
    {
        power_manage();
    }
}
