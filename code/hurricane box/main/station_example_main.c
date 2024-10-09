#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
//#include "souravconfig.h"  
#include "./ADXL343.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>             
#include <inttypes.h>  
#include "freertos/queue.h"  
#include "driver/gptimer.h"     
#include "driver/gpio.h"        
#include "driver/ledc.h"    
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <string.h>
#include <math.h>

#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"

static const char *TAG = "UDP";

#define WIFI_SSID "Group_5"
#define WIFI_PASS "smartsys"

#define UDP_SERVER_IP "192.168.1.35"
#define UDP_SERVER_PORT 3333

#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

// Hardware interrupt definitions
//#define GPIO_INPUT_IO_1       4
#define ESP_INTR_FLAG_DEFAULT 0
//#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1 // casting GPIO input to bitmap

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

#define GPIO_LED 33
static uint8_t ledState = 0;

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       false // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
// #define NACK_VAL                           0x01 // i2c nack value (Was FF)
#define NACK_VAL                           0x01 // i2c nack value (Was FF)

// ADXL343
#define SLAVE_ADDR                         ADXL343_ADDRESS // 0x53

static esp_adc_cal_characteristics_t *adc_chars_temp;
static esp_adc_cal_characteristics_t *adc_chars_volt;
#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t tempChannel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t volt_chanel = ADC_CHANNEL_3; 
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t tempChannel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t volt_chanel = ADC_CHANNEL_3 
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

float print_voltage;
float tempC;
int seconds = 0;
float roll;
float pitch;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;


static void i2c_master_init(){
  // Debug
  printf("\n>> i2c Config\n");
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  conf.clk_flags = 0;
  err = i2c_param_config(i2c_master_port, &conf);           // Configure
  if (err == ESP_OK) {printf("- parameters: ok\n");}

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK) {printf("- initialized: yes\n");}

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
  int32_t scanTimeout = 1000;
  printf("\n>> I2C scanning ..."  "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    // printf("0x%X%s",i,"\n");
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {printf("- No I2C devices found!" "\n");}
}

////////////////////////////////////////////////////////////////////////////////

// ADXL343 Functions ///////////////////////////////////////////////////////////

// Get Device ID
int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
  // YOUR CODE HERE
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Read register
uint8_t readRegister(uint8_t reg) {
  // YOUR CODE HERE
    uint8_t value = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &value, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return value;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {

    uint8_t low, high;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &low, ACK_VAL);
    i2c_master_read_byte(cmd, &high, NACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    return (int16_t)(high << 8 | low);
}

void setRange(range_t range) {
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL343_REG_DATA_FORMAT, format);

}

range_t getRange(void) {
  /* Red the data format register to preserve bits */
  return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void) {
  return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
}

////////////////////////////////////////////////////////////////////////////////

// function to get acceleration
void getAccel(float * xp, float *yp, float *zp) {
  *xp = read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  //printf("here\n");
  *yp = read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  //printf("here_1\n");
  *zp = read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  //printf("here_2\n");
 //printf("X: %.2f \t Y: %.2f \t Z: %.2f\n", *xp, *yp, *zp);
}

// function to print roll and pitch
void calcRP(float x, float y, float z){
  // YOUR CODE HERE
  roll = atan2(y, z) * 180 / M_PI;
  pitch = atan2((- x), sqrt(y * y + z * z)) * 180 / M_PI;
  printf("roll: %.2f \t pitch: %.2f \n", roll, pitch);
}


static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        //printf("eFuse Two Point: Supported\n");
    } else {
        //printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        //printf("eFuse Vref: Supported\n");
    } else {
        //printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}




// Global flag
bool flag = false;

// A simple structure for queue elements
typedef struct {
    uint64_t event_count;
} example_queue_element_t;

// Create a queue (FIFO) for timer-based events
example_queue_element_t ele;
QueueHandle_t timer_queue;
     

// Timer interrupt handler -- callback timer function -- from GPTimer guide example
static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t timer_queue1 = (QueueHandle_t)user_data;    // represents state info passed to callback, if needed
    example_queue_element_t ele = {
          .event_count = edata->count_value                   // Retrieve count value and send to queue
      };
    xQueueSendFromISR(timer_queue1, &ele, &high_task_awoken); // Puts data into queue and allerts other recipients
    return (high_task_awoken == pdTRUE);
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        //printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        //printf("Characterized using eFuse Vref\n");
    } else {
        //printf("Characterized using Default Vref\n");
    }
}


// Timer configuration -- from GPTimer guide example
static void alarm_init() {
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer)); // instantiates timer
  
    gptimer_event_callbacks_t cbs = { // Set alarm callback
      .on_alarm = timer_on_alarm_cb,  // This is a specific supported callback from callbacks list
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, timer_queue)); // This registers the callback
    ESP_ERROR_CHECK(gptimer_enable(gptimer));                                      // Enables timer interrupt ISR

    //ESP_LOGI(TAG_TIMER, "Start timer, update alarm value dynamically and auto reload"); 
    gptimer_alarm_config_t alarm_config = { // Configure the alarm 
      .reload_count = 0,                    // counter will reload with 0 on alarm event
      .alarm_count = 1*1000000,            // period = 1*1s = 1s
      .flags.auto_reload_on_alarm = true,   // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));  // this enacts the alarm config
    ESP_ERROR_CHECK(gptimer_start(gptimer));                            // this starts the timer
}

// Timer task -- what to do when the timer alarm triggers 
static void timer_evt_task(void *arg) {  //This function sets flags for other routines in the code such as the walk task
  while (1) {
    // Transfer from queue and do something if triggered
    if (xQueueReceive(timer_queue, &ele, pdMS_TO_TICKS(2000))) {
      flag = true;           // Set a flag to be used elsewhere 
      seconds++;
      //printf("Time to next step: %d seconds\n", seconds);
    }
  }
}


void adc_init() {
  //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) { //initialize all the channels for the sensors
        adc1_config_width(width);
        adc1_config_channel_atten(tempChannel, atten);
        adc1_config_channel_atten(volt_chanel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)tempChannel, atten); //Set the attenuation
        adc2_config_channel_atten((adc2_channel_t)volt_chanel, atten);
    }

    //Characterize ADC
    adc_chars_temp = calloc(1, sizeof(esp_adc_cal_characteristics_t));  //Set the characteristics
    adc_chars_volt = calloc(1, sizeof(esp_adc_cal_characteristics_t));

    esp_adc_cal_value_t val_type_temp = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars_temp);
    esp_adc_cal_value_t val_type_volt = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars_volt);
   
    print_char_val_type(val_type_temp);
    print_char_val_type(val_type_volt);
}

static void adc_task(void *arg) {  
  while (1) {

    uint32_t adc_reading_temp = 0;
    uint32_t adc_reading_volt = 0;

    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading_temp += adc1_get_raw((adc1_channel_t)tempChannel); //Get raw values from the sensor
            adc_reading_volt += adc1_get_raw((adc1_channel_t)volt_chanel);
        } else {
            int raw_temp;
            int raw_volt;
            adc2_get_raw((adc2_channel_t)tempChannel, width, &raw_temp); //Convert to volatge reading
            adc2_get_raw((adc2_channel_t)volt_chanel, width, &raw_volt);
            adc_reading_temp += raw_temp;
            adc_reading_volt += raw_volt;
        }
    }
    adc_reading_temp /= NO_OF_SAMPLES; //Get the average over the sample size
    adc_reading_volt /= NO_OF_SAMPLES;
    //printf("%lu, %lu\n", adc_reading_ir, adc_reading_us);

    //Convert adc_reading to voltage
    float tempVoltage = esp_adc_cal_raw_to_voltage(adc_reading_temp, adc_chars_temp); //Convert to volatge reading
    //printf("irVoltage")

    //voltage to temp (celsius) conversion
    float resistance = 10000 * tempVoltage / (3300 - tempVoltage);
    tempC = (1.0 / ((1.0 / 298.15) + (1.0 / 3435) * log(resistance / 10000))) - 273.1;
    float voltage = esp_adc_cal_raw_to_voltage(adc_reading_volt, adc_chars_volt);
    print_voltage = voltage/1000;
    //conversion to distance in mm

    //conversion to distance in cm/in for ir
    //cm_distance = -(log(((irVoltage / 1000) + 0.207) / 3.10)) / 0.017;
    //cm_distance = 29.988 * pow((irVoltage/ 1000), -1.173);;
    //cm_distance = -61.9 * (irVoltage / 1000) + 174.75;
    //in_distance = cm_distance / 2.54;
    printf("%d,%f, %f  ",seconds,tempC, print_voltage);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

static void adxl343_task(void *arg) {
    
    // Check for ADXL343
    uint8_t deviceID;
    getDeviceID(&deviceID);
    if (deviceID == 0xE5) {
        printf("\n>> Found ADAXL343\n");
    } else {
        printf("\n>> ADXL343 not found. Check wiring.\n");
        while(1) vTaskDelay(1000 / portTICK_PERIOD_MS); // Infinite loop to halt operation
    }

    // Setup ADXL343
    writeRegister(ADXL343_REG_INT_ENABLE, 0); // Disable interrupts
    setRange(ADXL343_RANGE_16_G); // Set range
    writeRegister(ADXL343_REG_POWER_CTL, 0x08); // Enable measurements

    // Continuously poll ADXL343
    while (1) {
        float xVal, yVal, zVal;
        getAccel(&xVal, &yVal, &zVal);
        calcRP(xVal, yVal, zVal);
        vTaskDelay(pdMS_TO_TICKS(2000));
        //vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void led_task(void *arg) {
    while (1) {
        gpio_set_level(GPIO_LED, ledState);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void configure_led(void) {
    gpio_reset_pin(GPIO_LED);
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
}

void udp_client_task(void *pvParameters) {
    char message[50];
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(UDP_SERVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_SERVER_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        sprintf(message,"%d,%f,%f,%f,%f",seconds,tempC,roll,pitch,print_voltage);
        int err = sendto(sock, message, strlen(message), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        } else {
            ESP_LOGI(TAG, "Message sent to server");
        }
        char rx_buffer[128];
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        } else {
            rx_buffer[len] = '\0';
            ESP_LOGI(TAG, "Received message from server: %s", rx_buffer);
        }
        if (rx_buffer[0] == '1') {
            ledState = 1;
        }
        else {
            ledState = 0;
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_main() {
    configure_led();
     timer_queue = xQueueCreate(10, sizeof(example_queue_element_t));
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();
    adc_init();
  i2c_master_init();
  i2c_scanner();

  // Create task to handle timer-based events 
  xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
  xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5-1, NULL);
  xTaskCreate(adc_task, "adc_task", 2048, NULL, 1, NULL);
  // Create ADXL343 task
  xTaskCreate(adxl343_task, "adxl343_task", 2048, NULL, 5, NULL);
   xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL); 
    //xTaskCreate(udp_receive_task, "udp_receiver", 4096, NULL, 5, NULL);
    alarm_init();
  
  vTaskDelay(100);
}
