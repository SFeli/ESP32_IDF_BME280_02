/*
 * Vorlage i2c-master simple von ESPREssif
 * und utkumaden/esp-idf-bmx280
 */
/* i2c - neuer i2c - master treiber
    und BME280 - API von Bosch
    dev_addr .. Device Address 0x76
    reg_addr .. Register Address z.B. 0xD0 zur Abfrage der Sensormodells BME280 -> 0x60
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"          // 
#include "freertos/task.h"              // FreeRTOS task management
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/i2c_master.h"          // Neuer I2C-Master 
#include "bme280.c"                     // Bosch BME280 API - Library

static const char *TAG = "BME280-I2C_Master";

#define I2C_MASTER_SCL_IO    CONFIG_BME280_I2C_MASTER_SCL   //  z.B. 48 GPIO number used for I2C master clock 
#define I2C_MASTER_SDA_IO    CONFIG_BME280_I2C_MASTER_SDA   //  z.B. 45 GPIO number used for I2C master data  
#define I2C_MASTER_NUM              I2C_NUM_0               // I2C port number for master dev 
#define I2C_MASTER_FREQ_HZ          100000                  // I2C master clock frequency 
#define I2C_MASTER_TIMEOUT_MS       500                     // war mal 1000

#define BME280_SENSOR_ADDR          BME280_I2C_ADDR_PRIM  // 0x76 Primary Address of the BME280 sensor 
#define BME280_WHO_AM_I_REG_ADDR    BME280_REG_CHIP_ID    // 0xD0 Register addresses of the "who am I" register 
#define BME280_PWR_MGMT_1_REG_ADDR  0xE0        // Register addresses of the power management register */
#define BOSCH_RESET_VALUE           0xB6

int8_t rslt;                // ReturnCode of Functions
uint8_t data[2];            // Temporary Variable
struct bme280_dev           bme;
struct bme280_settings      b_settings;
struct bme280_calib_data    b_calib_data;

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

/**
 *  @brief Function : Perform a single write-read transaction on the I2C bus without BME280 Library
 *              The transaction will be undergoing until it finishes or it reaches the timeout provided.
 *  @param[in]  dev_handle : I2C master device handle that created by i2c_master_bus_add_device.
 *  @param[in]  reg_addr   : Data bytes to send on the I2C bus. - Register
 *  @param[out] data       : Data bytes received from i2c bus.
 *  @param[in]  len        : Size, in bytes, of the read buffer.
 *  @return Status of the inquiry 0 .. OK 
 */
static esp_err_t bme280_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, sizeof(reg_addr), data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * Funktion for BME280 - API - Library - Format zum einlesen der Daten
 *      BOSCH-Format : typedef BME280_INTF_RET_TYPE (*bme280_read_fptr_t)
 *                              (uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
 *  @param[in]  reg_addr   : Register im BME280 - Sensor, der ausgelesen werden soll
 *  @param[out] *reg_data  : Data bytes received from i2c bus.
 *  @param[in]  len        : Size, in bytes, von den Daten 
 *  @param[in]  *intf_ptr  : Size, in bytes, of the read buffer.  
 */
static bme280_read_fptr_t bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    char* TAGR = "ESP32-BME280-Read ";
    switch (reg_addr)
    {
    case 0xd0:
        ESP_LOGD(TAGR, "Register to read  %x -> Chip-ID", reg_addr);
        break;
    case 0xf4:
        ESP_LOGD(TAGR, "Register to read  %x -> controls", reg_addr);
        break;
    case 0xf7:
        ESP_LOGD(TAGR, "Register to read  %x -> burst-read of values", reg_addr);
        break;
    case 0x88:
        ESP_LOGD(TAGR, "Register to read  %x -> trimming parameters", reg_addr);
        break;
    case 0xf2:
        ESP_LOGD(TAGR, "Register to read  %x -> Reg CTRL HUM", reg_addr);
        break;
    case 0xf3:
        ESP_LOGD(TAGR, "Register to read  %x -> Reg Status", reg_addr);
        break;  
    case 0xf5:
        ESP_LOGD(TAGR, "Register to read  %x -> Reg Config", reg_addr);
        break;
    case 0xe1:
        ESP_LOGD(TAGR, "Register to read  %x -> Humidity Calibration Data", reg_addr);
        break;
    default:
        ESP_LOGD(TAGR, "Register to read  %x -> not known", reg_addr);
        break;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    rslt = i2c_master_transmit_receive(dev_handle, &reg_addr, sizeof(reg_addr), reg_data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (rslt != ESP_OK) {
        ESP_LOGE(TAG, "bme2_i2c_read failed!");
    }
    return (bme280_read_fptr_t) rslt;
}

/**
 *  @brief Function : Perform to a write a single transaction on the I2C bus without BME280 - Library. 
 *         The transaction will be undergoing until it finishes or it reaches the timeout provided.
 *  @param[in]  dev_handle : I2C master device handle that created by i2c_master_bus_add_device.
 *  @param[in]  reg_addr   : Data bytes to send on the I2C bus. - Register
 *  @param[out] data       : Data bytes received from i2c bus.
 */
static esp_err_t bme280_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
} 

/**
 * Funktion for BME280 - API - Library - Format zum Schreiben der Daten
 *  BOSCH-Format : typedef BME280_INTF_RET_TYPE (*bme280_write_fptr_t)
 *                          (uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
 *  @param[in]  reg_addr   : Register im BME280 - Sensor, der ausgelesen werden soll
 *  @param[out] *reg_data  : Data bytes received from i2c bus.
 *  @param[in]  len        : Size, in bytes, von den Daten 
 *  @param[in]  *intf_ptr  : Size, in bytes, of the read buffer.  
 */
static bme280_write_fptr_t bme280_i2c_write(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    char* TAGW = "ESP32-BME280-Write";
    switch (reg_addr)
    {
    case 0xe0:
        ESP_LOGD(TAGW, "Register to write %x -> reset", reg_addr);
        break;
    case 0xf2:
        ESP_LOGD(TAGW, "Register to write %x -> CTRL_HUM", reg_addr);
        break;
    case 0xf4:
        ESP_LOGD(TAGW, "Register to write %x -> controls", reg_addr);
        break;
    case 0xf5:
        ESP_LOGD(TAGW, "Register to write %x -> Reg Config", reg_addr);
        break;
    default:
        ESP_LOGD(TAGW, "Register to write %x -> not known", reg_addr);
        break;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);      // 100 / 10
    uint8_t write_buf[2] = {reg_addr, (uint8_t)*reg_data};
    rslt = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (rslt != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_write_to_device failed!");
    }
    return (bme280_write_fptr_t) rslt;
}

/*!
 * Delay function nach BOSCH-Format
 * typedef void (*bme280_delay_us_fptr_t)(uint32_t period, void *intf_ptr) 
 */
static bme280_delay_us_fptr_t delay_ms(uint32_t period_ms, void *intf_ptr)
{
    //vTaskDelay(period_ms / portTICK_PERIOD_MS);
    ESP_LOGD("TAG", "Function delay 1");
    vTaskDelay(40);
    ESP_LOGD("TAG", "Function delay 2");
    return (bme280_delay_us_fptr_t) 0;
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    // 1. Step - creat I2C - Master Bus
    ESP_LOGI(TAG, "I2C_MASTER_PORT  :  %d", I2C_MASTER_NUM);
	ESP_LOGI(TAG, "I2C_MASTER_MODE  :  %d (1..master or 0..slave)", I2C_MODE_MASTER);
	ESP_LOGI(TAG, "I2C_MASTER_SDA_IO:  %d", I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "I2C_MASTER_SCL_IO:  %d", I2C_MASTER_SCL_IO);
   	ESP_LOGI(TAG, "I2C_MASTER_FREQ_HZ  %d", I2C_MASTER_FREQ_HZ);

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));
    ESP_LOGI("TAG", "I2C - Master bus erstellt");

    // 2. Step - creat I2C - Master device
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BME280_SENSOR_ADDR,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

void app_main(void)
{
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    /* Read the BME280 WHO_AM_I register, on power up the register should have the value 0x71 BMP280 and 0x60 BME280*/
    ESP_ERROR_CHECK(bme280_register_read(dev_handle, BME280_WHO_AM_I_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    /* Demonstrate writing by resetting the BME280 */
    ESP_ERROR_CHECK(bme280_register_write_byte(dev_handle, BME280_PWR_MGMT_1_REG_ADDR, 1 << BOSCH_RESET_VALUE));

    static uint8_t dev_addr;
    dev_addr  = BME280_SENSOR_ADDR;    // BME280_I2C_ADDR_PRIM = 0x76 oder BME280_I2C_ADDR_SEC = 0x77

    bme.chip_id =  BME280_CHIP_ID;   // uint8_t      Chip Id 
    bme.intf    =  BME280_I2C_INTF;  // Interface Selection For SPI = BME280_SPI_INTF For I2C = BME280_I2C_INTF
    bme.intf_ptr = &dev_addr;        // The interface pointer is used to enable the user
                                     // to link their interface descriptors for reference during the
                                     // implementation of the read and write interfaces to the hardware.
    //bme.intf_rslt =                // BME280_INTF_RET_TYPE  Variable to store result of read/write function 
    bme.read  = (bme280_read_fptr_t)  bme280_i2c_read;      // bme280_read_fptr_t               Read function pointer 
    bme.write = (bme280_write_fptr_t) bme280_i2c_write;    // bme280_write_fptr_t              Write function pointer 
    bme.delay_us = (bme280_delay_us_fptr_t)&delay_ms;      // delay_us bme280_delay_us_fptr_t  Delay function pointer 
    bme.calib_data = b_calib_data;   // bme280_calib_data                 Trim data 

    // 3. Step initialize BME280 - Sensor 
    rslt = bme280_init(&bme);
    ESP_LOGI(TAG, "bme280 initialization RC=%d", rslt);

    /* Always read the current settings before writing, especially when all the configuration is not modified */
    rslt = bme280_get_sensor_settings(&b_settings, &bme);
    ESP_LOGI("BME280", "BME280 get settings RC=%d", rslt);

    /* Configuring the over-sampling mode, filter coefficient and output data rate */
    b_settings.filter = BME280_FILTER_COEFF_2;

    /* Over-sampling mode is set as high resolution i.e., os_pres = 8x and os_temp = 1x */
    b_settings.osr_h = BME280_OVERSAMPLING_2X;
    b_settings.osr_p = BME280_OVERSAMPLING_2X;
    b_settings.osr_t = BME280_OVERSAMPLING_2X;
    b_settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

    rslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &b_settings, &bme);
    if (rslt != ESP_OK) {
        ESP_LOGE("BME280", "BME280 set Settings RC=%d", rslt);
    } 

    /* Set normal power mode */
    rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme);
    if (rslt != ESP_OK) {
        ESP_LOGE("BME280", "BME280 set power  RC=%d", rslt);
    }  

    /* Calculate measurement time in microseconds */
    uint32_t period;
    rslt = bme280_cal_meas_delay(&period, &b_settings);
    if (rslt != ESP_OK) {
        ESP_LOGE("BME280", "Cal Meas delay: rslt = %d", rslt);
    }   
    ESP_LOGI("BME280", "Measurement time : %lu us", (long unsigned int)period);

    struct bme280_data bme_results;

    for(int i = 0; i < 10; i++)  {
        rslt = bme280_get_sensor_data(BME280_ALL, &bme_results, &bme);
        if (rslt != ESP_OK) {
            ESP_LOGE("BME280", "Returncode von get data: rslt = %d", rslt);
        }  
        ESP_LOGI("BME280", "Read Values: temp = %2.2f, pres = %2.2f, hum = %2.2f", bme_results.temperature, bme_results.pressure/100, bme_results.humidity);
        vTaskDelay(500);
    }
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}