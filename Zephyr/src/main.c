#include <stdio.h>
#include <string.h>
#include <sys/_stdint.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/i2c.h>

#define TRANSMIT 1
#define RECEIVE 0

#define I2C_DEV DT_NODELABEL(i2c2)

#define GPS_ADDR 0x42
#define GPS_BUFFER_SIZE 256 // Buffer size for bulk read

#define SLEEP_TIME_MS 500
#define LED0_NODE DT_ALIAS(led0)

/* MS5611 register definitions */
#define MS5611_ADDR            0x77

/* Commands */
#define MS5611_RESET_CMD       0x1E
#define MS5611_PROM_READ_BASE  0xA2  /* C1 starts at 0xA2 and each coeff is 2 bytes */
#define MS5611_CONVERT_D1_4096 0x48  /* Oversampling = 4096 for Pressure */
#define MS5611_CONVERT_D2_4096 0x58  /* Oversampling = 4096 for Temperature */
#define MS5611_ADC_READ        0x00

/* Calibration coefficients */
struct ms5611_calib_data {
    uint16_t C1;
    uint16_t C2;
    uint16_t C3;
    uint16_t C4;
    uint16_t C5;
    uint16_t C6;
};

/* Structure to hold raw and computed results */
struct ms5611_data {
    struct ms5611_calib_data calib;
    int32_t temperature; /* in 0.01 °C */
    int32_t pressure;    /* in Pa     */
};

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static uint8_t gps_data_buffer[GPS_BUFFER_SIZE];
static size_t gps_data_len = 0;
static struct k_mutex gps_data_mutex;

/* Declare the stack */
K_THREAD_STACK_DEFINE(gps_reader_stack, 1024);
K_THREAD_STACK_DEFINE(nmea_parser_stack, 1024);
K_THREAD_STACK_DEFINE(ms5611_stack, 1024);

static struct k_thread gps_reader_thread_data;
static struct k_thread nmea_parser_thread_data;
static struct k_thread ms5611_thread_data;

static struct ms5611_data ms5611_dev_data;
static struct k_mutex ms5611_mutex;

// Function to configure LoRa
int lora_configure(const struct device *dev, bool transmit)
{
	int ret;
	struct lora_modem_config config;

	config.frequency = 916800000;
	config.bandwidth = BW_125_KHZ;
	config.datarate = SF_10;
	config.preamble_len = 8;
	config.coding_rate = CR_4_5;
	config.tx_power = 4;
	config.iq_inverted = false;
	config.public_network = false;
	config.tx = transmit;

	ret = lora_config(dev, &config);
	if (ret < 0) {
		printk("LoRa device configuration failed\n");
		return false;
	}

	return true;
}

void i2c_scan(const struct device *i2c_dev)
{
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready\n");
        return;
    }

    printk("Starting I2C scan...\n");

    for (uint8_t addr = 0x03; addr <= 0x77; addr++) { // Valid I2C 7-bit address range
        uint8_t dummy = 0;
        struct i2c_msg msgs[1];
        int ret;

        msgs[0].buf = &dummy;
        msgs[0].len = 1;
        msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

        ret = i2c_transfer(i2c_dev, msgs, 1, addr);
        if (ret == 0) {
            printk("Found device at 0x%02X\n", addr);
        }
    }

    printk("I2C scan complete.\n");
}

/* Send a reset command to the MS5611 */
static int ms5611_reset(const struct device *i2c_dev)
{
    uint8_t cmd = MS5611_RESET_CMD;
    return i2c_write(i2c_dev, &cmd, 1, MS5611_ADDR);
}

/* Read two bytes from the MS5611 PROM at a given address */
static int ms5611_read_prom_word(const struct device *i2c_dev, uint8_t addr, uint16_t *word)
{
    int ret;
    uint8_t buf[2];

    ret = i2c_write_read(i2c_dev, MS5611_ADDR, &addr, 1, buf, 2);
    if (ret < 0) {
        return ret;
    }

    /* Each word is 16 bits, big-endian */
    *word = (buf[0] << 8) | buf[1];
    return 0;
}

/* Read all calibration coefficients from PROM (C1..C6) */
static int ms5611_read_calibration(const struct device *i2c_dev, struct ms5611_calib_data *calib)
{
    int ret;
    uint8_t prom_addr;

    /* C1..C6 are at 0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC */
    for (int i = 0; i < 6; i++) {
        prom_addr = MS5611_PROM_READ_BASE + (i * 2);
        ret = ms5611_read_prom_word(i2c_dev, prom_addr, ((uint16_t *)calib) + i);
        if (ret < 0) {
            return ret;
        }
    }

    return 0;
}

/* Start the conversion for either Pressure (D1) or Temperature (D2) */
static int ms5611_start_conversion(const struct device *i2c_dev, uint8_t cmd)
{
    /* Write the command (e.g. 0x48 or 0x58) to start conversion */
    return i2c_write(i2c_dev, &cmd, 1, MS5611_ADDR);
}

/* Read the 24-bit ADC result */
static int ms5611_read_adc(const struct device *i2c_dev, uint32_t *adc_val)
{
    uint8_t rx_buf[3] = {0, 0, 0};
    int ret;

    /* ADC result at register 0x00, read 3 bytes */
    uint8_t cmd = MS5611_ADC_READ;
    ret = i2c_write_read(i2c_dev, MS5611_ADDR, &cmd, 1, rx_buf, 3);
    if (ret < 0) {
        return ret;
    }

    *adc_val = (rx_buf[0] << 16) | (rx_buf[1] << 8) | rx_buf[2];
    return 0;
}

/* 
 * Get compensated temperature (0.01 °C) and pressure (Pa) from the MS5611.
 * - The code here follows the MS5611 datasheet formulas.
 */
static int ms5611_get_data(const struct device *i2c_dev,
                           struct ms5611_calib_data *calib,
                           int32_t *temperature,
                           int32_t *pressure)
{
    int ret;
    uint32_t raw_temp, raw_pressure;

    /* Start temperature conversion (D2) */
    ret = ms5611_start_conversion(i2c_dev, MS5611_CONVERT_D2_4096);
    if (ret < 0) {
        return ret;
    }
    /* Data-sheet suggests max conversion time up to ~9.04 ms for OSR=4096 */
    k_msleep(10);

    /* Read raw temperature */
    ret = ms5611_read_adc(i2c_dev, &raw_temp);
    if (ret < 0) {
        return ret;
    }

    /* Start pressure conversion (D1) */
    ret = ms5611_start_conversion(i2c_dev, MS5611_CONVERT_D1_4096);
    if (ret < 0) {
        return ret;
    }
    k_msleep(10);

    /* Read raw pressure */
    ret = ms5611_read_adc(i2c_dev, &raw_pressure);
    if (ret < 0) {
        return ret;
    }

    /*  
     * Apply compensation (from the datasheet):
     *   dT    = D2 - C5 * 2^8
     *   TEMP  = 2000 + dT * C6 / 2^23
     *   OFF   = C2 * 2^16 + (C4 * dT) / 2^7
     *   SENS  = C1 * 2^15 + (C3 * dT) / 2^8
     *   P     = (D1 * SENS / 2^21 - OFF) / 2^15
     */
    int32_t dT   = (int32_t)raw_temp - ((int32_t)calib->C5 << 8);
    int32_t TEMP = 2000 + ((int64_t)dT * calib->C6) / 8388608;  /* 2^23 = 8388608 */

    int64_t OFF  = ((int64_t)calib->C2 << 16) + (((int64_t)calib->C4 * dT) >> 7);
    int64_t SENS = ((int64_t)calib->C1 << 15) + (((int64_t)calib->C3 * dT) >> 8);

    /* Second order temperature compensation, per datasheet, if you desire more accuracy 
       (especially below 20C). This snippet omits that for brevity. */

    int64_t P = ((((int64_t)raw_pressure * SENS) >> 21) - OFF) >> 15;

    /* Results */
    *temperature = TEMP;   /* in 0.01 °C */
    *pressure    = (int32_t)P;  /* in Pa */

    return 0;
}

void gps_reader_thread(void *p1, void *p2, void *p3) {
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_DEV);
    uint8_t buffer[GPS_BUFFER_SIZE];
	uint32_t sleep_time_ms = 5;

    if (!device_is_ready(i2c_dev)) {
        printk("I2C device is not ready\n");
        return;
    }

	i2c_scan(i2c_dev);

    while (1) {
        size_t read_len = 0;

        while (read_len < GPS_BUFFER_SIZE) {
            int ret = i2c_read(i2c_dev, &buffer[read_len], 1, GPS_ADDR);
            if (ret < 0) {
                printk("I2C read error\n");
                break;
            }

            uint8_t ch = buffer[read_len];

            if ((ch >= 32 && ch <= 126) || ch == '\r' || ch == '\n') {
                read_len++;
            } else {
                break;
            }
        }

        if (read_len > 0) {
            k_mutex_lock(&gps_data_mutex, K_FOREVER);
            memcpy(gps_data_buffer, buffer, read_len);
            gps_data_len = read_len;
            k_mutex_unlock(&gps_data_mutex);
        }

        k_msleep(sleep_time_ms);
    }
}

void nmea_parser_thread(void *p1, void *p2, void *p3) {
    uint8_t local_buffer[GPS_BUFFER_SIZE];

    while (1) {
        k_mutex_lock(&gps_data_mutex, K_FOREVER);
        size_t len = gps_data_len;
        memcpy(local_buffer, gps_data_buffer, len);
        gps_data_len = 0;
		uint32_t sleep_time_ms = 100;
        k_mutex_unlock(&gps_data_mutex);

        for (size_t i = 0; i < len; i++) {
            if (local_buffer[i] == '$') {
                size_t start = i;
                size_t end = i;

                while (end < len && local_buffer[end] != '*') {
                    end++;
                }

                if (end < len && local_buffer[end] == '*') {
                    end += 3; // Include checksum and 

                    // printk("NMEA Packet: ");
                    for (size_t j = start; j < end && j < len; j++) {
                        printk("%c", local_buffer[j]);
                    }
                    printk("\n");

                    i = end - 1; // Move index to end of packet
                }
            }
        }

        k_msleep(sleep_time_ms);
    }
}

void ms5611_thread(void *p1, void *p2, void *p3)
{
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_DEV);
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready for MS5611\n");
        return;
    }

    /* Reset the sensor */
    if (ms5611_reset(i2c_dev) < 0) {
        printk("MS5611 reset failed\n");
        return;
    }

    /* Wait after reset (per datasheet, 2.8 ms or so is typical) */
    k_msleep(3);

    /* Read calibration data */
    if (ms5611_read_calibration(i2c_dev, &ms5611_dev_data.calib) < 0) {
        printk("Failed to read MS5611 calibration\n");
        return;
    }

    while (1) {
        k_mutex_lock(&ms5611_mutex, K_FOREVER);
        if (ms5611_get_data(i2c_dev,
                            &ms5611_dev_data.calib,
                            &ms5611_dev_data.temperature,
                            &ms5611_dev_data.pressure) == 0) {

			int32_t temp_raw = ms5611_dev_data.temperature;  // 0.01 °C
			int32_t press_raw = ms5611_dev_data.pressure;    // Pa

			int32_t temp_int = temp_raw / 100;      // integer part of temperature
			int32_t temp_dec = temp_raw % 100;      // fractional part
			int32_t press_mbar = press_raw / 100;   // integer part of pressure in mbar
			int32_t press_dec  = press_raw % 100;   // fractional part in mbar

			printk("MS5611 -> Temperature: %d.%02d °C, Pressure: %d.%02d mbar\n",
				temp_int, temp_dec, press_mbar, press_dec);
        } else {
            printk("Failed to get data from MS5611\n");
        }
        k_mutex_unlock(&ms5611_mutex);

        /* Sleep for some interval (e.g. 1 second) */
        k_msleep(1000);
    }
}

int main(void) {
    int ret;
    const struct device *dev_lora;

    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    // Setup LoRa
    dev_lora = DEVICE_DT_GET(DT_ALIAS(lora0));
    if (!device_is_ready(dev_lora)) {
        printk("%s: device not ready\n", dev_lora->name);
        return 0;
    }

    if (lora_configure(dev_lora, TRANSMIT)) {
        printk("LoRa Device Configured\n");
    } else {
        return 0;
    }

    k_mutex_init(&gps_data_mutex);
    k_mutex_init(&ms5611_mutex);

    /* Create the I2C reader thread (already in your code) */
    k_tid_t gps_tid = k_thread_create(
        &gps_reader_thread_data,
        gps_reader_stack,
        K_THREAD_STACK_SIZEOF(gps_reader_stack),
        gps_reader_thread,
        NULL, NULL, NULL,
        7, 0, K_NO_WAIT
    );

    /* Create the NMEA parser thread (already in your code) */
    k_tid_t nmea_tid = k_thread_create(
        &nmea_parser_thread_data,
        nmea_parser_stack,
        K_THREAD_STACK_SIZEOF(nmea_parser_stack),
        nmea_parser_thread,
        NULL, NULL, NULL,
        7, 0, K_NO_WAIT
    );

    /* Create the MS5611 barometer thread */
    k_tid_t ms5611_tid = k_thread_create(
        &ms5611_thread_data,
        ms5611_stack,
        K_THREAD_STACK_SIZEOF(ms5611_stack),
        ms5611_thread,
        NULL, NULL, NULL,
        7, 0, K_NO_WAIT
    );

    while (1) {
        gpio_pin_toggle_dt(&led);
        k_msleep(SLEEP_TIME_MS);
    }

    return 0;
}
