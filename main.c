#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "bsec_integration.h"

#define I2C_PORT i2c1

const uint LED_PIN = 25;
bool led_state = false;

int8_t bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len) {
    int32_t ret = 0;
    uint8_t buff[16];

    buff[0] = reg_addr;
    for (size_t i = 0; i < data_len; i++) {
        buff[i + 1] = reg_data_ptr[i];
    }
    ret += i2c_write_blocking(I2C_PORT, dev_addr, buff, data_len + 1, false);
    // printf("I2C write[%d] %x -> ", ret, reg_addr);
    // for (size_t i = 0; i < data_len + 1; i++) {
    //     printf("%x ", buff[i]);
    // }
    // printf("\n");
    return 0;
}

int8_t bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data_ptr, uint16_t data_len) {
    int32_t ret = 0;
    ret += i2c_write_blocking(I2C_PORT, dev_addr, &reg_addr, 1, false);
    sleep_us(150);
    ret += i2c_read_blocking(I2C_PORT, dev_addr, reg_data_ptr, data_len, false);
    // printf("I2C read[%d] %x -> ", ret, reg_addr);
    // for (size_t i = 0; i < data_len; i++) {
    //     printf("%x ", reg_data_ptr[i]);
    // }
    // printf("\n");
    return 0;
}

void sleep(uint32_t t_ms) { sleep_ms(t_ms); }

int64_t get_timestamp_us() {
    int64_t system_current_time = (int64_t)(to_us_since_boot(get_absolute_time()));
    return system_current_time;
}

void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
     float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
     float static_iaq, float co2_equivalent, float breath_voc_equivalent) {
    printf("[%d]:\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
        iaq_accuracy, iaq, temperature, humidity, pressure,
        raw_temperature, raw_humidity, gas, static_iaq,
        co2_equivalent, breath_voc_equivalent);
    gpio_put(LED_PIN, led_state);
    led_state = !led_state;
}

void state_save(const uint8_t *state_buffer, uint32_t length) {}
uint32_t state_load(uint8_t *state_buffer, uint32_t n_buffer) { return 0; }
uint32_t config_load(uint8_t *config_buffer, uint32_t n_buffer) { return 0; }

void hang() {
    while (1) {
        printf("something went wrong\n");
        gpio_put(LED_PIN, led_state);
        led_state = !led_state;
        sleep_ms(500);
    }
}

int main() {
    bi_decl(bi_program_description("This is a climate control device."));

    stdio_init_all();

    i2c_init(I2C_PORT, 300 * 1000);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    return_values_init ret;

    sleep_ms(3000);

    uint8_t data = 0;
    bus_read(BME680_I2C_ADDR_PRIMARY, BME680_CHIP_ID_ADDR, &data, 1);
    printf("CHIP ID: %x\n", data);

    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, bus_write, bus_read, sleep, state_load, config_load);
    while (ret.bme680_status != 0 && ret.bsec_status != 0) {
        sleep_ms(2000);
        printf("Try to init again ...\n");
        ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, bus_write, bus_read, sleep, state_load, config_load);
        if (ret.bme680_status) {
            printf("Could not intialize BME680\n");
        }
        else if (ret.bsec_status) {
            printf("Could not intialize BSEC library\n");
        }
    }

    printf("Stsrting loop ...\n");
    bsec_iot_loop(sleep, get_timestamp_us, output_ready, state_save, 10000);

    hang();
    return 0;
}