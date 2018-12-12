#include "esp_common.h"
#include "gpio.h"

uint32 user_rf_cal_sector_set(void) {
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;
    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void LEDBlinkTask (void *pvParameters) {
    while(1) {
        vTaskDelay (2000/portTICK_RATE_MS);
        GPIO_OUTPUT_SET (2, 1);

        vTaskDelay (4000/portTICK_RATE_MS);
        GPIO_OUTPUT_SET (2, 0);
    }
}

void user_init(void) {
    printf("SDK version:%s\n", system_get_sdk_version());
    PIN_FUNC_SELECT (PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO2);
    xTaskCreate(LEDBlinkTask, (signed char *)"Blink", 256, NULL, 2, NULL);
}
