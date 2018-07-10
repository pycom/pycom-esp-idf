/**
 * @file
 * @brief
 *   This file includes the Pycom Esp32 hooks, that are called to debug openthread.
 */

 #ifndef ESP_HOOKS_H_
 #define ESP_HOOKS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* error codes */
typedef enum esp_hooks_codes_e{
    ESP_HOOKS_NONE = 0,
    ESP_HOOKS_1,
    ESP_HOOKS_2,
    ESP_HOOKS_NUM
} esp_hooks_codes_t;

void esp_hook_call(int code, int info_code);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* ESP_HOOKS_H_ */
