/*
 * Copyright (c) 2020, Pycom Limited.
 *
 * This software is licensed under the GNU GPL version 3 or any
 * later version, with permitted additional terms. For more information
 * see the Pycom Licence v1.0 document supplied with this file, or
 * available at https://www.pycom.io/opensource/licensing
 */

#include <string.h>
#include "esp_log.h"
#include "bootloader_flash.h"
#include "esp32/rom/spi_flash.h"
#include "esp32/rom/cache.h"
#include "esp32/rom/crc.h"
#include "esp_flash_encrypt.h"

#include "pycom_bootloader_support.h"


static const char* TAG = "boot";

esp_err_t pycom_read_otadata(const esp_partition_pos_t *ota_info, boot_info_t* boot_info)
{
    boot_info_t *_boot_info;

    if (ota_info->offset == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    // partition table has OTA data partition
    if (ota_info->size < 2 * sizeof(esp_partition_pos_t)) {
        ESP_LOGE(TAG, "ota_info partition size %d is too small (minimum %d bytes)", ota_info->size, sizeof(esp_ota_select_entry_t));
        return ESP_FAIL; // can't proceed
    }

    ESP_LOGD(TAG, "OTA data offset 0x%x", ota_info->offset);
    _boot_info = (boot_info_t *)bootloader_mmap(ota_info->offset, ota_info->size);
    if (!boot_info) {
        ESP_LOGE(TAG, "bootloader_mmap(0x%x, 0x%x) failed", ota_info->offset, ota_info->size);
        return ESP_FAIL; // can't proceed
    }
    memcpy(boot_info, _boot_info, sizeof(boot_info_t));
    bootloader_munmap(_boot_info);

    return ESP_OK;
}

IRAM_ATTR bool pycom_ota_write_boot_info(boot_info_t *boot_info, uint32_t offset) {

    boot_info->crc = pycom_bootloader_common_ota_select_crc(boot_info);

    esp_err_t err = bootloader_flash_erase_sector(offset / FLASH_SECTOR_SIZE);
    if (err == ESP_OK) {
        if (esp_flash_encryption_enabled()) {
            uint8_t buff[64] __attribute__((aligned (32)));
            memcpy(buff, (void *)boot_info, sizeof(boot_info_t));
            err = bootloader_flash_write(offset, boot_info, 64, esp_flash_encryption_enabled());
        }
        else{
            err = bootloader_flash_write(offset, boot_info, sizeof(boot_info_t), esp_flash_encryption_enabled());
        }
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error in pycom_ota_write_boot_info operation. err = 0x%x", err);
        return false;
    }

    return true;
}

uint32_t pycom_bootloader_common_ota_select_crc(const boot_info_t *s)
{
    return crc32_le(UINT32_MAX, (uint8_t*)&s->ActiveImg, sizeof(boot_info_t) - sizeof(s->crc));
}
