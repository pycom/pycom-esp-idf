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

IRAM_ATTR bool pycom_ota_write_boot_info (boot_info_t *boot_info, uint32_t offset) {
    esp_rom_spiflash_result_t write_result;

    boot_info->crc = pycom_bootloader_common_ota_select_crc(boot_info);
    Cache_Read_Disable(0);
    if (ESP_ROM_SPIFLASH_RESULT_OK != esp_rom_spiflash_erase_sector(offset / 0x1000)) {
        ESP_LOGE(TAG, SPI_ERROR_LOG);
        Cache_Read_Enable(0);
        return false;
    }

    if (esp_flash_encryption_enabled()) {
            // if flash is encrypted, then Write is done 32B chunks
        uint8_t buff[64] __attribute__((aligned (32)));
        memcpy(buff, (void *)boot_info, sizeof(boot_info_t));
        write_result = esp_rom_spiflash_write_encrypted(offset, (void *)boot_info, 64);
    }
    else {
            write_result = esp_rom_spiflash_write(offset, (void *)boot_info, sizeof(boot_info_t));
    }

    if (ESP_ROM_SPIFLASH_RESULT_OK != write_result) {
        ESP_LOGE(TAG, SPI_ERROR_LOG);
        Cache_Read_Enable(0);
        return false;
    }
    Cache_Read_Enable(0);
    return true;
}

uint32_t pycom_bootloader_common_ota_select_crc(const boot_info_t *s)
{
  return crc32_le(UINT32_MAX, (uint8_t*)&s->ActiveImg, sizeof(boot_info_t) - sizeof(s->crc));
}
