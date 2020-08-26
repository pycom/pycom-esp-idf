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
#include "esp_image_format.h"
#include "pycom_bootloader.h"

/* This file contains the Pycom specific helper function for the bootloader*/


static const char* TAG = "boot";
static uint32_t pycom_bootloader_common_ota_select_crc(const boot_info_t *s);

/* Public APIs */

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

int pycom_bootloader_utility_get_selected_boot_partition(const bootloader_state_t *bs)
{

    boot_info_t boot_info_object;
    boot_info_t *boot_info = &boot_info_object;
    int boot_index = FACTORY_INDEX;

    if (bs->ota_info.offset == 0) {
        return FACTORY_INDEX;
    }

    if (pycom_read_otadata(&bs->ota_info, boot_info) != ESP_OK) {
        return INVALID_INDEX;
    }

#ifndef RGB_LED_DISABLE
    mperror_init0();
#endif
    if (!pycom_bootloader_common_ota_select_valid(boot_info)) {
        ESP_LOGI(TAG, "Initializing OTA partition info");
        // init status flash
        boot_info->ActiveImg = IMG_ACT_FACTORY;
        boot_info->Status = IMG_STATUS_READY;
        boot_info->PrevImg = IMG_ACT_FACTORY;
        boot_info->safeboot = false;
        if (!pycom_ota_write_boot_info (boot_info, bs->ota_info.offset)) {
            ESP_LOGE(TAG, "Error writing boot info");
#ifndef RGB_LED_DISABLE
            mperror_fatal_error();
#endif
            return ESP_FAIL;
        }
        return FACTORY_INDEX;
    } else {
        // CRC is fine, check here the image that we need to load based on the status (ready or check)
        // if image is in status check then we must verify the MD5, and set the new status
        // if the MD5 fails, then we roll back to the previous image

        // do we have a new image that needs to be verified?
        if (boot_info->Status == IMG_STATUS_CHECK) {
            if (boot_info->ActiveImg == IMG_ACT_UPDATE2) {
                boot_info->ActiveImg = IMG_ACT_FACTORY;    // we only have space for 1 OTA image
            }

            // verify the active image (ota partition)
            esp_image_metadata_t data;
            esp_err_t ret;
            if(boot_info->ActiveImg == IMG_ACT_FACTORY) {
                ret = esp_image_verify(ESP_IMAGE_VERIFY, &bs->factory, &data);
            }
            else {
                ret = esp_image_verify(ESP_IMAGE_VERIFY, &bs->ota[0], &data);
            }

            if (ESP_OK != ret) {
                ets_printf("Cannot load Firmware img in the active partition! .. Defaulting back to previous partition\n");
                // switch to the previous image
                uint32_t tempimg = boot_info->ActiveImg;
                boot_info->ActiveImg = boot_info->PrevImg;
                boot_info->PrevImg = tempimg;
            }

            // in any case, change the status to "READY"
            boot_info->Status = IMG_STATUS_READY;
            // write the new boot info
            if (!pycom_ota_write_boot_info (boot_info, bs->ota_info.offset)) {
                ESP_LOGE(TAG, "Error writing boot info");
                mperror_fatal_error();
                return ESP_FAIL;
            }
        }

        // this one might modify the boot info hence it MUST be called after
        // bootmgr_verify! (so that the changes are not saved to flash)
        ESP_LOGI(TAG, "Checking safe boot pin");

        uint32_t ActiveImg = boot_info->ActiveImg;
        uint32_t safeboot = wait_for_safe_boot (boot_info, &ActiveImg);
        if (safeboot > 0) {
            ESP_LOGI(TAG, "Safe boot requested!");
        }
        if (safeboot != boot_info->safeboot) {
            if (boot_info->safeboot == SAFE_BOOT_SW) {
                boot_info->safeboot = SAFE_BOOT_HW;
            } else {
                boot_info->safeboot = safeboot;
            }
            // write the new boot info
            if (!pycom_ota_write_boot_info (boot_info, bs->ota_info.offset)) {
                ESP_LOGE(TAG, "Error writing boot info");
                mperror_fatal_error();
                return ESP_FAIL;
            }
        }

        // load the selected active image
        if(boot_info->ActiveImg == IMG_ACT_FACTORY) {
            return FACTORY_INDEX;
        }
        else {
            return 0; // OTA slot
        }
        return true;
    }

    return boot_index;
}

bool pycom_bootloader_common_ota_select_valid(const boot_info_t *s)
{
    uint32_t _crc = pycom_bootloader_common_ota_select_crc(s);
    ESP_LOGI(TAG, "Cal crc=%x, saved crc=%x", _crc, s->crc);
    return s->Status != UINT32_MAX && s->crc == _crc;

}

/* Private APIs */


static uint32_t pycom_bootloader_common_ota_select_crc(const boot_info_t *s)
{
  return crc32_le(UINT32_MAX, (uint8_t*)&s->ActiveImg, sizeof(boot_info_t) - sizeof(s->crc));
}


