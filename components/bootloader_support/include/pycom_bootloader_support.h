/*
 * Copyright (c) 2020, Pycom Limited.
 *
 * This software is licensed under the GNU GPL version 3 or any
 * later version, with permitted additional terms. For more information
 * see the Pycom Licence v1.0 document supplied with this file, or
 * available at https://www.pycom.io/opensource/licensing
 */

/* This file contains the Pycom specific bootloader support functions */

#ifndef __PYCOM_BOOTLOADER_SUPPORT_H__
#define __PYCOM_BOOTLOADER_SUPPORT_H__

#include <stdint.h>
#include "esp_flash_partitions.h"
#include "esp_attr.h"
#include "../../bootloader/subproject/main/pycom_bootloader.h"

#ifdef __cplusplus
extern "C"
{
#endif

extern esp_err_t pycom_read_otadata(const esp_partition_pos_t *ota_info, boot_info_t* boot_info);
extern IRAM_ATTR bool pycom_ota_write_boot_info (boot_info_t *boot_info, uint32_t offset);
uint32_t pycom_bootloader_common_ota_select_crc(const boot_info_t *s);

#ifdef __cplusplus
}
#endif

#endif /* __PYCOM_BOOTLOADER_SUPPORT_H__ */
