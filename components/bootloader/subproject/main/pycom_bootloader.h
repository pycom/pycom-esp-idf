/*
 * Copyright (c) 2020, Pycom Limited.
 *
 * This software is licensed under the GNU GPL version 3 or any
 * later version, with permitted additional terms. For more information
 * see the Pycom Licence v1.0 document supplied with this file, or
 * available at https://www.pycom.io/opensource/licensing
 */

/* This file contains the Pycom specific helper function for the bootloader*/

#ifndef __PYCOM_BOOTLOADER_H__
#define __PYCOM_BOOTLOADER_H__

#include <stdint.h>
#include "esp_flash_partitions.h"
#include "esp_attr.h"
#include "bootloader_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _boot_info_t
{
  uint32_t  ActiveImg;
  uint32_t  Status;
  uint32_t  PrevImg;
  uint32_t  size;
  uint32_t  safeboot;
  uint8_t   signature[16];
  uint32_t  crc;
} boot_info_t;

#define IMG_SIZE_8MB                            (1980 * 1024)
#define IMG_UPDATE1_OFFSET_8MB                  (2112 * 1024)  // taken from the partitions table

#define IMG_SIZE_4MB                            (1720 * 1024)
#define IMG_UPDATE1_OFFSET_4MB                  (1792 * 1024)  // taken from the partitions table

#define OTAA_DATA_SIZE                      (4 * 1024)
#define OTA_DATA_INDEX                      2
#define IMG_FACTORY_OFFSET                  (64 * 1024)


#define IMG_UPDATE2_OFFSET                  (IMG_FACTORY_OFFSET)

#define IMG_STATUS_CHECK                    0
#define IMG_STATUS_READY                    1
#define IMG_STATUS_PATCH                    2

#define IMG_ACT_FACTORY                     0
#define IMG_ACT_UPDATE1                     1
#define IMG_ACT_UPDATE2                     2

#define BOOT_VERSION                        "V0.3"
#define SPI_SEC_SIZE                        0x1000

#define PARTITIONS_COUNT_8MB                    5
#define PARTITIONS_COUNT_4MB                    7

#define PART_TYPE_APP                       0x00
#define PART_SUBTYPE_FACTORY                0x00
#define PART_SUBTYPE_OTA_FLAG               0x10
#define PART_SUBTYPE_OTA_MASK               0x0f
#define PART_SUBTYPE_TEST                   0x20

#define PART_TYPE_DATA                      0x01
#define PART_SUBTYPE_DATA_OTA               0x00
#define PART_SUBTYPE_DATA_RF                0x01
#define PART_SUBTYPE_DATA_WIFI              0x02

#define SAFE_BOOT_HW                        0x01
#define SAFE_BOOT_SW                        0x02

#define SPI_ERROR_LOG "spi flash error"

// Luckily these are the same for all boards, they are defined in mpconfigboard.h
#define MICROPY_HW_SAFE_PIN_NUM                                 (21)
#define MICROPY_HW_HB_PIN_NUM                                   (0)

extern int pycom_bootloader_utility_get_selected_boot_partition(const bootloader_state_t *bs);
extern bool pycom_bootloader_common_ota_select_valid(const boot_info_t *s);
extern void IRAM_ATTR mperror_set_rgb_color (uint32_t rgbcolor);
extern void mperror_heartbeat_switch_off (void);
extern void mperror_init0 (void);
extern __attribute__((noreturn)) void mperror_fatal_error (void);
extern uint32_t wait_for_safe_boot (const boot_info_t *boot_info, uint32_t *ActiveImg);


#ifdef __cplusplus
}
#endif

#endif /* __PYCOM_BOOTLOADER_H__ */
