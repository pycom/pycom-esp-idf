idf_component_register(SRCS "bootloader_start.c" "pycom_bootloader.c" "pycom_bootmgr.c" "pycom_gpio.c" "pycom_mperror.c"
                    REQUIRES bootloader bootloader_support spi_flash)

idf_build_get_property(target IDF_TARGET)
set(scripts "ld/${target}/bootloader.ld"
            "ld/${target}/bootloader.rom.ld")

target_linker_script(${COMPONENT_LIB} INTERFACE "${scripts}")

