#set(COMPONENT_SRCDIRS ".")
#set(COMPONENT_ADD_INCLUDEDIRS ".")
#set(COMPONENT_PRIV_INCLUDEDIRS "")
#set(COMPONENT_PRIV_REQUIRES spi_flash)
#register_component()


#spiffs_create_partition_image(<storage> <./spiffs_image> [FLASH_IN_PROJECT] [DEPENDS dep dep dep...])

idf_component_register(SRCS "esp_spiffs.c" "list.c" "mutex.c" "spiffs_cache.c" "spiffs_check.c" "spiffs_gc.c" "spiffs_hydrogen.c" "spiffs_nucleus.c" "spiffs_vfs.c"
                       INCLUDE_DIRS "./"
                       REQUIRES spi_flash )      #spi_flash

#spiffs_create_partition_image(storage ../spiffs_image FLASH_IN_PROJECT)