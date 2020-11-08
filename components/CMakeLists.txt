set(bme_srcs "BME280_driver/bme280.c")

idf_component_register(SRCS "${bme_srcs}"
                       INCLUDE_DIRS "BME280_driver/" ".")
