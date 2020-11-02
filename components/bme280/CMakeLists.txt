set(bme_srcs "driver/bme280.c")

idf_component_register(SRCS "${bme_srcs}"
                       INCLUDE_DIRS "driver" ".")
