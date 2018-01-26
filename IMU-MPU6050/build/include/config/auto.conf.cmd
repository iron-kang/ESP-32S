deps_config := \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/components/aws_iot/Kconfig \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/components/bt/Kconfig \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/components/esp32/Kconfig \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/components/ethernet/Kconfig \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/components/fatfs/Kconfig \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/components/freertos/Kconfig \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/components/log/Kconfig \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/components/lwip/Kconfig \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/components/mbedtls/Kconfig \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/components/openssl/Kconfig \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/components/spi_flash/Kconfig \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/components/bootloader/Kconfig.projbuild \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/components/esptool_py/Kconfig.projbuild \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/components/partition_table/Kconfig.projbuild \
	/home/rws205/embedded/esp32/iotbits_esp32/esp-idf/Kconfig

include/config/auto.conf: \
	$(deps_config)


$(deps_config): ;
