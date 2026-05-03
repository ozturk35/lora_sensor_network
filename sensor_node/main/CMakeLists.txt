idf_component_register(
    SRCS "main.c"
         "sx1262_driver.c"
         "mq135_driver.c"
         "dht22_driver.c"
         "bmp280_driver.c"
         "payload_encoder.c"
         "power_manager.c"
    INCLUDE_DIRS ".")
