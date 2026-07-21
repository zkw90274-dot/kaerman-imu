idf_component_register(SRCS "hello_world_main.c"
                            "hw_spi.c"
                            "icm42688.c"
                            "imu_math.c"
                            "ahrs.c"
                            "kalman.c"
                            "imu_quaternion.c"
                    PRIV_REQUIRES spi_flash esp_driver_gpio esp_driver_spi
                    INCLUDE_DIRS ".")