idf_component_register(SRCS "dataloss.c" "main.c"
                    PRIV_REQUIRES bt nvs_flash
                    INCLUDE_DIRS "."
                    REQUIRES driver)
