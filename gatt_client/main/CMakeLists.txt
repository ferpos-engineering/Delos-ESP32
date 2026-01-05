idf_component_register(SRCS "prettyprinter.c" "dataloss.c" "main.c"
                    PRIV_REQUIRES bt nvs_flash
                    INCLUDE_DIRS "."
                    REQUIRES driver)
