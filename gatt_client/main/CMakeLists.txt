idf_component_register(SRCS "main.c" "src/prettyprinter.c" "src/dataloss.c"
                    PRIV_REQUIRES bt nvs_flash
                    INCLUDE_DIRS "."
                    REQUIRES driver)
