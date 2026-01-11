idf_component_register(SRCS "src/prettyprinter.c" "src/dataloss.c" "src/nvs_manager.c" "main.c"
                    PRIV_REQUIRES bt nvs_flash
                    INCLUDE_DIRS "."
                    REQUIRES driver)
