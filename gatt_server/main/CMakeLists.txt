file(GLOB_RECURSE srcs "src/*.c")

idf_component_register(SRCS "main.c" "main-not-working.c" "main - Copy.c" "${srcs}"
                       INCLUDE_DIRS "./include")
