file(GLOB_RECURSE srcs "src/*.c")

idf_component_register(SRCS "main.c" "${srcs}"
                       INCLUDE_DIRS "./include")
