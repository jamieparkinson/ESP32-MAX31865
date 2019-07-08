idf_component_register(SRCS "Max31865.cpp"
        INCLUDE_DIRS "include")
target_compile_options(${COMPONENT_LIB} PUBLIC -std=c++11)
