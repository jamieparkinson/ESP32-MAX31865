idf_component_register(SRCS "src/Max31865.cpp" "src/numerics.cpp"
        INCLUDE_DIRS "include")
target_compile_options(${COMPONENT_LIB} PRIVATE -std=c++11)
