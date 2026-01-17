cmake_minimum_required(VERSION 3.20)

# Basic pipeline example
add_executable(basic_pipeline basic_pipeline.cpp)
target_link_libraries(basic_pipeline PRIVATE aura_core)

# Sensor simulation example
add_executable(sensor_simulation sensor_simulation.cpp)
target_link_libraries(sensor_simulation PRIVATE aura_core)
