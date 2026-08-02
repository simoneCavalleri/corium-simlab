# CMakeLists.txt for Corium SimLab C++ Samples

add_executable(sample_01_basic_environment 01_basic_environment.cpp)
target_link_libraries(sample_01_basic_environment PRIVATE corium_sim_core)

add_executable(sample_02_robotic_arm_manipulator 02_robotic_arm_manipulator.cpp)
target_link_libraries(sample_02_robotic_arm_manipulator PRIVATE corium_sim_core)

# Copy libwgpu_native.so next to sample executables
add_custom_command(TARGET sample_01_basic_environment POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${wgpu_native_SOURCE_DIR}/libwgpu_native.so"
    "$<TARGET_FILE_DIR:sample_01_basic_environment>"
)

add_custom_command(TARGET sample_02_robotic_arm_manipulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${wgpu_native_SOURCE_DIR}/libwgpu_native.so"
    "$<TARGET_FILE_DIR:sample_02_robotic_arm_manipulator>"
)
