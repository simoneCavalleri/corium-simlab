# CMakeLists.txt for Corium SimLab C++ Samples

add_executable(sample_01_physical_agent_arena 01_physical_agent_arena.cpp)
target_link_libraries(sample_01_physical_agent_arena PRIVATE corium_sim_core)

add_executable(sample_02_robotic_arm_manipulator 02_robotic_arm_manipulator.cpp)
target_link_libraries(sample_02_robotic_arm_manipulator PRIVATE corium_sim_core)

add_executable(sample_03_multi_agent_arena 03_multi_agent_arena.cpp)
target_link_libraries(sample_03_multi_agent_arena PRIVATE corium_sim_core)

add_executable(sample_04_service_communication_arena 04_service_communication_arena.cpp)
target_link_libraries(sample_04_service_communication_arena PRIVATE corium_sim_core)

# Copy libwgpu_native.so next to sample executables
add_custom_command(TARGET sample_01_physical_agent_arena POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${wgpu_native_SOURCE_DIR}/libwgpu_native.so"
    "$<TARGET_FILE_DIR:sample_01_physical_agent_arena>"
)

add_custom_command(TARGET sample_02_robotic_arm_manipulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${wgpu_native_SOURCE_DIR}/libwgpu_native.so"
    "$<TARGET_FILE_DIR:sample_02_robotic_arm_manipulator>"
)

add_custom_command(TARGET sample_03_multi_agent_arena POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${wgpu_native_SOURCE_DIR}/libwgpu_native.so"
    "$<TARGET_FILE_DIR:sample_03_multi_agent_arena>"
)

add_custom_command(TARGET sample_04_service_communication_arena POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${wgpu_native_SOURCE_DIR}/libwgpu_native.so"
    "$<TARGET_FILE_DIR:sample_04_service_communication_arena>"
)

