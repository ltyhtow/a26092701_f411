host_test(test_parameter_service test_parameter_service.c "${PROJECT_ROOT}/app/src/parameter_service.c")
target_include_directories(test_parameter_service PRIVATE stubs_parameter_service
    "${PROJECT_ROOT}/app/include" "${OS}/include" "${PROJECT_ROOT}/platform/include")
target_link_libraries(test_parameter_service PRIVATE parameters_core serial_codec)
foreach(case disabled_ram boot_load save_failures unsafe queue_fail_1 queue_fail_2 task_fail)
    add_test(NAME parameter_service_${case} COMMAND test_parameter_service ${case})
endforeach()
