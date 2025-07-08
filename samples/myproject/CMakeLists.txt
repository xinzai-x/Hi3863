if(DEFINED CONFIG_SAMPLE_SUPPORT_KERNEL_01_TASK)
    add_subdirectory_if_exist(kernel_01_task)
endif()

if(DEFINED CONFIG_SAMPLE_SUPPORT_BASE_02_UART)
    add_subdirectory_if_exist(base_02_uart)
endif()

if(DEFINED CONFIG_SAMPLE_SUPPORT_BASE_01_LEDBLINK)
    add_subdirectory_if_exist(base_01_ledblink)
endif()

if(DEFINED CONFIG_SAMPLE_SUPPORT_BASE_04_RGB)
    add_subdirectory_if_exist(base_04_rgb)
endif()

if(DEFINED CONFIG_SAMPLE_SUPPORT_WIFI_01_STA)
    add_subdirectory_if_exist(wifi_01_sta)
endif()

if(DEFINED CONFIG_SAMPLE_SUPPORT_WIFI_05_MQTT)
    add_subdirectory_if_exist(wifi_05_mqtt)
endif()

if(DEFINED CONFIG_SAMPLE_SUPPORT_TEST)
    add_subdirectory_if_exist(test)
endif()

set(SOURCES "${SOURCES}" PARENT_SCOPE)