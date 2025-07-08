if(DEFINED CONFIG_SAMPLE_SUPPORT_KERNEL_01_TASK)
    add_subdirectory_if_exist(kernel_01_task)
endif()

if(DEFINED CONFIG_SAMPLE_SUPPORT_MYSTARSEA)
    add_subdirectory_if_exist(mystarsea)
endif()

if(DEFINED CONFIG_SAMPLE_SUPPORT_SLE_CLICENT)
    add_subdirectory_if_exist(sle_client)
endif()

set(SOURCES "${SOURCES}" PARENT_SCOPE)
