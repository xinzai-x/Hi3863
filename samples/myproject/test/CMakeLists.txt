set(SOURCES_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_server_adv/sle_server.c
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_server_adv/sle_server_adv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/main.c
)


set(HEADER_LIST
    ${CMAKE_CURRENT_SOURCE_DIR}/sle_server_adv
)

set(SOURCES "${SOURCES}" ${SOURCES_LIST} PARENT_SCOPE)
