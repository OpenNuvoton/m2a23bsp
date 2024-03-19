create_project(ISP_CAN
    ${CMAKE_CURRENT_LIST_DIR}/ISP_CAN/main.c
)

create_project(ISP_RS485
    ${CMAKE_CURRENT_LIST_DIR}/ISP_RS485/main.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_RS485/fmc_user.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_RS485/isp_user.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_RS485/targetdev.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_RS485/uart_transfer.c
)

create_project(ISP_SPI
    ${CMAKE_CURRENT_LIST_DIR}/ISP_SPI/main.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_SPI/fmc_user.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_SPI/isp_user.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_SPI/targetdev.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_SPI/spi_transfer.c
)

create_project(ISP_UART
    ${CMAKE_CURRENT_LIST_DIR}/ISP_UART/main.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_UART/fmc_user.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_UART/isp_user.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_UART/targetdev.c
    ${CMAKE_CURRENT_LIST_DIR}/ISP_UART/uart_transfer.c
)
