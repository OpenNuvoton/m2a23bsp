create_project(fmc_ld_code ${CMAKE_CURRENT_LIST_DIR}/LDROM_iap.c)

enable_language(C ASM)
create_project(fmc_ap_main ${CMAKE_CURRENT_LIST_DIR}/APROM_main.c ${CMAKE_CURRENT_LIST_DIR}/Keil/ap_image.s)
add_dependencies(fmc_ap_main fmc_ld_code)

