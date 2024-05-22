create_project(fmc_ld_code ${CMAKE_CURRENT_LIST_DIR}/LDROM_iap.c)

enable_language(C ASM)
create_project(fmc_ap_main ${CMAKE_CURRENT_LIST_DIR}/APROM_main.c ${CMAKE_CURRENT_LIST_DIR}/GCC/ap_image.S)
target_compile_definitions(fmc_ap_main PUBLIC -DBIN_FILE="${CMAKE_BINARY_DIR}/SampleCode/StdDriver/FMC_IAP/fmc_ld_code.bin")
add_dependencies(fmc_ap_main fmc_ld_code)
