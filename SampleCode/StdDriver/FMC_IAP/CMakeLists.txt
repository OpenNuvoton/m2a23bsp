set(PRJ FMC_IAP)

create_project(LDROM_iap ${CMAKE_CURRENT_LIST_DIR}/LDROM_iap.c)

enable_language(C ASM)
create_project(${PRJ}_ap ${CMAKE_CURRENT_LIST_DIR}/APROM_main.c ${CMAKE_CURRENT_LIST_DIR}/GCC/ap_image.S)
target_include_directories(${PRJ}_ap PUBLIC ${CMAKE_BINARY_DIR}/SampleCode/StdDriver/FMC_IAP)
add_dependencies(${PRJ}_ap LDROM_iap)
