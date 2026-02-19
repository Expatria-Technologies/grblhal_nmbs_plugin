add_library(nanomodbus INTERFACE)

target_sources(nanomodbus INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/nanomodbus.c
  ${CMAKE_CURRENT_LIST_DIR}/plugin_nmbs.c
)

target_include_directories(nmbs INTERFACE ${CMAKE_CURRENT_LIST_DIR})