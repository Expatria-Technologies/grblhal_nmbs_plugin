add_library(nmbs INTERFACE)

target_sources(nmbs INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/nanomodbus.c
  ${CMAKE_CURRENT_LIST_DIR}/nmbs_plugin.c
)

target_include_directories(nmbs INTERFACE ${CMAKE_CURRENT_LIST_DIR})