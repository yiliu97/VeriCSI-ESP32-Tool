# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

# v2.0: shared sdkconfig.defaults files live in the workspace-level
# `sdkconfig/` directory. Both the common defaults and the per-target
# overrides are loaded; ${IDF_TARGET} is expanded by the IDF build system.
set(SDKCONFIG_DEFAULTS
        "${CMAKE_CURRENT_LIST_DIR}/../sdkconfig/sdkconfig.defaults"
        "${CMAKE_CURRENT_LIST_DIR}/../sdkconfig/sdkconfig.defaults.${IDF_TARGET}")

# UART console fallback for the USJ-primary targets (C3/S3/C6/C61):
# `idf.py -DUART_CONSOLE=1 reconfigure` appends the override that routes the
# primary console to UART0 at 921600 baud (docs/sink_guide.md). The cache
# variable persists across builds; -DUART_CONSOLE=0 reconfigure restores the
# per-target defaults.
if(UART_CONSOLE)
    list(APPEND SDKCONFIG_DEFAULTS
            "${CMAKE_CURRENT_LIST_DIR}/../sdkconfig/sdkconfig.uart_console")
endif()

# Note on the ESP32-C5 USJ-console breakage (docs/known_issues.md): excluding
# esp_driver_usb_serial_jtag via EXCLUDE_COMPONENTS does NOT work - `vfs`
# hard-PRIV_REQUIRES it (vfs/CMakeLists.txt:17, IDF-13984) and dependency
# resolution silently re-adds it. The actual fix lives in
# sdkconfig/sdkconfig.defaults.esp32c5 (UART primary + USJ secondary console).
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(active-ap)

# Console/port consistency check, run automatically before every flash: warns
# when the configured primary console will not produce output on the port
# being flashed (python_utils/check_console_port.py, diagnostic only).
idf_build_get_property(python PYTHON)
idf_build_get_property(sdkconfig_path SDKCONFIG)
add_custom_target(console_port_check
        COMMAND ${python} "${CMAKE_CURRENT_LIST_DIR}/../python_utils/check_console_port.py"
                --sdkconfig "${sdkconfig_path}"
        USES_TERMINAL)
foreach(flash_target flash app-flash encrypted-flash)
    if(TARGET ${flash_target})
        add_dependencies(${flash_target} console_port_check)
    endif()
endforeach()
