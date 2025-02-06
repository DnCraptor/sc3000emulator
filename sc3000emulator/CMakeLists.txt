add_executable(sc3000emulator)

# by default the header is generated into the build dir
pico_generate_pio_header(sc3000emulator ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(sc3000emulator ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(sc3000emulator ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)
# however, alternatively you can choose to generate it somewhere else (in this case in the source tree for check in)
#pico_generate_pio_header(pio_blink ${CMAKE_CURRENT_LIST_DIR}/blink.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

# Make sure TinyUSB can find tusb_config.h
target_include_directories(sc3000emulator PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_sources(sc3000emulator PRIVATE sc3000emulator.c Z80.c hid_app.c joystick.c hidparser/hidparser.c vga16_graphics.c lfs.c lfs_util.c tms9918/vrEmuTms9918.c tms9918/vrEmuTms9918Util.c)

pico_enable_stdio_usb(sc3000emulator 0)
pico_enable_stdio_uart(sc3000emulator 1)

set(Z80_SHARED_LIBS                 NO  CACHE BOOL "")
set(Z80_WITH_Q                      YES CACHE BOOL "")
set(Z80_WITH_ZILOG_NMOS_LD_A_IR_BUG YES CACHE BOOL "")
set(Z80_USE_LOCAL_HEADER            YES CACHE BOOL "")

target_link_libraries(sc3000emulator PRIVATE pico_stdlib hardware_pio hardware_timer hardware_dma hardware_uart hardware_flash pico_multicore hardware_pwm hardware_vreg tinyusb_host tinyusb_board)
pico_add_extra_outputs(sc3000emulator)
