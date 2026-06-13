idf_component_register(
    SRCS
        Mono_Bold_12.c
        Mono_Bold_14.c
        Mono_Bold_16.c
        cobalt_alien_17.c
        cobalt_alien_25.c
        RTCLib.cpp
        RTC_DS3231.cpp
        Adafruit_I2CDevice.cpp
        SiderealPlanets.cpp
        SiderealObjects.cpp
        strval.cpp
        main.cpp
        volcanos.c
        arg_parser.cpp
        config.cpp
        custommapping.cpp
        hextodig.cpp
        i2c_helper.cpp
        ins.cpp
        matrix.cpp
        meteors.cpp
        multiplexers.cpp
        multi_display_controller.cpp
        rss_server.cpp
        satio.cpp
        satio_file.cpp
        sdcard_helper.cpp
        serial_infocmd.cpp
        sidereal_helper.cpp
        system_data.cpp
        task_handler.cpp
        wit_c_sdk.cpp
        wt901.cpp
        wtgps300p.cpp
        astroclock.cpp
        satio_lvgl.cpp
        # srtm_reader.cpp
    INCLUDE_DIRS
        .
)

# add_definitions(-DARDUINO_USB_MODE -DARDUINO_USB_CDC_ON_BOOT)

# # Include extra CMake options to relax warnings-as-errors
# include("${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt.extra")

# Disable strict warnings for SiderealPlanets library (Arduino port)
set_source_files_properties(
    SiderealPlanets.cpp
    SiderealObjects.cpp
    PROPERTIES COMPILE_FLAGS 
    "-Wno-dangling-else -Wno-unused-variable -Wno-unused-but-set-variable -Wno-maybe-uninitialized"
)