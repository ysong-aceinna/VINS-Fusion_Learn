
set(DIR_LIB_SRCS driver.cpp cv_painter.cpp adapter.cpp simulator.cpp)

if (ENABLE_MYNT_SDK)
    set(DIR_LIB_SRCS ${DIR_LIB_SRCS} mynt_S1030.cpp)
elseif(ENABLE_MYNT_ROS)
    set(DIR_LIB_SRCS ${DIR_LIB_SRCS} mynt_S1030_ros.cpp)
endif()

add_library (driver_lib  ${DIR_LIB_SRCS})
# aux_source_directory(. DIR_LIB_SRCS)   #SHARED
