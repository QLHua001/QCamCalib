cmake_minimum_required(VERSION 3.0.0)
project(QCamCalib VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()

set(CMAKE_BUILD_TYPE Debug)
set(EXECUTABLE_OUTPUT_PATH ../output/)
message("CMAKE_SYSTEM: ${CMAKE_SYSTEM}")
message("CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")

if(CMAKE_HOST_WIN32)

elseif(CMAKE_HOST_UNIX)

    set(OpenCV_DIR "/QMy-File2/QPackage/opencv-4.8.0/output_host_ubuntu2004/")
    include_directories(${OpenCV_DIR}/include/opencv4/)
    link_directories(${OpenCV_DIR}/lib/)
    set(OpenCV_LIBS libopencv_world.so.4.8.0)

endif()

include_directories(./)
set(SRC_QCamCalib
    QCamCalib/QCamCalib.cpp)

set(LIB_LIST ${OpenCV_LIBS})
set(SRC_LIST main.cpp ${SRC_QCamCalib})

add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${LIB_LIST})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
