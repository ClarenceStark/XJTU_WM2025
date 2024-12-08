    cmake_minimum_required(VERSION 3.0.0)
    set(CMAKE_CXX_STANDARD 17)
    set(TARGET_NAME infantry_new)
    project(${TARGET_NAME})
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    # 开启多线程
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
    SET(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -pthread")
    find_package(OpenVINO REQUIRED COMPONENTS Runtime)
    # find_package(Ceres REQUIRED)
    set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
    find_package(OpenCV 4 QUIET)
    # aim debug
    # add_definitions(-DPOSESTIMATION)
    add_definitions(-DSHOW_FPS)
    # add_definitions(-DDEBUGMODE)
    # add_definitions(-DDEBUGCOLOR)
    # add_definitions(-DDEBUGNUM)
    add_definitions(-DVIRTUALGRAB)  # 打开虚拟取流 即用视频跑
    # add_definitions(-DUSETRIGGERMODE)
   add_definitions(-DNOPORT)    
    
    # add_definitions(-DTHREADANALYSIS)
    # 使用UI动态调参
   add_definitions(-DDEBUGHIT)

    add_definitions(-DSENDCAMERA)
    # add_definitions(-DRECORDPIG)
    # 使用神经网络识别
    add_definitions(-DUSEWMNET)
    # 记录视频
    # add_definitions(-DRECORDVIDEO)
    include_directories(
        /opt/MVS/include
        armor/include
        camera/include
        params/include
        serialPort/include
        # windmill/include
        ${CERES_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${OpenVINO_INCLUDE_DIRS}
        /usr/local/cuda/include
        )
    link_directories(
        /opt/MVS/lib/64
        /opt/MVS/lib/32
        /usr/local/lib
        )
    add_subdirectory(camera)
    add_subdirectory(params)
    add_subdirectory(serialPort)
    # add_subdirectory(windmill)
    add_subdirectory(armor)
    add_executable(infantry_new main.cpp)
    target_link_libraries(infantry_new
        params
        camera
        serialPort
        # windmill
        armor
        ${CERES_LIBRARIES}
        ${InferenceEngine_LIBRARIES}
        ${OpenCV_LIBS}
        MvCameraControl
        libMvCameraControl.so
        libglog.so
        openvino::runtime
        )
    set(CPACK_PROJECT_NAME ${PROJECT_NAME})
    set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
    include(CPack)
