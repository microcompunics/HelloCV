cmake_minimum_required(VERSION 3.6)

set(CMAKE_VERBOSE_MAKEFILE on)
set(libs "${CMAKE_SOURCE_DIR}/../../Common/OpenCV3.2_Android_SDK/native/jniLibs")
include_directories(${CMAKE_SOURCE_DIR}/../../Common/OpenCV3.2_Android_SDK/native/include)

add_library(libopencv_java3 SHARED IMPORTED )
set_target_properties(libopencv_java3 PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_java3.so")

add_library(libopencv_java SHARED IMPORTED )
set_target_properties(libopencv_java PROPERTIES
    IMPORTED_LOCATION "${libs}/${ANDROID_ABI}/libopencv_java.so")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fexceptions -frtti")

SET(SOURCE
        src/main/cpp/src/CCNF_patch_expert.cpp
        src/main/cpp/src/CLM.cpp
        src/main/cpp/src/CLM_utils.cpp
        src/main/cpp/src/CLMTracker.cpp
        src/main/cpp/src/DetectionValidator.cpp
        src/main/cpp/src/Patch_experts.cpp
        src/main/cpp/src/PAW.cpp
        src/main/cpp/src/PDM.cpp
        src/main/cpp/src/SVR_patch_expert.cpp
        src/main/cpp/src/common.cpp
        )

SET(HEADERS
        src/main/cpp/include/CCNF_patch_expert.h
        src/main/cpp/include/CLM.h
        src/main/cpp/include/CLM_utils.h
        src/main/cpp/include/CLMParameters.h
        src/main/cpp/include/CLMTracker.h
        src/main/cpp/include/DetectionValidator.h
        src/main/cpp/include/Patch_experts.h
        src/main/cpp/include/PAW.h
        src/main/cpp/include/PDM.h
        src/main/cpp/include/SVR_patch_expert.h
        src/main/cpp/include/common.h
        )

#add_library(dlib src/main/cpp/include/dlib/all/source.cpp)
add_library(CLMLib ${SOURCE} ${HEADERS})

add_library( # Sets the name of the library.
             native-lib

             # Sets the library as a shared library.
             SHARED

             # Provides a relative path to your source file(s).
             # Associated headers in the same location as their source
             # file are automatically included.
             src/main/cpp/native-lib.cpp )

find_library( # Sets the name of the path variable.
              log-lib

              # Specifies the name of the NDK library that
              # you want CMake to locate.
              log)

target_link_libraries(native-lib android log
    libopencv_java3 libopencv_java
    ${log-lib}
    CLMLib
    #dlib
    )

