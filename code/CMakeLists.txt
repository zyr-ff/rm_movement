cmake_minimum_required(VERSION 3.16)
project(rm_ekf_third CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===================== OpenCV =====================
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# ===================== ONNX Runtime =====================
find_path(ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime/onnxruntime_cxx_api.h
          onnxruntime/core/session/onnxruntime_cxx_api.h
    HINTS ${ONNXRUNTIME_DIR} /ucrt64/include /mingw64/include $ENV{ONNXRUNTIME_DIR}/include)
find_library(ONNXRUNTIME_LIBRARY
    NAMES onnxruntime
    HINTS ${ONNXRUNTIME_DIR} /ucrt64/lib /mingw64/lib $ENV{ONNXRUNTIME_DIR}/lib)
if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
    message(STATUS "ONNX Runtime found: ${ONNXRUNTIME_LIBRARY}")
    include_directories(${ONNXRUNTIME_INCLUDE_DIR})
endif()

# ===================== 目录 =====================
set(COMMON_INC ${CMAKE_SOURCE_DIR}/common/include)
set(TRAD_INC   ${CMAKE_SOURCE_DIR}/traditional/include)
set(NN_INC     ${CMAKE_SOURCE_DIR}/neural/include)
set(EKF_INC    ${CMAKE_SOURCE_DIR}/ekf)
# 让 #include "ekf/ekf.h" 这种按子目录的写法也能解析（源码根加入 include 路径）
set(SRC_ROOT    ${CMAKE_SOURCE_DIR})

# 第二周独立演示 common/src/main.cpp 自带 main()，与第三周 main_ekf.cpp 冲突，
# 故仅把可复用的 pnp_solver.cpp 编译进第三周主程序。
set(COMMON_SRC common/src/pnp_solver.cpp)
set(TRAD_SRC   traditional/src/armor_detector.cpp
               traditional/src/number_classifier.cpp)
set(NN_SRC     neural/src/neural_detector.cpp)
set(EKF_SRC    ekf/ekf.cpp
               ekf/observer_model.cpp
               ekf/imu_sync.cpp)

# ===================== 第三周主程序：EKF 整车状态估计 =====================
# 复用第二周 检测+数字识别+PnP，叠加第三周 IMU 同步 + EKF 预测/更新(匹配/门控)。
add_executable(rm_ekf_third
    main_ekf.cpp
    ${EKF_SRC}
    ${COMMON_SRC}
    ${TRAD_SRC}
    ${NN_SRC})
target_include_directories(rm_ekf_third PRIVATE
    ${COMMON_INC} ${TRAD_INC} ${NN_INC} ${EKF_INC} ${SRC_ROOT})
target_link_libraries(rm_ekf_third ${OpenCV_LIBS})
if(ONNXRUNTIME_LIBRARY)
    target_link_libraries(rm_ekf_third ${ONNXRUNTIME_LIBRARY})
endif()

# ===================== 纯数学自测（不需要 OpenCV/相机，验证 EKF 逻辑）=====================
# 用法: g++ -std=c++17 -Iekf ekf/test_ekf.cpp ekf/observer_model.cpp ekf/ekf.cpp -o test_ekf
# 已在交付前验证通过（收敛误差 < 1cm，野值被正确门控）。

# ===================== 运行时 DLL 自动拷贝（避开 System32 旧版 ORT）=====================
if(ONNXRUNTIME_LIBRARY)
    get_filename_component(ORT_LIB_DIR ${ONNXRUNTIME_LIBRARY} DIRECTORY)
    set(ORT_DLLS)
    foreach(d ${ORT_LIB_DIR}/../bin/onnxruntime.dll
              ${ORT_LIB_DIR}/../bin/onnxruntime_providers_shared.dll)
        if(EXISTS ${d})
            list(APPEND ORT_DLLS ${d})
        endif()
    endforeach()
    if(ORT_DLLS)
        add_custom_command(TARGET rm_ekf_third POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${ORT_DLLS} $<TARGET_FILE_DIR:rm_ekf_third>
            COMMENT "拷贝 onnxruntime DLL 到 rm_ekf_third 旁(避开 System32 旧版)")
    endif()
endif()
