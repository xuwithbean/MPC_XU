cmake_minimum_required(VERSION 3.16.3)
project(main)
set(PJT_SRC_DIR ${PROJECT_SOURCE_DIR}/Src)
set(PJT_INC_DIR ${PROJECT_SOURCE_DIR}/Inc)
set(PJT_LIB_DIR ${PROJECT_SOURCE_DIR}/Lib)
set(EIGEN_INC_DIR ${PROJECT_SOURCE_DIR}/eigen-3.4.0)
set(raisim_DIR "/home/xu/Documents/GitHub/raisimLib/raisim/linux/lib/cmake/raisim")
find_package(raisim CONFIG REQUIRED)
aux_source_directory(${PJT_SRC_DIR} SRC_LIST)
include_directories(
    ${PJT_INC_DIR} ${MOTOR_INC_DIR} ${EIGEN_INC_DIR} ${raisim_DIR}
)
link_directories(${PJT_LIB_DIR})
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} PUBLIC
raisim::raisim
pthread
qpOASES
)
target_compile_options(${PROJECT_NAME} PUBLIC "-march=native")
set(SOURCE_FOLDER "${CMAKE_SOURCE_DIR}/rsc")
set(DEST_FOLDER "${CMAKE_BINARY_DIR}/rsc")
file(COPY ${SOURCE_FOLDER} DESTINATION ${CMAKE_BINARY_DIR})

