cmake_minimum_required(VERSION 3.5)
project(acados_ocp_solver LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)

# set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

include_directories($ENV{ACADOS_SOURCE_DIR}/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/hpipm/include)
include_directories($ENV{ACADOS_SOURCE_DIR}/include/acados)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code)

# 载入ACADOS默认库
link_directories($ENV{ACADOS_SOURCE_DIR}/lib)

file(GLOB ocp_solver
        ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/acados_solver_path_smoother.c
        )

file(GLOB casadi_fun
        ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/path_smoother_model/*.c)

file(GLOB sim_solver
        ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/acados_sim_solver_path_smoother.c
        )

# 将原来C代码设置为库
add_library(ocp_shared_lib SHARED ${ocp_solver} ${casadi_fun} )
target_link_libraries(ocp_shared_lib acados hpipm blasfeo)
## 这里使用到仿真器，如果在实际实现的时候这部分就不需要了
add_library(sim_shared_lib SHARED ${sim_solver} ${casadi_fun})
target_link_libraries(sim_shared_lib acados hpipm blasfeo)

# 编译自己的例子
add_executable(${PROJECT_NAME}_main ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code/main_path_smoother.c)
target_link_libraries(${PROJECT_NAME}_main ocp_shared_lib)

include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/thirdparty/nlohmann_json
        )
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC)
add_library(${PROJECT_NAME} SHARED ${SRC})
target_link_libraries(${PROJECT_NAME} ocp_shared_lib)

add_executable(main test/main.cpp)
target_link_libraries(main PRIVATE ${PROJECT_NAME})