# Copyright (c) Facebook, Inc. and its affiliates.
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
cmake_minimum_required(VERSION 3.14)
project(torch_isolation)

add_subdirectory("pinocchio_isolation/")



# Respect an explicit Torch_DIR and interpreter in isolated builds.
if(NOT Torch_DIR)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    execute_process(COMMAND "${Python3_EXECUTABLE}" -c
        "import torch; print(torch.utils.cmake_prefix_path + '/Torch')"
        OUTPUT_VARIABLE Torch_DIR OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
endif()
find_package(Torch REQUIRED)

# Find Eigen
find_package(Eigen3 REQUIRED)

include_directories(
    "include"
    "${TORCH_INCLUDE_DIRS}"
    "${EIGEN3_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/pinocchio_isolation/include"
)

add_library(torch_server_ops SHARED src/torch_server_ops.cpp)
set_target_properties(torch_server_ops PROPERTIES CXX_VISIBILITY_PRESET hidden)
target_link_libraries(torch_server_ops PRIVATE
    "${TORCH_LIBRARIES}"
)

add_library(torchrot SHARED src/rotations.cpp)
set_target_properties(torchrot PROPERTIES CXX_VISIBILITY_PRESET hidden)
target_link_libraries(torchrot PRIVATE
    "${TORCH_LIBRARIES}"
)

add_library(torchscript_pinocchio SHARED src/pinocchio.cpp)
set_target_properties(torchscript_pinocchio PROPERTIES CXX_VISIBILITY_PRESET hidden)
target_link_libraries(torchscript_pinocchio PRIVATE
    pinocchio_wrapper
    "${TORCH_LIBRARIES}"
    torchrot
)

foreach(target torch_server_ops torchrot torchscript_pinocchio)
    target_compile_definitions(${target} PRIVATE _GLIBCXX_USE_CXX11_ABI=0)
    install(TARGETS ${target} LIBRARY DESTINATION lib)
endforeach()
