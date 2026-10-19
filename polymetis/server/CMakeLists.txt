cmake_minimum_required(VERSION 3.22)
project(polymetis_server_runtime LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(POLYMETIS_DEPS_PREFIX "" CACHE PATH "Legacy server dependency prefix")
set(POLYMETIS_SERVO_LIBRARY "" CACHE FILEPATH "Verified native Torch joint servo library")
if(NOT IS_DIRECTORY "${POLYMETIS_DEPS_PREFIX}/include" OR NOT EXISTS "${POLYMETIS_SERVO_LIBRARY}")
  message(FATAL_ERROR "Set the complete dependency prefix and built servo library")
endif()
list(PREPEND CMAKE_PREFIX_PATH "${POLYMETIS_DEPS_PREFIX}")
set(CMAKE_BUILD_RPATH "${POLYMETIS_DEPS_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib")
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
add_library(protobuf::libprotobuf SHARED IMPORTED)
set_target_properties(protobuf::libprotobuf PROPERTIES
  IMPORTED_LOCATION "${POLYMETIS_DEPS_PREFIX}/lib/libprotobuf.so"
  INTERFACE_INCLUDE_DIRECTORIES "${POLYMETIS_DEPS_PREFIX}/include")
find_package(gRPC CONFIG REQUIRED PATHS "${POLYMETIS_DEPS_PREFIX}/lib/cmake/grpc" NO_DEFAULT_PATH)
set(POLYMETIS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(PROTO "${POLYMETIS_ROOT}/protos/polymetis.proto")
add_custom_command(
  OUTPUT polymetis.pb.cc polymetis.pb.h polymetis.grpc.pb.cc polymetis.grpc.pb.h
  COMMAND "${POLYMETIS_DEPS_PREFIX}/bin/protoc"
    --python_out "${CMAKE_CURRENT_BINARY_DIR}"
    --grpc_python_out "${CMAKE_CURRENT_BINARY_DIR}"
    "--plugin=protoc-gen-grpc_python=${POLYMETIS_DEPS_PREFIX}/bin/grpc_python_plugin"
    --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
    --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
    "--plugin=protoc-gen-grpc=${POLYMETIS_DEPS_PREFIX}/bin/grpc_cpp_plugin"
    -I "${POLYMETIS_ROOT}/protos" -I "${POLYMETIS_DEPS_PREFIX}/include" "${PROTO}"
  DEPENDS "${PROTO}"
  VERBATIM)
add_library(runtime_proto STATIC polymetis.pb.cc polymetis.grpc.pb.cc)
target_link_libraries(runtime_proto PUBLIC gRPC::grpc++ protobuf::libprotobuf)
target_include_directories(runtime_proto PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")


add_subdirectory("${POLYMETIS_ROOT}/torch_isolation" torch_isolation)
add_library(controller_server "${POLYMETIS_ROOT}/src/polymetis_server.cpp")
target_include_directories(controller_server PUBLIC "${POLYMETIS_ROOT}/include" "${POLYMETIS_ROOT}/torch_isolation/include")
target_link_libraries(controller_server PUBLIC runtime_proto yaml-cpp spdlog::spdlog
  -Wl,--no-as-needed torchscript_pinocchio torchrot torch_server_ops "${POLYMETIS_SERVO_LIBRARY}")
add_executable(run_server "${POLYMETIS_ROOT}/src/run_server.cpp")
target_link_libraries(run_server PRIVATE controller_server pthread)
install(TARGETS run_server RUNTIME DESTINATION bin)
install(FILES "${POLYMETIS_SERVO_LIBRARY}" DESTINATION lib)
