cmake_minimum_required(VERSION 3.16)
project(polymetis_franka_runtime LANGUAGES CXX)

# Build hardware clients independently of the legacy Torch/Pinocchio server.
# The dependency prefix is explicit; never write into an activated environment.
set(POLYMETIS_DEPS_PREFIX "" CACHE PATH "Prefix containing gRPC, Protobuf, yaml-cpp and spdlog")
set(POLYMETIS_FRANKA_VERSION "0.18.2" CACHE STRING "Exact libfranka version selected for the robot image")
if(NOT IS_DIRECTORY "${POLYMETIS_DEPS_PREFIX}/include")
  message(FATAL_ERROR "Set POLYMETIS_DEPS_PREFIX to a complete dependency prefix")
endif()
set(POLYMETIS_MODEL_LIBDIR "" CACHE PATH "Directory containing the matching Pinocchio and hpp-fcl libraries")
if(NOT IS_DIRECTORY "${POLYMETIS_MODEL_LIBDIR}")
  message(FATAL_ERROR "Set POLYMETIS_MODEL_LIBDIR to the libfranka model dependency directory")
endif()
set(CMAKE_BUILD_RPATH "${POLYMETIS_MODEL_LIBDIR};${POLYMETIS_DEPS_PREFIX}/lib")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
list(PREPEND CMAKE_PREFIX_PATH "${POLYMETIS_DEPS_PREFIX}")
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
add_library(protobuf::libprotobuf SHARED IMPORTED)
set_target_properties(protobuf::libprotobuf PROPERTIES
  IMPORTED_LOCATION "${POLYMETIS_DEPS_PREFIX}/lib/libprotobuf.so"
  INTERFACE_INCLUDE_DIRECTORIES "${POLYMETIS_DEPS_PREFIX}/include")
find_package(gRPC CONFIG REQUIRED PATHS "${POLYMETIS_DEPS_PREFIX}/lib/cmake/grpc" NO_DEFAULT_PATH)
find_package(Franka ${POLYMETIS_FRANKA_VERSION} EXACT CONFIG REQUIRED)

set(POLYMETIS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(PROTO "${POLYMETIS_ROOT}/protos/polymetis.proto")
add_custom_command(
  OUTPUT polymetis.pb.cc polymetis.pb.h polymetis.grpc.pb.cc polymetis.grpc.pb.h
  COMMAND "${POLYMETIS_DEPS_PREFIX}/bin/protoc"
    --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
    --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
    "--plugin=protoc-gen-grpc=${POLYMETIS_DEPS_PREFIX}/bin/grpc_cpp_plugin"
    -I "${POLYMETIS_ROOT}/protos" -I "${POLYMETIS_DEPS_PREFIX}/include" "${PROTO}"
  DEPENDS "${PROTO}"
  VERBATIM)
add_library(runtime_proto STATIC polymetis.pb.cc polymetis.grpc.pb.cc)
target_link_libraries(runtime_proto PUBLIC gRPC::grpc++ protobuf::libprotobuf)
target_include_directories(runtime_proto PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")

foreach(client franka_panda_client franka_hand_client)
  add_executable(${client} "${POLYMETIS_ROOT}/src/clients/franka_panda_client/${client}.cpp")
  target_include_directories(${client} PRIVATE "${POLYMETIS_ROOT}/include")
  target_link_options(${client} PRIVATE "-Wl,-rpath-link,${POLYMETIS_MODEL_LIBDIR}")
  target_link_libraries(${client} PRIVATE "${POLYMETIS_MODEL_LIBDIR}/libhpp-fcl.so" runtime_proto Franka::Franka Eigen3::Eigen yaml-cpp spdlog::spdlog pthread)
  set_target_properties(${client} PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
  install(TARGETS ${client} RUNTIME DESTINATION bin)
endforeach()

enable_testing()
add_executable(gripper_command_test gripper_command_test.cpp)
target_include_directories(gripper_command_test PRIVATE "${POLYMETIS_ROOT}/include")
target_link_options(gripper_command_test PRIVATE "-Wl,-rpath-link,${POLYMETIS_MODEL_LIBDIR}")
target_link_libraries(gripper_command_test PRIVATE "${POLYMETIS_MODEL_LIBDIR}/libhpp-fcl.so" Franka::Franka spdlog::spdlog pthread)
add_test(NAME gripper_command_failure COMMAND gripper_command_test)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(NAME gripper_readonly_guard COMMAND "${Python3_EXECUTABLE}"
  "${CMAKE_CURRENT_SOURCE_DIR}/check_gripper_readonly.py" "$<TARGET_FILE:franka_hand_client>")
add_executable(runtime_test runtime_test.cpp "${POLYMETIS_ROOT}/src/clients/franka_panda_client/franka_panda_client.cpp")
target_compile_definitions(runtime_test PRIVATE POLYMETIS_CLIENT_NO_MAIN)
target_include_directories(runtime_test PRIVATE "${POLYMETIS_ROOT}/include")
target_link_options(runtime_test PRIVATE "-Wl,-rpath-link,${POLYMETIS_MODEL_LIBDIR}")
target_link_libraries(runtime_test PRIVATE "${POLYMETIS_MODEL_LIBDIR}/libhpp-fcl.so" runtime_proto Franka::Franka Eigen3::Eigen yaml-cpp spdlog::spdlog pthread)
add_test(NAME runtime_test COMMAND runtime_test "${POLYMETIS_ROOT}/conf/robot_client/franka_hardware.yaml" "${CMAKE_CURRENT_BINARY_DIR}/test-flight")
set_tests_properties(runtime_test PROPERTIES FIXTURES_SETUP flight_recording)
add_test(NAME flight_json COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/check_flight_recording.py" "${CMAKE_CURRENT_BINARY_DIR}/test-flight")
set_tests_properties(flight_json PROPERTIES FIXTURES_REQUIRED flight_recording)

add_executable(inspect_franka inspect_franka.cpp)
target_include_directories(inspect_franka PRIVATE "${POLYMETIS_ROOT}/include")
target_link_options(inspect_franka PRIVATE "-Wl,-rpath-link,${POLYMETIS_MODEL_LIBDIR}")
target_link_libraries(inspect_franka PRIVATE "${POLYMETIS_MODEL_LIBDIR}/libhpp-fcl.so" Franka::Franka)
set_target_properties(inspect_franka PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
install(TARGETS inspect_franka RUNTIME DESTINATION bin)

set(FRANKA_MODEL_TEST_URDF "" CACHE FILEPATH "Official libfranka model fixture with accelerometer branches")
if(EXISTS "${FRANKA_MODEL_TEST_URDF}")
  add_executable(model_test model_test.cpp)
  target_include_directories(model_test PRIVATE "${POLYMETIS_ROOT}/include")
  target_link_options(model_test PRIVATE "-Wl,-rpath-link,${POLYMETIS_MODEL_LIBDIR}")
  target_link_libraries(model_test PRIVATE "${POLYMETIS_MODEL_LIBDIR}/libhpp-fcl.so" Franka::Franka Eigen3::Eigen)
  add_test(NAME model_frames COMMAND model_test "${FRANKA_MODEL_TEST_URDF}")
endif()
