#include "polymetis/clients/franka_panda_client.hpp"
#include <filesystem>
#include <limits>

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
template <typename F> void rejects(F action, const char* message) {
  bool caught = false;
  try { action(); } catch (const std::exception&) { caught = true; }
  require(caught, message);
}

class FakeServer final : public PolymetisControllerServer::Service {
 public:
  int mode = 0;
  grpc::Status InitRobotClient(grpc::ServerContext*, const RobotClientMetadata*, Empty*) override {
    return grpc::Status::OK;
  }
  grpc::Status ControlUpdate(grpc::ServerContext*, const RobotState*, TorqueCommand* out) override {
    if (mode == 3) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    for (int i = 0; i < (mode == 1 ? 6 : 7); ++i)
      out->add_joint_torques(mode == 2 ? std::numeric_limits<double>::quiet_NaN() : 0.0);
    return grpc::Status::OK;
  }
};

class FrankaRuntimeTest {
 public:
  static void run(FrankaTorqueControlClient& client, FakeServer& server) {
    std::array<double, 7> torques{}, values{}, lower{}, upper{};
    lower.fill(-1); upper.fill(1); values.fill(2);
    for (int repeat = 0; repeat < 2; ++repeat)
      rejects([&] { client.computeSafetyReflex(values, lower, upper, false, torques, .1, 1, "Joint position"); },
              "repeated hard limit was not enforced");
    values.fill(std::numeric_limits<double>::quiet_NaN());
    rejects([&] { client.computeSafetyReflex(values, lower, upper, false, torques, .1, 1, "Joint position"); },
            "non-finite state accepted");
    franka::RobotState state{};
    client.updateServerCommand(state, torques);
    for (int mode : {1, 2, 3}) {
      server.mode = mode;
      const auto before = std::chrono::steady_clock::now();
      rejects([&] { client.updateServerCommand(state, torques); }, "invalid RPC response accepted");
      if (mode == 3)
        require(std::chrono::steady_clock::now() - before < std::chrono::milliseconds(45), "RPC deadline ignored");
    }
  }
};

int main(int argc, char** argv) {
  require(argc == 3, "runtime_test config output-dir");
  FakeServer service;
  grpc::ServerBuilder builder;
  int port;
  builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
  builder.RegisterService(&service);
  auto server = builder.BuildAndStart();
  auto config = YAML::LoadFile(argv[1])["robot_client"]["executable_cfg"];
  config["mock"] = true;
  config["readonly"] = false;
  config["control_rpc_timeout_ms"] = 5;
  auto directory = std::filesystem::path(argv[2]);
  std::filesystem::remove_all(directory);
  std::filesystem::create_directories(directory);
  RobotClientMetadata metadata;
  metadata.set_dof(7);
  const auto metadata_path = directory / "metadata.bin";
  std::ofstream out(metadata_path, std::ios::binary);
  metadata.SerializeToOstream(&out);
  out.close();
  config["robot_client_metadata_path"] = metadata_path.string();
  FrankaTorqueControlClient client(grpc::CreateChannel("127.0.0.1:" + std::to_string(port),
                                                     grpc::InsecureChannelCredentials()), config);
  FrankaRuntimeTest::run(client, service);
  polymetis_runtime::FlightRecorder recorder(directory.string(), 2);
  franka::RobotState state{};
  std::array<double, 7> torque{};
  for (int i = 1; i <= 3; ++i) {
    state.q[0] = i;
    recorder.record(state, torque, torque, polymetis_runtime::monotonic_ns(), .2, 1, true);
  }
  recorder.dump("test \"quoted\"\nreason");
  rejects([&] { polymetis_runtime::FlightRecorder invalid(directory.string(), 0); }, "zero recorder capacity");
  rejects([&] { polymetis_runtime::FlightRecorder invalid(directory.string(), 20001); }, "unbounded recorder capacity");
  server->Shutdown();
  return 0;
}
