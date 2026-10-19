// Synthetic feedback only. This executable does not link or open libfranka.
#include "polymetis/utils.h"
#include "real_time.hpp"
#include <grpcpp/grpcpp.h>
#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <thread>

int64_t monotonic_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct Args { std::string endpoint, metadata, output, feedback; int seconds; int failures = 0; };
struct Sample { int64_t start, received, computed, end; int code, servo; bool reference; };
void* run(void* input) {
  const auto& args = *static_cast<Args*>(input);
  auto stub = PolymetisControllerServer::NewStub(grpc::CreateChannel(args.endpoint, grpc::InsecureChannelCredentials()));
  RobotClientMetadata metadata;
  std::ifstream metadata_input(args.metadata, std::ios::binary);
  if (!metadata.ParseFromIstream(&metadata_input)) throw std::runtime_error("Invalid metadata");
  std::ifstream q_input(args.feedback);
  std::vector<std::array<float,14>> feedback;
  std::array<float,14> row;
  while (q_input >> row[0]) {
    for (int i=1; i<14; ++i)
      if (!(q_input >> row[i])) throw std::runtime_error("Incomplete feedback row");
    for (auto value : row)
      if (!std::isfinite(value)) throw std::runtime_error("Nonfinite feedback");
    feedback.push_back(row);
  }
  if (feedback.empty()) throw std::runtime_error("Feedback required");
  Empty empty;
  grpc::ClientContext init;
  init.set_deadline(std::chrono::system_clock::now()+std::chrono::seconds(3));
  auto initialized = stub->InitRobotClient(&init, metadata, &empty);
  if (!initialized.ok()) throw std::runtime_error(initialized.error_message());
  std::vector<Sample> samples(args.seconds * 1000);
  RobotState state;
  TorqueCommand torque;
  for (int i=0; i<7; ++i) {
    state.add_joint_positions(feedback[0][i]); state.add_joint_velocities(0);
    state.add_motor_torques_measured(0); state.add_motor_torques_external(0);
  }
  std::cout << "synthetic_client_ready" << std::endl;
  auto due = std::chrono::steady_clock::now();
  int failures=0, previous_servo=0;
  for (std::size_t i=0; i<samples.size(); ++i) {
    const auto& q = feedback[(i/33) % feedback.size()];
    for (int j=0; j<7; ++j) {
      state.set_joint_positions(j,q[j]); state.set_joint_velocities(j,q[7+j]);
    }
    setTimestampToNow(state.mutable_timestamp());
    grpc::ClientContext context;
    context.set_deadline(std::chrono::system_clock::now()+std::chrono::milliseconds(10));
    auto& sample=samples[i];
    torque.Clear();
    sample.start=monotonic_ns();
    state.set_native_request_monotonic_ns(sample.start);
    auto status=stub->ControlUpdate(&context,state,&torque);
    sample.end=monotonic_ns();
    sample.code=status.error_code();
    sample.servo=torque.servo_status();
    sample.reference=torque.has_servo_reference();
    if (sample.servo!=previous_servo) {
      std::cout<<"servo_status "<<i<<" "<<sample.servo<<std::endl;
      previous_servo=sample.servo;
    }
    sample.received=torque.server_received_monotonic_ns();
    sample.computed=torque.server_computed_monotonic_ns();
    if (!status.ok()) {
      ++failures;
      // Read the server-side copy after the deadline to locate the stall.
      sample.received=sample.computed=0;
      for (int retry=0; retry<30; ++retry) {
        grpc::ClientContext diagnostic;
        diagnostic.set_deadline(std::chrono::system_clock::now()+std::chrono::milliseconds(100));
        RobotState last;
        if (stub->GetRobotState(&diagnostic,empty,&last).ok() && last.native_request_monotonic_ns()==sample.start) {
          sample.received=last.server_received_monotonic_ns();
          sample.computed=last.server_computed_monotonic_ns();
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      std::cout << "deadline " << i << " input_us " << (sample.received ? (sample.received-sample.start)/1000. : -1)
                << " compute_us " << (sample.computed ? (sample.computed-sample.received)/1000. : -1)
                << " roundtrip_us " << (sample.end-sample.start)/1000. << std::endl;
    }
    due+=std::chrono::milliseconds(1);
    if (due<std::chrono::steady_clock::now()) due=std::chrono::steady_clock::now();
    std::this_thread::sleep_until(due);
  }
  std::ofstream out(args.output);
  out<<"start,received,computed,end,grpc_code,servo_status,has_reference\n";
  for (const auto& s:samples) out<<s.start<<','<<s.received<<','<<s.computed<<','<<s.end<<','<<s.code<<','<<s.servo<<','<<s.reference<<'\n';
  static_cast<Args*>(input)->failures=failures;
  std::cout<<"completed "<<samples.size()<<" failures "<<failures<<std::endl;
  return nullptr;
}
int main(int argc,char** argv) {
  if (argc!=6) { std::cerr<<"probe endpoint metadata feedback-text output-csv seconds\n";return 2; }
  Args args{argv[1],argv[2],argv[4],argv[3],std::stoi(argv[5])};
  if (args.seconds < 1 || args.seconds > 3600) return 2;
  create_real_time_thread(run,&args);
  return args.failures ? 1 : 0;
}
