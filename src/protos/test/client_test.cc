
#include "protos/client.h"

int main(int argc, char** argv) {
  std::string target_str;
  std::string arg_str("--target");
  if (argc > 1) {
    std::string arg_val = argv[1];
    size_t start_pos = arg_val.find(arg_str);
    if (start_pos != std::string::npos) {
      start_pos += arg_str.size();
      if (arg_val[start_pos] == '=') {
        target_str = arg_val.substr(start_pos + 1);
      } else {
        std::cout << "The only correct argument syntax is --target="
                  << std::endl;
        return 0;
      }
    } else {
      std::cout << "The only acceptable argument is --target=" << std::endl;
      return 0;
    }
  } else {
    target_str = "localhost:50051";
  }
  cartographer::stream::Client client(
      grpc::CreateChannel(target_str, grpc::InsecureChannelCredentials()));
  std::string user("string");
  Eigen::Vector3d position;
  Eigen::Quaterniond bearing;
  cartographer::common::Time time;
  client.GetRelocPose(user, &time, &bearing, &position);
  std::cout << "position is: " << position.transpose()
            << "time is: " << cartographer::common::ToUniversal(time)
            << std::endl;

  return 0;
}