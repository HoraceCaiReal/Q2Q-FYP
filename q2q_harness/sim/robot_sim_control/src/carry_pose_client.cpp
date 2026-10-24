// Persistent Fortress transport client for the explicitly kinematic carry model.
// stdin: x y z; stdout: 1 (service acknowledged true) or 0. No motion planning.
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <iostream>
#include <cmath>
#include <sstream>

int main(int argc, char **argv)
{
  if (argc != 3) return 2;
  ignition::transport::Node node;
  double x, y, z;
  std::string line;
  while (std::getline(std::cin, line)) {
    ignition::msgs::Pose request;
    ignition::msgs::Boolean reply;
    bool result = false;
    if (line == "detach") {
      x = y = z = 0.;
    } else {
      std::istringstream input(line);
      if (!(input >> x >> y >> z)) return 3;
      request.set_name(argv[2]);
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return 3;
    request.mutable_position()->set_x(x);
    request.mutable_position()->set_y(y);
    request.mutable_position()->set_z(z);
    request.mutable_orientation()->set_w(1.0);
    bool executed = node.Request(argv[1], request, 500u, reply, result);
    std::cout << (executed && result && reply.data() ? 1 : 0) << std::endl;
  }
  return 0;
}
