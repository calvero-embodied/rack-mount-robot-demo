#include <ignition/transport/Node.hh>
#include <ignition/msgs/empty.pb.h>
#include <chrono>
#include <thread>
#include <iostream>

int main(int argc, char **argv) {
  if (argc != 2) return 2;
  ignition::transport::Node node;
  auto pub = node.Advertise<ignition::msgs::Empty>(argv[1]);
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!pub.HasConnections() && std::chrono::steady_clock::now() < end)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  if (!pub.HasConnections()) { std::cerr << "No detach subscriber\n"; return 3; }
  ignition::msgs::Empty msg;
  if (!pub.Publish(msg)) return 4;
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
}
