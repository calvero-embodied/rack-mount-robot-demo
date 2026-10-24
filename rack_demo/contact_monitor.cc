#include <ignition/transport/Node.hh>
#include <ignition/msgs/contacts.pb.h>
#include <atomic>
#include <csignal>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <chrono>

std::atomic<bool> running{true};
std::mutex mutex;
struct Record { unsigned count=0; double depth=0; };
std::map<std::string,Record> records;
void stop(int) { running=false; }
void receive(const ignition::msgs::Contacts &msg) {
  std::lock_guard<std::mutex> guard(mutex);
  for (auto &c:msg.contact()) {
    auto a=c.collision1().name(), b=c.collision2().name();
    if (b<a) std::swap(a,b);
    // Names are Gazebo-generated identifiers without quotes.
    auto &r=records[a+" | "+b];++r.count;
    for (double d:c.depth()) r.depth=std::max(r.depth,d);
  }
}
int main(int argc,char **argv) {
  if (argc!=3) return 2;
  std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
  ignition::transport::Node node;
  std::ifstream topics(argv[1]);std::string topic;unsigned subscriptions=0;
  while (std::getline(topics,topic)) if (!topic.empty()) {
    if (!node.Subscribe(topic,receive)) return 3;
    ++subscriptions;
  }
  while (running) {
    {
      std::lock_guard<std::mutex> guard(mutex);
      std::ofstream file(std::string(argv[2])+".tmp");
      file<<"{\"subscriptions\":"<<subscriptions<<",\"pairs\":[";
      bool first=true;
      for (auto &entry:records) {
        if (!first) file<<",";first=false;
        file<<"{\"pair\":\""<<entry.first<<"\",\"messages\":"<<entry.second.count
            <<",\"maximum_depth_m\":"<<entry.second.depth<<"}";
      }
      file<<"]}";file.close();std::rename((std::string(argv[2])+".tmp").c_str(),argv[2]);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
}
