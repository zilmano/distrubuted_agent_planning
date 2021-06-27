#include "ros/ros.h"
//#include "std_msgs/String.h"

#include <sstream>
#include <sys/types.h>
#include <unistd.h>

#include "distributed_mapf/Vertex.h"
#include "distributed_mapf/PathMsg.h"
#include "monitor.h"  

monitor::Monitor* monitor_;

ros::Subscriber plan_sub_;
ros::Subscriber clock_sub_;
ros::Subscriber reg_sub_;

//ros::Publisher visualization_pub_;
//amrl_msgs::VisualizationMsg map_viz_msg_;


void planCallback(const distributed_mapf::PathMsg& msg) {
    monitor_->PlanMsgCallback(msg);
}

void clockCallback(const distributed_mapf::ClockMsg& clkmsg) {
    monitor_->ClockMsgCallback(clkmsg);
}

void registerCallback(const distributed_mapf::RegMsg& msg) {
    monitor_->RegisterMsgCallback(msg);
}


void initComm(ros::NodeHandle& n) {
  plan_sub_ = n.subscribe(defs::plan_topic, 1000, planCallback);
  clock_sub_= n.subscribe(defs::clock_topic, 1000, clockCallback);
  reg_sub_= n.subscribe(defs::register_topic, 1000, registerCallback);
  monitor_->InitPublishers();
}

int main(int argc, char **argv)
{
  
  //pid_t pid = 15;
  std::stringstream ss;
  ss << "monitor";
  std::string node_name = ss.str();

  ros::init(argc, argv, node_name);
  ros::NodeHandle n;

  monitor_ = new monitor::Monitor(&n);
  initComm(n);
  
  ros::Rate loop_rate(defs::central_clock_freq);
  while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
  }
  
  return 0;
}
