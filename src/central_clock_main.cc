#include <sstream>
#include "ros/ros.h"
#include "defs.h"
#include "std_msgs/String.h"
#include "distributed_mapf/ClockMsg.h"
#include <string>

using std::cout;
using std::endl;

int main(int argc, char **argv)
{
  
  pid_t pid = getpid();
  std::stringstream ss;
  ss << "central_clock_" << pid;
  std::string node_name = ss.str();
  ros::init(argc, argv, node_name);
  ros::NodeHandle n;

  ros::Rate loop_rate(defs::central_clock_freq);
  unsigned long clock_count = 1;

  ros::Publisher clock_pub = 
      n.advertise<distributed_mapf::ClockMsg>(defs::clock_topic, 1000);
      
  loop_rate.sleep();
  distributed_mapf::ClockMsg clock_msg;
  while (ros::ok()) {
    //cout << "Clock count:" << clock_count << endl;
    clock_msg.clock = clock_count;
    clock_pub.publish(clock_msg);
    clock_count++;
    loop_rate.sleep();

  }
  return 0;
}