#include "ros/ros.h"
//#include "std_msgs/String.h"

#include <sstream>
#include <sys/types.h>
#include <unistd.h>

#include "distributed_mapf/Vertex.h"
#include "distributed_mapf/PathMsg.h"
#include "agent.h"  

using std::list;

//Agent/Node State
agent::Agent* agent_;

ros::Publisher visualization_pub_;
amrl_msgs::VisualizationMsg map_viz_msg_;

void planCallback(const distributed_mapf::PathMsg& msg) {
    agent_->PlanMsgCallback(msg);
}


void initComm(ros::NodeHandle& n) {
  n.subscribe(plan_topic, 1000, planCallback);
  agent_->InitPublishers();
}

int main(int argc, char **argv)
{
  
  pid_t pid = 15;
  std::stringstream ss;
  ss << "agent_" << pid;
  std::string node_name = ss.str();

  ros::init(argc, argv, node_name);
  ros::NodeHandle n;

  agent::Params params;
  params.plan_grid_pitch = 1;
  params.plan_x_start = -50;
  params.plan_x_end = 50;
  params.plan_y_start = -30;
  params.plan_y_end = 30;
  params.plan_num_of_orient = 1;
  params.plan_margin_to_wall = 0.1;
    


  agent_ = new agent::Agent(&n, params, pid);
  initComm(n);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  ros::Rate loop_rate(10);
  int count = 0;

  while (ros::ok()) {
      /**
      * This is a message object. You stuff it with data, and then publish it.
      */
      
      /*std_msgs::String msg;
      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());*/
      distributed_mapf::PathMsg msg;
      msg.sender_id = (unsigned int)pid;

      /**
      * The publish() function is how you send messages. The parameter
      * is the message object. The type of this object must agree with the type
      * given as a template parameter to the advertise<>() call, as was done
      * in the constructor above.
      */
      
      agent_->PublishPlan(msg);

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
  }
  
  return 0;
}
