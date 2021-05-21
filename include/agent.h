#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include <stdlib.h>

const std::string plan_topic = "plan_topic";

class Agent {
private:
	static void plan_comm_callback(const distributed_mapf::PathMsg& msg) {
		if (agent_id_ != msg.sender_id) {
			ROS_INFO("I heard a plan message from agent [%s]", std::to_string(msg.sender_id).c_str());
		}
	}

public:
	Agent(ros::NodeHandle *n): n_(n) {}

    void initComm() {
    	plan_publish_ = n_->advertise<distributed_mapf::PathMsg>(plan_topic, 1000);
    	test_publish_ = n_->advertise<std_msgs::String>("chatter", 1000);

    	plan_subscribe_ = n_->subscribe(plan_topic, 1000, Agent::plan_comm_callback);
    }

    void publishPlan(const distributed_mapf::PathMsg& msg) {
    	plan_publish_.publish(msg);	
    }

    void publishTest(const std_msgs::String& msg) {
    	test_publish_.publish(msg);
    }

    static void setAgentId(unsigned int id) {
    	agent_id_ = id;
    }



private:
	ros::NodeHandle* n_;
	ros::Publisher plan_publish_;
	ros::Subscriber plan_subscribe_;
	ros::Publisher test_publish_;

	static unsigned int agent_id_;
	//vector clock 


};