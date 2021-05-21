#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include <stdlib.h>

#include "distributed_mapf/Vertex.h"
#include "distributed_mapf/PathMsg.h"

const std::string plan_topic = "plan_topic";

class Agent {
private:
	static void plan_subscribe_callback(const distributed_mapf::PathMsg& msg);
		
public:
	Agent(ros::NodeHandle *n): n_(n) {}

    void initComm() {
    	plan_publish_ = n_->advertise<distributed_mapf::PathMsg>(plan_topic, 1000);
    	test_publish_ = n_->advertise<std_msgs::String>("chatter", 1000);

    	plan_subscribe_ = n_->subscribe(plan_topic, 1000, Agent::plan_subscribe_callback);
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