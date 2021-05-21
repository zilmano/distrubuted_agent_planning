#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"

const std::string plan_topic = "plan_topic";

class Agent {
public:
	Agent(ros::NodeHandle *n): n_(n){}

    void initComm() {
    	path_publish_ = n_->advertise<distributed_mapf::PathMsg>(plan_topic, 1000);
    	test_publish_ = n_->advertise<std_msgs::String>("chatter", 1000);
    }

    void publishPath() {

    }

    void publishTest(const std_msgs::String& msg) {
    	test_publish_.publish(msg);
    }

private:
	ros::NodeHandle* n_;
	ros::Publisher path_publish_;
	ros::Subscriber path_subscribe_;
	ros::Publisher test_publish_;


};