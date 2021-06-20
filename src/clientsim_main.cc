#include "ros/ros.h"
#include "clientsim.h" 
#include <iostream>

#include "distributed_mapf/RegMsg.h"

using namespace simulator;
using std::cout;
using std::endl;


ros::Subscriber register_sub_;
ClientSim* clientsim_;

void registerCallback(const distributed_mapf::RegMsg& msg) {
    cout << "Client::Getting Register callback." << endl;
    clientsim_->RegisterMsgCallback(msg);

}

void initComm(ros::NodeHandle& n) {
  register_sub_ = n.subscribe(defs::register_topic, 1, &registerCallback);
  clientsim_->InitPublishers();
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "clientsim");
  	ros::NodeHandle n;

	simulator::Params params;
	params.plan_x_start = defs::plan_x_start;
	params.plan_x_end = defs::plan_x_end; 
	params.plan_y_start = defs::plan_y_start;;
	params.plan_y_end = defs::plan_y_end;
	params.goal_change_freq = 5;
	params.goal_change_prob = 0.3;
	
	clientsim_ = new ClientSim(&n, params);
	initComm(n);

	std::default_random_engine generator;
	generator.seed(time(0));
	std::uniform_int_distribution<unsigned int> goal_change_step_dist(
    			params.goal_change_freq-1, 
    			params.goal_change_freq+5
    );
    

	ros::Rate loop_rate(0.5);

	unsigned int step_cnt = 1;
	unsigned int goal_change_step = goal_change_step_dist(generator);
	
	while (ros::ok()) {
		if (step_cnt == goal_change_step) {
			cout << "Goal setting step reached." << endl;
			step_cnt = 0;
			clientsim_->SetNewGoal();
			goal_change_step = goal_change_step_dist(generator);

		}
 		ros::spinOnce();
 		step_cnt += 1;
 		loop_rate.sleep();
	}	
	return 0;
}



