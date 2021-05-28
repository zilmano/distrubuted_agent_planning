#include "../include/agent.h"

unsigned int Agent::agent_id_;
void Agent::plan_subscribe_callback(const distributed_mapf::PathMsg& msg) {
	if (Agent::agent_id_ != (unsigned int) msg.sender_id) {
		
		// Drop the packet if the random number is not multiple of ten

		srand (time(NULL));
		int first_random_num = rand() % 100 + 1;
		
		if(first_random_num%10!=0){
			// Delay the packet if the random number is multple of 3.

			int second_random_num = rand() % 90 + 1;

			if (second_random_num%3==0)
				sleep(2);
			
			ROS_INFO("Agent [%s]:I heard a plan message from agent [%s]",std::to_string(Agent::agent_id_).c_str(), std::to_string(msg.sender_id).c_str());
		
		}
	}
}
