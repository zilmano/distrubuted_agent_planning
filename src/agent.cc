#include "../include/agent.h"

unsigned int Agent::agent_id_;
void Agent::plan_subscribe_callback(const distributed_mapf::PathMsg& msg) {
	if (Agent::agent_id_ != (unsigned int) msg.sender_id) {
		ROS_INFO("Agent [%s]:I heard a plan message from agent [%s]",std::to_string(Agent::agent_id_).c_str(), std::to_string(msg.sender_id).c_str());
	}
}