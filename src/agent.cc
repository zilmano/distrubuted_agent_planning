#include "agent.h"

using namespace agent;

void Agent::PlanMsgCallback(const distributed_mapf::PathMsg& msg) {
	if (agent_id_ != (unsigned int) msg.sender_id) {
		ROS_INFO("Agent [%s]:I heard a plan message from agent [%s]",std::to_string(agent_id_).c_str(), std::to_string(msg.sender_id).c_str());
	}
}