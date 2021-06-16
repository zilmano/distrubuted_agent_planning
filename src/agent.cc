#include "agent.h"
#include <bits/stdc++.h>
using namespace agent;
//std::map <int, vector<planning::GraphIndex>>  all_paths;

//std::vector <vector<distributed_mapf::PathMsg::_path_type>>  all_paths;
std::vector<list<planning::GraphIndex>> all_paths;
std::vector <int> agent_ids;
time_t start_time=-1;
time_t end_time=-1;

void Agent::networkModel() {

}


void Agent::add_path(int sender_id, list<planning::GraphIndex> path_list){

	//find index from agent_ids array
	
	int agent_index = -1;

	for(unsigned int i=0;i<agent_ids.size();i++){
		if(agent_ids[i]==sender_id)
			agent_index=i;
	
	}

	//Add path corresponding to agent_id

	if(agent_index==-1){
		agent_ids.push_back(sender_id);
		all_paths.push_back(path_list);
	
	}else{
		all_paths[agent_index]=path_list;
	
	}

}

void Agent::check_collisions(int sender_id){

//	ROS_INFO("\nDISPLAYING ALL PATHS and size of all_paths is %d\n",all_paths.size());
//	ROS_INFO("\nSender ID is %d and agent ID is %d\n",sender_id,agent_id_);

	for(unsigned int i = 0;i<all_paths.size();i++){

//		ROS_INFO("\nPrinting PATH number %d\n",i);

		distributed_mapf::PathMsg msg_i;
		ConvertGraphIndexListToPathMsg(all_paths[i], msg_i);


		int step_i = 0;

		for (const auto& vertex_i: msg_i.path) {

                        unsigned int x_cord = vertex_i.x_id;
                        unsigned int y_cord = vertex_i.y_id;
			int step_j = 0;
              //          std::cout<<x_cord<<" "<<y_cord<<"\";


			for(unsigned int j = 0;j<all_paths.size();j++){

				if(i!=j){
					distributed_mapf::PathMsg msg_j;
					ConvertGraphIndexListToPathMsg(all_paths[j], msg_j);

					for (const auto& vertex_j: msg_j.path) {

						if(vertex_j.x_id==vertex_i.x_id && vertex_j.y_id==vertex_i.y_id && step_i==step_j){

							ROS_INFO("COLLISION IN PATHS!!!!!\n");
							return;

						}
					}
				}

			step_j++;
			}

		
		step_i++;
               }


	}

	ROS_INFO("\nexiting check collision\n");
}

void Agent::PlanMsgCallback(const distributed_mapf::PathMsg& msg) {

	if (start_time==-1){
	time(&start_time);
	}


	time(&end_time);

	int time_taken = int(end_time - start_time);

	if(time_taken%120!=0){

		check_collisions((int) msg.sender_id);

	}

	check_collisions((int) msg.sender_id);





	// OLEG TODO: move the following if's contect to networkModel function,
	//            so that we can use the same network simulation for all topics.
	
	list<planning::GraphIndex> recievedPlan_temp;
	ConvertPathMsgToGraphIndexList(msg, recievedPlan_temp);
	add_path(msg.sender_id, recievedPlan_temp);

	if (agent_id_ != (unsigned int) msg.sender_id) {

	

		
		cout << "Got msg" << endl;
		// Drop the packet if the random number is not multiple of ten

		srand (time(NULL));
		int first_random_num = rand() % 100 + 1;
		
		if(first_random_num%10!=0){
			// Delay the packet if the random number is multple of 3.

			int second_random_num = rand() % 90 + 1;

			if (second_random_num%3==0)
				sleep(2);
			
			ROS_INFO("Agent [%s]:I heard a plan message from agent [%s], change command [%s]",
				std::to_string(agent_id_).c_str(), 
				std::to_string(msg.sender_id).c_str(),
				std::to_string(msg.set_new_plan).c_str());

			if(!msg.set_new_plan) {
				//message from agent A to B notifying of A' change of plan.
				list<planning::GraphIndex> recievedPlan;
				ConvertPathMsgToGraphIndexList(msg, recievedPlan);
				
				if (DetectCollision(recievedPlan)) {
					JointReplan(recievedPlan, msg.sender_id);
				

					for (unsigned int i = 0; mapf_Astar_.NumOfAgents(); i++) {
						if (mapf_Astar_.GetAgentIdFromIndex(i) == agent_id_)
							continue;

						distributed_mapf::PathMsg reply_msg;
			      		reply_msg.sender_id = agent_id_;
		    	  		reply_msg.target_id = mapf_Astar_.GetAgentIdFromIndex(i); 
		    	  		reply_msg.set_new_plan = true;
		    	  		ConvertGraphIndexListToPathMsg(mapf_Astar_.GetAgentPath(i), reply_msg);
		    	  		PublishPlan(reply_msg);
					}
				} 
			} else {
				   // message B from A after B detected a collision and recalculated a joint plan
				   // for both B and A
				   ChangePlan(msg);

				
			}
		}
	}
}

void Agent::ChangePlan(const distributed_mapf::PathMsg& msg) {
	cout << "changing plan: " << endl;
	list<planning::GraphIndex> new_plan;
	ConvertPathMsgToGraphIndexList(msg, new_plan);
	my_plan_ = new_plan;
}


void Agent::JointReplan(const list<planning::GraphIndex>& recieved_plan, unsigned int other_agent_id) {
	cout << "Agent::" << agent_id_ << "::Starting Joint Replan..." << endl;
	
	mapf_graph_.Init();
    mapf_graph_.AddAgentGraph(graph_); // OLEG TODO: Need to "redistrict it" 
    								   // aroud the collision point
    mapf_graph_.AddAgentToJointSpace(agent_id_, GetStartVertex(),
                                    GetGoalVertex());
    mapf_graph_.AddAgentToJointSpace(other_agent_id, recieved_plan.front(), 
    								recieved_plan.back());
    mapf_graph_.MergeAgentGraphs();

    mapf_Astar_ = planning::MultiAgentAstar(mapf_graph_, 1, 0.5);
    mapf_Astar_.GeneratePath();

}





