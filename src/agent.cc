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

void Agent::PlanMsgCallback(const distributed_mapf::PathMsg& msg) {

    if (clock_cnt_ == 0) {
        cout << endl;
        return; // agent has not synced with the central clock server yet.
    }

    if (agent_id_ != (unsigned int) msg.sender_id) {
        cout << "Got msg" << endl;
        if (!ideal_) {
            /*srand (time(NULL));
            int first_random_num = rand() % 100 + 1;
            if(first_random_num%10==0)
                return;
            // Delay the packet if the random number is multple of 3.
            int second_random_num = rand() % 90 + 1;
            if (second_random_num%3==0)
                sleep(2);*/

            // It is C++ not C, let's use std::random shall we....
            if (!msg.delayed && msg_drop_bernoulli_(gen_)) {
                cout << "\n\n-------------------------------------" << endl
                 << "NETWORK DELAY IN PROGRES......................" << endl
                 << "--------------------------------------------\n\n";
                cout << "Network issue - Message dropped." << endl << endl;
                return;
            } else if(!msg.delayed && msg_delay_bernoulli_(gen_)) {
                cout << "\n\n-------------------------------------" << endl
                 << "NETWORK DELAY IN PROGRES......................" << endl
                 << "--------------------------------------------\n\n";
                cout << "Network issue - Message delayed" << endl << endl;
                std::thread pubthread([this](distributed_mapf::PathMsg msg){
                    unsigned int delay = msg_delay_time_gen_(gen_);
                    sleep(delay);
                    cout << "\n--------------------------------------------" << endl;
                    cout << "Thread:: Message delay finished, publishing............." << endl
                          << "---------------------------------------------\n\n";
                    msg.delayed = true;
                    PublishPlan(msg);
                }, msg);
                pubthread.detach();
                return;
            }
        }
        

        /*ROS_INFO("Agent [%s]:I heard a plan message from agent [%s],"
                 "change command [%s]",
            std::to_string(agent_id_).c_str(), 
            std::to_string(msg.sender_id).c_str(),
            std::to_string(msg.set_new_plan).c_str());*/
        cout << "[ INFO] Agent ["<< agent_id_ << "]:" 
             << "I heard a plan message from agent [" <<  msg.sender_id << "],"
             <<    "change command [" << msg.set_new_plan<< "]" << endl;
            
        if(!msg.set_new_plan) {
            if (vector_clk_.count(msg.sender_id) > 0 
                     && msg.agent_vector_clk <= vector_clk_[msg.sender_id]) {
                cout << "Got msg" << endl
                << "Information message. But vector clock says that message is stale. Ignoring." << endl << endl;
                return;
            }
            PublishPlan(msg);
            vector_clk_[msg.sender_id]  = msg.agent_vector_clk;
            //message from agent A to B notifying of A' change of plan.
            list<planning::GraphIndex> recieved_plan;
            ConvertPathMsgToGraphIndexList(msg, recieved_plan);
            UpdatePlanToCurrentTime(msg.clock, clock_cnt_, recieved_plan);
            cout << "Agent::" << agent_id_ << ":: Working on detecting a collision..." << endl;
            bool collision_found = DetectCollision(recieved_plan);
            if (collision_found && recieved_plan.front() == my_plan_.front()) {
                cout << "Unfortunatly agents start at the same location. Can't resolve this, moving on." << endl;
            } else if (collision_found) {
                cout << "Agent::" << agent_id_ << ":: "<< "Collision detected. Starting joint plan" << endl;
                cout << "Agent::" << agent_id_ << ":: Collision point:" << collision_vertex_.pprint(true,true);
                JointReplan(recieved_plan, msg.sender_id);
                
                for (unsigned int i = 0; i < mapf_Astar_.NumOfAgents(); i++) {
                    if (mapf_Astar_.GetAgentIdFromIndex(i) == agent_id_) {
                        mapf_Astar_.PlugAgentPath(my_plan_, i);
                        cout << "New plan after joint astar:" << endl;
                        planning::PrintPlan(my_plan_);
                        if (mapf_Astar_.HasPlanChanged(i, my_plan_))
                            own_vector_clk_++;
                        else 
                            cout << "My plan hasn't changed." << endl;
                        
                        // TODO: notify other agents that plan has changed.
                        continue;
                    }

                    list<planning::GraphIndex> agent_path; 
                    agent_path = mapf_Astar_.PlugAgentPath(i);
                    if (mapf_Astar_.HasPlanChanged(i, agent_path)) {
                        distributed_mapf::PathMsg reply_msg;
                        reply_msg.sender_id = agent_id_;
                        reply_msg.target_id = mapf_Astar_.GetAgentIdFromIndex(i); 
                        reply_msg.set_new_plan = true;
                        reply_msg.clock = clock_cnt_;
                        reply_msg.agent_vector_clk = own_vector_clk_;
                        reply_msg.delayed = false;
                        ConvertGraphIndexListToPathMsg(agent_path, reply_msg);
                        cout << "Agent::" << agent_id_ 
                             << ":: Send new plan to other agent " << reply_msg.target_id  
                             << ":" << endl; 
                        planning::PrintPlan(my_plan_);
                        if (vector_clk_.count(reply_msg.target_id) == 0) {
                            cout << "ERROR: vector clock for agent " << reply_msg.target_id
                                 << " has not been initialized." << endl;
                            throw;
                        }
                        cout << "Plan was sent! --" << endl << endl;
                        vector_clk_[reply_msg.target_id] += 1;
                        PublishPlan(reply_msg);
                        issued_command_to.insert(reply_msg.target_id);
                    }
                }
                // reset the graph to save memory.    
                //mapf_Astar_ = planning::MultiAgentAstar(); // Init mapf_Astar to conserve memory.
    
            } else {
                cout << "No collision detected. Carry on." << endl;
            }

        } else if (agent_id_ == msg.target_id){
           if (vector_clk_.count(msg.sender_id) > 0 
                     && msg.agent_vector_clk < vector_clk_[msg.sender_id]) {
                cout << "Got msg" << endl
                << "set gpa message. But vector clock says that message is stale. Ignoring." << endl << endl;
                return;
            }
           vector_clk_[msg.sender_id]  = msg.agent_vector_clk;
           // message B from A after B detected a collision and recalculated a joint plan
           // for both B and A
           if (issued_command_to.count(msg.sender_id) == 0) {
            cout << "Haven't Issued command to " << msg.sender_id << " So yielding " << endl;
            own_vector_clk_++;
            ChangePlan(msg);
           }
           else if ( agent_id_> msg.sender_id)  {
            cout << "Issued command to " << msg.sender_id << " But yielding as my id is bigger" << endl;
            own_vector_clk_++;
            ChangePlan(msg);

           } else {
            cout << "Issued command to " << msg.sender_id << " But not yielding since my id is smaller" << endl;
           }
        }
    } else {
        cout << endl;
    }
}

void Agent::GoalMsgCallback(const distributed_mapf::GoalMsg& msg) {
    if (clock_cnt_ == 0) 
        return; //agent hasn't synced with central clock yet.
    if (agent_id_ == (unsigned int) msg.target_id) {
        asked_for_new_goal_ = false;
        cout << "\n\n *****  Got new goal from client.. ******" << endl;
        cout << "x and y corrdinates of new goal are x: "<< msg.vertex.loc_x<<" y: "<<msg.vertex.loc_y<<endl;

//      navigation::PoseSE2 goal(15,9, 0);  // Debugging with this goal

        navigation::PoseSE2 goal_pose(msg.vertex.loc_x, msg.vertex.loc_y, 0);
        Eigen::Vector2f start_loc = graph_.GetLocFromVertexIndex(
                                                    current_loc_->x,
                                                    current_loc_->y); 
        navigation::PoseSE2 start_pose(start_loc.x(), start_loc.y(), 0);
        if (Plan(start_pose, goal_pose)) {
            cout<<"Plan of new goal about to be published!"<<endl;
            distributed_mapf::PathMsg plan_msg;
            plan_msg.sender_id = agent_id_;
            plan_msg.target_id = (-1); // TODO: Set appropriate target ID
            plan_msg.set_new_plan = false;
            plan_msg.clock = clock_cnt_;
            plan_msg.agent_vector_clk = own_vector_clk_; // Is this coorect?
            plan_msg.delayed = false;
            cout<<"\nVector clock for newly published plan is "<<own_vector_clk_<<endl;
            ConvertGraphIndexListToPathMsg(my_plan_, plan_msg);
            PublishPlan(plan_msg);
            cout<<"!!!!NEW GOAL PLAN PUBLISHED!!!!!\n\n"<<endl;
            issued_command_to.insert(plan_msg.target_id);
        } else {
            cout << "Could not find a valid path to the new goal. Continuing with the old plan.\n" << endl;
        }
    }
}

void Agent::ChangePlan(const distributed_mapf::PathMsg& msg) {
    cout << "changing plan: " << endl;
    list<planning::GraphIndex> new_plan;
    ConvertPathMsgToGraphIndexList(msg, new_plan);
    UpdatePlanToCurrentTime(msg.clock, clock_cnt_, new_plan);
    my_plan_ = new_plan;

    distributed_mapf::PathMsg plan_msg;
    plan_msg.sender_id = agent_id_;
    plan_msg.target_id = (-1); // TODO: Set appropriate target ID
    plan_msg.set_new_plan = false;
    plan_msg.clock = clock_cnt_;
    plan_msg.agent_vector_clk = own_vector_clk_;
    plan_msg.delayed = false;
    //cout<<"\nVector clock for newly published plan is "<<own_vector_clk_<<endl;
    ConvertGraphIndexListToPathMsg(my_plan_, plan_msg);
    PublishPlan(plan_msg);
}

void Agent::JointReplan(const list<planning::GraphIndex>& recieved_plan, unsigned int other_agent_id) {
    cout << "Agent::" << agent_id_ << ":: Starting Joint Replan..." << endl;
    
    std::shared_ptr<planning::MultiAgentGraph> mapf_graph = 
                            std::make_shared<planning::MultiAgentGraph>();
    
    mapf_graph->AddAgentGraph(graph_); // OLEG TODO: Need to "redistrict it" 
                                       // aroud the collision point
    mapf_graph->SetWindow(collision_vertex_, 
                          params_.window_size_x,
                          params_.window_size_y);
    cout << "my plan. ";
    mapf_graph->AddAgentToJointSpace(agent_id_, my_plan_);
    cout << "recieved plan. ";
    mapf_graph->AddAgentToJointSpace(agent_id_, my_plan_);
    mapf_graph->AddAgentToJointSpace(other_agent_id, recieved_plan);
    mapf_graph->MergeAgentGraphs();
    cout << "Done merge joint graph." << endl;
    mapf_Astar_ = planning::MultiAgentAstar(mapf_graph, 1, 0.5);
    mapf_Astar_.GeneratePath();
}





