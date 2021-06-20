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
ros::Subscriber plan_sub_;



void planCallback(const distributed_mapf::PathMsg& msg) {
    agent_->PlanMsgCallback(msg);
}


void initComm(ros::NodeHandle& n) {
  plan_sub_ = n.subscribe(defs::plan_topic, 1000, planCallback);
  agent_->InitPublishers();
}

void initVisualizer(ros::NodeHandle& n) {
  visualization_pub_ =
    n.advertise<amrl_msgs::VisualizationMsg>("visualization", 1);
  map_viz_msg_ = 
    visualization::NewVisualizationMessage("map", "agent");
}


void testVisualizeGraph(planning::Graph& graph){

    planning::Vertices V = graph.GetVertices();
    //visualization::ClearVisualizationMsg(map_viz_msg_);

    //visualization::DrawCross(Eigen::Vector2f(0,0), 0.5, 0x000FF, map_viz_msg_);
    for (int x_id = 0; x_id < (int)V.size(); ++x_id) {
        for (int y_id = 0; y_id < (int)V[0].size(); ++y_id) {
            Eigen::Vector2f vertex_loc = graph.GetLocFromVertexIndex(x_id,y_id);
            for (int o_id = 0; o_id < (int)V[0][0].size(); ++o_id) {
                planning::GraphIndex curr_vertex(x_id,y_id,o_id);
                list<planning::GraphIndex> neighbors = graph.GetVertexNeighbors(curr_vertex);
                for ( auto &neighbor : neighbors) {
                    //geometry::line2f edge(
                    //        graph.GetLocFromVertexIndex(x_id,y_id),
                    //        graph.GetLocFromVertexIndex(neighbor.x,neighbor.y));
                    visualization::DrawLine(
                            graph.GetLocFromVertexIndex(x_id,y_id),
                            graph.GetLocFromVertexIndex(neighbor.x,neighbor.y),
                            0x0000000,
                            map_viz_msg_);
                }
            }

            visualization::DrawCross(vertex_loc, 0.25, 0x000FF, map_viz_msg_);

        }
    }
    visualization_pub_.publish(map_viz_msg_);
}

void testVisualizePath(planning::Graph graph, 
                       list<planning::GraphIndex> plan, uint32_t color=0x000FF) {
  visualization::ClearVisualizationMsg(map_viz_msg_);
  for(const auto& node : plan)
  {
    Eigen::Vector2f node_loc = graph.GetLocFromVertexIndex(node.x,node.y);
    std::cout << "[" << node.x << " " << node.y << "] ";
    visualization::DrawCross(node_loc, 0.25, color, map_viz_msg_);
  }
  std::cout << std::endl;
  visualization_pub_.publish(map_viz_msg_);
}


void testGenGraph(ros::NodeHandle& n) {
  visualization_pub_ =
    n.advertise<amrl_msgs::VisualizationMsg>("visualization", 1);
  
  map_viz_msg_ = 
    visualization::NewVisualizationMessage("map", "agent");

  agent_->LoadMap();

  //navigation::PoseSE2 start(-25, 6, 0);
  //navigation::PoseSE2 goal(35, 12, 0);
  navigation::PoseSE2 start(-25, 15, 0);
  navigation::PoseSE2 goal(33, -10, 0);
  
  agent_->Plan(start, goal);
  
}

distributed_mapf::PathMsg makeChangeMsg(pid_t pid) {
  distributed_mapf::PathMsg cmd_msg;
  cmd_msg.sender_id = (unsigned int)pid;
  cmd_msg.target_id = -1; 
  cmd_msg.set_new_plan = true;
  agent::ConvertGraphIndexListToPathMsg(agent_->GetPlan(), cmd_msg);
  return cmd_msg;
}

distributed_mapf::PathMsg makeNotifyMsg(pid_t pid) {
  distributed_mapf::PathMsg cmd_msg;
  cmd_msg.sender_id = (unsigned int)pid;
  cmd_msg.target_id = -1; 
  cmd_msg.set_new_plan = false;
  agent::ConvertGraphIndexListToPathMsg(agent_->GetPlan(), cmd_msg);
  return cmd_msg;
}
  

int main(int argc,char **argv) {

  pid_t pid = getpid();
  std::stringstream ss;
  ss << "agent_" << pid;
  std::string node_name = ss.str();

  ros::init(argc, argv, node_name);
  ros::NodeHandle n;
  agent::Params params;
  params.plan_grid_pitch = 1;
  params.plan_x_start = -15;
  params.plan_x_end = 3;
  params.plan_y_start = 7;
  params.plan_y_end = 21;
  params.plan_num_of_orient = 1;
  params.plan_margin_to_wall = 0.3;

  agent_ = new agent::Agent(&n, params, pid);
  initComm(n);
  initVisualizer(n);
  ros::Rate loop_rate(0.5);
  int count = 0;

  agent_->LoadMap();
  //planning::Graph agentGraph = agent_->GetLocalGraph();
  navigation::PoseSE2 start = navigation::PoseSE2(-1, 9, 0);
  navigation::PoseSE2 goal = navigation::PoseSE2(-9, 20, 0);

  agent_->Plan(start, goal);
  planning::Graph agentGraph = agent_->GetLocalGraph();

  testVisualizePath(agentGraph, agent_->GetPlan(), 0x00FF00);
  
  distributed_mapf::PathMsg cmd_msg = makeNotifyMsg(pid);
  ros::Duration(20.0).sleep();
  agent_->PublishPlan(cmd_msg);

  
  while (ros::ok()) {
      /**
      * This is a message object. You stuff it with data, and then publish it.
      */
      
      /*std_msgs::String msg;
      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());*/

      agent_->PublishPlan(cmd_msg);
      distributed_mapf::PathMsg cmd_msg = makeNotifyMsg(pid);

      //testVisualizeGraph(testGraph);
      //testVisualizePath(testGraph, agent_->GetPlan());
      testVisualizePath(agentGraph, agent_->GetPlan(),0x00FF00);
      //testVisualizePath(testGraph, path2_,0x00FF00);
      //testVisualizePath(testGraph, path3_,0xFF0000);
      
      ros::spinOnce();

      loop_rate.sleep();
      ++count;
  }
  return 0;
}



