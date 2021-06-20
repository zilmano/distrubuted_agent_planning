#include "ros/ros.h"
//#include "std_msgs/String.h"

#include <sstream>
#include <sys/types.h>
#include <unistd.h>

#include "distributed_mapf/Vertex.h"
#include "distributed_mapf/PathMsg.h"
#include "agent.h"  
#include "defs.h"


using std::list;

//Agent/Node State
agent::Agent* agent_;

ros::Publisher visualization_pub_;
amrl_msgs::VisualizationMsg map_viz_msg_;
ros::Subscriber plan_sub_;
ros::Subscriber goal_sub_;



void planCallback(const distributed_mapf::PathMsg& msg) {
    cout << "Getting plan callback." << endl;
    agent_->PlanMsgCallback(msg);
}

void goalCallback(const distributed_mapf::GoalMsg& msg) {
    cout << "Getting plan callback." << endl;
    agent_->GoalMsgCallback(msg);
}


void initComm(ros::NodeHandle& n) {
  plan_sub_ = n.subscribe(defs::plan_topic, 1000, &planCallback);
  goal_sub_ = n.subscribe(defs::new_goal_topic, 1000, &goalCallback);
  
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

void testVisualizePath(planning::Graph graph, list<planning::GraphIndex> plan,
                       uint32_t color=0x000FF) {
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

distributed_mapf::PathMsg makeNotifyMsg(pid_t pid) {
  distributed_mapf::PathMsg cmd_msg;
  cmd_msg.sender_id = (unsigned int)pid;
  cmd_msg.target_id = -1; 
  cmd_msg.set_new_plan = false;
  agent::ConvertGraphIndexListToPathMsg(agent_->GetPlan(), cmd_msg);
  return cmd_msg;
}



void test_plan_comm(int argc,char **argv) {

  navigation::PoseSE2 start;
  navigation::PoseSE2 goal;
  uint32_t color = 0x000FF;

  /* example available colors:
  https://www.rapidtables.com/web/color/RGB_Color.html
  0x990000 Red
  0xFF6666
  0xFF9999
  0x999900 Yellow
  0xFFFF00
  0x4C9900 Green
  0xB2FF66
  0x006600
  0x00FF00
  0x00FFFF Blue
  0x006666
  0x000066
  0x004C99
  0x0000FF
  0x6666FF Purple
  0x7F00FF 
  0xCC00CC
  0xFF99FF
  0xFF007F
  0x606060 Gray
  0x808080
  0xC0C0C0
  */
  
  if (argc < 5) {
    start = navigation::PoseSE2(-14, 9, 0);
    goal = navigation::PoseSE2(0, 18, 0);
  } else {
    if (argc != 5 && argc != 6) {
      cout << "ERROR: need to provide start and goal" << endl;
      throw;
    }
    start = navigation::PoseSE2(std::stoi(argv[1]), std::stoi(argv[2]), 0);
    goal =  navigation::PoseSE2(std::stoi(argv[3]), std::stoi(argv[4]), 0);
    if (argc == 6)
      color = std::stoi(argv[5],0,16); 
  }

  debug::print_loc(start.loc,"Start loc:", false);
  debug::print_loc(goal.loc," end loc:", true);

  pid_t pid = getpid();
  std::stringstream ss;
  ss << "agent_" << pid;
  std::string node_name = ss.str();

  ros::init(argc, argv, node_name);
  ros::NodeHandle n;
  agent::Params params;
  params.plan_grid_pitch = defs::plan_grid_pitch;
  params.plan_x_start = defs::plan_x_start;
  params.plan_x_end = defs::plan_x_end;
  params.plan_y_start = defs::plan_y_start;
  params.plan_y_end = defs::plan_y_end;
  params.window_size_x = defs::window_size_x;
  params.window_size_y = defs::window_size_y;
  params.plan_num_of_orient = defs::plan_num_of_orient;
  params.plan_margin_to_wall = defs::plan_margin_to_wall;

  agent_ = new agent::Agent(&n, params, pid);
  agent_->SetIdeal();
  initComm(n);
  initVisualizer(n);
  ros::Rate loop_rate(0.1);
  int count = 0;

  agent_->LoadMap();
  //planning::Graph agentGraph = agent_->GetLocalGraph();
  
  agent_->Plan(start, goal);
  planning::Graph agentGraph = agent_->GetLocalGraph();
  
  while (ros::ok()) {
      /**
      * This is a message object. You stuff it with data, and then publish it.
      */
      
      /*std_msgs::String msg;
      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());*/
      //distributed_mapf::PathMsg msg;
      //msg.sender_id = (unsigned int)pid;

      distributed_mapf::PathMsg cmd_msg = makeNotifyMsg(pid);
      agent_->PublishPlan(cmd_msg);
      agent_->PublishRegister();
      agent_->ClearIssuedCommands(); // This, using the 'issued_command_to' member
                                     // is a temporary solution to a corner case
                                     // where both agent had change of plans simultaniosly
                                     // and notifiy and command each other simulataniously.


      auto path = agent_->GetPlan(); 
      //testVisualizeGraph(testGraph);
      //testVisualizePath(testGraph, agent_->GetPlan());
      testVisualizePath(agentGraph, path, color);
      
      ros::spinOnce();

      loop_rate.sleep();
      ++count;
  }
}


int main(int argc, char **argv)
{
  
  test_plan_comm(argc, argv);
  
}