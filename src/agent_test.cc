#include "ros/ros.h"
//#include "std_msgs/String.h"

#include <sstream>
#include <sys/types.h>
#include <unistd.h>

#include "distributed_mapf/Vertex.h"
#include "distributed_mapf/PathMsg.h"
#include "agent.h"  


using std::list;
using planning::GraphIndex;
using std::cout;
using std::endl;
using navigation::PoseSE2;

//Agent/Node State
agent::Agent* agent_;

ros::Publisher visualization_pub_;
amrl_msgs::VisualizationMsg map_viz_msg_;

list<planning::GraphIndex> path1_;
list<planning::GraphIndex> path2_ ;
list<planning::GraphIndex> path3_; 
planning::MultiAgentGraph mapfGraph_;
planning::MultiAgentAstar mapfAstar_;
  

void planCallback(const distributed_mapf::PathMsg& msg) {
    agent_->PlanMsgCallback(msg);
}


void initComm(ros::NodeHandle& n) {
  n.subscribe(defs::plan_topic, 1000, planCallback);
  agent_->InitPublishers();
}

void printPlan(const list<GraphIndex>& plan) {
  cout << "[ ";
  for (auto &v: plan) {
    cout << v.pprint(true); 
  }
  cout << " ]" << endl; 

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
                       uint32_t color=0x000FF){
  visualization::ClearVisualizationMsg(map_viz_msg_);
  for(const auto& node : plan)
  {
    Eigen::Vector2f node_loc = graph.GetLocFromVertexIndex(node.x,node.y);
    //std::cout << "[" << node.x << " " << node.y << "] ";
    visualization::DrawCross(node_loc, 0.25, color, map_viz_msg_);
  }
  //std::cout << std::endl;
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

  //agent_->Plan(start, goal);
  
}

void testAddAgents(ros::NodeHandle& n) {
  visualization_pub_ =
    n.advertise<amrl_msgs::VisualizationMsg>("visualization", 1);
  
  map_viz_msg_ = 
    visualization::NewVisualizationMessage("map", "agent");
  
  cout << "Loading Map..." << endl;
  agent_->LoadMap();
  cout << "Done Loading Map..." << endl;
  
  
  planning::Graph agentGraph = agent_->GetLocalGraph();

  cout << "Planning agent 1..." << endl;
  //navigation::PoseSE2 start(-25, 6, 0);
  //navigation::PoseSE2 goal(35, 12, 0);
  navigation::PoseSE2 start(-14, 9, 0);
  navigation::PoseSE2 goal(0, 18, 0);
  
  agent_->Plan(start, goal);
  path1_ = agent_->GetPlan(); 

  mapfGraph_.AddAgentGraph(agentGraph);
  mapfGraph_.AddAgentToJointSpace(1, path1_);
  cout << "Agent start/end vertices: " << agent_->GetStartVertex().pprint(true) << ", " 
                                       << agent_->GetGoalVertex().pprint(true, true) << endl;


  cout << "Planning agent 2..." << endl;

  //start = navigation::PoseSE2(-28, 18, 0);
  //goal = navigation::PoseSE2(16, 18, 0);
  start = navigation::PoseSE2(1, 9, 0);
  goal = navigation::PoseSE2(-9, 20, 0);

  agent_->Plan(start, goal);
  path2_ = agent_->GetPlan(); 
  mapfGraph_.AddAgentToJointSpace(2, path2_);
  cout << "Agent start/end vertices: " << agent_->GetStartVertex().pprint(true) << ", "
                                       << agent_->GetGoalVertex().pprint(true, true) << endl;


  cout << "Planning agent 3..." << endl;

  start = navigation::PoseSE2(-1, 20, 0);
  goal = navigation::PoseSE2(-13, 18, 0);
  agent_->Plan(start, goal);
  path3_ = agent_->GetPlan(); 
  mapfGraph_.AddAgentToJointSpace(3, path3_);
  cout << "Agent start/end vertices: " << agent_->GetStartVertex().pprint(true) << ", "
                                       << agent_->GetGoalVertex().pprint(true, true);

  cout << "size individual graph:: " <<  agent_->GetLocalGraph().getNumVerticesX()
                                          * 
                                          agent_->GetLocalGraph().getNumVerticesY()
                                      << endl << endl;

}

void testJointPlan(ros::NodeHandle& n) {
   cout << "Start planning joint..." << endl;
   cout << "Merge Graphs..." << endl;

   cout << "size individual graph:: " <<  agent_->GetLocalGraph().getNumVerticesX()
                                          * 
                                          agent_->GetLocalGraph().getNumVerticesY()
                                      << endl;
   mapfGraph_.MergeAgentGraphs();

   


   cout << "reminder size individual graph:: " <<  agent_->GetLocalGraph().getNumVerticesX()
                                          * 
                                          agent_->GetLocalGraph().getNumVerticesY()
                                      << endl;
   cout << "size multi agent graph:" << mapfGraph_.GetGraphSize() << endl;
   
   cout << "Make MapfAstar..." << endl;

   mapfAstar_ = planning::MultiAgentAstar(mapfGraph_, 1, 0.5);
 
   cout << "Generate Path..." << endl;

   mapfAstar_.GeneratePath();

   cout << "Done Generate Path..." << endl;

   //for (unsigned int i = 0; i < mapfAstar_.NumOfAgents(); i++) {
   //
   //}
   mapfAstar_.GetAgentPath(path1_, 0);
   cout << "Agent1 start:" << path1_.front().x << "," << path1_.front().y 
        << "end: "<< path1_.back().x << "," << path1_.back().y 
        << "Length:" << path1_.size() << endl;
   mapfAstar_.GetAgentPath(path2_, 1);
   cout << "Agent1 start:" << path2_.front().x << "," << path2_.front().y 
        << "end: "<< path2_.back().x << "," << path2_.back().y 
        << "Length:" << path2_.size() << endl;
   //path3_ = mapfAstar_.GetAgentPath(2);
}


int main_test_graph(int argc, char **argv) {

  pid_t pid = getpid();
  std::stringstream ss;
  ss << "agent_" << pid;
  std::string node_name = ss.str();

  ros::init(argc, argv, node_name);
  ros::NodeHandle n;

  agent::Params params;
  params.plan_x_start = -35;
  params.plan_x_end = 40;
  params.plan_y_start = 0;
  params.plan_y_end = 30;
  params.plan_num_of_orient = 1;
  params.plan_margin_to_wall = 0.00001;
  params.plan_grid_pitch = 1;
  
  /*params.plan_grid_pitch = 1.5;
  params.plan_x_start = -15;
  params.plan_x_end = 3;
  params.plan_y_start = 7;
  params.plan_y_end = 21;
  params.plan_num_of_orient = 1;
  params.plan_margin_to_wall = 0.3;*/


  agent_ = new agent::Agent(&n, params, pid);
  initComm(n);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  ros::Rate loop_rate(500);
  int count = 0;
  testGenGraph(n);
  //testMapf(n);
  //testJointPlan(n);

  planning::Graph testGraph = agent_->GetLocalGraph();

  while (ros::ok()) {
      /**
      * This is a message object. You stuff it with data, and then publish it.
      */
      
      /*std_msgs::String msg;
      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());*/
      distributed_mapf::PathMsg msg;
      msg.sender_id = (unsigned int)pid;

      /**
      * The publish() function is how you send messages. The parameter
      * is the message object. The type of this object must agree with the type
      * given as a template parameter to the advertise<>() call, as was done
      * in the constructor above.
      */
      //agent.publishTest(msg);
      agent_->PublishPlan(msg);

      testVisualizeGraph(testGraph);
      //testVisualizePath(testGraph, agent_->GetPlan());
      //testVisualizePath(testGraph, path1_);
      //testVisualizePath(testGraph, path2_,0x00FF00);
      //testVisualizePath(testGraph, path3_,0xFF0000);


      ros::spinOnce();

      loop_rate.sleep();
      ++count;
  }
  
  return 0;
}

int main_test_astar(int argc, char **argv) {
  pid_t pid = getpid();
  std::stringstream ss;
  ss << "agent_" << pid;
  std::string node_name = ss.str();

  ros::init(argc, argv, node_name);
  ros::NodeHandle n;

  agent::Params params;
  params.plan_grid_pitch = 3;
  params.plan_x_start = -35;
  params.plan_x_end = 40;
  params.plan_y_start = 0;
  params.plan_y_end = 30;
  params.plan_num_of_orient = 1;
  params.plan_margin_to_wall = 0.00001;
    


  agent_ = new agent::Agent(&n, params, pid);
  initComm(n);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  ros::Rate loop_rate(500);
  int count = 0;
  //testGenGraph(n);
  testAddAgents(n);
  //testJointPlan(n);

  planning::Graph testGraph = agent_->GetLocalGraph();

  while (ros::ok()) {
      /**
      * This is a message object. You stuff it with data, and then publish it.
      */
      
      /*std_msgs::String msg;
      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());*/
      distributed_mapf::PathMsg msg;
      msg.sender_id = (unsigned int)pid;

      /**
      * The publish() function is how you send messages. The parameter
      * is the message object. The type of this object must agree with the type
      * given as a template parameter to the advertise<>() call, as was done
      * in the constructor above.
      */
      //agent.publishTest(msg);
      //agent_->PublishPlan(msg);

      //testVisualizeGraph(testGraph);
      //testVisualizePath(testGraph, agent_->GetPlan());
      testVisualizePath(testGraph, path1_);
      testVisualizePath(testGraph, path2_,0x00FF00);
      testVisualizePath(testGraph, path3_,0xFF0000);


      ros::spinOnce();

      loop_rate.sleep();
      ++count;
  }
  
  return 0;

}

int main_test_mapf(int argc, char **argv) {
  pid_t pid = getpid();
  std::stringstream ss;
  ss << "agent_" << pid;
  std::string node_name = ss.str();

  ros::init(argc, argv, node_name);
  ros::NodeHandle n;

  agent::Params params;
  /*params.plan_grid_pitch = 3;
  params.plan_x_start = -35;
  params.plan_x_end = 40;
  params.plan_y_start = 0;
  params.plan_y_end = 30;
  params.plan_num_of_orient = 1;
  params.plan_margin_to_wall = 0.00001;*/
  params.plan_grid_pitch = 1.5;
  params.plan_x_start = -15;
  params.plan_x_end = 3;
  params.plan_y_start = 7;
  params.plan_y_end = 21;
  params.plan_num_of_orient = 1;
  params.plan_margin_to_wall = 0.3;

    


  agent_ = new agent::Agent(&n, params, pid);
  initComm(n);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  ros::Rate loop_rate(500);
  int count = 0;
  //testGenGraph(n);
  testAddAgents(n);
  //testJointPlan(n);

  planning::Graph testGraph = agent_->GetLocalGraph();

  while (ros::ok()) {
      /**
      * This is a message object. You stuff it with data, and then publish it.
      */
      
      /*std_msgs::String msg;
      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());*/
      distributed_mapf::PathMsg msg;
      msg.sender_id = (unsigned int)pid;

      /**
      * The publish() function is how you send messages. The parameter
      * is the message object. The type of this object must agree with the type
      * given as a template parameter to the advertise<>() call, as was done
      * in the constructor above.
      */
      //agent.publishTest(msg);
      agent_->PublishPlan(msg);



      //testVisualizeGraph(testGraph);
      //testVisualizePath(testGraph, agent_->GetPlan());
      testVisualizePath(testGraph, path1_);
      testVisualizePath(testGraph, path2_,0x00FF00);
      testVisualizePath(testGraph, path3_,0xFF0000);


      ros::spinOnce();

      loop_rate.sleep();
      ++count;

      
  }

  return 0;
  
}

void testTemp(ros::NodeHandle& n) {
  visualization_pub_ =
    n.advertise<amrl_msgs::VisualizationMsg>("visualization", 1);
  
  map_viz_msg_ = 
    visualization::NewVisualizationMessage("map", "agent");
  
  cout << "Loading Map..." << endl;
  agent_->LoadMap();
  cout << "Done Loading Map..." << endl;
  
  
  planning::Graph agentGraph = agent_->GetLocalGraph();
  agentGraph.SetWindow(agentGraph.GetClosestVertex(PoseSE2(-6,9, 0)),
                       defs::window_size_x,
                       defs::window_size_y);

  // Test IsVectorInWindow
  /*
  vector<PoseSE2> ps = {PoseSE2(-12, 3, 0), PoseSE2(-12, 9, 0), PoseSE2(-10, 15,0), 
                        PoseSE2(-6, 14, 0), PoseSE2(0, 4, 0), PoseSE2(20, 9, 0), 
                        PoseSE2(-10, 5, 0), PoseSE2(-7, 14, 0), PoseSE2(-5, 14, 0), 
                        PoseSE2(-6, 14, 0)};
  for (auto& p : ps) {
    cout << agentGraph.GetClosestVertex(p).pprint(true) << " ";
    if (agentGraph.IsVertexInWindow(agentGraph.GetClosestVertex(p))) {
      cout << "Is in Window." << endl;
    } else {
      cout << "Not in Window" << endl;
    }    
  }*/

  // Test GetVertexNeighbors
  /*
  ps = {PoseSE2(-6, 14, 0), PoseSE2(-12, 6, 0), 
        PoseSE2(-8, 11, 0), PoseSE2( 20, 9, 0)};

  for (auto &p: ps) {
    cout << agentGraph.GetClosestVertex(p).pprint(true,false) << "neighbors:";
    for (auto &v: agentGraph.GetVertexNeighbors(agentGraph.GetClosestVertex(p))) {
      cout << v.pprint(true, false) << ", ";
    }
    cout << endl;
  }*/
  PoseSE2 start;
  PoseSE2 goal;

  start = PoseSE2(-14, 8, 0);
  goal = PoseSE2 (8, 14, 0);
  agent_->Plan(start, goal);
  path1_ = agent_->GetPlan(); 

  start = PoseSE2(1, 9, 0);
  goal  =  PoseSE2(-23, 16, 0);
  agent_->Plan(start, goal);
  path2_ = agent_->GetPlan();

  cout << "Detect colision..." << endl;
  if (!agent_->DetectCollision(path1_))
    cout << "NO COLLISION DETECTED." << endl;
  mapfGraph_.AddAgentGraph(agentGraph);
  mapfGraph_.SetWindow(agent_->GetCollisionVertex(), 
                 defs::window_size_x, 
                 defs::window_size_y);
  cout << "collision vertex:" << agent_->GetCollisionVertex().pprint(true) << endl;
  

  
  cout << "Planning agent 1..." << endl;
  mapfGraph_.AddAgentToJointSpace(1, path1_);
  cout << "Agent start/end vertices: " << agent_->GetStartVertex().pprint(true) << ", " 
                                       << agent_->GetGoalVertex().pprint(true, true) << endl;


  cout << "Planning agent 2..." << endl;
  mapfGraph_.AddAgentToJointSpace(2, path2_);
  cout << "Agent start/end vertices: " << agent_->GetStartVertex().pprint(true) << ", "
                                       << agent_->GetGoalVertex().pprint(true, true) << endl;

  mapfGraph_.MergeAgentGraphs();    
  cout << "size multi agent graph:" << mapfGraph_.GetGraphSize() << endl; 

  cout << "Make MapfAstar..." << endl;
  mapfAstar_ = planning::MultiAgentAstar(mapfGraph_, 1, 0.5);
  cout << "Generate Path..." << endl;
  mapfAstar_.GeneratePath();
  cout << "Done Generate Path..." << endl;

  list<GraphIndex> win_path1, win_path2;
  mapfAstar_.GetAgentPath(win_path1, 0);
  mapfAstar_.GetAgentPath(win_path2, 1);
  cout << "Plan 1:" << endl;
  printPlan(win_path1);
  cout << "Plan 2:" << endl;
  printPlan(win_path2);

  cout << "Original plan " << endl;
  printPlan(path1_);
  cout << "Fixed plan " << endl;
  mapfAstar_.PlugAgentPath(path1_, 0);
  printPlan(path1_);
  

}




int main_test_temp(int argc, char **argv) {
  id_t pid = getpid();
  std::stringstream ss;
  ss << "agent_" << pid;
  std::string node_name = ss.str();
  ros::init(argc, argv, node_name);
  ros::NodeHandle n;

  agent::Params params;
  params.plan_x_start = -35;
  params.plan_x_end = 40;
  params.plan_y_start = 0;
  params.plan_y_end = 30;
  params.plan_num_of_orient = 1;
  params.plan_margin_to_wall = 0.00001;
  params.plan_grid_pitch = 1;
  
  agent_ = new agent::Agent(&n, params, pid);
  initComm(n);
  ros::Rate loop_rate(0.5);
  int count = 0;

  testTemp(n);
  
  return 0;
}




int main(int argc, char **argv)
{
  

  main_test_temp(argc, argv);
  //main_test_graph(argc, argv);


  return 0;
  
}