#ifndef INCLUDE_DEFS_
#define INCLUDE_DEFS_

namespace defs {
    
    // Single agent map borders.
    const static int plan_x_start = -41;
    const static int plan_x_end = 39;
    const static int plan_y_start = 4;
    const static int plan_y_end = 20.5;
    //other single agent graphs params 
    const static float plan_grid_pitch = 1;
    const static int plan_num_of_orient = 1;
    const static float plan_margin_to_wall = 0.0001;

    // mapf window size
    const static float window_size_x = 10;
    const static float window_size_y = 10;
    
    // rates and clock freq
    const static float agent_spin_freq = 1; // 1 sec; The delays should be relative to move rate
    const static float central_clock_freq = 0.5; // 5 sec - we move every 5 sec.;
    
    // network loss model params
    const static float msg_drop_prob = 0.4;
    const static float msg_delay_prob = 0.4;
    const static unsigned int msg_delay_time_upper = 6;
    const static unsigned int msg_delay_time_lower = 2;

    // client sim params
    const static unsigned int reached_goal_wait_time_upper = 9;
    const static unsigned int reached_goal_wait_time_lower = 3; 
    
    const static unsigned int agent_heartbeat_timeout = 10;

    // names
    const static std::string plan_topic = "plan_topic";
	const static std::string new_goal_topic = "new_goal_topic";
	const static std::string register_topic = "register_topic";
    const static std::string clock_topic = "clock_topic";
	const static std::string default_map = "GDC1";

}

#endif





