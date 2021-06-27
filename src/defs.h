#ifndef INCLUDE_DEFS_
#define INCLUDE_DEFS_

namespace defs {
    
    // Single agent map borders.
    const static int plan_x_start = -41;
    const static int plan_x_end = 39;
    const static int plan_y_start = 4;
    const static int plan_y_end = 20.5;
    // mapf window size
    const static float window_size_x = 10;
    const static float window_size_y = 10;
    //other single agent graphs params 
    const static float plan_grid_pitch = 1;
    const static int plan_num_of_orient = 1;
    const static float plan_margin_to_wall = 0.0001;

    // rates and clock freq
    const static float agent_spin_freq = 1; // 1 sec; The delays should be relative to move rate
    const static float central_clock_freq = 0.2; // 5 sec - we move every 5 sec.;
    
    // names
    const static std::string plan_topic = "plan_topic";
	const static std::string new_goal_topic = "new_goal_topic";
	const static std::string register_topic = "register_topic";
    const static std::string clock_topic = "clock_topic";
	const static std::string default_map = "GDC1";

}

#endif





