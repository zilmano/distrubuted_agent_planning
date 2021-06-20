#ifndef INCLUDE_DEFS_
#define INCLUDE_DEFS_

namespace defs {
    
    // Single agent map borders.
    const static int plan_x_start = -41;
    const static int plan_x_end = 39;
    const static int plan_y_start = 4;
    const static int plan_y_end = 20.5;

    const static float window_size_x = 10;
    const static float window_size_y = 10;

    /*
    params.plan_x_start = -15;
  	params.plan_x_end = 3;
  	params.plan_y_start = 7;
  	params.plan_y_end = 21;
    */

    //other single agent graphs params 
    const static float plan_grid_pitch = 1;
    const static int plan_num_of_orient = 1;
    const static float plan_margin_to_wall = 0.0001;
    
    // mapf window size

    // names
    const static std::string plan_topic = "plan_topic";
	const static std::string new_goal_topic = "new_goal_topic";
	const static std::string register_topic = "register_topic";
	const static std::string default_map = "GDC1";

}

#endif





