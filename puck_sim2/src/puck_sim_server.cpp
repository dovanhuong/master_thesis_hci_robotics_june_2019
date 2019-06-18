#include <ros/ros.h>
#include "../include/puck_sim2/mySim.h"
#include <vector>

bool run(puck_sim2::puck_sim::Request &req,
         puck_sim2::puck_sim::Response &res){
    mySim sim;
    myCoord hand_init, hand_fin, obj_init, obj_fin;
    srand(time(NULL));

    hand_init.m_x = req.hand_init[0];
    hand_init.m_y = req.hand_init[1];
    hand_fin.m_x = req.hand_fin[0];
    hand_fin.m_y = req.hand_fin[1];
    obj_init.m_x = req.obj_init[0];
    obj_init.m_y = req.obj_init[1];

    char filename[255];
    sprintf(filename, "%s/puckSim/example.png", getenv("HOME"));//.c_str());
    obj_fin = sim.run(obj_init, hand_init, hand_fin, filename);

    std::vector<double> temp_res;
    temp_res.push_back(obj_fin.m_x);
    temp_res.push_back(obj_fin.m_y);
    res.obj_fin = temp_res;

    printf("%f   %f \n", obj_fin.m_x, obj_fin.m_y);

    return true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "puck_sim_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("puck_sim", run);
    ROS_INFO("Ready for puck_sim");
    ros::spin();

    return 0;
}
