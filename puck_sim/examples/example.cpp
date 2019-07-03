
#include "ros/ros.h"
#include "puck_sim/mySim.h"

// Text input and output
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <cstdlib>
#include <ctime>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "planner");
	
	ros::NodeHandle n;
    unsigned int j;
    char filename[255];

    system("rm /home/skkim/puckSim/*.png");
    
    mySim sim;
    myCoord hand_init, hand_fin, obj_init, obj_fin;
    srand (time(NULL));

    double r;
    do
    {
        r = (double)rand() / RAND_MAX;
        hand_init.m_x = MINX + r*(MAXX-MINX);
        r = (double)rand() / RAND_MAX;
        hand_init.m_y = MINY + r*(MAXY-MINY);
        r = (double)rand() / RAND_MAX;
        hand_fin.m_x = MINX + r*(MAXX-MINX);
        r = (double)rand() / RAND_MAX;
        hand_fin.m_y = MINY + r*(MAXY-MINY);
        r = (double)rand() / RAND_MAX;
        obj_init.m_x = MINX + r*(MAXX-MINX);
        r = (double)rand() / RAND_MAX;
        obj_init.m_y = MINY + r*(MAXY-MINY);

        sprintf(filename, "%s/puckSim/example.png", getenv("HOME"));//.c_str());
        obj_fin = sim.run(obj_init, hand_init, hand_fin, filename);
    } while(obj_init.distTo(obj_fin) < 1.0e-6 || obj_init.distTo(hand_init) < GRIP_RADIUS+PUCK_RADIUS);
    return (0);
}


