/******************************************************************************************
*                                                                                        *
*    Optimal Path Planner to Separate Different-colored Beans with Two Agents            *
*    Version 1.0                                                                         *
*    ----------------------------------------------------------                          *
*    Copyright (C) 2012  Soonkyum Kim                                                    *
*                                                                                        *
*    This program is free software: you can redistribute it and/or modify                *
*    it under the terms of the GNU General Public License as published by                *
*    the Free Software Foundation, either version 3 of the License, or                   *
*    (at your option) any later version.                                                 *
*                                                                                        *
*    This program is distributed in the hope that it will be useful,                     *
*    but WITHOUT ANY WARRANTY; without even the implied warranty of                      *
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                       *
*    GNU General Public License for more details <http://www.gnu.org/licenses/>.         *
*                                                                                        *
*                                                                                        *
*    Contact: kim.soonkyum@gmail.com                                                     *
*                                                                                        *
*                                                                                        *
******************************************************************************************/
#include "ros/ros.h"
#include "../include/puck_sim2/mySim.h"

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

//    system("rm /home/skkim/puckSim/*.png");
    
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


