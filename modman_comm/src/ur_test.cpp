#include "ros/ros.h"

#include "stdio.h"
#include "stdlib.h"
#include <iostream>
#include "string.h"
#include "time.h"
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include "std_msgs/String.h"

#include "modman_msgs/modman_state.h"

#include "opencv_object_tracking/position_publish.h"

// paramter for log
#define SHOW_LOG (true)

#define UR_IP "192.168.1.102"
#define PORT_NUMBER 30003
#define MESSAGE_FREQ 125
#define BUF_LEN 216

#define READY_0 (0.0)
#define READY_1 (-90.0*M_PI/180.0)
#define READY_2 (0.0)
#define READY_3 (-90.0*M_PI/180.0)
#define READY_4 (0.0)
#define READY_5 (0.0)

#define POSE_0 (0.0)
#define POSE_1 (-45.0*M_PI/180.0)
#define POSE_2 (0.0)
#define POSE_3 (0.0*M_PI/180.0)
#define POSE_4 (45.0*M_PI/180.0)
#define POSE_5 (45.0*M_PI/180.0)

/*#define WORK0_0 (-0.350)
#define WORK0_1 (0.300)
#define WORK0_2 (0.055) 
#define WORK0_3 (90.0*M_PI/180.0)
#define WORK0_4 (0.0*M_PI/180.0)
#define WORK0_5 (0.0*M_PI/180.0)
*/
#define WORK0_0 (-0.450)
#define WORK0_1 (-0.100)
//#define WORK0_1 (0.300)
#define WORK0_2 (0.055) 
#define WORK0_3 (90.0*M_PI/180.0)
#define WORK0_4 (0.0*M_PI/180.0)
#define WORK0_5 (0.0*M_PI/180.0)


#define VIS_CAL_0_0 (-0.450)
#define VIS_CAL_0_1 (0.300)
#define VIS_CAL_0_2 (0.2) 
#define VIS_CAL_0_3 (180.0*M_PI/180.0)
#define VIS_CAL_0_4 (0.0*M_PI/180.0)
#define VIS_CAL_0_5 (0.0*M_PI/180.0)

#define VIS_CAL_1_0 (-0.550)
#define VIS_CAL_1_1 (-0.100)
#define VIS_CAL_1_2 (0.045) 
#define VIS_CAL_1_3 (180.0*M_PI/180.0)
#define VIS_CAL_1_4 (0.0*M_PI/180.0)
#define VIS_CAL_1_5 (0.0*M_PI/180.0)

#define MANIPULATION_WAIT_TIME (1.0)

using namespace std;

void error(const char *msg) {
    perror(msg);
    exit(1);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "ur_client");
    ros::NodeHandle nh;

    ros::Rate loop_rate(MESSAGE_FREQ); // set the rate as defined in the macro MESSAGE_FREQ

    int sockfd, portno, n, choice = 1;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[BUF_LEN];
    bool echoMode = false;

    portno = PORT_NUMBER;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");

    server = gethostbyname(UR_IP);
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");

#if SHOW_LOG
    ROS_INFO("[main()] UR Client is runnign now.");
#endif

    //unsigned int count(0);
    char command[BUF_LEN];
    bzero(command, BUF_LEN);

    int g = 1;
    ros::Time start;
    start = ros::Time::now();
    while(ros::ok())
    {
        bzero(buffer,BUF_LEN);
        n = read(sockfd,buffer,BUF_LEN);
        if (n < 0)
            error("ERROR reading from socket");
	    //ROS_INFO("[while()] I receive the message of %s", buffer);

        bzero(command, BUF_LEN);
        if( (ros::Time::now()-start).toSec() > MANIPULATION_WAIT_TIME )
        {
            if( g == 0 )
            {
//                sprintf(command,"movej([%.6f, %.6f, %.6f, %.6f, %.6f, %.6f])\n",
//                                        READY_0, READY_1, READY_2, READY_3, READY_5, READY_5);
//                sprintf(command,"movel(p[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f], a=5.0, v=2.0)\n",
//                                        WORK0_0, WORK0_1, WORK0_2, WORK0_3, WORK0_5, WORK0_5);
                g++;
            }
            else if( g == 1 )
            {
//                sprintf(command,"movel(p[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f], a=1.5, v=0.2)\n",
//                                        WORK0_0, 0.3, WORK0_2, WORK0_3, WORK0_4, WORK0_5);
                sprintf(command,"movel(p[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f], a=1.0, v=0.2)\n",
                                        VIS_CAL_0_0, VIS_CAL_0_1, VIS_CAL_0_2, VIS_CAL_0_3, VIS_CAL_0_4, VIS_CAL_0_5);
//                                        -0.55, -0.5, VIS_CAL_1_2, VIS_CAL_1_3, VIS_CAL_1_4, VIS_CAL_1_5);
//                                        VIS_CAL_1_0, VIS_CAL_1_1, VIS_CAL_1_2, VIS_CAL_1_3, VIS_CAL_1_4, VIS_CAL_1_5);
                g++;
            }
            start = ros::Time::now();
        }
        else 
            bzero(command, BUF_LEN);
	
        n = write(sockfd, command, strlen(command));
	    if (n < 0) 
	         error("ERROR writing to socket");
	    if( strlen(command) > 2 )
            ROS_INFO("[while()] I sent message of %s", command);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

