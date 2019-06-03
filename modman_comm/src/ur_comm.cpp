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

// parameter for log
#define SHOW_LOG (true)
// some parameters
#define OBJECT_CONVERGE_MARGIN (0.03)
#define OBJECT_CONVERGE_MARGIN_SQ (OBJECT_CONVERGE_MARGIN*OBJECT_CONVERGE_MARGIN)
#define MIN_TRAVEL_DIST (0.2)
#define MIN_TRAVEL_DISTSQ (MIN_TRAVEL_DIST*MIN_TRAVEL_DIST)
#define OBJECT_CONVERGE_PERIOD (10)
#define MANIPULATION_HEIGHT (0.055 + 0.0)
#define MANIPULATION_WAIT_TIME (4.0)
#define UR_VEL (2.5)
#define UR_ACC (5.0)

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

#define READY_POSE_X (-0.450)
#define READY_POSE_Y (0.300)
#define READY_POSE_Z (MANIPULATION_HEIGHT) 
#define READY_POSE_RX (90.0*M_PI/180.0)
#define READY_POSE_RY (0.0*M_PI/180.0)
#define READY_POSE_RZ (0.0*M_PI/180.0)

using namespace std;

modman_msgs::modman_state g_state;
opencv_object_tracking::position_publish g_last_position;
opencv_object_tracking::position_publish g_start_position;
int g_position_count;

double distBtObjects(const opencv_object_tracking::position_publish& o1, const opencv_object_tracking::position_publish& o2)
{
    return sqrt((o1.Position_XYZ[0].x-o2.Position_XYZ[0].x)*(o1.Position_XYZ[0].x-o2.Position_XYZ[0].x)+(o1.Position_XYZ[0].y-o2.Position_XYZ[0].y)*(o1.Position_XYZ[0].y-o2.Position_XYZ[0].y));
}

double distSQBtObjects(const opencv_object_tracking::position_publish& o1, const opencv_object_tracking::position_publish& o2)
{
    return (o1.Position_XYZ[0].x-o2.Position_XYZ[0].x)*(o1.Position_XYZ[0].x-o2.Position_XYZ[0].x)+(o1.Position_XYZ[0].y-o2.Position_XYZ[0].y)*(o1.Position_XYZ[0].y-o2.Position_XYZ[0].y);
}

void positionCallback(const opencv_object_tracking::position_publish& msg)
{
    //ROS_INFO("g_position_count = %u, msg.count = %u.", g_position_count, msg.counter);
    if( g_position_count < 0 )
    {
        g_last_position = msg;
        g_position_count = 0;
        return;
    }
    if( msg.counter < 1 )
    {
        g_position_count = 0;
        return;
    }

    if( distSQBtObjects(g_last_position, msg) < OBJECT_CONVERGE_MARGIN_SQ )
    {
        g_position_count++;
        //if( msg.counter > g_last_position.counter + OBJECT_CONVERGE_PERIOD )
        if( g_position_count > OBJECT_CONVERGE_PERIOD )
        {
            g_position_count = 0;
            switch( g_state.state )
            {
                case 0 : // if ready, update goal
                    g_state.object_goal_x = -1.0*msg.Position_XYZ[0].y;
                    g_state.object_goal_y = -1.0*msg.Position_XYZ[0].x - 0.61;
                    g_state.goal_pixel_x = msg.center_pixel_x;
                    g_state.goal_pixel_y = msg.center_pixel_y;
                    g_start_position = msg;
                    g_state.state++;
#if SHOW_LOG
                    ROS_INFO("[positionCallback()] Update g_state.state = %d, object_goal = (%.2f, %.2f).", g_state.state, g_state.object_goal_x, g_state.object_goal_y);
#endif
                    break;
                case 1 : // if get goal, update start
                    if( distSQBtObjects(msg, g_start_position) > MIN_TRAVEL_DISTSQ )
                    {
                        //g_state.object_start_x = -0.5*(msg.Position_XYZ[0].y + g_last_position.Position_XYZ[0].y);
                        //g_state.object_start_y = -0.5*(msg.Position_XYZ[0].x + g_last_position.Position_XYZ[0].x) - 0.61;
                        g_state.object_start_x = -1.0*msg.Position_XYZ[0].y;
                        g_state.object_start_y = -1.0*msg.Position_XYZ[0].x - 0.61;
                        g_state.start_pixel_x = msg.center_pixel_x;
                        g_state.start_pixel_y = msg.center_pixel_y;
                        g_state.state++;
#if SHOW_LOG
                        ROS_INFO("[positionCallback()] Update g_state.state = %d, object_start = (%.2f, %.2f).", g_state.state, g_state.object_start_x, g_state.object_start_y);
#endif
                    }
                    break;
                case 7 : // motion of modman done and waiting for final position
                    g_state.object_final_x = -0.5*(msg.Position_XYZ[0].y + g_last_position.Position_XYZ[0].y);
                    g_state.object_final_y = -0.5*(msg.Position_XYZ[0].x + g_last_position.Position_XYZ[0].x) - 0.61;
                    g_state.state++;
#if SHOW_LOG
                    ROS_INFO("[positionCallback()] Update g_state.state = %d, object_final = (%.2f, %.2f).", g_state.state, g_state.object_final_x, g_state.object_final_y);
#endif
                    break;
            }
        }
    }
    else
    {
        g_last_position = msg;
        g_position_count = 0;
    }
}

void stateCallback(const modman_msgs::modman_state& msg)
{
    // update g_State when the state and msg are valid states
    if( g_state.state == 2 && msg.state == 3 )
    {
        g_state.robot_start_x = msg.robot_start_x;
        g_state.robot_start_y = msg.robot_start_y;
        g_state.robot_end_x = msg.robot_end_x;
        g_state.robot_end_y = msg.robot_end_y;
        g_state.state = msg.state;
#if SHOW_LOG
        ROS_INFO("[stateCallback()] Update g_state.state = %d, robot motion : (%.2f, %.2f) -> (%.2f, %.2f).", g_state.state, g_state.robot_start_x, g_state.robot_start_y, g_state.robot_end_x, g_state.robot_end_y);
#endif
    }
    else if( msg.state == 9 && g_state.state == 8 )
    {
#if SHOW_LOG
        ROS_INFO("[stateCallback()] Update g_state.state = %d. This episode it done.", g_state.state);
#endif
        char c('n');
        while( c != 'y' )
        {
            cout << "Type y <Enter> to continue.";
            cin >> c;
        }
        g_state.state = 0;
#if SHOW_LOG
        ROS_INFO("[stateCallback()] Now we initialize the state, g_state.state = %d. This episode it done.", g_state.state);
#endif
    }
}

void error(const char *msg) {
    perror(msg);
    exit(1);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "ur_client");
    ros::NodeHandle nh;

    double theta;
    g_position_count = -1;
//    ros::Publisher server_pub = nh.advertise<std_msgs::String>("/server_messages/", 10);
    ros::Publisher state_pub = nh.advertise<modman_msgs::modman_state>("/drl_state/", 10);

    ros::Subscriber subP = nh.subscribe("/position_object", 10, positionCallback);
    ros::Subscriber subS = nh.subscribe("/drl_actions", 10, stateCallback);

    // initialize the problem
    g_state.state = 0;
    g_state.object_final_x = -1000.0;
    g_state.object_final_y = -1000.0;
#if SHOW_LOG
    ROS_INFO("[main()] Initialize g_state.state = %d.", g_state.state);
#endif

    ros::Rate loop_rate(MESSAGE_FREQ); // set the rate as defined in the macro MESSAGE_FREQ
//	Listener listener;

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
    ROS_INFO("[main()] Server is running now.");
#endif

    //unsigned int count(0);
    char command[BUF_LEN];
    bzero(command, BUF_LEN);

#if SHOW_LOG
    const unsigned int show_log_period(100);
    unsigned int count = 0;
    unsigned int state_prev;
#endif
    // send to ready pose
    bzero(buffer,BUF_LEN);
    n = read(sockfd,buffer,BUF_LEN);
    if (n < 0)
        error("ERROR reading from socket");
    bzero(command, BUF_LEN);
    sprintf(command,"movel(p[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f])\n",
                            READY_POSE_X, READY_POSE_Y, READY_POSE_Z, READY_POSE_RX, READY_POSE_RY, READY_POSE_RZ);
    n = write(sockfd, command, strlen(command));

    ros::Duration(5.0).sleep();

    // Start loop
    ros::Time start;
    while(ros::ok())
    {
//	    ROS_INFO("[Server] The server is running.");
        bzero(buffer,BUF_LEN);
        n = read(sockfd,buffer,BUF_LEN);
        if (n < 0)
            error("ERROR reading from socket");
	    //ROS_INFO("[while()] I receive the message of %s", buffer);
#if SHOW_LOG
	    state_prev = g_state.state;
#endif
        bzero(command, BUF_LEN);
	    switch( g_state.state )
	    {
	        case 0 : // 0 ~ 2 : waiting for 
	        case 1 : // position of objects and motion generation
	        case 2 : // do nothing
	        case 7 : // 7 : end of motion
            case 8 : // 8 : final position is sent to gym
            case 9 : // 9 : end of episode
                // DO nothing
//                sprintf(command,"movej([%.6f, %.6f, %.6f, %.6f, %.6f, %.6f])\n",
//                                        READY_0, READY_1, READY_2, READY_3, READY_5, READY_5);
	            break;
	        case 3 : // request to move right arm to start pose
                theta = M_PI*0.5 + atan2(g_state.robot_end_y-g_state.robot_start_y, g_state.robot_end_x-g_state.robot_start_x);
                sprintf(command,"movej(p[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f])\n",
                                        g_state.robot_start_x, g_state.robot_start_y, MANIPULATION_HEIGHT,
                                        READY_POSE_RX, theta, theta);
	            start = ros::Time::now();
	            g_state.state++;
	            break;
	        case 4 : // waiting to move start pose
	            if( (ros::Time::now()-start).toSec() > MANIPULATION_WAIT_TIME )
	            {
   	                g_state.state++;
   	            }
   	            break;
    	    case 5 : // request to move end pose
                theta = M_PI*0.5 + atan2(g_state.robot_end_y-g_state.robot_start_y, g_state.robot_end_x-g_state.robot_start_x);
                sprintf(command,"movel(p[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f], a=%.1f, v=%.1f)\n",
                                        g_state.robot_end_x, g_state.robot_end_y, MANIPULATION_HEIGHT,
                                        READY_POSE_RX, theta, theta, UR_ACC, UR_VEL);
   	            start = ros::Time::now();
   	            g_state.state++;
	            break;
	        case 6 : // waiting to move end pose 
	            if( (ros::Time::now()-start).toSec() > MANIPULATION_WAIT_TIME )
	            {
                    sprintf(command,"movel(p[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f])\n",
                                            READY_POSE_X, READY_POSE_Y, READY_POSE_Z, READY_POSE_RX, READY_POSE_RY, READY_POSE_RZ);
   	                g_state.state++;
   	            }
   	            break;
	    }
#if SHOW_LOG
	    if( g_state.state == state_prev )
	    {
	        count++;
	        if( count > show_log_period )
	        {
	            count = 0;
	            ROS_INFO("[while()] Running without change for %u periods. s_state.state = %u.", show_log_period, g_state.state);
	        }
	    }
	    else
	    {
            ROS_INFO("[while()] Update g_state.state = %d, command = %s.", g_state.state, command);
	        count = 0;
	    }
#endif
	    n = write(sockfd, command, strlen(command));
	    if (n < 0) 
	         error("ERROR writing to socket");
	    //ROS_INFO("[Server] I sent message of %s", buffer);
        state_pub.publish(g_state);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

