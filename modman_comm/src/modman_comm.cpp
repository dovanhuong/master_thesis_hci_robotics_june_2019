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
#include "std_msgs/String.h"

#include "modman_comm/modman_comm.h"
#include "modman_msgs/modman_state.h"

//#include "find_object_2d/position_object.h"
#include "opencv_object_tracking/position_publish.h"

// paramter for log
#define SHOW_LOG (true)
/*
#if SHOW_LOG
ROS_INFO("");
#endif
*/
// some parameters
#define OBJECT_CONVERGE_MARGIN (0.03)
#define OBJECT_CONVERGE_MARGIN_SQ (OBJECT_CONVERGE_MARGIN*OBJECT_CONVERGE_MARGIN)
#define MIN_TRAVEL_DIST (0.2)
#define MIN_TRAVEL_DISTSQ (MIN_TRAVEL_DIST*MIN_TRAVEL_DIST)
#define OBJECT_CONVERGE_PERIOD (10)
#define MANIPULATION_HEIGHT (0.2)
#define MANIPULATION_WAIT_TIME (4.0)

using namespace std;

modman_msgs::modman_state g_state;
//find_object_2d::position_object g_last_position;
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

//void positionCallback(const find_object_2d::position_object& msg)
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
    //if( distBtObjects(g_last_position, msg) < OBJECT_CONVERGE_MARGIN )
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
                    g_state.object_goal_x = (msg.Position_XYZ[0].x + g_last_position.Position_XYZ[0].x)*0.5;
                    g_state.object_goal_y = (msg.Position_XYZ[0].y + g_last_position.Position_XYZ[0].y)*0.5;
                    g_start_position = msg;
                    g_state.state++;
#if SHOW_LOG
                    ROS_INFO("[positionCallback()] Update g_state.state = %d, object_goal = (%.2f, %.2f).", g_state.state, g_state.object_goal_x, g_state.object_goal_y);
#endif
                    break;
                case 1 : // if get goal, update start
                    if( distSQBtObjects(msg, g_start_position) > MIN_TRAVEL_DISTSQ )
                    {
                        g_state.object_start_x = (msg.Position_XYZ[0].x + g_last_position.Position_XYZ[0].x)*0.5;
                        g_state.object_start_y = (msg.Position_XYZ[0].y + g_last_position.Position_XYZ[0].y)*0.5;
                        g_state.state++;
#if SHOW_LOG
                        ROS_INFO("[positionCallback()] Update g_state.state = %d, object_start = (%.2f, %.2f).", g_state.state, g_state.object_start_x, g_state.object_start_y);
#endif
                    }
                    break;
                case 7 : // motion of modman done and waiting for final position
                    g_state.object_final_x = (msg.Position_XYZ[0].x + g_last_position.Position_XYZ[0].x)*0.5;
                    g_state.object_final_y = (msg.Position_XYZ[0].y + g_last_position.Position_XYZ[0].y)*0.5;
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
    ros::init(argc, argv, "server_node");
    ros::NodeHandle nh;

    g_position_count = -1;
    ros::Publisher server_pub = nh.advertise<std_msgs::String>("/server_messages/", 10);
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

    //Testing package is working fine
    int sockfd, newsockfd, portno; //Socket file descriptors and port number
    socklen_t clilen; //object clilen of type socklen_t
    char buffer[BUF_LEN]; //buffer array of size 256
    struct sockaddr_in serv_addr, cli_addr; ///two objects to store client and server address
    std_msgs::String message;
    std::stringstream ss;
    int n;
    ros::Duration d(0.1); // 100Hz
    // port number is fix
    /*if (argc < 2) {
        fprintf(stderr,"ERROR, no port provided\n");
        exit(1);
    }*/
    //portno = atoi(argv[1]);
    portno = PORT_NUMBER;

//    cout << "Hello there! This node is listening on port " << portno << " for incoming connections" << endl;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    int enable = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0)
        error("setsockopt(SO_REUSEADDR) failed");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if( bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0 )
        error("ERROR on binding");
    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);
    if (newsockfd < 0)
        error("ERROR on accept");
#if SHOW_LOG
    ROS_INFO("[main()] Server is runnign now.");
#endif

    //unsigned int count(0);
    struct modman_command command;
    command.right_arm_command = 1;
    command.right_arm_data_type = 2;
    command.right_arm_target[0] = 1.0;
    command.right_arm_target[1] = -2.0;
    command.right_arm_target[2] = 3.0;
    command.right_arm_target[3] = 4.0;
    command.right_arm_target[4] = -5.0;
    command.right_arm_target[5] = 6.0;
    command.right_arm_led = 4;
    command.left_arm_command = 2;
    command.left_arm_data_type = 1;
    command.left_arm_target[0] = -6.0;
    command.left_arm_target[1] = -5.0;
    command.left_arm_target[2] = 4.0;
    command.left_arm_target[3] = -3.0;
    command.left_arm_target[4] = 2.0;
    command.left_arm_target[5] = -1.0;
    command.left_arm_led = 3;

#if SHOW_LOG
    const unsigned int show_log_period(100);
    unsigned int count = 0;
    unsigned int state_prev;
#endif
    ros::Time start;
    while(ros::ok())
    {
//	    ROS_INFO("[Server] The server is running.");
        bzero(buffer,BUF_LEN);
        n = read(sockfd,buffer,BUF_LEN);
//	    ROS_INFO("[Server] n = %d, I receive message of %s", n, buffer);
       	//sprintf(buffer,"I want to send message of %u", count++);
	    // update command based on g_State
#if SHOW_LOG
	    state_prev = g_state.state;
#endif
	    switch( g_state.state )
	    {
	        case 0 : // 0 ~ 2 : waiting for 
	        case 1 : // position of objects and motion generation
	        case 2 : // do nothing
	        case 7 : // 7 : end of motion
                case 8 : // 8 : final position is sent to gym
                case 9 : // 9 : end of episode
	            command.right_arm_command = 0;
	            command.right_arm_led = 0;
	            command.left_arm_command = 0;
	            command.left_arm_led = 0;
	            break;
	        case 3 : // request to move right arm to start pose
	            command.right_arm_command = 1;
	            command.right_arm_data_type = 2;
	            command.right_arm_target[0] = g_state.robot_start_x;
	            command.right_arm_target[1] = g_state.robot_start_y;
	            command.right_arm_target[2] = MANIPULATION_HEIGHT;
	            command.right_arm_target[3] = 0.0;
	            command.right_arm_target[4] = 0.0;
	            command.right_arm_target[5] = atan2(g_state.robot_end_y-g_state.robot_start_y,g_state.robot_end_x-g_state.robot_start_x);
	            command.right_arm_led = 0;
	            command.left_arm_command = 0;
	            command.left_arm_led = 0;
	            start = ros::Time::now();
	            g_state.state++;
	            break;
	        case 4 : // waiting to move start pose
	            if( (ros::Time::now()-start).toSec() > MANIPULATION_WAIT_TIME )
	            {
    	                command.right_arm_command = 0;
    	                command.right_arm_led = 0;
    	                command.left_arm_command = 0;
    	                command.left_arm_led = 0;
    	                g_state.state++;
    	            }
    	            break;
    	    case 5 : // request to move end pose
  	            command.right_arm_command = 2;
   	            command.right_arm_data_type = 2;
   	            command.right_arm_target[0] = g_state.robot_end_x;
   	            command.right_arm_target[1] = g_state.robot_end_y;
   	            command.right_arm_target[2] = MANIPULATION_HEIGHT;
   	            command.right_arm_target[3] = 0.0;
   	            command.right_arm_target[4] = 0.0;
   	            command.right_arm_target[5] = atan2(g_state.robot_end_y-g_state.robot_start_y,g_state.robot_end_x-g_state.robot_start_x);
   	            command.right_arm_led = 0;
   	            command.left_arm_command = 0;
   	            command.left_arm_led = 0;
   	            start = ros::Time::now();
   	            g_state.state++;
	            break;
	        case 6 : // waiting to move end pose 
	            if( (ros::Time::now()-start).toSec() > MANIPULATION_WAIT_TIME )
	            {
    	                command.right_arm_command = 0;
    	                command.right_arm_led = 0;
    	                command.left_arm_command = 0;
    	                command.left_arm_led = 0;
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
	        if( 3 < g_state.state && g_state.state < 8 )
	        {
	            ROS_INFO("[while()] Update g_state.state = %d, command = [%d, %.2f, %.2f, %.2f, %.2f].", g_state.state, command.right_arm_command, command.right_arm_target[0], command.right_arm_target[1], command.right_arm_target[2], command.right_arm_target[5]*180.0/M_PI);
	        }
	        else
	        {
	            ROS_INFO("[while()] Update g_state.state = %d.", g_state.state);
	        }
	        count = 0;
	    }
#endif
	    commandToString(command,buffer);
	    n = write(newsockfd,buffer,strlen(buffer));
	    if (n < 0) 
	         error("ERROR writing to socket");
	    //ROS_INFO("[Server] I sent message of %s", buffer);
        state_pub.publish(g_state);
        ros::spinOnce();
        d.sleep();
/*        ss.str(std::string()); //Clear contents of string stream
        bzero(buffer,256);
        n = read(newsockfd,buffer,255);
        if (n < 0)
            error("ERROR reading from socket");
        // printf("Here is the message: %s\n",buffer);
        ss << buffer;
        message.data = ss.str();
        ROS_INFO("%s", message.data.c_str());
        server_pub.publish(message);
        n = write(newsockfd,"I got your message",18);
        if (n < 0) error("ERROR writing to socket");
        //close(newsockfd);
        //close(sockfd);
        //ros::spinOnce();
        //d.sleep();
*/
    }
    return 0;
}

