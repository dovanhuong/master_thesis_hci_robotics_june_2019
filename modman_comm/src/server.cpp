#include "ros/ros.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "time.h"
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include "std_msgs/String.h"

#include "modman_comm/modman_comm.h"

using namespace std;

void error(const char *msg) {
    perror(msg);
    exit(1);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "server_node");
    ros::NodeHandle nh;
    ros::Publisher server_pub = nh.advertise<std_msgs::String>("/server_messages/", 1000);

    //Testing package is working fine
    int sockfd, newsockfd, portno; //Socket file descriptors and port number
    socklen_t clilen; //object clilen of type socklen_t
    char buffer[BUF_LEN]; //buffer array of size 256
    struct sockaddr_in serv_addr, cli_addr; ///two objects to store client and server address
    std_msgs::String message;
    std::stringstream ss;
    int n;
    ros::Duration d(0.01); // 100Hz
    // port number is fix
    /*if (argc < 2) {
        fprintf(stderr,"ERROR, no port provided\n");
        exit(1);
    }*/
    //portno = atoi(argv[1]);
    portno = PORT_NUMBER;

    cout << "Hello there! This node is listening on port " << portno << " for incoming connections" << endl;
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

    unsigned int count(0);
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

    while(ros::ok())
    {
	    ROS_INFO("[Server] The server is running.");
        bzero(buffer,256);
        n = read(sockfd,buffer,BUF_LEN);
	    ROS_INFO("[Server] n = %d, I receive message of %s", n, buffer);
       	//sprintf(buffer,"I want to send message of %u", count++);
	    commandToString(command,buffer);
	    n = write(newsockfd,buffer,strlen(buffer));
	    if (n < 0) 
	         error("ERROR writing to socket");
	    ROS_INFO("[Server] I sent message of %s", buffer);
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

