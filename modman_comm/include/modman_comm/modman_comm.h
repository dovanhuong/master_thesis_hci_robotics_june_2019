#ifndef _MODMAN_COMM_H_
#define _MODMAN_COMM_H_

#define PORT_NUMBER 5500
#define BUF_LEN 512

struct modman_command
{
    int right_arm_command; // 0 stand still, 1 move in joint space, 2 move straight in workspace
    int right_arm_data_type; // 0 not in use, 1 joint value, 2 workspace value[x,y,z,r,p,y]
    double right_arm_target[6]; // target value
    int right_arm_led; // 0 no change, 1 blue, 2 green, 3 red, 4 turn off

    int left_arm_command; // 0 stand still, 1 move in joint space, 2 move straight in workspace
    int left_arm_data_type; // 0 not in use, 1 joint value, 2 workspace value[x,y,z,r,p,y]
    double left_arm_target[6]; // target value
    int left_arm_led; // 0 no change, 1 blue, 2 green, 3 red, 4 turn off
};

void commandToString(const struct modman_command& command, char* string)
{
    sprintf(string,"%d%d%10f%10f%10f%10f%10f%10f%d%d%d%10f%10f%10f%10f%10f%10f%d",
    command.right_arm_command, command.right_arm_data_type, command.right_arm_target[0], command.right_arm_target[1], command.right_arm_target[2], command.right_arm_target[3], command.right_arm_target[4], command.right_arm_target[5], command.right_arm_led,
    command.left_arm_command, command.left_arm_data_type, command.left_arm_target[0], command.left_arm_target[1], command.left_arm_target[2], command.left_arm_target[3], command.left_arm_target[4], command.left_arm_target[5], command.left_arm_led);
    return;
}

void stringToCommand(const char* string, struct modman_command& command)
{
    char buffer[16];
    bzero(buffer,16);
    memcpy(buffer, string, 1);
    command.right_arm_command = atoi(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+1, 1);
    command.right_arm_data_type = atoi(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+2, 10);
    command.right_arm_target[0] = atof(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+12, 10);
    command.right_arm_target[1] = atof(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+22, 10);
    command.right_arm_target[2] = atof(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+32, 10);
    command.right_arm_target[3] = atof(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+42, 10);
    command.right_arm_target[4] = atof(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+52, 10);
    command.right_arm_target[5] = atof(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+62, 1);
    command.right_arm_led = atoi(buffer);

    bzero(buffer,16);
    memcpy(buffer, string+63, 1);
    command.left_arm_command = atoi(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+64, 1);
    command.left_arm_data_type = atoi(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+65, 10);
    command.left_arm_target[0] = atof(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+75, 10);
    command.left_arm_target[1] = atof(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+85, 10);
    command.left_arm_target[2] = atof(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+95, 10);
    command.left_arm_target[3] = atof(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+105, 10);
    command.left_arm_target[4] = atof(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+115, 10);
    command.left_arm_target[5] = atof(buffer);
    bzero(buffer,16);
    memcpy(buffer, string+125, 1);
    command.left_arm_led = atoi(buffer);
}

void print_command(const struct modman_command& command)
{
    printf("[Right Arm] command = %d.\n", command.right_arm_command);
    printf("[Right Arm] data_type = %d.\n", command.right_arm_data_type);
    printf("[Right Arm] target = [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f].\n", command.right_arm_target[0], command.right_arm_target[1], command.right_arm_target[2], command.right_arm_target[3], command.right_arm_target[4], command.right_arm_target[5]);
    printf("[Right Arm] arm_led = %d.\n", command.right_arm_led);

    printf("[Left  Arm] command = %d.\n", command.left_arm_command);
    printf("[Left  Arm] data_type = %d.\n", command.left_arm_data_type);
    printf("[Left  Arm] target = [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f].\n", command.left_arm_target[0], command.left_arm_target[1], command.left_arm_target[2], command.left_arm_target[3], command.left_arm_target[4], command.left_arm_target[5]);
    printf("[Left  Arm] arm_led = %d.\n", command.left_arm_led);
}

#endif // _MODMAN_COMM_H_
