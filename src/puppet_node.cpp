#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <ecn_common/token_handle.h>
#include <ros/ros.h>
#include <vector>

//Global variables
baxter_core_msgs::JointCommand cmd_msg; // Command message
ros::Publisher pub_command ;//The publisher of the command to move the joint
std::string MODE;
int signs[7] = {-1, 1, -1, 1, -1, 1, -1};  // Mirroring effect
std::vector<std::string> jointNames = {"left_e0", "left_e1", "left_s0", "left_s1", "left_w0", "left_w1", "left_w2"}; // Left arm joints
std::vector<double> jointCommands(7);


void stateCallBack(const sensor_msgs::JointState msg)
{
    jointCommands.clear();
    // Iterate over the right joints
    for( unsigned i = 9; i <= 15 ; i++ ) 
    {
        if(MODE == "pos"){
            jointCommands.push_back(signs[i-9]*msg.position[i]) ;
        }else{
            jointCommands.push_back(signs[i-9]*msg.velocity[i]) ;
        }
    }
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "puppet_node");
    ros::NodeHandle nh("~"); // Use local namespace

    ecn::TokenHandle token("Groupe4");

    // Getting the control mode from the "mode" parameter
    if(!nh.getParam("mode", MODE) )
    {
        ROS_INFO("Couldn't find parameter: mode\n");
    } else {
        ROS_INFO("Puppet Mode: %s\n",MODE.c_str()) ;
    }

    // Publisher declaration
    pub_command = nh.advertise<baxter_core_msgs::JointCommand>("/joint_command", 1);

    // Prepare cmd_msg to control in position the left arm
    cmd_msg.mode = MODE == "pos" ?  cmd_msg.POSITION_MODE : cmd_msg.VELOCITY_MODE; 
    cmd_msg.names = jointNames;

    //Subscribing
    ROS_INFO("Subscribing to topics\n");
    ros::Subscriber baxter_joint_state = nh.subscribe<sensor_msgs::JointState> ("/robot/joint_states"  , 1, stateCallBack);

    int sampling_rate = 100;
    ros::Rate loop(sampling_rate);
    
    // main loop, will continue until ROS is down or Ctrl-C or another node registers with the same name
    while(ros::ok())
    {
        cmd_msg.command = jointCommands;// set the joint commands
        pub_command.publish(cmd_msg); // Publish the command
        // update token, sync with sampling time and activate publish / subscribe
        token.update();
        loop.sleep();
        ros::spinOnce();
    }
}

