#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

geometry_msgs::PoseStamped pose;

void update_pose(){
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = cos(ros::Time::now().toSec());
    pose.pose.position.y = sin(ros::Time::now().toSec());
}

int main(int argc, char** argv){
    ros::init(argc, argv, "quad");
    ros::NodeHandle nh;
    ros::Rate rate(30);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("quad_pose", 1);

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    pose.header.frame_id = "/map";

    ROS_INFO("Quad ready");

    while(ros::ok()){
        update_pose();
        pose_pub.publish(pose);
        rate.sleep();
    }
}
