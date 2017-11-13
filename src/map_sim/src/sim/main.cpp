#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

geometry_msgs::Pose quad_pose;
std::vector<geometry_msgs::Pose> bot_poses;

void quad_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    quad_pose = msg->pose;
}

void bot_callback(const geometry_msgs::PoseArray::ConstPtr& msg){
    bot_poses = msg->poses;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "main_sim");
    ros::NodeHandle nh;
    ros::Rate rate(30);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("quad_marker", 1);
    ros::Publisher bots_pub = nh.advertise<visualization_msgs::MarkerArray>("bot_markers", 1);
    ros::Subscriber quad_sub = nh.subscribe("quad_pose", 1, quad_callback);
    ros::Subscriber bots_sub = nh.subscribe("bot_poses", 10, bot_callback);

    ROS_INFO("Sim Ready");
    while(ros::ok()){

        visualization_msgs::Marker marker;

        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "quad";
        marker.id = 0;
        
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        marker.pose = quad_pose;
        marker_pub.publish(marker);

        visualization_msgs::MarkerArray markers;
        for(int i = 0; i < bot_poses.size(); i++){
            visualization_msgs::Marker m;
            m.header.frame_id = "/map";
            m.header.stamp = ros::Time::now();
            m.ns = "bots";
            m.id = i;

            m.type = visualization_msgs::Marker::CYLINDER;
            m.action = visualization_msgs::Marker::ADD;

            m.pose = bot_poses.at(i);
            m.scale.x = 0.5;
            m.scale.y = 0.5;
            m.scale.z = 0.1;

            m.color.r = 1.0f;
            m.color.g = 1.0f;
            m.color.b = 1.0f;
            m.color.a = 1.0;

            m.lifetime = ros::Duration();

            markers.markers.push_back(m);
        }

        bots_pub.publish(markers);

        rate.sleep();

        ros::spinOnce();
    }

    return 1;
}
