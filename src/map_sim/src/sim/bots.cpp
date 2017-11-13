#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>

class Bot{
    public:
        int id;
        geometry_msgs::Pose pose;
        geometry_msgs::Twist twist;
        Bot(geometry_msgs::Pose starting_pose){
            this->pose = starting_pose;
        }
        geometry_msgs::Pose update_pose(double dt){
            pose.position.x += twist.linear.x * dt;
            pose.position.y += twist.linear.y * dt;

            return pose;
        }
};

geometry_msgs::PoseArray bot_poses;
std::vector<Bot> bots;

const int BOT_COUNT = 10;

void update_poses(double dt){
    for(int i = 0; i < BOT_COUNT; i++){
        bot_poses.poses.at(i) = bots.at(i).update_pose(dt);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "bots_node");
    ros::NodeHandle nh;
    ros::Publisher poses_pub = nh.advertise<geometry_msgs::PoseArray>("bot_poses", BOT_COUNT);
    ros::Rate rate(30);
    ros::Time last_t = ros::Time::now();
    bot_poses.header.frame_id = "/map";

    for(int i = 0; i < BOT_COUNT; i++){
        geometry_msgs::Pose p;
        p.position.x = 0;
        p.position.y = 0;
        p.position.z = 0;
        p.orientation.x = 0;
        p.orientation.y = 0;
        p.orientation.z = 0;
        p.orientation.w = 1;

        Bot b(p);
        b.id = i;

        bots.push_back(b);
        bot_poses.poses.push_back(p);
    }

    ROS_INFO("%d Bots initialized", BOT_COUNT);

    while(ros::ok()){
        double dt = (ros::Time::now() - last_t).toSec();
        last_t = ros::Time::now();
        update_poses(dt);

        bot_poses.header.stamp = ros::Time::now();
        poses_pub.publish(bot_poses);
        rate.sleep();
    }
}
