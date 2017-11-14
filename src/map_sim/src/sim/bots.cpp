#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <tf/tf.h>

const double PI = 3.1415926;
const double robot_radius = 1;
const double obstacle_radius = 5;
const int BOT_COUNT = 10;
const int OBSTACLE_COUNT = 4;

class Bot{
    private:
        ros::Time last_time;
        ros::Time last_noise;
        ros::Time last_reversal;

        const double speed = 0.33;
        const double top_rotation = PI/4;
        const double reversal_rotation = PI;
        const double noise_amplitude = PI/9;
        
        const ros::Duration reversal_period = ros::Duration(20.0);
        const ros::Duration noise_period = ros::Duration(5.0);

    public:
        int id;
        geometry_msgs::Pose pose;
        Bot(geometry_msgs::Pose starting_pose){
            last_time = ros::Time::now();
            this->pose = starting_pose;
        }
        geometry_msgs::Pose update_pose(ros::Time now){
            ros::Duration dt = now - last_time;
            last_time = now;

            if(now - last_noise > noise_period)
;
            else if (now - last_reversal > reversal_period)
;
            else{
                
            }

            return pose;
        }
};

class ObstacleBot{
    private:
        ros::Time last_time;

        const double speed = 0.33;
        //Clockwise circular trajectory
        const double trajectory_radius = 5;
    public:
        int id;
        geometry_msgs::Pose pose;
        ObstacleBot(geometry_msgs::Pose starting_pose){
            this->pose = starting_pose;
        }
        geometry_msgs::Pose update_pose(ros::Time now){

            return pose;
        }
};

geometry_msgs::PoseArray bot_poses;
std::vector<Bot> bots;

geometry_msgs::PoseArray obs_poses;;
std::vector<ObstacleBot> obs;

void update_poses(ros::Time now){
    for(int i = 0; i < BOT_COUNT; i++){
        bot_poses.poses.at(i) = bots.at(i).update_pose(now);
    }
    for(int i = 0; i < OBSTACLE_COUNT; i++){
        obs_poses.poses.at(i) = obs.at(i).update_pose(now);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "bots_node");
    ros::NodeHandle nh;
    ros::Publisher bots_pub = nh.advertise<geometry_msgs::PoseArray>("bot_poses", BOT_COUNT);
    ros::Publisher obs_pub = nh.advertise<geometry_msgs::PoseArray>("obstacle_poses", OBSTACLE_COUNT);
    ros::Rate rate(30);
    ros::Time last_t = ros::Time::now();
    bot_poses.header.frame_id = "/map";
    obs_poses.header.frame_id = "/map";

    double dtheta = 2*PI / BOT_COUNT;
    for(int i = 0; i < BOT_COUNT; i++){
        geometry_msgs::Pose p;
        p.position.x = cos(dtheta*i) * robot_radius;
        p.position.y = sin(dtheta*i) * robot_radius;
        p.position.z = 0;

        tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, dtheta*i);

        quaternionTFToMsg(q, p.orientation);

        Bot b(p);
        b.id = i;

        bots.push_back(b);
        bot_poses.poses.push_back(p);
    }

    dtheta = 2*PI / OBSTACLE_COUNT;
    for(int i = 0; i < OBSTACLE_COUNT; i++){
        geometry_msgs::Pose p;
        p.position.x = cos(dtheta*i) * obstacle_radius;
        p.position.y = sin(dtheta*i) * obstacle_radius;
        p.position.z = 0;
        p.orientation.x = 0;
        p.orientation.y = 0;
        p.orientation.z = 0;
        p.orientation.w = 1;

        ObstacleBot o(p);
        o.id = i;

        obs.push_back(o);
        obs_poses.poses.push_back(p);
    }

    ROS_INFO("%d Bots initialized", BOT_COUNT);

    while(ros::ok()){
        ros::Time now = ros::Time::now();
        update_poses(now);

        bot_poses.header.stamp = now;
        bots_pub.publish(bot_poses);
        obs_poses.header.stamp = now;
        obs_pub.publish(obs_poses);
        rate.sleep();
    }
}
