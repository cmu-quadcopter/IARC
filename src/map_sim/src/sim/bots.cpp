#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <random>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

const double PI = 3.1415926;
const double robot_radius = 1;
const double obstacle_radius = 5;
const int BOT_COUNT = 10;
const int OBSTACLE_COUNT = 4;

class Bot{
    private:
        //Robot properties
        const double wheel_base = 0.24; //Separation between wheels and center of robot
        const double speed = 0.33;	       //normal travel speed
        const double rotation_speed = speed/2; //speed during rotations (left wheel negative, right wheel positive)

        const ros::Duration reversal_period = ros::Duration(20.0); //auto reverse occurs every 20 seconds
        const ros::Duration noise_period = ros::Duration(5.0);	   //noise injection occurs every 5 seconds

        const double randomMax = PI/9; //noise up to 20 degrees 

        const ros::Duration noise_duration = ros::Duration(0.85);
        //180 degrees auto reversal or collision reversal
        const ros::Duration reversal_duration = ros::Duration(PI * wheel_base / rotation_speed); 
        //45 degrees top touch rotation
        const ros::Duration touch_duration = ros::Duration(reversal_duration.toSec()/4);    

        //State parameters
        int state = 0; //0: idle, 1:normal, 2: noise injection, 3:touch turning, 4:reversing, 5:colliding

        std::mt19937 gen;
        std::uniform_real_distribution<> dis;
        double current_noise;

        ros::Time last_time; 	 //time at the last update
        ros::Time last_noise; 	 //time at the last noise injection
        ros::Time last_reversal; //time at the last reversal
        ros::Time last_collision; //time at the last reversal
        ros::Time last_touch;	 //time at the last top touch

    public:
        int id;
        geometry_msgs::Pose pose;
        Bot(geometry_msgs::Pose starting_pose){
            last_time = ros::Time::now();
            last_reversal = last_time;
            last_noise = last_time;
            this->pose = starting_pose;
            state = 1;

            std::random_device rd;
            gen = std::mt19937(rd());
            dis = std::uniform_real_distribution<>(-1*randomMax, randomMax);
        }

        void check_collisions(ros::Time now, const std::vector<geometry_msgs::Pose> &poses){
            state = 4;
            //TODO implement collision detection
            return;
            last_collision = now;
        }

        geometry_msgs::Pose update_pose(ros::Time now){
            double dt = (now - last_time).toSec();
            last_time = now;

            double yaw;
            yaw = tf::getYaw(pose.orientation);

            switch(state){
                case 0: //Idle
                    break;
                case 1: //Normal
                    pose.position.x += speed * dt*cos(yaw);
                    pose.position.y += speed * dt*sin(yaw);

                    if(now - last_reversal > reversal_period){
                        ROS_INFO("REVERSING");
                        state = 4;
                        last_reversal = now;
                    }
                    else if(now - last_noise > noise_period){
                        ROS_INFO("Injecting noise");
                        state = 2;
                        last_noise = now;
                        current_noise = dis(gen);
                    }
                    break;
                case 2: //Noise injection
                    yaw += dt/noise_duration.toSec() * current_noise;
                    pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                    pose.position.x += speed * dt*cos(yaw);
                    pose.position.y += speed * dt*sin(yaw);

                    if(now - last_noise > noise_duration){
                        state = 1;
                    }
                    break;
                case 3: //Touch turning
                    yaw += dt/touch_duration.toSec() * PI;
                    pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                    if(now - last_touch > touch_duration){
                        state = 1;
                    }
                    break;
                case 4: //Reversing
                    yaw += dt/reversal_duration.toSec() * PI;
                    pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

                    if(now - last_reversal > reversal_duration){
                        state = 1;
                    }
                    break;
                case 5: //Colliding
                    yaw += dt/reversal_duration.toSec() * PI;
                    pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

                    if(now - last_collision > reversal_duration){
                        state = 1;
                    }
                    break;
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
    for(int i = 0; i < OBSTACLE_COUNT; i++){
        obs_poses.poses.at(i) = obs.at(i).update_pose(now);
    }
    for(int i = 0; i < BOT_COUNT; i++){
        bot_poses.poses.at(i) = bots.at(i).update_pose(now);
    }
    for(int i = 0; i < BOT_COUNT; i++){
        bots.at(i).check_collisions(now, bot_poses.poses);
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

    ROS_INFO("%d bots initialized", BOT_COUNT);

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

    ROS_INFO("%d obstacles initialized", OBSTACLE_COUNT);

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
