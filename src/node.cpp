#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "pathfinder_ros/Path.h"
#include "pathfinder_ros/PathSegment.h"
#include "tf/transform_listener.h"
using namespace ros;

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
using namespace std;

#define PI 3.1415926535

#define STATE_IDLE        0
#define STATE_ALIGN_START 1
#define STATE_FOLLOW      2
#define STATE_ALIGN_END   3

int state = STATE_IDLE;

pathfinder_ros::Path::ConstPtr path;
int pathHz; // Path rate (hertz)
ros::Rate loopRate(0);
std::string pathTf;

double getYaw(tf::Transform tf) {
    tf::Matrix3x3 m(tf.getRotation());
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if(yaw < 0.0) {
        yaw += PI * 2.0;
    }
    return yaw;
}

void pathUpdate(const pathfinder_ros::Path::ConstPtr& msg) {
    cout << "Recieved Path" << endl;
    path = msg;
    pathHz = static_cast<int>(1.0 / path->path[0].dt);
    loopRate = ros::Rate(pathHz);

    pathTf = path->header.frame_id;
    state = STATE_ALIGN_START;

    cout << pathHz << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "reftracking_node");
    NodeHandle n;
    NodeHandle params("~");

    float zeta = params.param("zeta", 0.0);
    float b = params.param("b", 0.0);
    float alignP = params.param("align_coeff", 1.0);
    float alignTolerance = params.param("align_tolerance", 0.2);
    float goalPosTolerance = params.param("goal_tolerance", 0.2);
    std::string robotTf = params.param<std::string>("robot_tf", "/base_link");
    std::string pathTopic = params.param<std::string>("path_topic", "/pathfinder_ros/path_references");

    // TODO: replace with proper rosout print
    cout << "====================================================" << endl;
    cout << "Path Follower Config: " << endl;
    cout << "Zeta: " << zeta << endl;
    cout << "B: " << b << endl;
    cout << "Align Proportional Coefficient: " << alignP << endl;
    cout << "Align Tolerance: " << alignTolerance << endl;
    cout << "Goal Tolerance: " << goalPosTolerance << endl;
    cout << "Robot TF: " << robotTf << endl;
    cout << "Path Topic: " << pathTopic << endl;
    cout << "====================================================" << endl;

    Subscriber sub = n.subscribe(pathTopic, 1000, pathUpdate);

    Publisher publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    tf::TransformListener listener;

    while(ros::ok()) {
        geometry_msgs::Twist msg;
        msg.linear.x  = 0.0; msg.linear.y  = 0.0; msg.linear.z  = 0.0;
        msg.angular.x = 0.0; msg.angular.y = 0.0; msg.angular.z = 0.0;

        try {
            tf::StampedTransform transform;
            listener.lookupTransform(robotTf, pathTf, ros::Time(0), transform);
            robotX = 

            if(state == STATE_IDLE) {
                // Don't move
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
            } else if(state == STATE_ALIGN_START) {
                
            }

        } catch (tf::TransformException &ex) {
            ROS_ERROR("Transform lookup error: %s", ex.what());

            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
        }

        publisher.publish(vel_msg);
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
