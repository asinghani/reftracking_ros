#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "pathfinder_ros/Path.h"
#include "pathfinder_ros/PathSegment.h"
#include "tf/transform_listener.h"
using namespace ros;

#include <iostream>
#include <vector>
#include <cstdlib>
#include <math.h>
using namespace std;

#define PI 3.1415926535

#define STATE_IDLE        0
#define STATE_ALIGN_START 1
#define STATE_FOLLOW      2
#define STATE_ALIGN_END   3

int state = STATE_IDLE;

pathfinder_ros::Path::ConstPtr path;
int pathHz; // Path rate (hertz)
ros::Rate* loopRate = NULL;
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

double convertAngle(double angle) {
    double n = angle + 0.0;
    while(n > PI) {
        n -= 2.0 * PI;
    }

    while(n < -1.0 * PI) {
        n += 2.0 * PI;
    }

    return n;
}

double gainFunc(double linvel, double angvel, double b, double zeta) {
    return 2.0 * zeta * sqrt( (angvel * angvel) + ((b) * (linvel * linvel)) );
}

void pathUpdate(const pathfinder_ros::Path::ConstPtr& msg) {
    cout << "Recieved Path" << endl;
    path = msg;
    pathHz = static_cast<int>(1.0 / path->path[0].dt);
    ros::Rate tempRate(pathHz);
    loopRate = &tempRate;

    pathTf = msg->header.frame_id;
    cout << "Entering STATE_ALIGN_START" << endl;
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

    ros::Rate waitForMap(20);

    int count = 0;

    while(ros::ok()) {
        geometry_msgs::Twist msg;
        msg.linear.x  = 0.0; msg.linear.y  = 0.0; msg.linear.z  = 0.0;
        msg.angular.x = 0.0; msg.angular.y = 0.0; msg.angular.z = 0.0;

        try {
            tf::StampedTransform transform;
            listener.lookupTransform(robotTf, pathTf, ros::Time(0), transform);
            double robotX = (double) (transform.getOrigin().getX());
            double robotY = (double) (transform.getOrigin().getY());
            double robotTheta = getYaw(transform);

            if(state == STATE_IDLE) {
                // Don't move
                msg.linear.x = 0.0;
                msg.angular.z = 0.0;
            } else if(state == STATE_ALIGN_START) {
                msg.linear.x = 0.0;

                pathfinder_ros::PathSegment seg = path->path[0];
                double angVel = alignP * (robotTheta - seg.heading);
                msg.angular.z = angVel;

                if(abs(robotTheta - seg.heading) <= alignTolerance) {
                    cout << "Entering STATE_FOLLOW" << endl;
                    state = STATE_FOLLOW;
                    count = 0;
                    msg.angular.z = 0.0;
                }
            } else if(state == STATE_ALIGN_END) {
                msg.linear.x = 0.0;

                pathfinder_ros::PathSegment seg = path->path[path->path.size() - 1];
                double angVel = alignP * (robotTheta - seg.heading);
                msg.angular.z = angVel;

                if(abs(robotTheta - seg.heading) <= alignTolerance) {
                    cout << "Entering STATE_IDLE" << endl;
                    state = STATE_IDLE;
                    msg.angular.z = 0.0;
                }
            } else if(state == STATE_FOLLOW) {
                pathfinder_ros::PathSegment seg = path->path[count];
                double xError = seg.x - robotX;
                double yError = seg.y - robotY;
                double thetaError = convertAngle(seg.heading - robotTheta);
                if(abs(thetaError) < 0.0001) { thetaError = 0.0001; }

                double vFF = seg.velocity;
                double wFF = seg.angular_velocity;
                double theta = seg.heading;

                // https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf (5.12)
                double linvel = (vFF * cos(thetaError)) + (gainFunc(vFF, wFF, b, zeta) * (xError * cos(theta) + yError * sin(theta)));
                double angvel = wFF + (b * vFF * (sin(thetaError) / thetaError) * (yError * cos(theta) - xError * sin(theta))) +
                                (gainFunc(vFF, wFF, b, zeta) * thetaError);
                
                msg.linear.x = linvel;
                msg.angular.z = angvel;

                if(count < (path->path.size() - 1)) {
                    count++;
                } else {
                    cout << "Entering STATE_ALIGN_END" << endl;
                    state = STATE_ALIGN_END;
                    msg.linear.x = 0.0;
                    msg.angular.z = 0.0;
                }
            }

        } catch (tf::TransformException &ex) {
            ROS_ERROR("Transform lookup error: %s", ex.what());

            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
        }

        publisher.publish(msg);
        ros::spinOnce();
        if(loopRate == NULL) {
            waitForMap.sleep(); 
        } else {
            loopRate->sleep();
        }
    }

    return 0;
}
