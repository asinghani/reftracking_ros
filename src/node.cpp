#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "pathfinder_ros/Path.h"
#include "pathfinder_ros/PathSegment.h"
#include "tf/transform_listener.h"
using namespace ros;

#include <iostream>
#include <vector>
using namespace std;

#define PI 3.1415926535

#define STATE_IDLE        0
#define STATE_ALIGN_START 1
#define STATE_FOLLOW      2
#define STATE_ALIGN_END   3

int state = STATE_IDLE;

void pathUpdate(const pathfinder_ros::Path::ConstPtr& msg) {

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "reftracking_node");
    NodeHandle n;
    NodeHandle params("~");

    float zeta = params.param("zeta", 0.0);
    float b = params.param("b", 0.0);

    // TODO: replace with proper rosout print
    cout << "====================================================" << endl;
    cout << "Path Follower Config: " << endl;
    cout << "Zeta: " << zeta << endl;
    cout << "B: " << b << endl;
    cout << "====================================================" << endl;

    Subscriber sub = n.subscribe("/reftracking_node/path_references", 1000, pathUpdate);

    // Primarily for visualization
    Publisher publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::spin();

    return 0;
}
