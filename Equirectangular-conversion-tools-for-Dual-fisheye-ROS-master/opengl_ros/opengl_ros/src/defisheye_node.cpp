#include "defisheye_nodecore.h"

using namespace opengl_ros;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "defisheye");

    DefisheyeNode node(ros::NodeHandle(), ros::NodeHandle("~"));
    node.run();
}
