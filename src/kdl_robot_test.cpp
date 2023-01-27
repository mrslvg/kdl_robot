#include "kdl_ros_control/kdl_robot.h"
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"

// Functions
KDLRobot createRobot(std::string robot_string)
{
    
    KDL::Tree robot_tree;
    urdf::Model my_model;
    if (!my_model.initFile(robot_string))
    {
        printf("Failed to parse urdf robot model \n");
    }
    if (!kdl_parser::treeFromUrdfModel(my_model, robot_tree))
    {
        printf("Failed to construct kdl tree \n");
    }
    
    KDLRobot robot(robot_tree);
    return robot;
}



int main(int argc, char **argv)
{
    // Create robot
    KDLRobot robot = createRobot("../../../UKAEA_5_DOFS.urdf");
    std::vector<double> q(5, 0.0), qd(5, 0.0);
    robot.update(q, qd);
    KDL::Frame F = robot.getEEFrame();
    std::cout << F << std::endl;
    return 0;
}
