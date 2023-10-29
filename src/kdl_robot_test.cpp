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

// Main
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Please, provide a path to a URDF file...\n");
        return 0;
    }

    // Create robot
    KDLRobot robot = createRobot(argv[1]);
    int nrJnts = robot.getNrJnts();

    // Jnt values
    std::vector<double> q(nrJnts, 1.0), qd(nrJnts, 0.0);
    robot.update(q, qd);
    robot.addEE(KDL::Frame::Identity());

    // Direct kinematics
    KDL::Frame F = robot.getEEFrame();
    std::cout << F << std::endl;

    // Inverse kinematics
    KDL::Frame Fd = KDL::Frame(robot.getEEFrame().M, robot.getEEFrame().p - KDL::Vector(0,0,0.1));
    KDL::JntArray q_kdl(7);
    q_kdl.data << q[0], q[1], q[2], q[3], q[4], q[5], q[6];  
    q_kdl = robot.getInvKin(q_kdl, Fd);

    for(unsigned int i = 0; i < q_kdl.data.size(); i++)
        std::cout << "joint " << i << "= " << q_kdl.data[i] << std::endl;

    return 0;
}
