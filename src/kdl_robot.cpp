#include "kdl_ros_control/kdl_robot.h"

KDLRobot::KDLRobot()
{

}

KDLRobot::KDLRobot(KDL::Tree &robot_tree)
{
    createChain(robot_tree);
    n_ = chain_.getNrOfJoints();
    grav_ = KDL::JntArray(n_);
    s_J_ee_ = KDL::Jacobian(n_);
    b_J_ee_ = KDL::Jacobian(n_);
    s_J_dot_ee_ = KDL::Jacobian(n_);
    b_J_dot_ee_ = KDL::Jacobian(n_);
    s_J_ee_.data.setZero();
    b_J_ee_.data.setZero();
    s_J_dot_ee_.data.setZero();
    b_J_dot_ee_.data.setZero();
    jntArray_ = KDL::JntArray(n_);
    jntVel_ = KDL::JntArray(n_);
    coriol_ = KDL::JntArray(n_);
    dynParam_ = new KDL::ChainDynParam(chain_,KDL::Vector(0,0,-9.81));
    jacSol_ = new KDL::ChainJntToJacSolver(chain_);
    jntJacDotSol_ = new KDL::ChainJntToJacDotSolver(chain_);
    fkSol_ = new KDL::ChainFkSolverPos_recursive(chain_);
    fkVelSol_ = new KDL::ChainFkSolverVel_recursive(chain_);
    idSolver_ = new KDL::ChainIdSolver_RNE(chain_,KDL::Vector(0,0,-9.81));
    jsim_.resize(n_);
    grav_.resize(n_);
    q_min_.data.resize(n_);
    q_max_.data.resize(n_);
    q_min_.data << -2.96,-2.09,-2.96,-2.09,-2.96, -2.09,-2.96;//
    q_max_.data <<  2.96,2.09,2.96,2.09,2.96, 2.09, 2.96;
    ikVelSol_ = new KDL::ChainIkSolverVel_wdls(chain_);
    ikSol_ = new KDL::ChainIkSolverPos_NR_JL(chain_, q_min_, q_max_, *fkSol_, *ikVelSol_);
    // jntArray_out_ = KDL::JntArray(n_);
}

void KDLRobot::update(std::vector<double> _jnt_values, std::vector<double> _jnt_vel)
{
    KDL::Twist s_T_f;
    KDL::Frame s_F_f;
    KDL::Jacobian s_J_f(7);
    KDL::Jacobian s_J_dot_f(7);
    KDL::FrameVel s_Fv_f;
    KDL::JntArrayVel jntVel(jntArray_,jntVel_);
    KDL::Twist s_J_dot_q_dot_f;

    // joints space
    updateJnts(_jnt_values, _jnt_vel);
    dynParam_->JntToMass(jntArray_, jsim_);
    dynParam_->JntToCoriolis(jntArray_, jntVel_, coriol_);
    dynParam_->JntToGravity(jntArray_, grav_);

    // robot flange
    fkVelSol_->JntToCart(jntVel, s_Fv_f);
    s_T_f = s_Fv_f.GetTwist();
    s_F_f = s_Fv_f.GetFrame();
    int err = fkSol_->JntToCart(jntArray_,s_F_f);
    err = jacSol_->JntToJac(jntArray_, s_J_f);
    err = jntJacDotSol_->JntToJacDot(jntVel, s_J_dot_q_dot_f);
    err = jntJacDotSol_->JntToJacDot(jntVel, s_J_dot_f);

    // robot end-effector
    s_F_ee_ = s_F_f*f_F_ee_;
    KDL::Vector s_p_f_ee = s_F_ee_.p - s_F_f.p;
    KDL::changeRefPoint(s_J_f, s_p_f_ee, s_J_ee_);
    KDL::changeRefPoint(s_J_dot_f, s_p_f_ee, s_J_dot_ee_);
    KDL::changeBase(s_J_ee_, s_F_ee_.M.Inverse(), b_J_ee_);
    KDL::changeBase(s_J_dot_ee_, s_F_ee_.M.Inverse(), b_J_dot_ee_);
    s_V_ee_ = s_T_f.RefPoint(s_p_f_ee);

}


////////////////////////////////////////////////////////////////////////////////
//                                 CHAIN                                      //
////////////////////////////////////////////////////////////////////////////////

void KDLRobot::createChain(KDL::Tree &robot_tree)
{
    //if(!robot_tree.getChain(robot_tree.getRootSegment()->first, "lbr_iiwa_link_7",chain_))
    if(!robot_tree.getChain(robot_tree.getRootSegment()->first, 
        std::prev(std::prev(robot_tree.getSegments().end()))->first, chain_))
    {
        std::cout << "Failed to create KDL robot" << std::endl;
        return;
    }
    std::cout << "KDL robot model created" << std::endl;
    std::cout << "with " << chain_.getNrOfJoints() << " joints" << std::endl;
    std::cout << "and " << chain_.getNrOfSegments() << " segments" << std::endl;
}

unsigned int KDLRobot::getNrJnts()
{
    return n_;
}

unsigned int KDLRobot::getNrSgmts()
{
    return chain_.getNrOfSegments();
}

////////////////////////////////////////////////////////////////////////////////
//                                 JOINTS                                     //
////////////////////////////////////////////////////////////////////////////////

void KDLRobot::updateJnts(std::vector<double> _jnt_pos, std::vector<double> _jnt_vel)
{
    for (unsigned int i = 0; i < n_; i++)
    {
        //std::cout << _jnt_pos[i] << std::endl;
        jntArray_(i) = _jnt_pos[i];
        jntVel_(i) = _jnt_vel[i];
    }
}
Eigen::VectorXd KDLRobot::getJntValues()
{
    return jntArray_.data;
}

Eigen::VectorXd KDLRobot::getJntVelocities()
{
    return jntVel_.data;
}

Eigen::MatrixXd KDLRobot::getJntLimits()
{
    Eigen::MatrixXd jntLim;
    jntLim.resize(n_,2);

    jntLim.col(0) = q_min_.data;
    jntLim.col(1) = q_max_.data;

    return jntLim;
}

Eigen::MatrixXd KDLRobot::getJsim()
{
    return jsim_.data;
}

Eigen::VectorXd KDLRobot::getCoriolis()
{
    return coriol_.data;
}

Eigen::VectorXd KDLRobot::getGravity()
{
    return grav_.data;
}

Eigen::VectorXd KDLRobot::getID(const KDL::JntArray &q,
                                const KDL::JntArray &q_dot,
                                const KDL::JntArray &q_dotdot,
                                const KDL::Wrenches &f_ext)
{
    Eigen::VectorXd t;
    t.resize(chain_.getNrOfJoints());
    KDL::JntArray torques(chain_.getNrOfJoints());
    int r = idSolver_->CartToJnt(q,q_dot,q_dotdot,f_ext,torques);
    std::cout << "idSolver result: " << idSolver_->strError(r) << std::endl;
    // std::cout << "torques: " << torques.data.transpose() << std::endl;
    t = torques.data;
    return t;
}

KDL::JntArray KDLRobot::getInvKin(const KDL::JntArray &q,
                        const KDL::Frame &eeFrame)
{
    KDL::JntArray jntArray_out_;
    jntArray_out_.resize(chain_.getNrOfJoints());
    int err = ikSol_->CartToJnt(q, eeFrame, jntArray_out_);
    if (err != 0)
    {
        printf("inverse kinematics failed with error: %d \n", err);
    }
    return jntArray_out_;
}
////////////////////////////////////////////////////////////////////////////////
//                              END-EFFECTOR                                  //
////////////////////////////////////////////////////////////////////////////////

KDL::Frame KDLRobot::getEEFrame()
{
    return s_F_ee_;
}


KDL::Frame KDLRobot::getFlangeEE()
{
    return f_F_ee_;
}


KDL::Twist KDLRobot::getEEVelocity()
{
    return s_V_ee_;
}

KDL::Twist KDLRobot::getEEBodyVelocity()
{
    return s_V_ee_;
}

KDL::Jacobian KDLRobot::getEEJacobian()
{
    return s_J_ee_;
}

KDL::Jacobian KDLRobot::getEEBodyJacobian()
{
    return b_J_ee_;
}

Eigen::VectorXd KDLRobot::getEEJacDotqDot()
{
    return s_J_dot_ee_.data;
}

void KDLRobot::addEE(const KDL::Frame &_f_F_ee)
{
    f_F_ee_ = _f_F_ee;
    this->update(toStdVector(this->jntArray_.data), toStdVector(this->jntVel_.data));
}