#include <memory>

#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/Dense>

#include <ros/ros.h>

#include <coordinated_motion_controllers/kinematics.h>

static const std::string NAME = "coordinated_motion_test";

// ros parameters
static const std::string BASE_LINK = "rob1_base_link";
static const std::string TIP_LINK = "rob1_tool0";
static const std::string POS_LINK = "positioner";

namespace coordinated_motion_controllers
{

class CoordinatedMotionTest
{
public:
  CoordinatedMotionTest(ros::NodeHandle& nh) : nh_(nh) {}

  bool init()
  {
    std::string robot_description;
    if (!ros::param::search("robot_description", robot_description))
    {
      ROS_ERROR(
          "Searched enclosing namespaces for robot_description but "
          "nothing found");
      return false;
    }
    if (!nh_.getParam(robot_description, robot_description))
    {
      ROS_ERROR_STREAM("Failed to load " << robot_description
                                         << " from parameter server");
      return false;
    }

    urdf::Model robot_model;
    if (!robot_model.initString(robot_description))
    {
      ROS_ERROR("Failed to initialize urdf model from 'robot_description'");
      return false;
    }
    if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree_))
    {
      ROS_FATAL("Failed to parse KDL tree from  urdf model");
      return false;
    }

    return true;
  }

  bool run()
  {
    static const double alpha = 0.1;  // pinv damping value

    /* single robot chain */
    KDL::Chain robot_chain;
    if (!robot_tree_.getChain(BASE_LINK, TIP_LINK, robot_chain))
    {
      ROS_FATAL_STREAM("Failed to build kinematic chain from '"
                       << BASE_LINK << "' to '" << TIP_LINK
                       << "'. Make sure these links exist in the URDF.");
      return false;
    }
    std::unique_ptr<KDL::ChainJntToJacSolver> robot_jac_solver =
        std::make_unique<KDL::ChainJntToJacSolver>(robot_chain);

    KDL::JntArray positions;
    positions.data = Eigen::Matrix<double, 6, 1>::Zero();
    KDL::Jacobian jac(positions.rows());

    robot_jac_solver->JntToJac(positions, jac);
    std::cout << "robot jacobian:\n" << jac.data << std::endl;

    Eigen::Matrix<double, 6, 6> identity6x6 =
        Eigen::Matrix<double, 6, 6>::Identity();

    Eigen::Matrix<double, 6, 6> pinv_jac =
        jac.data.transpose() *
        (jac.data * jac.data.transpose() + alpha * alpha * identity6x6)
            .inverse();

    std::cout << "pinv:\n" << pinv_jac << std::endl;

    /* robot positioner chain */
    KDL::Chain robot_pos_chain;
    if (!robot_tree_.getChain(POS_LINK, TIP_LINK, robot_pos_chain))
    {
      ROS_FATAL_STREAM("Failed to build kinematic chain from '"
                       << POS_LINK << "' to '" << TIP_LINK
                       << "'. Make sure these links exist in the URDF.");
      return false;
    }
    std::unique_ptr<KDL::ChainJntToJacSolver> robot_pos_jac_solver =
        std::make_unique<KDL::ChainJntToJacSolver>(robot_pos_chain);

    KDL::JntArray positions2(7);
    positions2.data << 0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    KDL::Jacobian jac2(7);

    robot_pos_jac_solver->JntToJac(positions2, jac2);
    std::cout << "robot-pos jacobian:\n" << jac2.data << std::endl;

    Eigen::Matrix<double, 7, 6> pinv_jac2 =
        jac2.data.transpose() *
        (jac2.data * jac2.data.transpose() + alpha * alpha * identity6x6)
            .inverse();

    std::cout << "pinv:\n" << pinv_jac2 << std::endl;

    return true;
  }

private:
  ros::NodeHandle nh_;
  KDL::Tree robot_tree_;
};

}  // namespace coordinated_motion_controllers

int main(int argc, char** argv)
{
  ros::init(argc, argv, NAME);
  ros::NodeHandle nh(NAME);

  coordinated_motion_controllers::CoordinatedMotionTest app(nh);
  if (!app.init())
  {
    ROS_ERROR_NAMED(NAME, "Failed to initialize server");
    return 1;
  }

  app.run();
  ROS_INFO("Executed successfully");
  return 0;
}

