#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <ros/ros.h>

#include <coordinated_motion_controllers/kinematics.h>

static const std::string NAME = "coordinated_motion_test";

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
    coordinated_motion_controllers::KinematicChain chain(
        robot_tree_, "positioner", "rob1_base", "rob1_typhoon_extruder");

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

  ROS_INFO("Executed successfully");
  // ros::spin();
  return 0;
}
