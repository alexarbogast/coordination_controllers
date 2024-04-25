#pragma once

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>

namespace coordinated_motion_controllers
{

// splits a kinematic chain   "work" ----- "base" ----- "eef"
class KinematicChain
{
public:
  KinematicChain(const KDL::Tree& robot_tree, const std::string& work_frame,
                 const std::string& base_frame, const std::string& eef_frame)
    : work_frame_(work_frame), base_frame_(base_frame), eef_frame_(eef_frame)
  {
  }

private:
  std::string work_frame_, base_frame_, eef_frame_;
  KDL::Chain chain_;
};

}  // namespace coordinated_motion_controllers
