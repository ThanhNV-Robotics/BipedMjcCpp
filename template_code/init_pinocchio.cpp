// Sanity-check test: build a Pinocchio model from a URDF and run forward
// kinematics at the neutral configuration. Run with no arguments to load
// models/urdf/left_leg_6dof.urdf, or pass an explicit URDF path to override
// it.

#include <cstdio>
#include <iostream>
#include <string>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

int main(int argc, char** argv) {
  const std::string default_urdf = "models/urdf/left_leg_6dof.urdf";
  const std::string urdf_path = (argc > 1) ? argv[1] : default_urdf;

  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_path, model);
  pinocchio::Data data(model);

  std::printf("Loaded %s: nq=%d nv=%d njoints=%d\n", urdf_path.c_str(), model.nq, model.nv, model.njoints);

  const Eigen::VectorXd q = pinocchio::neutral(model);
  pinocchio::forwardKinematics(model, data, q);

  for (pinocchio::JointIndex joint_id = 1; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id) {
    std::cout << model.names[joint_id] << ": " << data.oMi[joint_id].translation().transpose() << "\n";
  }
  return 0;
}
