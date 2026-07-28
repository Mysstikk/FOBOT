#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Dense>
#include <random>
#include <algorithm>
#include <iostream>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
      "singularity_probe",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  robot_model_loader::RobotModelLoader loader(node, "robot_description");
  auto model = loader.getModel();
  moveit::core::RobotState state(model);

  const std::string group_name = "fobot_arm";
  const std::string tip_link = "link_tcp";
  const auto* jmg = model->getJointModelGroup(group_name);
  if (!jmg) {
    std::cerr << "Could not find group '" << group_name << "' — check the name.\n";
    return 1;
  }

  std::vector<double> lower, upper;
  for (const auto* joint : jmg->getActiveJointModels()) {
    const auto& bounds = joint->getVariableBounds(joint->getName());
    lower.push_back(bounds.min_position_);
    upper.push_back(bounds.max_position_);
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::vector<std::uniform_real_distribution<double>> dists;
  for (size_t i = 0; i < lower.size(); ++i)
    dists.emplace_back(lower[i], upper[i]);

  const int N = 5000;
  std::vector<double> conds;
  conds.reserve(N);

  for (int s = 0; s < N; ++s) {
    std::vector<double> q(lower.size());
    for (size_t i = 0; i < q.size(); ++i) q[i] = dists[i](gen);
    state.setJointGroupPositions(jmg, q);
    state.update();

    Eigen::MatrixXd jacobian;
    state.getJacobian(jmg, state.getLinkModel(tip_link), Eigen::Vector3d::Zero(), jacobian);

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
    const auto& sv = svd.singularValues();
    conds.push_back(sv(0) / sv(sv.size() - 1));
  }

  std::sort(conds.begin(), conds.end());
  std::cout << "min: "      << conds.front() << "\n"
            << "median: "   << conds[conds.size() / 2] << "\n"
            << "95th pct: " << conds[static_cast<size_t>(conds.size() * 0.95)] << "\n"
            << "99th pct: " << conds[static_cast<size_t>(conds.size() * 0.99)] << "\n"
            << "max: "      << conds.back() << "\n";

  rclcpp::shutdown();
  return 0;
}
