#include <yaml-cpp/yaml.h>

#include <iostream>

#include "ovinf/ovinf_factory.hpp"
int main() {
  YAML::Node config =
      YAML::LoadFile("/home/dknt/Project/dl_deploy/ovinf/config/humanoid.yaml");

  auto policy = ovinf::PolicyFactory::CreatePolicy(config["inference"]);

  for (size_t i = 0; i < 20; ++i) {
    policy->WarmUp({
        .command = Eigen::Vector3f{0.0, 0.0, 0.0},
        .ang_vel = Eigen::Vector3f{0.0, 0.0, 0.0},
        .proj_gravity = Eigen::Vector3f{0.0, 0.0, 0.0},
        .joint_pos = Eigen::VectorXf::Zero(12),
        .joint_vel = Eigen::VectorXf::Zero(12),
    });
    while (!policy->GetResult().has_value());
  }

  bool status;
  status = policy->InferUnsync({
      .command = Eigen::Vector3f{0.0, 0.0, 0.0},
      .ang_vel = Eigen::Vector3f{0.0, 0.0, 0.0},
      .proj_gravity = Eigen::Vector3f{0.0, 0.0, 0.0},
      .joint_pos = Eigen::VectorXf::Zero(12),
      .joint_vel = Eigen::VectorXf::Zero(12),
  });
  status = policy->InferUnsync({
      .command = Eigen::Vector3f{0.0, 0.0, 0.0},
      .ang_vel = Eigen::Vector3f{0.0, 0.0, 0.0},
      .proj_gravity = Eigen::Vector3f{0.0, 0.0, 0.0},
      .joint_pos = Eigen::VectorXf::Zero(12),
      .joint_vel = Eigen::VectorXf::Zero(12),
  });
  std::cout << "Inference error: " << status << std::endl;

  auto res = policy->GetResult();
  if (res.has_value()) {
    std::cout << "Result: " << res.value().transpose() << std::endl;
  } else {
    std::cout << "Result not ready" << std::endl;
  }

  return 0;
}
