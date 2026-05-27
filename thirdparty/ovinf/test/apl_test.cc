#include "utils/antiparallelogram.hpp"

int main() {
  ovinf::AntiparallelogramLinkage<float> apl{
      {.r = 0.033,
       .l = 0.120,
       .theta_bias = -0.6730129222030294,  // 22.5 degree
       .phi_bias = 0.39269908169872414}};

  {
    auto res = apl.InverseKinematics(M_PI / 2);
    std::cout << "Motor angle: " << res / M_PI * 180.0 << std::endl;
  }
  {
    auto res = apl.ForwardKinematics(-99.8311 / 180.0 * M_PI);
    // auto res = apl.ForwardKinematics(-67.86 / 180.0 * M_PI);
    // auto res = apl.ForwardKinematics(0.0 / 180.0 * M_PI);
    std::cout << "Joint angle: " << res / M_PI * 180.0 << std::endl;
    auto torque = apl.TorqueRemapping(1);
    std::cout << "Torque: " << torque << std::endl;
  }
  return 0;
}
