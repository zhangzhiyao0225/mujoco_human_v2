
#include "efc_ankle.hpp"
// #include "ovinf/utils/efc_ankle.hpp"

int main(int argc, char** argv) {
  ovinf::EfcAnkle<float> ankle({
      .l_bar1 = 0.023,
      .l_rod1 = 0.173,
      .r_a1 = {-0.0285, 0.0, 0.1679},
      .r_b1_0 = {-0.0285, 0.023, 0.1679},
      .r_c1_0 = {-0.03, 0.0155, -0.002},
      .l_bar2 = 0.023,
      .l_rod2 = 0.2299,
      .r_a2 = {-0.0285, 0.0, 0.2249},
      .r_b2_0 = {-0.0285, -0.023, 0.2249},
      .r_c2_0 = {-0.03, -0.0155, -0.002},
      .r_op = {0.0, 0.0, -0.016},
  });

  // {-pitch, roll, -left, -right} in degrees
  std::vector<Eigen::Vector4f> test_points = {
      {-35, 0, -63.16, 63.73},    {-15, 0, -28.03, 28.36},
      {-15, 20, -40.83, 12.58},   {-15, -20, -12.27, 41.14},
      {0, 0, -7.26, 7.49},        {0, 20, -18.14, -7.56},
      {0, -20, 7.87, 18.21},      {15, 0, 11.98, -11.74},
      {15, 20, 1.86, -27.12},     {15, -20, 27.48, 1.85},
      {40, 0, 43.71, -43.37},     {40, 18.5, 33.76, -63.12},
      {40, -18.5, 63.11, -33.58}, {53, 0, 63.08, -63.02}};

  for (auto const& point : test_points) {
    auto ik_res = ankle.InverseKinematics(-point(0) / 180.0 * M_PI,
                                          point(1) / 180.0 * M_PI);
    std::cout << "IK res (L,R): " << ik_res(0) / M_PI * 180 << ", "
              << ik_res(1) / M_PI * 180 << " => gt (L,R): " << -point(2) << ", "
              << -point(3) << std::endl;

    // ankle.last_pitch_ = -point(0) / 180.0 * M_PI;
    // ankle.last_roll_ = point(1) / 180.0 * M_PI;
    ankle.last_pitch_ = 0.0;
    ankle.last_roll_ = 0.0;

    auto fk_res = ankle.ForwardKinematics(-point(2) / 180.0 * M_PI,
                                          -point(3) / 180.0 * M_PI);
    std::cout << "FK Result (P,R): " << fk_res(0) / M_PI * 180 << ", "
              << fk_res(1) / M_PI * 180 << " => gt (P,R): " << -point(0) << ", "
              << point(1)

              << std::endl;
    std::cout << "------------------------" << std::endl;
  }

  return 0;
}
