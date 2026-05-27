/**
 * 4-bar anti-parallelogram linkage solver.
 *
 * Dknt 2025.5
 */

#ifndef ANTIPARALLELOGRAM_HPP
#define ANTIPARALLELOGRAM_HPP

#include <cmath>
#include <iostream>

namespace ovinf {

template <typename T = double>
class AntiparallelogramLinkage {
 public:
  constexpr static T TPi = T(M_PI);
  struct APLParameters {
    T r = 10.0;
    T l = 100.0;
    T theta_bias = 0.0;
    T phi_bias = 0.0;
  };

  AntiparallelogramLinkage() = delete;
  ~AntiparallelogramLinkage() = default;

  AntiparallelogramLinkage(APLParameters const& config) {
    r_ = config.r;
    l_ = config.l;
    theta_bias_ = config.theta_bias;
    phi_bias_ = config.phi_bias;
    J_ = 1.0;
    theta_ = 0.0;
    phi_ = 0.0;
  }

  /**
   * @brief Forward kinematics of the 4 bar anti-parallelogram linkage.
   *
   * @param[in] theta Motor angle in rad
   * @return Joint angle in rad
   */
  T ForwardKinematics(const T theta) {
    theta_ = theta + theta_bias_;
    if (theta_ > 0 || theta_ < -TPi) {
      std::cout << "[AntiparallelogramLinkage] "
                << "Invalid input theta: " << phi_ << std::endl;
      theta_ = std::max<T>(-TPi, std::min<T>(TPi, theta_));
    }

    // Solve phi
    T s = std::sqrt(r_ * r_ + l_ * l_ + 2.0 * r_ * l_ * std::cos(theta_));
    T A = (l_ * std::cos(theta_) + r_) / s;
    T alpha = std::acos(A);
    T phi = theta_ + 2.0 * alpha;

    // Compute Jacobian
    J_ = -std::sin(theta_) / std::sin(phi);

    return phi - phi_bias_;
  }

  /**
   * @brief Inverse kinematics of the 4 bar anti-parallelogram linkage.
   *
   * @param[in] phi Joint angle in rad
   * @return Motor angle in rad
   */
  T InverseKinematics(const T phi) {
    phi_ = phi + phi_bias_;
    if (phi_ < 0 || phi_ > TPi) {
      std::cout << "[AntiparallelogramLinkage] "
                << "Invalid input phi: " << phi_ << std::endl;
      phi_ = std::max<T>(0.0, std::min<T>(TPi, phi_));
    }

    T s = std::sqrt(r_ * r_ + l_ * l_ - 2.0 * r_ * l_ * std::cos(phi_));
    T A = (-l_ * std::cos(phi_) + r_) / s;
    T alpha = std::acos(A);
    T theta = 2.0 * (alpha - TPi) + phi_;

    return theta - theta_bias_;
  }

  /**
   * @brief Velocity mapping
   *
   * @param[in] theta_dot Motor speed in rad/s
   * @return Joint speed
   */
  T VelocityMapping(const T theta_dot) { return theta_dot / J_; }

  /**
   * @brief Torque remapping
   *
   * @param[in] tau Joint torque in Nm
   * @return Motor torque in Nm
   */
  T TorqueRemapping(const T tau) { return tau / J_; }

 private:
  T r_;
  T l_;
  T theta_bias_;
  T phi_bias_;

  T theta_;  // Motor angle in rad
  T phi_;    // Joint angle in rad

  T J_;  // Jacobian
};

}  // namespace ovinf

#endif  // !ANTIPARALLELOGRAM_HPP
