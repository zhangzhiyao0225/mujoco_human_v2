#ifndef EFC_ANKLE_HPP
#define EFC_ANKLE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

namespace ovinf {

template <typename T = double>
class EfcAnkle {
  using Vector2 = Eigen::Matrix<T, 2, 1>;
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector6 = Eigen::Matrix<T, 6, 1>;
  using Matrix2 = Eigen::Matrix<T, 2, 2>;
  using Matrix3 = Eigen::Matrix<T, 3, 3>;
  using Isometry3 = Eigen::Transform<T, 3, Eigen::Isometry>;
  using AngleAxis = Eigen::AngleAxis<T>;

 public:
  struct AnkleParameters {
    T l_bar1 = 0.1;
    T l_rod1 = 0.2;
    Vector3 r_a1 = Vector3::Zero();
    Vector3 r_b1_0 = Vector3::Zero();
    Vector3 r_c1_0 = Vector3::Zero();
    T l_bar2 = 0.1;
    T l_rod2 = 0.2;
    Vector3 r_a2 = Vector3::Zero();
    Vector3 r_b2_0 = Vector3::Zero();
    Vector3 r_c2_0 = Vector3::Zero();

    Vector3 r_op = {0, 0, -0.05};
  };

  EfcAnkle(const AnkleParameters& params, const T threshold = 1e-6)
      : params_(params), threshold_(threshold) {
    l_rod1_ = params_.l_rod1;
    l_bar1_ = params_.l_bar1;
    l_rod2_ = params_.l_rod2;
    l_bar2_ = params_.l_bar2;

    r_op_ = params_.r_op;

    r_a1_ = params_.r_a1;
    r_b1_0_ = params_.r_b1_0;
    r_fc1_ = params_.r_c1_0 - r_op_;

    r_a2_ = params_.r_a2;
    r_b2_0_ = params_.r_b2_0;
    r_fc2_ = params_.r_c2_0 - r_op_;

    T_op_0_ = Isometry3::Identity();
    T_op_0_.pretranslate(r_op_);
  }

  /**
   * @brief Inverse Kinematics.
   *
   * @param[in] pitch Pitch in rad
   * @param[in] roll Roll in rad
   *
   * @return Joint positions [j_left, j_right]
   */
  Vector2 InverseKinematics(const T pitch, const T roll) {
    Isometry3 T_of = Isometry3(AngleAxis(pitch, Vector3::UnitY())) * T_op_0_ *
                     Isometry3(AngleAxis(roll, Vector3::UnitX()));

    Vector3 r_c1 = T_of * (this->r_fc1_);
    Vector3 r_c2 = T_of * (this->r_fc2_);

    T a1 = (this->r_a1_ - r_c1)(1);
    T b1 = (this->r_a1_ - r_c1)(2);
    T c1 = (-std::pow(this->l_rod1_, 2) + std::pow(this->l_bar1_, 2) +
            std::pow((this->r_a1_ - r_c1).norm(), 2)) /
           (2 * this->l_bar1_);

    T a2 = (this->r_a2_ - r_c2)(1);
    T b2 = (this->r_a2_ - r_c2)(2);
    T c2 = -(-std::pow(this->l_rod2_, 2) + std::pow(this->l_bar2_, 2) +
             std::pow((this->r_a2_ - r_c2).norm(), 2)) /
           (2 * this->l_bar2_);

    Vector2 theta_res(2);

    theta_res(0, 0) = std::asin(
        (-b1 * c1 + std::sqrt(b1 * b1 * c1 * c1 -
                              (a1 * a1 + b1 * b1) * (c1 * c1 - a1 * a1))) /
        (a1 * a1 + b1 * b1));
    theta_res(1, 0) = std::asin(
        (-b2 * c2 - std::sqrt(b2 * b2 * c2 * c2 -
                              (a2 * a2 + b2 * b2) * (c2 * c2 - a2 * a2))) /
        (a2 * a2 + b2 * b2));

    return theta_res;
  }

  /**
   * @brief Compute Jacobian, save it in member J_c_.
   *
   * @param[in] pitch Pitch in rad
   * @param[in] roll Roll in rad
   * @param[in] theta1 Left joint (y+) position in rad
   * @param[in] theta2 Right joint (y-) position in rad
   */
  void ComputeJacobian(const T pitch, const T roll, const T theta1,
                       const T theta2) {
    // Joint B position
    Vector3 r_b1 = this->r_a1_ + Matrix3(AngleAxis(theta1, Vector3::UnitX())) *
                                     (this->r_b1_0_ - this->r_a1_);
    Vector3 r_b2 = this->r_a2_ + Matrix3(AngleAxis(theta2, Vector3::UnitX())) *
                                     (this->r_b2_0_ - this->r_a2_);

    Isometry3 T_of = Isometry3(AngleAxis(pitch, Vector3::UnitY())) * T_op_0_ *
                     Isometry3(AngleAxis(roll, Vector3::UnitX()));
    Vector3 r_c1 = T_of * (this->r_fc1_);
    Vector3 r_c2 = T_of * (this->r_fc2_);

    Vector3 r_pc1 = AngleAxis(pitch, Vector3::UnitY()) *
                    AngleAxis(roll, Vector3::UnitX()) * this->r_fc1_;
    Vector3 r_pc2 = AngleAxis(pitch, Vector3::UnitY()) *
                    AngleAxis(roll, Vector3::UnitX()) * this->r_fc2_;

    Vector3 r_bar_1 = r_b1 - this->r_a1_;
    Vector3 r_bar_2 = r_b2 - this->r_a2_;
    Vector3 r_rod_1 = r_c1 - r_b1;
    Vector3 r_rod_2 = r_c2 - r_b2;

    Vector3 s_31 = Vector3::UnitY();
    Vector3 s_32 = Vector3::UnitX();

    Vector3 s_11 = Vector3::UnitX();
    Vector3 s_21 = Vector3::UnitX();

    Matrix3 Ry_pitch(AngleAxis(pitch, Vector3::UnitY()));

    Matrix2 A = Matrix2::Zero();
    Matrix2 B = Matrix2::Zero();

    A(0, 0) = s_11.dot(r_bar_1.cross(r_rod_1));
    A(1, 1) = s_21.dot(r_bar_2.cross(r_rod_2));

    B(0, 0) = s_31.dot(r_c1.cross(r_rod_1));
    B(0, 1) = (Ry_pitch * s_32).dot(r_pc1.cross(r_rod_1));
    B(1, 0) = s_31.dot(r_c2.cross(r_rod_2));
    B(1, 1) = (Ry_pitch * s_32).dot(r_pc2.cross(r_rod_2));
    this->J_c_ = A.inverse() * B;
  }

  /**
   * @brief Compute forward kinematics.
   * @param[in] theta1 Joint position 1 (left)
   * @param[in] theta2 Joint position 2 (right)
   * @return Foot pitch and roll [pitch, roll]
   */
  Vector2 ForwardKinematics(const T theta1, const T theta2) {
    if (std::isnan(this->last_pitch_) || std::isnan(this->last_roll_)) {
      std::cout << "Ankle solver fatal error. Fatal count: "
                << this->fatal_count_++ << std::endl;
      this->last_pitch_ = 0.0;
      this->last_roll_ = 0.0;
    }
    Vector2 theta_ref(theta1, theta2);

    for (size_t i = 0; i < 10; ++i) {
      Vector2 theta_k;
      theta_k = this->InverseKinematics(this->last_pitch_, this->last_roll_);
      Vector2 err = theta_k - theta_ref;

      this->ComputeJacobian(last_pitch_, last_roll_, theta_k(0, 0),
                            theta_k(1, 0));

      // Check
      if (std::isnan(this->last_pitch_) || std::isnan(this->last_roll_)) {
        // First check.
        std::cout << "Ankle solving failed. Error count: "
                  << this->error_count_++ << std::endl;
        this->last_pitch_ = 0.0;
        this->last_roll_ = 0.0;
        theta_k = this->InverseKinematics(this->last_pitch_, this->last_roll_);
        this->ComputeJacobian(last_pitch_, last_roll_, theta_k(0, 0),
                              theta_k(1, 0));
        break;
      } else if (err.norm() < this->threshold_) {
        break;
      } else {
        Vector2 x_c_k(this->last_pitch_, this->last_roll_);
        // Error clip to avoid divergence
        err = err.cwiseMin(0.6).cwiseMax(-0.6);
        x_c_k -= this->J_c_.inverse() * err;

        this->last_pitch_ = x_c_k(0, 0);
        this->last_roll_ = x_c_k(1, 0);
      }
    }

    Vector2 pitch_roll(this->last_pitch_, this->last_roll_);
    return pitch_roll;
  }

  inline Vector2 TorqueS2P(const T torque_pitch, const T torque_roll) {
    Vector2 torque_ankle(torque_pitch, torque_roll);
    Vector2 torque_motor = this->J_c_.inverse().transpose() * torque_ankle;
    return torque_motor;
  }

  inline Vector2 TorqueP2S(const T t1, const T t2) {
    Vector2 torque_motor(t1, t2);
    Vector2 torque_ankle = this->J_c_.transpose() * torque_motor;
    return torque_ankle;
  }

  inline Vector2 VelocityP2S(const T omega1, const T omega2) {
    Vector2 vel_motor(omega1, omega2);
    Vector2 vel_ankle = this->J_c_.inverse() * vel_motor;
    return vel_ankle;
  }

  class PDTargetSolverInput {
   public:
    Vector2 serial_kp = Vector2::Zero();
    Vector2 serial_kd = Vector2::Zero();
    Vector2 serial_q = Vector2::Zero();
    Vector2 serial_target_q = Vector2::Zero();
    Vector2 parallel_q = Vector2::Zero();
    Vector2 parallel_dq = Vector2::Zero();
  };

  class PDTargetSolverOutput {
   public:
    Vector2 parallel_target_pos = Vector2::Zero();
    Vector2 parallel_kp = Vector2::Zero();
    Vector2 parallel_kd = Vector2::Zero();
    Vector2 feedforward_torque = Vector2::Zero();
  };

  inline PDTargetSolverOutput PDTargetS2P(const PDTargetSolverInput& input) {
    Matrix2 parallel_kp = this->J_c_.inverse().transpose() *
                          input.serial_kp.asDiagonal() * this->J_c_.inverse();
    Matrix2 parallel_kd = this->J_c_.inverse().transpose() *
                          input.serial_kd.asDiagonal() * this->J_c_.inverse();
    Vector2 parallel_target_pos =
        this->J_c_ * (input.serial_target_q - input.serial_q) +
        input.parallel_q;

    Matrix2 parallel_kp_off_diag = parallel_kp;
    parallel_kp_off_diag(0, 0) = 0.0;
    parallel_kp_off_diag(1, 1) = 0.0;
    Matrix2 parallel_kd_off_diag = parallel_kd;
    parallel_kd_off_diag(0, 0) = 0.0;
    parallel_kd_off_diag(1, 1) = 0.0;

    PDTargetSolverOutput res;
    // PD transform
    res.parallel_target_pos = parallel_target_pos;
    res.parallel_kp(0, 0) = parallel_kp(0, 0);
    res.parallel_kp(1, 0) = parallel_kp(1, 1);
    res.parallel_kd(0, 0) = parallel_kd(0, 0);
    res.parallel_kd(1, 0) = parallel_kd(1, 1);
    // res.feedforward_torque =
    //     parallel_kp_off_diag * (parallel_target_pos - input.parallel_q) -
    //     parallel_kd_off_diag * input.parallel_dq;

    auto fft = parallel_kp_off_diag * (parallel_target_pos - input.parallel_q) -
               parallel_kd_off_diag * input.parallel_dq;
    res.parallel_target_pos += res.parallel_kp.asDiagonal().inverse() * fft;

    res.feedforward_torque.setZero();

    // res.feedforward_torque(0, 0) *= -1;

    // TODO: Test this
    // res.feedforward_torque.setZero();
    // res.feedforward_torque(0, 0) = 0.1;
    // res.feedforward_torque(1, 0) = 0.1;

    // Torque transform
    // res.parallel_kp.setZero();
    // res.parallel_kd.setZero();
    // res.parallel_target_pos.setZero();
    // res.feedforward_torque = this->J_c_.inverse().transpose() *
    //                          (input.serial_kp.asDiagonal() *
    //                               (input.serial_target_q - input.serial_q) -
    //                           input.serial_kd.asDiagonal() *
    //                               this->J_c_.inverse() * input.parallel_dq);

    return res;
  }

 public:
  size_t error_count_ = 0;
  size_t fatal_count_ = 0;
  AnkleParameters params_;
  T l_rod1_;
  T l_bar1_;
  T l_rod2_;
  T l_bar2_;
  T threshold_;

  Vector3 r_op_;

  Vector3 r_a1_;
  Vector3 r_b1_0_;
  Vector3 r_fc1_;  // r_pc1_0

  Vector3 r_a2_;
  Vector3 r_b2_0_;
  Vector3 r_fc2_;

  Isometry3 T_op_0_;

  // Values for iteration
  T last_pitch_ = 0.0;
  T last_roll_ = 0.0;
  Matrix2 J_c_ = Matrix2::Identity();
};

}  // namespace ovinf

#endif  // !EFC_ANKLE_HPP
