#ifndef EFC_JOINT_H
#define EFC_JOINT_H

#include <pugixml.hpp>

#include "device/efc_device.hpp"

namespace bitbot
{

  enum class EfcJointType
  {
    NONE = 0,
    PositionForceHybrid,
  };

  class EfcJoint final : public EfcDevice
  {
  public:
    EfcJoint(const pugi::xml_node &device_node);
    ~EfcJoint();

    inline double GetActualPosition() { return actual_position_; }

    inline double GetActualVelocity() { return actual_velocity_; }

    inline double GetActualCurrent() { return actual_current_; }

    inline double GetEstimatedTorque() { return estimated_torque_; }

    inline float GetMotorTemperature() { return motor_temp_; }
    inline float GetMosTemperature() { return mos_temp_; }

    inline void SetTargetPDGains(double p_gain, double d_gain)
    {
      p_gain_ = p_gain;
      d_gain_ = d_gain;
    }

    inline void SetTargetPosition(double pos) { target_position_ = pos; }

    inline void SetTargetVelocity(double vel) { target_velocity_ = vel; }

    inline void SetTargetTorque(double torque) { target_torque_ = torque; }

  private:
    virtual void Input(const DdsInterface::Ptr dds_interface) final;
    virtual void Output(const DdsInterface::Ptr dds_interface) final;
    virtual void UpdateModel(const DdsInterface::Ptr dds_interface) final;
    virtual void UpdateRuntimeData() final;

  private:
    EfcJointType joint_type_;
    bool enable_ = true;

    unsigned int dds_motor_index_;
    unsigned int model_id_;
    double ct_scale_ = 1.0;
    double current_limit_ = 0.0;
    double pos_bias_ = 0.0;
    double motor_dir_ = 1.0;

    double actual_position_ = 0.0;
    double actual_velocity_ = 0.0;
    double actual_current_ = 0.0;
    double estimated_torque_ = 0.0;

    double target_position_ = 0.0;
    double target_velocity_ = 0.0; // Not used for biped
    double target_torque_ = 0.0;

    double p_gain_ = 0.0;
    double d_gain_ = 0.0;

    float motor_temp_ = 0.;
    float mos_temp_ = 0.;
  };

} // namespace bitbot

#endif // !EFC_JOINT_H
