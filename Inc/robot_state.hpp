#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP
#include "params.hpp"
#include "utils.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
namespace ConvexMPC {
class RobotState {
public:
    RobotState() = default;
    RobotState(const Eigen::Ref<const Eigen::Vector3d>& euler_angle,
               const Eigen::Ref<const Eigen::Vector3d>& position,
               const Eigen::Ref<const Eigen::Vector3d>& angular_velocity,
               const Eigen::Ref<const Eigen::Vector3d>& linear_velocity);
    RobotState(const Eigen::Ref<const Eigen::Vector3d>& euler_angle,
               const Eigen::Ref<const Eigen::Vector3d>& position,
               const Eigen::Ref<const Eigen::Vector3d>& angular_velocity,
               const Eigen::Ref<const Eigen::Vector3d>& linear_velocity,
               const std::array<Eigen::Vector3d, LEG_NUM>& foot_position);
    RobotState(const RobotState& other);
    RobotState(const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& mpc_state);
    void updateContactState(const std::array<bool, LEG_NUM>& contact_state);
    void updateContactState(const size_t& leg_idx, const bool& state);
    void updateState(const Eigen::Ref<const Eigen::Vector3d>& euler_angle,
                     const Eigen::Ref<const Eigen::Vector3d>& position,
                     const Eigen::Ref<const Eigen::Vector3d>& angular_velocity,
                     const Eigen::Ref<const Eigen::Vector3d>& linear_velocity);
    void updateFootPosition(const std::array<Eigen::Vector3d, LEG_NUM>& foot_position);
    void updateFootVelocity(const std::array<Eigen::Vector3d, LEG_NUM>& foot_velocity);
    const Eigen::Vector3d& position() const { return position_; }
    const Eigen::Vector3d& euler_angle() const { return euler_angle_; }
    const Eigen::Vector3d& linear_velocity() const { return linear_velocity_; }
    const Eigen::Vector3d& angular_velocity() const { return angular_velocity_; }
    const Eigen::Vector<double, MPC_STATE_DIM>& mpc_state() const { return mpc_state_; }
    const Eigen::Vector3d foot_position(const size_t& leg_idx) const { return foot_position_[leg_idx]; }
    const std::array<Eigen::Vector3d, LEG_NUM>& foot_position() const { return foot_position_; }
    const Eigen::Vector3d foot_velocity(const size_t& leg_idx) const { return foot_velocity_[leg_idx]; }
    const std::array<Eigen::Vector3d, LEG_NUM>& foot_velocity() const { return foot_velocity_; }
    const std::array<bool, LEG_NUM>& contact_state() const { return contact_state_; }
    bool contact_state(const size_t& leg_idx) const { return contact_state_[leg_idx]; }
    Eigen::Matrix3d rotation_matrix() const { return utils::euler_to_matrix(euler_angle_); }
    Eigen::Quaterniond body_quaternion() const { return utils::euler_to_quaternion(euler_angle_); }
private:
    Eigen::Vector3d euler_angle_;
    Eigen::Vector3d position_;
    Eigen::Vector3d angular_velocity_;
    Eigen::Vector3d linear_velocity_;
    Eigen::Vector<double, MPC_STATE_DIM> mpc_state_;
    Eigen::Vector3d linear_acceleration;
    std::array<Eigen::Vector3d, LEG_NUM> foot_position_;
    std::array<Eigen::Vector3d, LEG_NUM> foot_velocity_;
    std::array<bool, LEG_NUM> contact_state_;
};
}
#endif