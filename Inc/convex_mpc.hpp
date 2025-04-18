#ifndef CONVEX_MPC_HPP
#define CONVEX_MPC_HPP
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <qpOASES.hpp>
#include "params.hpp"
#include "robot_model.hpp"
#include "robot_state.hpp"
namespace ConvexMPC {
class ConvexMPC {
public:
    ConvexMPC(const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& state_weight,
              const Eigen::Ref<const Eigen::Vector<double, MPC_INPUT_DIM>>& input_weight,
              const Eigen::Ref<const Eigen::Vector<double, MPC_CONSTRAINT_DIM>>& lower_bound,
              const Eigen::Ref<const Eigen::Vector<double, MPC_CONSTRAINT_DIM>>& upper_bound,
              const Eigen::Ref<const Eigen::MatrixXd>& constraint_coefficient);
    void updateQP(const Eigen::Ref<const Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>>& Ad,
                  const Eigen::Ref<const Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM>>& Bd,
                  const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& x0,
                  const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM * MPC_HORIZON>>& y);
    const Eigen::VectorXd solve();
    const Eigen::MatrixXd& L() const { return L_; }
    const Eigen::MatrixXd& K() const { return K_; }
    const Eigen::MatrixXd& A_qp() const { return A_qp_; }
    const Eigen::MatrixXd& B_qp() const { return B_qp_; }
    const Eigen::MatrixXd& hessian() const { return hessian_; }
    const Eigen::MatrixXd& linear_constraints() const { return linear_constraints_; }
    const Eigen::VectorXd& gradient() const { return gradient_; }
    const Eigen::VectorXd& lb() const { return lb_; }
    const Eigen::VectorXd& ub() const { return ub_; }
private:
    Eigen::MatrixXd L_;
    Eigen::MatrixXd K_;
    Eigen::MatrixXd A_qp_;
    Eigen::MatrixXd B_qp_;
    Eigen::MatrixXd hessian_;
    Eigen::MatrixXd linear_constraints_;
    Eigen::VectorXd gradient_;
    Eigen::VectorXd lb_;
    Eigen::VectorXd ub_;
};
} 
#endif