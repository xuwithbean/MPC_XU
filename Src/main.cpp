#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#include "params.hpp"
#include "robot_controller.hpp"
#include "robot_model.hpp"
#include "robot_state.hpp"
using namespace ConvexMPC;
int main() {
    raisim::World world;
    double mpc_dt = 0.025;
    double time_step = 0.001;
    world.setTimeStep(time_step);
    auto ground = world.addGround();
    raisim::ArticulatedSystem* dogrobort = world.addArticulatedSystem("./rsc/anymal_c/anymal.urdf");
    std::string leg_list[4] = {
        "LF_shank_fixed_LF_FOOT", "RF_shank_fixed_RF_FOOT", "LH_shank_fixed_LH_FOOT", "RH_shank_fixed_RH_FOOT"};//anymal
    Eigen::Matrix3d inertia;
    inertia.setZero();
    inertia <<   3.45791789e-01,  7.73636030e-03,  -9.35025187e-04, 
				 7.73636030e-03,  1.02294188e+00,   2.63540980e-03, 
				-9.35025187e-04,  2.63540980e-03,   7.89181283e-01;
    RobotModel robot_model(inertia,
                           52.13485,//anymal
                           params::gravity,
                           0.6,
                           mpc_dt,
                           0,
                           700);
    Eigen::Vector3d cmd_vel;
    std::atomic<bool> control_execute{};
    control_execute.store(true, std::memory_order_release);
    std::array<Eigen::Vector3d, LEG_NUM> foot_position_b, foot_velocity;
    std::array<Eigen::Vector3d, LEG_NUM> grf, feedback, feedforward;
    std::array<Eigen::Vector3d, LEG_NUM> grf_torque, feedback_torque, feedforward_torque;
    std::array<Eigen::Matrix3d, LEG_NUM> foot_jacobian;
    Eigen::VectorXd jointNominalConfig(dogrobort->getGeneralizedCoordinateDim()), jointVelocityTarget(dogrobort->getDOF());
    Eigen::VectorXd current_jointVelocity(dogrobort->getDOF()), current_jointConfig(dogrobort->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPgain(dogrobort->getDOF()), jointDgain(dogrobort->getDOF()), jointForce(dogrobort->getDOF()),dynamics_torque(dogrobort->getDOF());
    Eigen::Vector3d body_position, euler, linear_velocity, angular_velocity;
    Eigen::Vector4d quat;
    Eigen::VectorXd q_weights(13), r_weights(12);
    Eigen::MatrixXd lambda;
    jointPgain.setZero();
    jointDgain.setZero();
    jointVelocityTarget.setZero();
    jointNominalConfig << 0, 0, 0.54, 1, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
    q_weights << 50, 10, 10, 10, 10, 100, 0.05, 0.05, 0.05, 10, 10, 10, 0.;
    r_weights << 1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6;
    dogrobort->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    dogrobort->setPdTarget(jointNominalConfig, jointVelocityTarget);
    dogrobort->setName("dogrobort");
    dogrobort->setPdGains(jointPgain, jointDgain);
    raisim::RaisimServer server(&world);
    server.focusOn(dogrobort);
    server.launchServer();
    dogrobort->setGeneralizedCoordinate(jointNominalConfig);
    current_jointConfig = dogrobort->getGeneralizedCoordinate().e();
    body_position = current_jointConfig.segment(0, 3);
    quat = current_jointConfig.segment(3, 4);
    euler = utils::quaternion_to_euler(quat);
    linear_velocity = current_jointVelocity.segment(0, 3);
    angular_velocity = current_jointVelocity.segment(3, 3);
    for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        raisim::Vec<3> foot_position_w;
        dogrobort->getFramePosition(leg_list[leg_idx], foot_position_w);
        foot_position_b[leg_idx] = foot_position_w.e();
    }
    RobotState robot_state(euler, body_position, angular_velocity, linear_velocity, foot_position_b);
    RobotController robot_controller(robot_model, robot_state, q_weights, r_weights, 700, 40, 0.2, 0.2);
    robot_state.updateContactState({false, true, true, false});
    dogrobort->setGeneralizedCoordinate(jointNominalConfig);
    for (int i = 0; i < 10000000; i++) {
        RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
        current_jointConfig = dogrobort->getGeneralizedCoordinate().e();
        current_jointVelocity = dogrobort->getGeneralizedVelocity().e();
        body_position = current_jointConfig.segment(0, 3);
        quat = current_jointConfig.segment(3, 4);
        euler = utils::quaternion_to_euler(quat);
        linear_velocity = current_jointVelocity.segment(0, 3);
        angular_velocity = current_jointVelocity.segment(3, 3);
        dynamics_torque = dogrobort->getNonlinearities({0, 0, params::gravity}).e();
        robot_state.updateState(euler, body_position, angular_velocity, linear_velocity);
        cmd_vel = {-1.0, 0, 0};
        for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
            raisim::Vec<3> foot_position_w;
            Eigen::MatrixXd full_jacobian(3, dogrobort->getDOF());
            dogrobort->getFramePosition(leg_list[leg_idx], foot_position_w);
            dogrobort->getDenseFrameJacobian(leg_list[leg_idx], full_jacobian);
            foot_jacobian[leg_idx] = full_jacobian.block(0, 6 + 3 * leg_idx, 3, 3);
            foot_position_b[leg_idx] = foot_position_w.e() - body_position;
            foot_velocity[leg_idx] = foot_jacobian[leg_idx] * current_jointVelocity.segment(6 + 3 * leg_idx, 3);
        }
        robot_state.updateFootPosition(foot_position_b);
        robot_state.updateFootVelocity(foot_velocity);
        if (std::fmod(i, (mpc_dt / time_step)) == 0) {
            grf = robot_controller.computeGRF(robot_state, cmd_vel);
        }
        feedback = robot_controller.computeSwingForce(robot_state, cmd_vel);
        for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
            grf_torque[leg_idx] = -foot_jacobian[leg_idx].transpose() * 1.2 * grf[leg_idx];
            feedforward[leg_idx] = dynamics_torque.segment(6 + 3 * leg_idx, 3);
            feedback_torque[leg_idx] = foot_jacobian[leg_idx].transpose() * feedback[leg_idx];
        }
        jointForce << Eigen::VectorXd::Zero(6), 
						grf_torque[0] + feedback_torque[0] + feedforward[0], 
						grf_torque[1] + feedback_torque[1] + feedforward[1],
						grf_torque[2] + feedback_torque[2] + feedforward[2],
						grf_torque[3] + feedback_torque[3] + feedforward[3];
        dogrobort->setGeneralizedForce(jointForce);
        server.integrateWorldThreadSafe();
    }
    server.killServer();
}