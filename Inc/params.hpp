#ifndef ROBOT_PARAM_HPP
#define ROBOT_PARAM_HPP
namespace ConvexMPC {
#define MPC_STATE_DIM 13
#define MPC_INPUT_DIM 12
#define MPC_CONSTRAINT_DIM 20
#define MPC_HORIZON 16
#define LEG_NUM 4
#define LEG_DOF 3
class params {
public:
    static constexpr double gravity = -9.81;
};
}
#endif