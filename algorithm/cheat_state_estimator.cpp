#include "cheat_state_estimator.h"

CheatStateEstimator::CheatStateEstimator(const mjModel *model, const std::vector<std::string> &jointNames)
{
    int freeJointId = mj_name2id(model, mjOBJ_JOINT, "floating_base_joint");
    freeQposAdr_ = model->jnt_qposadr[freeJointId];
    freeQvelAdr_ = model->jnt_dofadr[freeJointId];

    jointQposAdr_.resize(jointNames.size());
    jointQvelAdr_.resize(jointNames.size());
    for (size_t i = 0; i < jointNames.size(); i++)
    {
        int jid = mj_name2id(model, mjOBJ_JOINT, jointNames[i].c_str());
        jointQposAdr_[i] = model->jnt_qposadr[jid];
        jointQvelAdr_[i] = model->jnt_dofadr[jid];
    }
}

void CheatStateEstimator::update(const mjModel *model, const mjData *data, RobotWrapper &robot_wrapper)
{
    // base position, world frame -- direct, no conversion needed.
    robot_wrapper.q(0) = data->qpos[freeQposAdr_ + 0];
    robot_wrapper.q(1) = data->qpos[freeQposAdr_ + 1];
    robot_wrapper.q(2) = data->qpos[freeQposAdr_ + 2];

    // base orientation: MuJoCo's free-joint qpos quat order is (w,x,y,z);
    // Pinocchio/robot_wrapper's q expects (x,y,z,w) -- the same convention
    // inversion used by every puppeting block elsewhere in this codebase.
    robot_wrapper.q(3) = data->qpos[freeQposAdr_ + 4]; // x
    robot_wrapper.q(4) = data->qpos[freeQposAdr_ + 5]; // y
    robot_wrapper.q(5) = data->qpos[freeQposAdr_ + 6]; // z
    robot_wrapper.q(6) = data->qpos[freeQposAdr_ + 3]; // w

    // base velocity: read the free joint's own 6 qvel DOFs directly rather
    // than mj_objectVelocity(mjOBJ_BODY, ...) -- that function returns
    // velocity about the BODY's CoM (via cvel), which is generally offset
    // from the free joint's own reference point (the same point qpos/q
    // track), silently injecting an omega x r_offset error into the linear
    // part. The free joint's qvel has no such ambiguity: qvel[0:3] is the
    // WORLD-frame linear velocity of the joint's own reference point,
    // qvel[3:6] is already the BODY-LOCAL angular velocity (MuJoCo's
    // standard free-joint convention). robot_wrapper.dq wants BOTH in the
    // local frame, so only the linear part needs rotating.
    Eigen::Quaterniond quat_wb(data->qpos[freeQposAdr_ + 3], data->qpos[freeQposAdr_ + 4],
                               data->qpos[freeQposAdr_ + 5], data->qpos[freeQposAdr_ + 6]); // world_R_base, (w,x,y,z)
    Eigen::Vector3d linVelWorld(data->qvel[freeQvelAdr_ + 0], data->qvel[freeQvelAdr_ + 1], data->qvel[freeQvelAdr_ + 2]);
    Eigen::Vector3d linVelLocal = quat_wb.toRotationMatrix().transpose() * linVelWorld;
    robot_wrapper.dq(0) = linVelLocal(0);
    robot_wrapper.dq(1) = linVelLocal(1);
    robot_wrapper.dq(2) = linVelLocal(2);
    robot_wrapper.dq(3) = data->qvel[freeQvelAdr_ + 3]; // already body-local
    robot_wrapper.dq(4) = data->qvel[freeQvelAdr_ + 4];
    robot_wrapper.dq(5) = data->qvel[freeQvelAdr_ + 5];

    // joint position/velocity: read directly from MuJoCo's qpos/qvel --
    // exact either way (a real robot's joint encoders give this exactly
    // too, no estimation involved), just skipping RobotSensor/MJ_Interface
    // as an intermediate.
    for (size_t j = 0; j < jointQposAdr_.size(); j++)
    {
        robot_wrapper.q(7 + j) = data->qpos[jointQposAdr_[j]];
        robot_wrapper.dq(6 + j) = data->qvel[jointQvelAdr_[j]];
    }
}
