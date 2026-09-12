// Author: Van Thanh Nguyen

#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>


#include <pinocchio/multibody/fwd.hpp>
// Forward declaration -- avoids a circular include since MyStateEstimator.h
// itself includes robot_wrapper.h. Only a reference is used in updateRobotState(),
// so a forward declaration is sufficient here; the full header is included in
// robot_wrapper.cpp where the implementation needs the complete type.
class StateEstimator;
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/aba.hpp"


#include "data_bus.h"
#include <string>
#include <vector>

#include "data_type.h"
#include <exception>
#include "useful_math.h"

class RobotWrapper {

    public:
        //*********************************************** */
        // model info
        int model_nq_; // position vector dim
        int model_nv_; // velocity vector dim
        int model_njoint_; // number of joint
        int model_na_; // number of actuated joint

        VectorXd min_joint_pos_;
        VectorXd max_joint_pos_;
        VectorXd joint_vel_limit_;
        VectorXd joint_torque_limit_;

        // actuated joint names, in Pinocchio joint order (URDF declaration
        // order) -- the same order used by q/dq's joint segments, min/max_joint_pos_,
        // joint_vel/torque_limit_, computeInitial_Stand()'s return value, etc.
        std::vector<std::string> jointNames_;

        //*********************************************** */
        // configuration space
        // TODO: Move q, dq, ddq to private once test files are refactored to not write to them directly.
        VectorXd q, dq, ddq;  //note for dq:  dq = [local_base_velocity_linear, local_base_velocity_angular, joint_velocities]

        // Read-only accessors
        const VectorXd& getQ() const { return q; }
        const VectorXd& getDq() const { return dq; }
        const VectorXd& getDdq() const { return ddq; }

        // computed Jacobian matrix, in World frame
        MatrixXd J_base_W, J_Lfeet_W, J_Rfeet_W;
        MatrixXd Jcom_W;

        std::vector<const MatrixXd*> J_array;
        std::vector<const MatrixXd*> dJ_array;
        std::vector<const Vector3d*> pos_array;
        std::vector<const Vector3d*> vel_array;

        // Jacobian derivative dJ
        MatrixXd dJ_Rfeet_W, dJ_Lfeet_W, dJ_base_W;

        // computed frame position
        Vector3d pos_R_feet_W, pos_L_feet_W, pos_base_W, pos_CoM_W; // in world frame
        Vector3d pos_R_feet_B, pos_L_feet_B, pos_base_B; // in body base/local frame

        // computed frame orientation/ rotation matrices
        Matrix3d rot_R_feet_W, rot_L_feet_W; // in world frame
        Matrix3d rot_R_feet_B, rot_L_feet_B; // in base frame

        // computed frame velocity
        Vector3d vel_base_W;
        Vector3d vel_R_feet_W, vel_L_feet_W;
        Vector3d vel_R_feet_B, vel_L_feet_B;
        Vector3d vel_CoM_W;

        //*********************************************** */
        // computed dynamics terms
        // Mass, Mass^-1, Coriollis, Gravity, Centroidal Momentumn Matrix (CMM) Ag(q) and dAg(q)
        MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_G, dyn_Ag, dyn_dAg;
        Eigen::VectorXd dyn_Non; // C + G

        //*********************************************** */
        // useful function
        void printModelInfo ();

        void printFixedBaseModelInfo ();
        
        void updateRobotState (StateEstimator &state_estimator);
        void updateRobotState (RobotConfiguration q, RobotSpatialVelocity v);
        void computeKin ();
        void computeDyn ();
        std::vector<double> getMaxTorque();

        // advance this->q by a tangent-space step delta_q (dim model_nv_),
        // e.g. a differential-IK correction -- uses pinocchio::integrate so
        // the floating-base quaternion is composed correctly, unlike a naive
        // q += delta_q (which doesn't make sense for the quaternion block)
        void integrateConfig (const VectorXd &delta_q);

        Vector12d computeFootInBase (ActuatorState actuator_state);

        struct IkRes
        {
            int status;
            int itr;
            Eigen::VectorXd err;
            Eigen::VectorXd jointPosRes;
        };

        IkRes computeInK_Leg(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R, const Eigen::Vector3d &Pdes_R);
        VectorXd computeInitial_Stand(const double base_height);
        //Constructor
        RobotWrapper(const std::string& urdf_path);

    private:

        pinocchio::Model pin_model_;
        pinocchio::Data pin_data_;

        pinocchio::Model model_fixedbase_;
        pinocchio::Data data_fixedbase_;

        std::vector<pinocchio::JointIndex> left_leg_joint_ids_;  // joint IDs in left leg subtree
        std::vector<pinocchio::JointIndex> right_leg_joint_ids_; // joint IDs in right leg subtree

        std::string robot_urdf_path_;
};