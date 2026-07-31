#include "MyStateEstimator.h"

// constructor
StateEstimator::StateEstimator(double dt, bool verbose) // verbose is option to print out filter param or note
{
    // ------------------------
    // Init Measurement vars
    // ------------------------
    this->dt_ = dt;
    joint_pos_mea_.setZero();
    joint_vel_mea_.setZero();
    joint_tor_mea_.setZero();

    imu_acceleration_mea_.setZero();
    imu_angular_vel_mea_.setZero();

    imu_quaternion_ << 0, 0, 0, 1; // pinocchio style (x,y,z,w), identity Rotation matrix

    // Init Kalman filter
    // state vector:
    // x = [base_pos(3), base linear velocity(3), foot position(dim_contact_)]
    int dim_state = 6 + dim_contact_; // base_pos + base_vel + feet positions
    Eigen::Matrix<double , 3, 3> I = Eigen::Matrix<double, 3 ,3>::Identity(); // Identity matrix

    // Init matrices
    this->A_ = Eigen::MatrixXd::Identity(dim_state, dim_state);
    this->A_.block<3,3>(0,3) = dt*I;

    this->B_ = Eigen::MatrixXd::Zero(dim_state, 3);
    this->B_.block<3,3>(0,0) = dt*dt*I;
    this->B_.block<3,3>(3,0) = dt*I;

    // Init matrix C
    for (int i = 0; i < this->num_contact_; i++)
    {
        this->C_.block<3,3>
    }


    if (verbose)
    {
        // print out kalman filter paramter for checking
        std::cout << "dim_state = " << dim_state << std::endl;
        std::cout << "A_ =\n" << this->A_ << std::endl;
        std::cout << "B_ =\n" << this->B_ << std::endl;
    }
}

void StateEstimator::getSensorMeansurement (DataBus& Data)
{
    // assign from DataBus
    // joint_pos_mea_ is Eigen::Matrix but Data.joint_pos_cur is std::vector<double>
    // so we have to use Eigen::Map here
    this->joint_pos_mea_ = Eigen::Map<const Eigen::Matrix<double,12,1>>(Data.joint_pos_cur.data());
    this->joint_vel_mea_ = Eigen::Map<const Eigen::Matrix<double,12,1>>(Data.joint_vel_cur.data());
    this->joint_tor_mea_ = Eigen::Map<const Eigen::Matrix<double,12,1>>(Data.joint_tor_cur.data());

    this->imu_acceleration_mea_ = Eigen::Map<Eigen::Vector3d>(Data.baseAcc);
    this->imu_angular_vel_mea_ = Eigen::Map<Eigen::Vector3d>(Data.baseAngVel);
    auto base_quat = Data.eul2quat(Data.rpy[0], Data.rpy[1], Data.rpy[2]);
    this->imu_quaternion_ << base_quat.x(), base_quat.y(), base_quat.z(), base_quat.w();
}

Eigen::Matrix<double, 4,1> StateEstimator::getImuquaternion()
{
    return this->imu_quaternion_;
}

Eigen::Matrix<double, 12,1> StateEstimator::get_qj() // return joint position
{
    return this->joint_pos_mea_;
}
Eigen::Matrix<double, 12,1> StateEstimator::get_qjd() // return joint velocity
{
    return this->joint_vel_mea_;
}

// Eigen::Matrix<double, 6,1> StateEstimator::get_qb() // return base pose (position, rpy)
// {
//     return this->joint_vel_mea_;
// }
// Eigen::Matrix<double, 6,1> StateEstimator::get_qbd() // return base linear velocity and angular velocity