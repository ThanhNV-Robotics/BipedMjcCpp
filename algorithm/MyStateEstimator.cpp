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
    Eigen::Matrix<double , 3, 3> I = Eigen::Matrix<double, 3 ,3>::Identity(); // Identity matrix

    // Init matrices
    this->A_ = Eigen::MatrixXd::Identity(this->dimState_, this->dimState_);
    this->A_.block<3,3>(0,3) = dt*I;

    this->B_ = Eigen::MatrixXd::Zero(this->dimState_, 3);
    this->B_.block<3,3>(0,0) = dt*dt*I;
    this->B_.block<3,3>(3,0) = dt*I;

    // Init matrix C
    // measurement vector, stacked per contact i = 0..numContact_-1:
    //   rows [0            : dimContact_)          : p_foot_i - p_base   = [ I_3, 0, -I_3 ] (foot position, 6)
    //   rows [dimContact_   : 2*dimContact_)        : v_base             = [ 0, I_3,  0   ] (foot velocity, 6)
    //   rows [2*dimContact_ : 2*dimContact_+numContact_) : p_foot_i.z    = [ 0, 0,  H    ] (foot height, 2)
    // dimObserve_ = 2*dimContact_ + numContact_ = 14
    this->C_ = Eigen::MatrixXd::Zero(this->dimObserve_, this->dimState_);

    for (Eigen::Index i = 0; i < this->numContact_; ++i)
    {
        // position-residual rows for foot i: [ I_3, 0, -I_3 ]
        this->C_.block<3, 3>(3 * i, 0) = Eigen::Matrix3d::Identity();
        this->C_.block<3, 3>(3 * i, 6 + 3 * i) = -Eigen::Matrix3d::Identity();

        // velocity-residual rows for foot i: [ 0, I_3, 0 ]
        this->C_.block<3, 3>(this->dimContact_ + 3 * i, 3) = Eigen::Matrix3d::Identity();

        // height-residual row for foot i: H = [0, 0, 1] picking foot_i_pos.z
        this->C_(2 * this->dimContact_ + i, 6 + 3 * i + 2) = 1.0;
    }

    // construct Q, P, R vector
    this->Q_ = Eigen::MatrixXd::Identity(this->dimState_, this->dimState_); // Process noise covariance
    this->P_ = 100*this->Q_; //Error covariance matrix
    this->R_ = Eigen::MatrixXd::Identity(this->dimObserve_, this->dimObserve_);

    this->feetHeights_.setZero(this->numContact_); // init footh height to [0,0]

    if (verbose)
    {
        // print out kalman filter paramter for checking
        std::cout << "dim_state = " << this->dimState_ << std::endl;
        std::cout << "A_ =\n" << this->A_ << std::endl;
        std::cout << "B_ =\n" << this->B_ << std::endl;
        std::cout << "C_ =\n" << this->C_ << std::endl;

        std::cout << "Q_ =\n" << this->Q_ << std::endl;
        std::cout << "P_ =\n" << this->P_ << std::endl;
        std::cout << "R_ =\n" << this->R_ << std::endl;
    }
}

void StateEstimator::update()
{
    // check the leg contact status
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