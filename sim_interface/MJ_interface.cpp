#include "MJ_interface.h"

// constructor
MJ_Interface::MJ_Interface(mjModel *mj_modelIn, mjData *mj_dataIn, const char* jsonPath)
{
    this->mj_model = mj_modelIn;
    this->mj_data = mj_dataIn;

    // Read json file
    Json::Reader reader;
    Json::Value root_read;
    std::ifstream in(jsonPath,std::ios::binary);

    reader.parse(in,root_read);

    JointName = root_read.getMemberNames(); // joint names come from the config file's top-level keys
    this->jointNum=JointName.size();
    this->jntId_qpos.assign(this->jointNum,0); //init jntId position, resize the vector to jointNum and set value to 0
    this->jntId_qvel.assign(this->jointNum,0);
    this->jntId_dctl.assign(this->jointNum,0);

    this->motor_pos.assign(this->jointNum,0);
    this->motor_vel.assign(this->jointNum,0);
    this->motor_pos_Old.assign(this->jointNum, 0);

    // Match joint id with the JointName list order
    for (int i = 0; i < this->jointNum; i++)
    {
        int tmpId = mj_name2id(this->mj_model, mjOBJ_JOINT, this->JointName[i].c_str());

        if (tmpId == -1) // No jointName[i] found
        {
            std::cout<<this->JointName[i].c_str() << " not found in XML \n";
        }

        jntId_qpos[i] = this->mj_model->jnt_qposadr[tmpId];
        jntId_qvel[i] = mj_model->jnt_dofadr[tmpId];

        // Find the actuator that drives this joint by transmission target,
        // rather than guessing a name convention (e.g. "M" + jointName) --
        // the XML's actuator names don't follow any fixed pattern relative
        // to their joint names (e.g. "right_hip_pitch_joint" is driven by
        // actuator "Right_Hip_Pitch_m").
        int actuatorId = -1;
        for (int a = 0; a < mj_model->nu; a++)
        {
            if (mj_model->actuator_trntype[a] == mjTRN_JOINT && mj_model->actuator_trnid[2 * a] == tmpId)
            {
                actuatorId = a;
                break;
            }
        }
        if (actuatorId == -1)
        {
            std::cerr << JointName[i] << ": no actuator drives this joint in the XML file!" << std::endl;
            std::terminate();
        }
        jntId_dctl[i] = actuatorId;
    }
}

void MJ_Interface::updateSensorValues()
{
    // update joint position
    for (int i = 0; i < this->jointNum; i++)
    {
        this->motor_pos_Old[i] = this->motor_pos[i];
        this->motor_pos[i] = this->mj_data->qpos[this->jntId_qpos[i]];
        this->motor_vel[i] = this->mj_data->qvel[this->jntId_qvel[i]];
    }
    return;
}

std::vector<double> MJ_Interface::getJointPos()
{
    return this->motor_pos;
}

std::vector<double> MJ_Interface::getJointVel()
{
    return this->motor_vel;
}

void MJ_Interface::setMotorsTorque(std::vector<double> &tauIn)
{
    for (int i = 0; i < jointNum; i++)
        mj_data->ctrl[jntId_dctl[i]] = tauIn.at(i);
    return;
}

void MJ_Interface::dataBusWrite(DataBus &busIn)
{
    // update to data bus so other controller can read motor states through data bus
    busIn.motors_pos_cur = motor_pos;
    busIn.motors_vel_cur = motor_vel;
}

// Printing out function
void MJ_Interface::printInfo()
{
    std::printf("Number of joint: %d\n", this->jointNum);
    std::printf("List of joints:\n");

    for (int i = 0; i < this->jointNum; i++)
    {
        std::printf("  [%2d] %-25s \n", i, this->JointName[i].c_str());
    }

    std::printf("Corresponding joint_pos id: ");

    for (int i = 0; i < this->jointNum; i++)
    {
        if (i == this->jointNum - 1) std::printf("  %d \n", this->jntId_qpos[i]);
        else std::printf("  %d, ", this->jntId_qpos[i]);
    }

    std::printf("Corresponding joint_velocity id: ");

    for (int i = 0; i < this->jointNum; i++)
    {
        if (i == this->jointNum - 1) std::printf("  %d \n", this->jntId_qvel[i]);
        else std::printf("  %d, ", this->jntId_qvel[i]);
    }

    std::printf("Corresponding actuator id: ");

    for (int i = 0; i < this->jointNum; i++)
    {
        if (i == this->jointNum - 1) std::printf("  %d \n", this->jntId_dctl[i]);
        else std::printf("  %d, ", this->jntId_dctl[i]);
    }
}

void MJ_Interface::printJointPos()
{
    for (int i = 0; i < this->jointNum; i++)
    {
        std::printf("Joint %-25s position: %.3f\n", this->JointName[i].c_str(), this->motor_pos[this->jntId_qpos[i]]);
    }
}
