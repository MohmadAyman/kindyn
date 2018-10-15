#include "kindyn/controller/cardsflow_command_interface.hpp"

namespace hardware_interface
{


    CardsflowHandle::CardsflowHandle() : CardsflowStateHandle(){}

    /**
     * \param js This joint's state handle
     * \param cmd A pointer to the storage for this joint's output command
     */
    CardsflowHandle::CardsflowHandle(const CardsflowStateHandle& js, double* joint_position_cmd,
                                     double* joint_velocity_cmd, VectorXd *motor_cmd)
            : CardsflowStateHandle(js), joint_position_cmd_(joint_position_cmd), joint_velocity_cmd_(joint_velocity_cmd), motor_cmd_(motor_cmd)
    {
    }

    void CardsflowHandle::setMotorCommand(VectorXd command) {*motor_cmd_ = command;}
    double CardsflowHandle::getJointPositionCommand() const {return *joint_position_cmd_;}
    double CardsflowHandle::getJointVelocityCommand() const {return *joint_velocity_cmd_;}

}
