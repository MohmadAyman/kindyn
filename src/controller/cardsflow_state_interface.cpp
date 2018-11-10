#include "kindyn/controller/cardsflow_state_interface.hpp"

namespace hardware_interface
{

    CardsflowStateHandle::CardsflowStateHandle(){}
     /**
      *
      * @param name The name of the joint
      * @param joint_index index of the joint in the robot model
      * @param pos A pointer to the storage for this joint's position
      * @param vel A pointer to the storage for this joint's velocity

      */
    CardsflowStateHandle::CardsflowStateHandle(const std::string& name, int joint_index, const double* pos, const double* vel, const double* acc,
                                               const MatrixXd *L, const MatrixXd *M, const VectorXd *CG, const VectorXd *l, const VectorXd *l_target)
            : name_(name), joint_index_(joint_index), pos_(pos), vel_(vel), acc_(acc), L_(L), M_(M), CG_(CG), l_(l), l_target_(l_target)
    {
        if (!pos)
        {
            throw HardwareInterfaceException("Cannot create handle '" + name + "'. Position data pointer is null.");
        }
        if (!vel)
        {
            throw HardwareInterfaceException("Cannot create handle '" + name + "'. Velocity data pointer is null.");
        }
        if (!acc)
        {
            throw HardwareInterfaceException("Cannot create handle '" + name + "'. Acceleration data pointer is null.");
        }
    }

    std::string CardsflowStateHandle::getName() const {return name_;}
    int CardsflowStateHandle::getJointIndex()  const { return joint_index_;}
    double CardsflowStateHandle::getPosition()  const {assert(pos_); return *pos_;}
    double CardsflowStateHandle::getVelocity()  const {assert(vel_); return *vel_;}
    double CardsflowStateHandle::getAcceleration()  const {assert(acc_); return *acc_;}
    VectorXd CardsflowStateHandle::getl()    const {return *l_;}
    VectorXd CardsflowStateHandle::getl_target()    const {return *l_target_;}
    MatrixXd CardsflowStateHandle::getL()    const {return *L_;}
    MatrixXd CardsflowStateHandle::getM()    const {return *M_;}
    VectorXd CardsflowStateHandle::getCG()    const {return *CG_;}

}