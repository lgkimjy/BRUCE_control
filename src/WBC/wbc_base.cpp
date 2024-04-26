#include "WBC/wbc_base.hpp"
#include "bruce_controller.hpp"

wbcBase::wbcBase()
{
    std::cout << "wbcBase Constructor" << std::endl;
    numDecisionVars_ = 0;

    // Sf.setZero();
    // Sf = Eigen::Matrix<double, 6, 6>::Identity();
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////// Updates measured and desired states values -> by segmenting the vectors
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd wbcBase::update(Robot &robot, BRUCEController &controller)
{
    // Update Memebr Variables from Robot and Controller Class,
    // which will be used to formulate Tasks and constraints.
    // Also, resize the decision varaiables according to the contact forces and contact states
    std::cout << "Base::update()" << std::endl;
    
    robot_ = robot;

    if(controller.stateMachine == DOUBLE_STANCE) {
        numDecisionVars_ = robot_.nv_ + (2 * 6);      // generalized coordinates + ( # x polyhedral approximation size )
    } else if(controller.stateMachine == RIGHT_CONTACT || controller.stateMachine == LEFT_CONTACT){
        numDecisionVars_ = robot_.nv_ + 6;            // generalized coordinates + ( polyhedral approximation size )
    } else {
        std::cerr << "Invalid State Machine" << std::endl;
    }

    // std::cout << controller.p_EE.size() << std::endl;
    // for(int i=0; i<4; i++)
    // {
    //     std::cout << i << " " << std::endl;
    //     p_EE_[i] = controller.p_EE[i];
    // }

    return {};
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////// Task Formulation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Task wbcBase::formulateMomentumTask()
{
    // matrix_t a(6, numDecisionVars_);
    // a.setZero();
    // a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

    // vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
    // inputLast_ = inputDesired;
    // mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);

    // const auto& model = pinocchioInterfaceDesired_.getModel();
    // auto& data = pinocchioInterfaceDesired_.getData();
    // const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
    // const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

    // const auto& A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
    // const Matrix6 Ab = A.template leftCols<6>();
    // const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
    // const auto Aj = A.rightCols(info_.actuatedDofNum);
    // const auto ADot = pinocchio::dccrba(model, data, qDesired, vDesired);
    // Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
    // centroidalMomentumRate.noalias() -= ADot * vDesired;
    // centroidalMomentumRate.noalias() -= Aj * jointAccel;

    // Vector6 b = AbInv * centroidalMomentumRate;

    // return {a, b, matrix_t(), vector_t()};
    return {};
}

Task wbcBase::formulateContactTask()
{
    return {};
}

Task wbcBase::formulateTorqueLimitsTask()
{
    return {};
}

Task wbcBase::formulateSwingLegTask()
{
    // eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
    // std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
    // std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
    // eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
    // std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
    // std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

    // matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    // vector_t b(a.rows());
    // a.setZero();
    // b.setZero();
    // size_t j = 0;
    // for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    //     if (!contactFlag_[i]) {
    //     vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
    //     a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
    //     b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
    //     j++;
    //     }
    // }

    // return {a, b, matrix_t(), vector_t()};
    return {};
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////// Constraints Formulation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Task wbcBase::formulateFloatingBaseConstraint()
{
    // auto& data = pinocchioInterfaceMeasured_.getData();

    // matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
    // s.block(0, 0, info_.actuatedDofNum, 6).setZero();
    // s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

    // matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose()).finished();
    // vector_t b = -data.nle;

    // return {a, b, matrix_t(), vector_t()};

    // Mqddot + Cqdot + G = Sf * f
    // Mqddot + Cqdot + G = Sf * U * R_contact * rho
    
    // header -> U, Sf
    // input -> qddot, rho
    // robot -> M, C, G,
    
    // needed -> R_contact, qdot
    
    // return {matrix_t(), vector_t(), matrix_t(), vector_t()};
    return {};
}

Task wbcBase::formulateContactConstraint()
{
    // Jdotqdot + Jqddot = 0

    // needed -> J, Jdot, qdot, qddot

    return {};
}

Task wbcBase::formulateFrictionConeConstraint()
{
    Eigen::MatrixXd a;
    Eigen::VectorXd b;
    Eigen::MatrixXd d;
    Eigen::VectorXd f;

    // f = U * R_contact & rho > 0 
    // rho > 0

    // header -> U, rho
    // needed -> R_contact

    return {a, b, d, f};
}