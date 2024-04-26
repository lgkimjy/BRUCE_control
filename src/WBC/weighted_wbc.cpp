#include "WBC/weighted_wbc.hpp"
#include "bruce_controller.hpp"

// Eigen::VectorXd WeightedWBC::update(Robot &robot)
Eigen::VectorXd WeightedWBC::update(Robot &robot, BRUCEController &controller)
{
    std::cout << "WeightedWBC::update()" << std::endl;
    // Formulate Tasks and Constraints
    wbcBase::update(robot, controller);

    Task constraints = formulateConstraints();
    size_t numConstraints = constraints.b_.size() + constraints.f_.size();

    // Constraints (inequality, equality)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
    vector_t lbA(numConstraints), ubA(numConstraints);  // clang-format off
    // A << constraints.a_,
    //     constraints.d_;

    // lbA << constraints.b_,
    //         -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
    // ubA << constraints.b_,
    //         constraints.f_;  // clang-format on

    // Objective Function (Cost)
    Task weightedTask = formulateWeightedTask();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weightedTask.a_.transpose() * weightedTask.a_;
    Eigen::VectorXd g = -weightedTask.a_.transpose() * weightedTask.b_;

    // Solve
    auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    options.enableEqualities = qpOASES::BT_TRUE;
    qpProblem.setOptions(options);
    int nWsr = 20;

    // qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
    Eigen::VectorXd qpSol(getNumDecisionVars());
    qpProblem.getPrimalSolution(qpSol.data());

    return qpSol;
}

Task WeightedWBC::formulateConstraints()
{
    return formulateFloatingBaseConstraint() + \
            formulateContactConstraint() + \
            formulateFrictionConeConstraint();
}

Task WeightedWBC::formulateWeightedTask()
{
    return formulateSwingLegTask() * swingLegWeight_ + \
           formulateMomentumTask() * baseAccelWeight_ + \
           formulateContactTask() * contactForceWeight_;
}