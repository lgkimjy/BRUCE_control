#include "WBC/wbic.hpp"
#include "bruce_controller.hpp"

  // related terms, + \ddot{q}_\text{cmd} + GRFs
Eigen::VectorXd WBIC::update(Robot &robot, BRUCEController& controller, Eigen::VectorXd qcmd, Eigen::VectorXd rho)
{
    std::cout << "WBIC::update()" << std::endl;
    // Formulate Tasks and Constraints
    wbcBase::update(robot, controller);
    
    Task constraints = formulateConstraints(qcmd, rho);
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
    Task relaxationTask = formulateRelaxationTask();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = relaxationTask.a_.transpose() * relaxationTask.a_;
    Eigen::VectorXd g = -relaxationTask.a_.transpose() * relaxationTask.b_;

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

Task WBIC::formulateConstraints(Eigen::VectorXd qcmd, Eigen::VectorXd rho)
{
    Task relaxed_task1, relaxed_task2, relaxed_task3;

    // equality constraints
    relaxed_task1.b_ = formulateFloatingBaseConstraint().b_ + Sf_ * (robot_.M_mat_q * qcmd - Jp_contact_.transpose() * rho);
    relaxed_task2.b_ = formulateContactConstraint().b_ + Jp_contact_ * qcmd;

    // inequality constraints
    relaxed_task3.f_ = formulateFrictionConeConstraint().f_ + rho;

    return relaxed_task1 + relaxed_task2 + relaxed_task3;
}

Task WBIC::formulateRelaxationTask()
{
    return {};
}