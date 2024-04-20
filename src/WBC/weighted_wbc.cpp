#include "WBC/weighted_wbc.hpp"

Eigen::VectorXd WeightedWBC::update()
{
    // Formulate Tasks and Constraints
    wbcBase::update();

    // Constraints (inequality, equality)
    
    // Objective Function (Cost)

    // Solve
    // auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
    // qpOASES::Options options;
    // options.setToMPC();
    // options.printLevel = qpOASES::PL_LOW;
    // options.enableEqualities = qpOASES::BT_TRUE;
    // qpProblem.setOptions(options);
    // int nWsr = 20;

    // qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
    // vector_t qpSol(getNumDecisionVars());

    // qpProblem.getPrimalSolution(qpSol.data());
    // return qpSol;
    return Eigen::VectorXd::Zero(10);
}