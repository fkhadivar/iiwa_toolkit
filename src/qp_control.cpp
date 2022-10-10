#include "qp_control.h"

QP_Control::QP_Control(){}
QP_Control::~QP_Control(){}


void QP_Control::initialise(const Eigen::MatrixXd &H, const Eigen::VectorXd &g){

    int nV = 7;
    Eigen::VectorXd lb(nV), ub(nV);
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(7,7);

    lb << -1.48, -1.48, -1.74, -1.30, -2.26, -2.35, -2.35;
    ub << 1.48, 1.48, 1.74, 1.30, 2.26, 2.35, 2.35;

    myqp
        .setHessianMatrix(H)
        .setGradientVector(g)
        .setLinearConstraints(A)
        .setLinearConstraintsBoundaries(lb, ub)
        .setVariablesBoundaries(lb, ub)
        .init();
    
    is_initialised = true;
}

Eigen::VectorXd QP_Control::solve(const Eigen::MatrixXd &H, const Eigen::VectorXd &g){

    myqp
        .setHessianMatrix(H)
        .setGradientVector(g)
        // .setVariablesBoundaries(lb, ub)
        .setRecalculation(100)
        .optimize();

    return myqp.solution();
}

