#include "qp_control.h"

QP_Control::QP_Control(){}
QP_Control::~QP_Control(){}

void QP_Control::optimize(const Eigen::MatrixXd &H, const Eigen::VectorXd &g){

    int nV = 7;
    Eigen::VectorXd lb(nV), ub(nV);

    lb << -1.48, -1.48, -1.74, -1.30, -2.26, -2.35, -2.35;
    ub << 1.48, 1.48, 1.74, 1.30, 2.26, 2.35, 2.35;

    QpoasesOptimizer myqp;
    myqp
        .setHessianMatrix(H)
        .setGradientVector(g)
        .setVariablesBoundaries(lb, ub)
        .init();

    std::cout << "init: " << myqp.solution().transpose() << std::endl;

    myqp
        .setGradientVector(g)
        .setVariablesBoundaries(lb, ub)
        .setRecalculation(10)
        .optimize();

    std::cout << "hotstart: " << myqp.solution().transpose() << std::endl;
  
    // return 0;
}

