#pragma once

#include <cstdio>
#include <iostream>
#include <numeric>
#include "math.h"
#include <vector>
#include <string>
#include <fstream>
#include <optimization_lib/QpoasesOptimizer.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace optimization_lib;

class QP_Control{

public:
    QpoasesOptimizer<SQProblem> myqp;

    bool is_initialised = false;
    QP_Control();
    ~QP_Control();
    void initialise(const Eigen::MatrixXd &H, const Eigen::VectorXd &g);
    Eigen::VectorXd solve(const Eigen::MatrixXd &H, const Eigen::VectorXd &g);

};


    