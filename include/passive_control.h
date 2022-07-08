//|
//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Farshad Khadivr (maintainer)
//|    email:   farshad.khadivar@epfl.ch
//|    website: lasa.epfl.ch
//|
//|    This file is part of iiwa_toolkit.
//|
//|    iiwa_toolkit is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    iiwa_toolkit is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|
#ifndef __PASSIVE_CONTROL__
#define __PASSIVE_CONTROL__
#include "ros/ros.h"
#include <ros/package.h>

#include <mutex>
#include <fstream>
#include <pthread.h>
#include <vector>
#include <memory>

#include <Eigen/Dense>
#include <iiwa_tools/iiwa_tools.h>
#include <RBDyn/FD.h>
#include "thirdparty/Utils.h"


struct Robot
{
    unsigned int no_joints = 7;
    Eigen::VectorXd jnt_position = Eigen::VectorXd(no_joints);
    Eigen::VectorXd jnt_velocity = Eigen::VectorXd(no_joints);
    Eigen::VectorXd jnt_torque = Eigen::VectorXd(no_joints);
    
    Eigen::VectorXd nulljnt_position = Eigen::VectorXd(no_joints);
    std::string name = "robot_";

    Eigen::Vector3d ee_pos, ee_vel, ee_acc, ee_angVel, ee_angAcc;
    Eigen::Vector4d ee_quat;

    Eigen::Vector3d ee_des_pos, ee_des_vel, ee_des_acc, ee_des_angVel, ee_des_angAcc;
    Eigen::Vector4d ee_des_quat;


    Eigen::MatrixXd jacob       = Eigen::MatrixXd(6, 7);
    Eigen::MatrixXd jacob_drv   = Eigen::MatrixXd(6, 7);
    Eigen::MatrixXd jacob_t_pinv= Eigen::MatrixXd(7, 6);
    Eigen::MatrixXd jacobPos    = Eigen::MatrixXd(3, 7);
    Eigen::MatrixXd jacobAng    = Eigen::MatrixXd(3, 7);

    Eigen::MatrixXd pseudo_inv_jacob        = Eigen::MatrixXd(6,6);
    Eigen::MatrixXd pseudo_inv_jacobJnt     = Eigen::MatrixXd(7,7);
    Eigen::MatrixXd pseudo_inv_jacobPos     = Eigen::MatrixXd(3,3);
    Eigen::MatrixXd pseudo_inv_jacobPJnt    = Eigen::MatrixXd(7,7);
    Eigen::MatrixXd joint_inertia           = Eigen::MatrixXd(7,7);
    Eigen::MatrixXd task_inertia            = Eigen::MatrixXd(6,6);
    Eigen::MatrixXd task_inertiaPos         = Eigen::MatrixXd(3,3);
    Eigen::MatrixXd task_inertiaAng         = Eigen::MatrixXd(3,3);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> pseudo_inverse(const MatT& mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance) {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}
class PassiveDS
{
private:
    double eigVal0;
    double eigVal1;
    Eigen::Matrix3d damping_eigval = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d baseMat = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d Dmat = Eigen::Matrix3d::Identity();
    Eigen::Vector3d control_output = Eigen::Vector3d::Zero();
    void updateDampingMatrix(const Eigen::Vector3d& ref_vel);
public:
    PassiveDS(const double& lam0, const double& lam1);
    ~PassiveDS();
    void set_damping_eigval(const double& lam0, const double& lam1);
    void update(const Eigen::Vector3d& vel, const Eigen::Vector3d& des_vel);
    Eigen::Vector3d get_output();
};




class PassiveControl
{
private:
    Robot _robot;
    iiwa_tools::IiwaTools _tools;
    rbd::ForwardDynamics _fdyn;

    bool first = true;
    bool is_just_velocity = false;
    double dsGain_pos;
    double dsGain_ori;
    double load_added = 0.;

    Eigen::VectorXd _trq_cmd = Eigen::VectorXd::Zero(7);
    void computeTorqueCmd();
   
    std::unique_ptr<PassiveDS> dsContPos;
    std::unique_ptr<PassiveDS> dsContOri;
    
public:
    PassiveControl();
    PassiveControl(const std::string& urdf_string,const std::string& end_effector);
    ~PassiveControl();
    void updateRobot(const Eigen::VectorXd& jnt_p,const Eigen::VectorXd& jnt_v,const Eigen::VectorXd& jnt_t);
    
    void set_desired_pose(const Eigen::Vector3d& pos, const Eigen::Vector4d& quat);
    void set_desired_position(const Eigen::Vector3d& pos);
    void set_desired_quat(const Eigen::Vector4d& quat);
    void set_desired_velocity(const Eigen::Vector3d& vel);


    void set_pos_gains(const double& ds, const double& lambda0,const double& lambda1);
    void set_ori_gains(const double& ds, const double& lambda0,const double& lambda1);
    void set_null_pos(const Eigen::VectorXd& nullPosition);
    void set_load(const double& mass);


    Eigen::VectorXd getCmd(){
        computeTorqueCmd();
        return _trq_cmd;}
    Eigen::Vector3d getEEpos();
    Eigen::Vector3d getEEVel();
    Eigen::Vector3d getEEAngVel();

    Eigen::Vector4d getEEquat();
    
    Eigen::MatrixXd getTaskInertiaPos();
    Eigen::MatrixXd jointToTaskInertia(const Eigen::MatrixXd& Jac, const Eigen::MatrixXd& joint_inertia);

};

#endif