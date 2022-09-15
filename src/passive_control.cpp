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

#include "passive_control.h"

PassiveDS::PassiveDS(const double& lam0, const double& lam1):eigVal0(lam0),eigVal1(lam1){
    set_damping_eigval(lam0,lam1);
}

PassiveDS::~PassiveDS(){}
void PassiveDS::set_damping_eigval(const double& lam0, const double& lam1){
    if((lam0 > 0)&&(lam1 > 0)){
        eigVal0 = lam0;
        eigVal1 = lam1;
        damping_eigval(0,0) = eigVal0;
        damping_eigval(1,1) = eigVal1;
        damping_eigval(2,2) = eigVal1;
    }else{
        std::cerr << "wrong values for the eigenvalues"<<"\n";
    }
}
void PassiveDS::updateDampingMatrix(const Eigen::Vector3d& ref_vel){ 

    if(ref_vel.norm() > 1e-6){
        baseMat.setRandom();
        baseMat.col(0) = ref_vel.normalized();
        for(uint i=1;i<3;i++){
            for(uint j=0;j<i;j++)
                baseMat.col(i) -= baseMat.col(j).dot(baseMat.col(i))*baseMat.col(j);
            baseMat.col(i).normalize();
        }
        Dmat = baseMat*damping_eigval*baseMat.transpose();
    }else{
        Dmat = Eigen::Matrix3d::Identity();
    }
    // otherwise just use the last computed basis
}

void PassiveDS::update(const Eigen::Vector3d& vel, const Eigen::Vector3d& des_vel){
    // compute damping
    updateDampingMatrix(des_vel);
    // dissipate
    control_output = - Dmat * vel;
    // compute control
    control_output += eigVal0*des_vel;
}
Eigen::Vector3d PassiveDS::get_output(){ return control_output;}

//************************************************

PassiveControl::PassiveControl(const std::string& urdf_string,const std::string& end_effector)
{
    _tools.init_rbdyn(urdf_string, end_effector);

    dsGain_pos = 5.00;
    dsGain_ori = 2.50;

    dsContPos = std::make_unique<PassiveDS>( 100., 100.);
    dsContOri = std::make_unique<PassiveDS>(5., 5.);
    
  
    _robot.name +=std::to_string(0);
    _robot.jnt_position.setZero();
    _robot.jnt_velocity.setZero();
    _robot.jnt_torque.setZero();
    _robot.nulljnt_position.setZero();
    _robot.ee_pos.setZero(); 
    _robot.ee_vel.setZero();   
    _robot.ee_acc.setZero();

    
    double angle0 = 0.25*M_PI;
    _robot.ee_quat[0] = (std::cos(angle0/2));
    _robot.ee_quat.segment(1,3) = (std::sin(angle0/2))* Eigen::Vector3d::UnitZ();
    
    _robot.ee_angVel.setZero();
    _robot.ee_angAcc.setZero();
   
   //* desired things
    _robot.ee_des_pos = {-0.5 , 0.5, 0.2}; 
    double angled = 1.0*M_PI;
    _robot.ee_des_quat[0] = (std::cos(angled/2));
    _robot.ee_des_quat.segment(1,3) = (std::sin(angled/2))* Eigen::Vector3d::UnitX();
    
    //** do we need these parts in here?
    _robot.ee_des_vel.setZero();   
    _robot.ee_des_acc.setZero();
    _robot.ee_des_angVel.setZero();
    _robot.ee_des_angAcc.setZero();


    _robot.jacob.setZero();
    _robot.jacob.setZero();       
    _robot.jacob_drv.setZero();   
    _robot.jacob_t_pinv.setZero();
    _robot.jacobPos.setZero();   
    _robot.jacobAng.setZero();
    _robot.pseudo_inv_jacob.setZero();   
    _robot.pseudo_inv_jacobPos.setZero();

    _robot.nulljnt_position << 0.0, 0.0, 0.0, -.75, 0., 0.0, 0.0;


}

PassiveControl::~PassiveControl(){}



void PassiveControl::updateRobot(const Eigen::VectorXd& jnt_p,const Eigen::VectorXd& jnt_v,const Eigen::VectorXd& jnt_t){
    

    _robot.jnt_position = jnt_p;
    _robot.jnt_velocity = jnt_v;
    _robot.jnt_torque   = jnt_t;

    iiwa_tools::RobotState robot_state;
    robot_state.position.resize(jnt_p.size());
    robot_state.velocity.resize(jnt_p.size());
    for (size_t i = 0; i < jnt_p.size(); i++) {
        robot_state.position[i] = _robot.jnt_position[i];
        robot_state.velocity[i] = _robot.jnt_velocity[i];
    }

    std::tie(_robot.jacob, _robot.jacob_drv) = _tools.jacobians(robot_state);
    _robot.jacobPos =  _robot.jacob.bottomRows(3);
    _robot.jacobAng =  _robot.jacob.topRows(3);

    _robot.pseudo_inv_jacob    = pseudo_inverse(Eigen::MatrixXd(_robot.jacob * _robot.jacob.transpose()) );
    _robot.pseudo_inv_jacobPos = pseudo_inverse(Eigen::MatrixXd(_robot.jacobPos * _robot.jacobPos.transpose()) );
    // _robot.pseudo_inv_jacobPJnt = pseudo_inverse(Eigen::MatrixXd(_robot.jacobPos.transpose() * _robot.jacobPos ) );
    _robot.pseudo_inv_jacobJnt = pseudo_inverse(Eigen::MatrixXd(_robot.jacob.transpose() * _robot.jacob ) );
    

    _robot.joint_inertia = _tools.get_joint_inertia(robot_state);
    _robot.task_inertiaPos = jointToTaskInertia(_robot.jacobPos, _robot.joint_inertia);
    _robot.dir_task_inertia_grad = getDirInertiaGrad(robot_state, _robot.direction);
    
    auto ee_state = _tools.perform_fk(robot_state);
    _robot.ee_pos = ee_state.translation;
    _robot.ee_quat[0] = ee_state.orientation.w();
    _robot.ee_quat.segment(1,3) = ee_state.orientation.vec();
    

    Eigen::VectorXd vel = _robot.jacob * _robot.jnt_velocity;
    _robot.ee_vel    = vel.tail(3); // check whether this is better or filtering position derivitive

    _robot.ee_angVel = vel.head(3); // compare it with your quaternion derivitive equation

    // for(int i = 0; i < 3; i++)
    //     _plotVar.data[i] = (_robot.ee_des_vel - _robot.ee_vel)[i];
}

Eigen::Vector3d PassiveControl::getEEpos(){
    return _robot.ee_pos;
}

Eigen::Vector4d PassiveControl::getEEquat(){
    return _robot.ee_quat;
}
Eigen::Vector3d PassiveControl::getEEVel(){
    return _robot.ee_vel;
}
Eigen::Vector3d PassiveControl::getEEAngVel(){
    return _robot.ee_angVel;
}

Eigen::MatrixXd PassiveControl::jointToTaskInertia(const Eigen::MatrixXd& Jac, const Eigen::MatrixXd& joint_inertia){
    Eigen::MatrixXd task_inertia_inverse = Jac * joint_inertia.inverse() * Jac.transpose();
    Eigen::MatrixXd task_inertia = task_inertia_inverse.inverse();
    return task_inertia;
}

Eigen::MatrixXd PassiveControl::getTaskInertiaPos(){
    return _robot.task_inertiaPos;
}

Eigen::VectorXd PassiveControl::getDirInertiaGrad(iiwa_tools::RobotState &current_state, Eigen::Vector3d &direction){
    Eigen::MatrixXd task_inertia = _robot.task_inertiaPos;
    double dir_inertia = direction.transpose() * task_inertia * direction;
    Eigen::VectorXd grad = Eigen::VectorXd(7);
    Eigen::MatrixXd duplicate_joint_inertia = Eigen::MatrixXd(7,7);
    Eigen::MatrixXd duplicate_task_inertiaPos = Eigen::MatrixXd(6,6);
    Eigen::MatrixXd jacob = Eigen::MatrixXd(6, 7);
    Eigen::MatrixXd jacob_drv = Eigen::MatrixXd(6, 7);
    Eigen::MatrixXd jacobPos = Eigen::MatrixXd(3, 7);
    double duplicate_dir_inertia;

    // create a duplicate current state
    iiwa_tools::RobotState duplicate_state = current_state;
    double dq = 0.001;
    for(int i = 0; i < 7; ++i){
        duplicate_state.position[i]+=dq;
        duplicate_joint_inertia = _tools.get_joint_inertia(duplicate_state);

        std::tie(jacob, jacob_drv) = _tools.jacobians(duplicate_state);
        jacobPos = jacob.bottomRows(3);
        duplicate_task_inertiaPos = jointToTaskInertia(jacobPos, duplicate_joint_inertia);
        duplicate_dir_inertia = direction.transpose() * duplicate_task_inertiaPos * direction;

        grad[i] = (duplicate_dir_inertia - dir_inertia) / dq;

        duplicate_state.position[i]-=dq;
    }

    return grad;
}


void PassiveControl::set_pos_gains(const double& ds, const double& lambda0,const double& lambda1){
    dsGain_pos = ds;
    dsContPos->set_damping_eigval(lambda0,lambda1);

}
void PassiveControl::set_ori_gains(const double& ds, const double& lambda0,const double& lambda1){
    dsGain_ori = ds;
    dsContOri->set_damping_eigval(lambda0,lambda1);
}
void PassiveControl::set_null_pos(const Eigen::VectorXd& nullPosition){
    if (nullPosition.size() == _robot.nulljnt_position.size() )
    {
        _robot.nulljnt_position = nullPosition;
    }else{
        ROS_ERROR("wrong size for the null joint position");
    }
}


void PassiveControl::set_desired_pose(const Eigen::Vector3d& pos, const Eigen::Vector4d& quat){
    _robot.ee_des_pos = pos;
    _robot.ee_des_quat = quat;
    is_just_velocity = false;
}
void PassiveControl::set_desired_position(const Eigen::Vector3d& pos){
    _robot.ee_des_pos = pos;
     is_just_velocity = false;
}
void PassiveControl::set_desired_quat(const Eigen::Vector4d& quat){
    _robot.ee_des_quat = quat;
}
void PassiveControl::set_desired_velocity(const Eigen::Vector3d& vel){
     _robot.ee_des_vel = vel;
     is_just_velocity = true;
}


void PassiveControl::set_load(const double& mass ){
    load_added = mass;
}

Eigen::VectorXd PassiveControl::computeInertiaTorqueNull(float des_dir_lambda, Eigen::Vector3d& des_vel){
    
    Eigen::Vector3d direction = des_vel / des_vel.norm();
    float inertia_error = direction.transpose() * _robot.task_inertiaPos * direction - des_dir_lambda;
    std::cout << "inertia dir: " <<  direction.transpose() * _robot.task_inertiaPos * direction << std::endl; 
    // std::cout << "inertia error: " <<  inertia_error  << std::endl; 
    Eigen::VectorXd null_torque = 1.0 * _robot.dir_task_inertia_grad * inertia_error;

    // std::cout << "torque: " << null_torque << std::endl;
    return null_torque;
}

void PassiveControl::computeTorqueCmd(){
    
    // desired position values
    Eigen::Vector3d deltaX = _robot.ee_des_pos - _robot.ee_pos;
    double maxDx = 0.1;
    if (deltaX.norm() > maxDx)
        deltaX = maxDx * deltaX.normalized();
    
    double theta_g = (-.5/(4*maxDx*maxDx)) * deltaX.transpose() * deltaX;
    
    Eigen::Matrix3d zgain = Eigen::Matrix3d::Identity();
    zgain(0,0) *= 1.5; 
    zgain(2,2) *= 1.5; 

    Eigen::Matrix3d xgain = Eigen::Matrix3d::Identity();
    xgain(0,0) *= 1.5; 

    if(!is_just_velocity)
        _robot.ee_des_vel   = zgain * dsGain_pos*(Eigen::Matrix3d::Identity()+xgain*std::exp(theta_g)) *deltaX;

    // desired angular values
    Eigen::Vector4d dqd = Utils<double>::slerpQuaternion(_robot.ee_quat, _robot.ee_des_quat, 0.5);    
    Eigen::Vector4d deltaQ = dqd -  _robot.ee_quat;

    Eigen::Vector4d qconj = _robot.ee_quat;
    qconj.segment(1,3) = -1 * qconj.segment(1,3);
    Eigen::Vector4d temp_angVel = Utils<double>::quaternionProduct(deltaQ, qconj);

    Eigen::Vector3d tmp_angular_vel = temp_angVel.segment(1,3);
    double maxDq = 0.2;
    if (tmp_angular_vel.norm() > maxDq)
        tmp_angular_vel = maxDq * tmp_angular_vel.normalized();

    double theta_gq = (-.5/(4*maxDq*maxDq)) * tmp_angular_vel.transpose() * tmp_angular_vel;
    _robot.ee_des_angVel  = 2 * dsGain_ori*(1+std::exp(theta_gq)) * tmp_angular_vel;

    // -----------------------get desired force in task space
    dsContPos->update(_robot.ee_vel,_robot.ee_des_vel);
    Eigen::Vector3d wrenchPos = dsContPos->get_output() + load_added * 9.8*Eigen::Vector3d::UnitZ();   
    Eigen::VectorXd tmp_jnt_trq_pos = _robot.jacobPos.transpose() * wrenchPos;

    // Orientation
    dsContOri->update(_robot.ee_angVel,_robot.ee_des_angVel);
    Eigen::Vector3d wrenchAng   = dsContOri->get_output();
    Eigen::VectorXd tmp_jnt_trq_ang = _robot.jacobAng.transpose() * wrenchAng;


    //sum up:
    Eigen::VectorXd tmp_jnt_trq = tmp_jnt_trq_pos + tmp_jnt_trq_ang;

    // null pos control
    Eigen::MatrixXd tempMat2 =  Eigen::MatrixXd::Identity(7,7) - _robot.jacob.transpose()* _robot.pseudo_inv_jacob* _robot.jacob;
    Eigen::VectorXd nullgains = Eigen::VectorXd::Zero(7);
    // nullgains << 5.,80,10.,30,5.,2.,2.;
    nullgains << 5.,15.,25.,5.,20.,40.,40.;
    Eigen::VectorXd er_null;
    if(!is_just_velocity){
        er_null = 1.0*computeInertiaTorqueNull(6.0, _robot.direction);
    }
    else{
        er_null = 1.0*computeInertiaTorqueNull(6.0, _robot.ee_des_vel); 
    }
    // Eigen::VectorXd er_null = _robot.jnt_position -_robot.nulljnt_position;

    // std::cout << "null space error is: " << er_null << std::endl;
    if(er_null.norm()<1.5){
        first = false;
    }
    if(er_null.norm()>2e-1){
        er_null = 0.2*er_null.normalized();
    }
    Eigen::VectorXd tmp_null_trq = Eigen::VectorXd::Zero(7);
    for (int i =0; i<7; i++){ 
        tmp_null_trq[i] = -nullgains[i] * er_null[i];
        tmp_null_trq[i] +=-1. * _robot.jnt_velocity[i];
    }
    if (first){
        _trq_cmd = tmp_null_trq;
        ROS_INFO_ONCE("going to the first pose ");                 
    }else{
        ROS_INFO_ONCE("Tracking in process");
        _trq_cmd = tmp_jnt_trq + 10.*tempMat2 * tmp_null_trq;
    }
    
    
    
    // Gravity Compensationn
    // the gravity compensation should've been here, but a server form iiwa tools is doing the job.
   

}