#include "passive_control.h"

passive_control::passive_control(const std::string& urdf_string,const std::string& end_effector)
{
    _tools.init_rbdyn(urdf_string, end_effector);

    dsGain_pos = 5.00;
    dsGain_ori = 2.50;

    dsContPos = std::make_unique<DSController>(3, 10., 10.);
    dsContOri = std::make_unique<DSController>(3, 5., 5.);
    
  
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

    _robot.nulljnt_position << 0.0, 0.0, -0.10, 0.0, 0.1, 0.1, 0.10;


}

passive_control::~passive_control()
{
}

void passive_control::updateRobot(const Eigen::VectorXd& jnt_p,const Eigen::VectorXd& jnt_v,const Eigen::VectorXd& jnt_t){
    

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

void passive_control::set_desired_pose(const Eigen::Vector3d& pos, const Eigen::Vector4d& quat){
    _robot.ee_des_pos = pos;
    _robot.ee_des_quat = quat;

    //! maybe computing desired vel and acc here?
    // desired position values
    Eigen::Vector3d deltaX = (_robot.ee_des_pos - _robot.ee_pos);
    if (deltaX.norm() > 0.10)
        deltaX = 0.1 * deltaX.normalized();
    _robot.ee_des_vel   = dsGain_pos * Eigen::Matrix3d::Identity(3,3) *deltaX;

    // desired angular values
    Eigen::Vector4d dqd = Utils<double>::slerpQuaternion(_robot.ee_quat, _robot.ee_des_quat, 0.5);    
    Eigen::Vector4d deltaQ;
    // for some reason this for loop is necessary!
    for (int i =0; i<4; i++)
      deltaQ[i] = dqd[i] -  _robot.ee_quat[i];

    Eigen::Vector4d qconj = _robot.ee_quat;
    qconj.segment(1,3) = -1 * qconj.segment(1,3);
    Eigen::Vector4d temp_angVel = Utils<double>::quaternionProduct(deltaQ, qconj);

    Eigen::Vector3d tmp_angular_vel = temp_angVel.segment(1,3);
    if (tmp_angular_vel.norm() > 0.2)
        tmp_angular_vel = 0.2 * tmp_angular_vel.normalized();
    _robot.ee_des_angVel    = 2 * dsGain_ori * tmp_angular_vel;

}

void passive_control::set_pos_gains(const double& ds, const double& lambda0,const double& lambda1){
    dsGain_pos = ds;
    dsContPos->set_damping_eigval(lambda0,lambda1);

}
void passive_control::set_ori_gains(const double& ds, const double& lambda0,const double& lambda1){
    dsGain_ori = ds;
    dsContOri->set_damping_eigval(lambda0,lambda1);
}
void passive_control::set_null_pos(const Eigen::VectorXd& nullPosition){
    if (nullPosition.size() == _robot.nulljnt_position.size() )
    {
        _robot.nulljnt_position = nullPosition;
    }else{
        ROS_ERROR("wrong size for the null joint position");
    }
}
void passive_control::computeTorqueCmd(){
    
    // -----------------------get desired force in task space
    dsContPos->Update(_robot.ee_vel,_robot.ee_des_vel);
    Eigen::Vector3d wrenchPos = dsContPos->control_output();   
    Eigen::VectorXd tmp_jnt_trq_pos = _robot.jacobPos.transpose() * wrenchPos;

    // Orientation
    dsContOri->Update(_robot.ee_angVel,_robot.ee_des_angVel);
    Eigen::Vector3d wrenchAng   = dsContOri->control_output();
    Eigen::VectorXd tmp_jnt_trq_ang = _robot.jacobAng.transpose() * wrenchAng;


    //sum up:
    Eigen::VectorXd tmp_jnt_trq = tmp_jnt_trq_pos + tmp_jnt_trq_ang;

    // null pos control
    Eigen::MatrixXd tempMat2 =  Eigen::MatrixXd::Identity(7,7) - _robot.jacob.transpose()* _robot.pseudo_inv_jacob* _robot.jacob;

    Eigen::VectorXd tmp_null_trq = Eigen::VectorXd::Zero(7);
    for (int i =0; i<7; i++){ 
        tmp_null_trq[i] = -1. * (_robot.jnt_position[i] -_robot.nulljnt_position[i]);
        tmp_null_trq[i] +=-1. * _robot.jnt_velocity[i];
    }
    tmp_jnt_trq += tempMat2 * tmp_null_trq;
    
    // Gravity Compensationn
    // the gravity compensation should've been here, but a server form iiwa tools is doing the job.
   _trq_cmd = tmp_jnt_trq;

}