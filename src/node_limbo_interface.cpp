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
#include <mutex>
#include <fstream>
#include <pthread.h>

#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>


// #include <iiwa_tools/iiwa_tools.h>


#define No_JOINTS 7
#define No_Robots 1
#define TOTAL_No_MARKERS 2

struct Options
{
    bool is_optitrack_on;
    double filter_gain = 0.;
};

struct feedback
{
    Eigen::VectorXd jnt_position = Eigen::VectorXd(No_JOINTS);
    Eigen::VectorXd jnt_velocity = Eigen::VectorXd(No_JOINTS);
    Eigen::VectorXd jnt_torque = Eigen::VectorXd(No_JOINTS);
};

class IiwaRosMaster 
{
  public:
    IiwaRosMaster(ros::NodeHandle &n,double frequency, Options options):
    _n(n), _loopRate(frequency), _dt(1.0f/frequency),_options(options){
        _stop =false;

    }

    ~IiwaRosMaster(){}

    bool init(){
        
        _feedback.jnt_position.setZero();
        _feedback.jnt_velocity.setZero();
        _feedback.jnt_torque.setZero();
        command_trq.setZero();
        command_plt.setZero();
        
        //!
        _subRobotStates[0]= _n.subscribe<sensor_msgs::JointState> ("/iiwa/joint_states", 1,
                boost::bind(&IiwaRosMaster::updateRobotStates,this,_1,0),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
        
        _torqueReceiver[0]= _n.subscribe<std_msgs::Float64MultiArray> ("/limbo/TorqueController/command", 1,
                boost::bind(&IiwaRosMaster::updateTorqueCommand,this,_1,0),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
        
        _plotReceiver[0] = _n.subscribe<std_msgs::Float64MultiArray> ("/limbo/plotvar", 1,
                boost::bind(&IiwaRosMaster::updatePlotVariable,this,_1,0),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
        // _subOptitrack[0] = _n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/baseHand/pose", 1,
            // boost::bind(&IiwaRosMaster::updateOptitrack,this,_1,0),ros::VoidPtr(),ros::TransportHints().reliable().tcpNoDelay());
        _TrqCmdPublisher = _n.advertise<std_msgs::Float64MultiArray>("/iiwa/TorqueController/command",1);

        // // Get the URDF XML from the parameter server
        // std::string urdf_string, full_param;
        // std::string robot_description = "robot_description";
        // std::string end_effector;
        // // gets the location of the robot description on the parameter server
        // if (!_n.searchParam(robot_description, full_param)) {
        //     ROS_ERROR("Could not find parameter %s on parameter server", robot_description.c_str());
        //     return false;
        // }
        // // search and wait for robot_description on param server
        // while (urdf_string.empty()) {
        //     ROS_INFO_ONCE_NAMED("Controller", "Controller is waiting for model"
        //                                                     " URDF in parameter [%s] on the ROS param server.",
        //         robot_description.c_str());
        //     _n.getParam(full_param, urdf_string);
        //     usleep(100000);
        // }
        // ROS_INFO_STREAM_NAMED("Controller", "Received urdf from param server, parsing...");

        // // Get the end-effector
        // _n.param<std::string>("params/end_effector", end_effector, "iiwa_link_ee");
        // // Initialize iiwa tools
        // _tools->init_rbdyn(urdf_string, end_effector);
        // size_t max_iteration = 5;
        // _controller = std::make_unique<control::TrackControl>(max_iteration);


        // plotting
        _plotPublisher = _n.advertise<std_msgs::Float64MultiArray>("/iiwa/plotvar",1);

        //todo condition here
        return true;
    }
    // run node
    void run(){
        while(!_stop && ros::ok()){ 
            _mutex.lock();
            

            // _controller->setInput(_feedback.jnt_position, _feedback.jnt_velocity,_feedback.jnt_torque);
            // std::cout << "pos: " << _feedback.jnt_position.transpose() << "\n";
            // std::cout << "vel: " << _feedback.jnt_velocity.transpose() << "\n";
            // std::cout << "trq: " << _feedback.jnt_torque.transpose() << "\n";
            // std::cout << "cmd Trq: " << command_trq.transpose() << "\n";
            if(!receiveing_msg){
                Eigen::VectorXd _jp(7);
                _jp << M_PI / 2., .3, 0., -M_PI / 2., 0., -.8, 0.;
                set_default_position(_jp);
            }
            //--- second joint friction:
            if (std::abs(_feedback.jnt_position[1]) >0.6)
            {
                double rand_trq = 1-2*( (double)std::rand()/RAND_MAX);
                command_trq[1] += 4*rand_trq; // pour lutter contre joint friction
                // std::cout << " second joints torque:  " << command_trq[1] << "randon torque added:  " << 5*rand_trq << "\n";
            }
            
            publishCommandTorque(command_trq);
            publishPlotVariable(command_plt);
            // publishPlotVariable(command_trq.tail(3));


            // publishPlotVariable(_controller->getPlotVariable());
                
            
            _mutex.unlock();

            receiveing_msg = false;    
            ros::spinOnce();
            _loopRate.sleep();
        }
        publishCommandTorque(Eigen::VectorXd::Zero(No_JOINTS));
        ros::spinOnce();
        _loopRate.sleep();
        ros::shutdown();
    }

  protected:
    double _dt;
    Options _options; 

    ros::NodeHandle _n;
    ros::Rate _loopRate;

    ros::Subscriber _subRobotStates[No_Robots];
    ros::Subscriber _torqueReceiver[No_Robots];
    ros::Subscriber _plotReceiver[1];


    ros::Subscriber _subOptitrack[TOTAL_No_MARKERS];  // optitrack markers pose

    ros::Publisher _TrqCmdPublisher;
    ros::Publisher _plotPublisher;

    feedback _feedback;
    // std::shared_ptr<iiwa_tools::IiwaTools> _tools;
    Eigen::VectorXd command_trq = Eigen::VectorXd(No_JOINTS);
    Eigen::VectorXd command_plt = Eigen::VectorXd(3);

    bool _stop;                        // Check for CTRL+C
    bool receiveing_msg = false;
    std::mutex _mutex;

    // std::unique_ptr<control::TrackControl> _controller;

  private:
    void set_default_position(const Eigen::VectorXd& jointPose){
        Eigen::VectorXd gain_i = Eigen::VectorXd(7);
        gain_i << 30, 300, 75, 150, 30, 40, 5;
        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(gain_i.size(), gain_i.size());
        A.diagonal() = gain_i;

        Eigen::VectorXd _erj = jointPose - _feedback.jnt_position;
        for (size_t i = 0; i < _erj.size(); i++)
        {
            if(abs(_erj[i]) > .5){
                _erj[i] *= 0.5/abs(_erj[i]); 
            }
        }
        
        command_trq = A * _erj - 0.2 * A * _feedback.jnt_velocity;

    }
    void updateRobotStates(const sensor_msgs::JointState::ConstPtr &msg, int k){
       for (int i = 0; i < No_JOINTS; i++){
            _feedback.jnt_position[i] = (double)msg->position[i];
            _feedback.jnt_velocity[i] = (double)msg->velocity[i];
            _feedback.jnt_torque[i]   = (double)msg->effort[i];
        }
    }
    
    void updateTorqueCommand(const std_msgs::Float64MultiArray::ConstPtr &msg, int k){
       for (int i = 0; i < No_JOINTS; i++){
            command_trq[i] = (double)msg->data[i];
        }
        receiveing_msg = true;
    }
    
    void updatePlotVariable(const std_msgs::Float64MultiArray::ConstPtr &msg, int k){
       for (int i = 0; i < command_plt.size(); i++){
            command_plt[i] = (double)msg->data[i];
        }
    }
    // void updateBioTac(const biotac_sensors::BioTacHand &msg);
    void publishCommandTorque(const Eigen::VectorXd& cmdTrq){
        std_msgs::Float64MultiArray _cmd_jnt_torque;
        _cmd_jnt_torque.data.resize(No_JOINTS);

        if (cmdTrq.size() == No_JOINTS){
            for(int i = 0; i < No_JOINTS; i++)
                _cmd_jnt_torque.data[i] = cmdTrq[i];
            _TrqCmdPublisher.publish(_cmd_jnt_torque);
        }
    }
    void publishPlotVariable(const Eigen::VectorXd& pltVar){
        std_msgs::Float64MultiArray _plotVar;
        _plotVar.data.resize(pltVar.size());
        for (size_t i = 0; i < pltVar.size(); i++)
            _plotVar.data[i] = pltVar[i];
        _plotPublisher.publish(_plotVar);
    }
    //TODO clean the optitrack

    // void updateOptitrack(const geometry_msgs::PoseStamped::ConstPtr& msg, int k){
    // }
    // uint16_t checkTrackedMarker(float a, float b){
    // }
};

//****************************************************
//****************************************************
int main (int argc, char **argv)
{
    float frequency = 200.0f;
    ros::init(argc,argv, "iiwa_track");
    ros::NodeHandle n;

    Options options;
    while(!n.getParam("options/filter_gain", options.filter_gain)){ROS_INFO("waiting ....");};


    std::unique_ptr<IiwaRosMaster> IiwaTrack = std::make_unique<IiwaRosMaster>(n,frequency,options);

    if (!IiwaTrack->init()){
        return -1;
    }else{
        IiwaTrack->run();
    }
    return 0;
}