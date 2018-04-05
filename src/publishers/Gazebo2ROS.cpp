/*
MIT License

Copyright (c) 2018 Group of Electronic Technology and Communications. University of A Coruña.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64.h>
#include "opticalFlow.pb.h"

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
//#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include <gazebo/gazebo_client.hh>
#include <gazebo/sensors/sensors.hh>


typedef const boost::shared_ptr<const opticalFlow_msgs::msgs::opticalFlow> OpticalFlowPtr;
typedef const boost::shared_ptr<const gazebo::msgs::IMU>ImuPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Magnetometer>MagPtr;


#include <iostream>

ros::Publisher pubPx4flow, pubErleImu, pubErleMagneticField, pubErleMagneticFieldInterfered;
bool airborne, hasLastPx4Imu= false;
int seq = 0;

gazebo::msgs::Vector3d lastPx4FlowAngularVelocity, lastPx4FlowLinearAcceleration;
gazebo::msgs::Quaternion lastPx4FlowOrientation;


void newPx4FlowMsg(OpticalFlowPtr& opticalFlow_message){
  mavros_msgs::OpticalFlowRad sensor_msg;

  //ROS_INFO("New Px4Flow MSG\n");

  double optflow_xgyro;
  double optflow_ygyro;
  double optflow_zgyro;


  sensor_msg.header.seq = seq;
  seq+=1;
  if (seq>=256){
    seq=0;
  }

  ros::Time currentTime = ros::Time::now();
  sensor_msg.header.stamp= currentTime;



  sensor_msg.integration_time_us = opticalFlow_message->integration_time_us();
  sensor_msg.integrated_x = -opticalFlow_message->integrated_x();
  sensor_msg.integrated_y = opticalFlow_message->integrated_y();

  sensor_msg.integrated_xgyro = 0;
  sensor_msg.integrated_ygyro = 0;
  sensor_msg.integrated_zgyro = 0;


  sensor_msg.temperature = opticalFlow_message->temperature();
  sensor_msg.quality = opticalFlow_message->quality();
  sensor_msg.time_delta_distance_us = opticalFlow_message->time_delta_distance_us();


  if (hasLastPx4Imu){
    //Rellenamos los valores que faltan con los ultimos recibidos de la imu
  optflow_xgyro = lastPx4FlowAngularVelocity.x();
  optflow_ygyro = lastPx4FlowAngularVelocity.y();
  optflow_zgyro = lastPx4FlowAngularVelocity.z();

  sensor_msg.integrated_xgyro = optflow_ygyro * opticalFlow_message->integration_time_us() / 1000000.0; //xy switched
  sensor_msg.integrated_ygyro = optflow_xgyro * opticalFlow_message->integration_time_us() / 1000000.0; //xy switched

  //TODO: En la siguiente linea usamos voluntariamente la coordenada x 
  sensor_msg.integrated_zgyro = -optflow_xgyro * opticalFlow_message->integration_time_us() / 1000000.0; //change direction
  
  //sensor_msg.integrated_zgyro = optflow_zgyro* opticalFlow_message->integration_time_us() / 1000000.0;
  }

  //La distancia en el ejemplo viene del lidar por separado
  sensor_msg.distance = 2.0;

  pubPx4flow.publish(sensor_msg);
}

void newPx4FlowImuMsg(ImuPtr& imu_message){

    lastPx4FlowAngularVelocity = imu_message->angular_velocity();
    lastPx4FlowLinearAcceleration = imu_message->linear_acceleration();
    lastPx4FlowOrientation = imu_message->orientation();
    hasLastPx4Imu = true;
}


void newErleImuMsg(ImuPtr& imu_message){
  sensor_msgs::Imu rosImuMsg;


  rosImuMsg.orientation.x = imu_message->orientation().x();
  rosImuMsg.orientation.y = imu_message->orientation().y();
  rosImuMsg.orientation.z = imu_message->orientation().z();
  rosImuMsg.orientation.w = imu_message->orientation().w();

  rosImuMsg.angular_velocity.x = imu_message->angular_velocity().x();
  rosImuMsg.angular_velocity.y = imu_message->angular_velocity().y();
  rosImuMsg.angular_velocity.z = imu_message->angular_velocity().z();

  rosImuMsg.linear_acceleration.x = imu_message->linear_acceleration().x();
  rosImuMsg.linear_acceleration.y = imu_message->linear_acceleration().y();
  rosImuMsg.linear_acceleration.z = imu_message->linear_acceleration().z();

//TODO revisar las covarianzas
//float64[9] orientation_covariance
//float64[9] angular_velocity_covariance
//float64[9] linear_acceleration_covariance

  pubErleImu.publish(rosImuMsg);

}



void newErleMagMsg(MagPtr& mag_message){
  sensor_msgs::MagneticField rosMagMsg;
  std_msgs::Float64 angleMsg;

  rosMagMsg.magnetic_field.x = mag_message->field_tesla().x();
  rosMagMsg.magnetic_field.y = mag_message->field_tesla().y();
  rosMagMsg.magnetic_field.z = mag_message->field_tesla().z();



  double angle = atan2(rosMagMsg.magnetic_field.y, rosMagMsg.magnetic_field.x);
  angleMsg.data = angle*180.0/M_PI;
//TODO revisar las covarianzas


  pubErleMagneticField.publish(angleMsg);

}

void newErleMagMsgInterfered(MagPtr& mag_message){
  sensor_msgs::MagneticField rosMagMsg;
  std_msgs::Float64 angleMsg;

  rosMagMsg.magnetic_field.x = mag_message->field_tesla().x();
  rosMagMsg.magnetic_field.y = mag_message->field_tesla().y();
  rosMagMsg.magnetic_field.z = mag_message->field_tesla().z();



  double angle = atan2(rosMagMsg.magnetic_field.y, rosMagMsg.magnetic_field.x);
  angleMsg.data = angle*180.0/M_PI;
//TODO revisar las covarianzas


  pubErleMagneticFieldInterfered.publish(angleMsg);

}



int main(int _argc, char **_argv){
  // Set variables
  airborne = false;

  // Load Gazebo & ROS
  gazebo::client::setup(_argc, _argv);
  ros::init(_argc, _argv, "gazebo2ros");

  // Create Gazebo node and init
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Create ROS node and init
  ros::NodeHandle n("~");

  //Definicion de publishers de ROS
  pubPx4flow = n.advertise<mavros_msgs::OpticalFlowRad>("/gtec/gazebo/px4flow", 1000);
  pubErleImu = n.advertise<sensor_msgs::Imu>("/gtec/gazebo/erle/imu", 1000);
  pubErleMagneticField = n.advertise<std_msgs::Float64>("/gtec/gazebo/erle/mag", 1000);
  pubErleMagneticFieldInterfered = n.advertise<std_msgs::Float64>("/gtec/gazebo/erle/maginterfered", 1000);


  // Definicion de suscripciones de topics de gazebo
  gazebo::transport::SubscriberPtr subPx4Flow = node->Subscribe("/gazebo/default/forklift_full/camera/link/opticalFlow", newPx4FlowMsg);
  gazebo::transport::SubscriberPtr subPx4FlowImu = node->Subscribe("/gazebo/default/forklift_full/camera/link/px4_imu/imu", newPx4FlowImuMsg);
  gazebo::transport::SubscriberPtr subErleImu = node->Subscribe("/gazebo/default/forklift_full/forkliftSensors/link_erle/erle_imu/imu", newErleImuMsg);
  gazebo::transport::SubscriberPtr subErleMag = node->Subscribe("/gazebo/default/forklift_full/forkliftSensors/link_erle/erle_magnetometer", newErleMagMsg);
  gazebo::transport::SubscriberPtr subErleMagInterfered = node->Subscribe("/gazebo/default/forklift_full/forkliftSensors/link_erle/erle_magnetometer/interfered", newErleMagMsgInterfered);



  ros::spin();

  node->Fini();
  gazebo::client::shutdown();

  
  exit(1);

/*  // Busy wait loop...replace with your own code as needed.
  while (true) {
    gazebo::common::Time::MSleep(20);
    
    // Spin ROS (needed for publisher)
    ros::spinOnce();
  }


  // Mayke sure to shut everything down.
  gazebo::client::shutdown();*/

}
