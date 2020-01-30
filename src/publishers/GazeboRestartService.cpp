/*
MIT License

Copyright (c) 2020 Group of Electronic Technology and Communications. University of A Coruna.

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
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include "ros/console.h"

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include <gazebo/gazebo_client.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <std_srvs/Empty.h>

#include <random>

class GazeboRestartService
{
public:
  GazeboRestartService();

private:
  bool doRestart(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  ros::NodeHandle mNodeHandle;

  ros::ServiceServer mThisService;

  ros::ServiceClient mAutoGuideSC;
  ros::ServiceClient mGazeboResetSimulationSC;
  ros::ServiceClient mGazeboSetModelStateSC;
  ros::ServiceClient mGazeboGetModelStateSC;
  ros::ServiceClient mGazeboPauseSC;
  ros::ServiceClient mGazeboUnpauseSC;
  ros::ServiceClient mGazeboResetWorldSC;

  double mMinRandomX, mMaxRandomX, mMinRandomY, mMaxRandomY, mMinRandomZ, mMaxRandomZ;
  std::uniform_real_distribution<double> unifX;
  std::uniform_real_distribution<double> unifY;
  std::uniform_real_distribution<double> unifZ;
  std::default_random_engine re;

};

GazeboRestartService::GazeboRestartService():
  mNodeHandle("~")
{

  mNodeHandle.param("min_random_x", mMinRandomX, 0.0);
  mNodeHandle.param("max_random_x", mMaxRandomX, 0.0);
  mNodeHandle.param("min_random_y", mMinRandomY, 0.0);
  mNodeHandle.param("max_random_y", mMaxRandomY, 0.0);
  mNodeHandle.param("min_random_z", mMinRandomZ, 0.0);
  mNodeHandle.param("max_random_z", mMaxRandomZ, 0.0);

  ROS_INFO("max_random_x %f \n",mMaxRandomX );
  ROS_INFO("max_random_y %f \n",mMaxRandomY );
  ROS_INFO("max_random_z %f \n",mMaxRandomZ );

  unifX=std::uniform_real_distribution<double>(mMinRandomX,mMaxRandomX);
  unifY=std::uniform_real_distribution<double>(mMinRandomY,mMaxRandomY);
  unifZ=std::uniform_real_distribution<double>(mMinRandomZ,mMaxRandomZ);

  mThisService = mNodeHandle.advertiseService("gazebo_restart", &GazeboRestartService::doRestart, this);

  mGazeboResetSimulationSC = mNodeHandle.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
    mGazeboResetWorldSC = mNodeHandle.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
  mGazeboSetModelStateSC = mNodeHandle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  mGazeboPauseSC = mNodeHandle.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  mGazeboUnpauseSC = mNodeHandle.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  mGazeboGetModelStateSC = mNodeHandle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
 
 ros::spin();

}

bool GazeboRestartService::doRestart(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){

  ROS_INFO("GazeboRestartService doRestart");

  std_srvs::Empty srvEmpty;
  
  unifX.reset();
  unifY.reset();
  unifZ.reset();

  //Pause
  
  mGazeboPauseSC.call(srvEmpty);

  //ResetSimulation
  mGazeboResetSimulationSC.call(srvEmpty);

    //ResetSimulation
  mGazeboResetWorldSC.call(srvEmpty);


  //Change magnetic interference position

  //TODO: get number of magnetic interferences
  for (int i = 0; i < 10; ++i)
  {
    gazebo_msgs::GetModelState srvGetModelState;
    srvGetModelState.request.model_name = "interferer_clone_" + std::to_string(i);

    if (mGazeboGetModelStateSC.call(srvGetModelState)){
        if ((bool)srvGetModelState.response.success){
          gazebo_msgs::SetModelState srvSetModelState;
          gazebo_msgs::ModelState newModelState;
          geometry_msgs::Pose newPose;

          newModelState.model_name = "interferer_clone_" + std::to_string(i);
          ROS_INFO("GazeboRestartService moving %s", srvGetModelState.request.model_name.c_str());
          newModelState.twist = (geometry_msgs::Twist) srvGetModelState.response.twist;
          newPose = (geometry_msgs::Pose) srvGetModelState.response.pose;
          newPose.position.x = unifX(re);
          newPose.position.y = unifY(re);
          newPose.position.z = unifZ(re);

          ROS_INFO("newPose.position.x %f \n",newPose.position.x );
          ROS_INFO("newPose.position.y %f \n",newPose.position.y );
          ROS_INFO("newPose.position.z %f \n",newPose.position.z );
          newModelState.pose = newPose;

          srvSetModelState.request.model_state = newModelState;
          mGazeboSetModelStateSC.call(srvSetModelState);
        }
    }
  }


    //Unpause simulation
    mGazeboUnpauseSC.call(srvEmpty);

    return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_restart_service");
  GazeboRestartService gazebo_restart_service;

  ros::spin();
}
