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
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/assert.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "ros/console.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define STEP_SET 2
#define STEP_WALK 3
#define STEP_STOP 4


#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians *180.0/ M_PI)

class GazeboGuideRTLS
{
public:
  GazeboGuideRTLS();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish(double x, double y, double z);
  void currentStepEnded(void);
  void startNextStep(void);

  ros::NodeHandle mHandle, mNodeHandle;


  int mStartButton;
  bool mCanPublish, mCanStart;
  std::string routeFile;


  ros::Publisher mPosPublisher;
  ros::Subscriber mJoySubscriber;

  boost::mutex publish_mutex_;
  ros::Timer mTimer, mStepTimer;

  double mLinearFactor, mAngularFactor;


  typedef struct
  {
    int  type;
    double time;
    double x;
    double y;
    double z;
  } route_step_t;

  std::vector<route_step_t> mSteps;

  int mCurrentStep;

  double mInitTime, mEndTime;
  double mCurrentX, mCurrentY,  mCurrentZ, mCurrentAngle, mWalkSpeed, mRunSpeed;

};

GazeboGuideRTLS::GazeboGuideRTLS():
  mHandle("~"),
  mStartButton(5),
  mLinearFactor(0.5),
  mAngularFactor(2.0),
  mWalkSpeed(0.85),
  mRunSpeed(4.0)
{
  mHandle.param("start_button", mStartButton, mStartButton);
  mHandle.getParam("route_file", routeFile);

  mCanStart = false;
  mCurrentStep = 0;


  //Cargamos el xml con la ruta
  try {

    //************************************
    //Cargamos la configuracion de las suscripciones
    //************************************

    boost::property_tree::ptree routeTree;
    std::stringstream ssRoute;
    ssRoute << routeFile;
    boost::property_tree::read_xml(ssRoute, routeTree);

    BOOST_FOREACH(const boost::property_tree::ptree::value_type & v, routeTree.get_child("route")) {
      if (v.first.compare("set") == 0) {
        route_step_t routeStep;
        routeStep.type = STEP_SET;
        routeStep.x = v.second.get<double>("<xmlattr>.x", 1.0);
        routeStep.y = v.second.get<double>("<xmlattr>.y", 1.0);
        routeStep.z = v.second.get<double>("<xmlattr>.z", 1.0);
        mSteps.push_back(routeStep);
      } else if (v.first.compare("walk") == 0) {
        route_step_t routeStep;
        routeStep.type = STEP_WALK;
        routeStep.x = v.second.get<double>("<xmlattr>.x", 1.0);
        routeStep.y = v.second.get<double>("<xmlattr>.y", 1.0);
        routeStep.z = v.second.get<double>("<xmlattr>.z", 1.0);
        mSteps.push_back(routeStep);
      } else if (v.first.compare("stop") == 0) {
        route_step_t routeStep;
        routeStep.type = STEP_STOP;
        routeStep.time = v.second.get<double>("<xmlattr>.time", 1.0);
        mSteps.push_back(routeStep);
      }
    }

  } catch (boost::exception const &ex) {
    ROS_INFO("Route read error");
  }

  ROS_INFO("ROUTE LOADED");
  ROS_INFO("Num Steps: %d", (int) mSteps.size());

  mPosPublisher =  mNodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/gtec/kfpos", 1000);
  mJoySubscriber = mNodeHandle.subscribe<sensor_msgs::Joy>("joy", 10, &GazeboGuideRTLS::joyCallback, this);
}


void GazeboGuideRTLS::currentStepEnded(void) {
  ROS_INFO("Num Steps: %d", (int) mSteps.size());
  mCurrentStep += 1;
  if (mCurrentStep < mSteps.size()) {
    startNextStep();
  } 
}

void GazeboGuideRTLS::startNextStep(void) {

  route_step_t step = mSteps[mCurrentStep];

  if (step.type == STEP_SET) {
    //Solo ponemos la posicion inicial para no tener q suscribirnos al topic de posicion
    ROS_INFO("STEP SET");
    mCurrentX = step.x;
    mCurrentY = step.y;
    mCurrentZ = step.z;
    mStepTimer = mNodeHandle.createTimer(ros::Duration(0), boost::bind(&GazeboGuideRTLS::currentStepEnded, this), true);
    return;
  } else if (step.type == STEP_WALK) {
    double toDestX = step.x - mCurrentX;
    double toDestY = step.y - mCurrentY;
    double toDestZ = step.z - mCurrentZ;
    double distanceToMove = sqrt(pow(step.x - mCurrentX, 2) + pow(step.y - mCurrentY, 2)+ pow(step.z - mCurrentZ, 2));
    double time = distanceToMove / mWalkSpeed;
    mCurrentX = step.x;
    mCurrentY = step.y;
    mCurrentZ = step.z;
    //Publish new position
    publish(step.x, step.y, step.z);

    //Timer to next step
    mStepTimer = mNodeHandle.createTimer(ros::Duration(time), boost::bind(&GazeboGuideRTLS::currentStepEnded, this), true);
    return;
  } else if (step.type == STEP_STOP) {
    //Timer to next step
    mStepTimer = mNodeHandle.createTimer(ros::Duration(step.time), boost::bind(&GazeboGuideRTLS::currentStepEnded, this), true);
    return;
  }

}

void GazeboGuideRTLS::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  if (joy->buttons[mStartButton]) {
    if (!mCanStart) {
      ROS_INFO("Starting ROUTE -----------");
      mCanStart = true;
      startNextStep();
    }
  }
}

void GazeboGuideRTLS::publish(double x, double y, double z)
{
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;
  pose.pose.pose.position.z = z;
  pose.pose.pose.orientation.x = 0;
  pose.pose.pose.orientation.y = 0;
  pose.pose.pose.orientation.z = 0;
  pose.pose.pose.orientation.w = 0;
  pose.header.frame_id = "world";
  pose.header.stamp = ros::Time::now();

  mPosPublisher.publish(pose);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_guide_rtls");
  GazeboGuideRTLS gazebo_guide_rtls;

  ros::spin();
}
