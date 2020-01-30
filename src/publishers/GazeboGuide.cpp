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
#include <sensor_msgs/Joy.h>
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
#include "ros/console.h"

#define STEP_MOVE 0
#define STEP_SPIN 1
#define STEP_SET 2
#define STEP_MOVE_TO 3
#define STEP_STOP 4


#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians *180.0/ M_PI)

class GazeboGuide
{
public:
  GazeboGuide();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();
  void currentStepEnded(void);
  void startNextStep(void);
  void stop(void);

  ros::NodeHandle mHandle, mNodeHandle;

  int mStartButton;
  bool mCanPublish, mCanStart;
  std::string routeFile;


  ros::Publisher mVelPublisher;
  ros::Subscriber mJoySubscriber;

  geometry_msgs::Twist mLastPublished;
  boost::mutex publish_mutex_;
  ros::Timer mTimer, mStepTimer;

  double mLinearFactor, mAngularFactor;


  typedef struct
{
    int  type;
    double time;
    double distance;
    double angle;
    double x;
    double y;
} route_step_t;

  std::vector<route_step_t> mSteps;

  int mCurrentStep;

  double mInitTime, mEndTime;
  double mCurrentX, mCurrentY, mCurrentAngle;

};

GazeboGuide::GazeboGuide():
  mHandle("~"),
  mStartButton(5),
  mLinearFactor(0.5),
  mAngularFactor(2.0)
{
  mHandle.param("start_button", mStartButton, mStartButton);
  mHandle.getParam("route_file", routeFile);

  mCanPublish = false;
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
      if (v.first.compare("move") == 0) {
        route_step_t routeStep;
        routeStep.type = STEP_MOVE;
        routeStep.distance = v.second.get<double>("<xmlattr>.distance", 1.0);
        routeStep.time = v.second.get<double>("<xmlattr>.time", 1.0);
        mSteps.push_back(routeStep);
      } else if (v.first.compare("spin") == 0) {
        route_step_t routeStep;
        routeStep.type = STEP_SPIN;
        routeStep.angle = v.second.get<double>("<xmlattr>.angle", 1.0);
        routeStep.time = v.second.get<double>("<xmlattr>.time", 1.0);
        mSteps.push_back(routeStep);
      }else if (v.first.compare("set") == 0) {
        route_step_t routeStep;
        routeStep.type = STEP_SET;
        routeStep.angle = v.second.get<double>("<xmlattr>.angle", 1.0);
        routeStep.x = v.second.get<double>("<xmlattr>.x", 1.0);
        routeStep.y = v.second.get<double>("<xmlattr>.y", 1.0);
        mSteps.push_back(routeStep);
      }else if (v.first.compare("moveto") == 0) {
        route_step_t routeStep;
        routeStep.type = STEP_MOVE_TO;
        routeStep.time = v.second.get<double>("<xmlattr>.time", 1.0);
        routeStep.x = v.second.get<double>("<xmlattr>.x", 1.0);
        routeStep.y = v.second.get<double>("<xmlattr>.y", 1.0);
        mSteps.push_back(routeStep);
      }else if (v.first.compare("stop") == 0) {
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



  mVelPublisher = mHandle.advertise<geometry_msgs::Twist>("guide_cmd_vel", 1, true);
  mJoySubscriber = mNodeHandle.subscribe<sensor_msgs::Joy>("joy", 10, &GazeboGuide::joyCallback, this);

  mTimer = mNodeHandle.createTimer(ros::Duration(0.1), boost::bind(&GazeboGuide::publish, this));
}


void GazeboGuide::currentStepEnded(void){
  mEndTime =ros::Time::now().toSec();
  double seconds = mEndTime-mInitTime;
  ROS_INFO("Num Steps: %d", (int) mSteps.size());
  mCurrentStep+=1;

  if (mCurrentStep<mSteps.size()){
      startNextStep();
  } else {
    //Llegamos al final
    stop();
  }
}

void GazeboGuide::stop(void){
  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.angular.z=0;
  mLastPublished = vel;
  mCanPublish = true;
}

void GazeboGuide::startNextStep(void){

  route_step_t step = mSteps[mCurrentStep];
  geometry_msgs::Twist vel;

  //Calculamos la velocidad
  vel.linear.x = 0;
  vel.angular.z=0;

  if (step.type==STEP_SET){
    //Solo ponemos la posicion inicial para no tener q suscribirnos al topic de posicion
    ROS_INFO("STEP SET");
    mCurrentAngle = step.angle;
    mCurrentX = step.x;
    mCurrentY = step.y;
    mStepTimer = mNodeHandle.createTimer(ros::Duration(0), boost::bind(&GazeboGuide::currentStepEnded, this), true);
    return;   
  } else if (step.type==STEP_MOVE){
    ROS_INFO("STEP MOVE  distance: %f, time %f from (%f, %f)",step.distance, step.time, mCurrentX, mCurrentY);


    vel.linear.x = mLinearFactor*(step.distance/step.time);
    mCurrentX = mCurrentX + cos(degreesToRadians(mCurrentAngle))*step.distance;
    mCurrentY = mCurrentY + sin(degreesToRadians(mCurrentAngle))*step.distance;

    ROS_INFO("STEP MOVE  vel linear x: %f",vel.linear.x);

  } else if (step.type==STEP_SPIN){
    vel.angular.z = mAngularFactor * degreesToRadians(step.angle)/step.time;
    mCurrentAngle = fmod(mCurrentAngle + step.angle, 360.0);
    ROS_INFO("STEP SPIN  AngleToSpin: %f, time: %f, CurrentAngleAfter: %f", step.angle, step.time,mCurrentAngle);

  } else if (step.type==STEP_MOVE_TO){
    //Primero hacemos un spin y despues un move

    double toDestX = step.x - mCurrentX;
    double toDestY = step.y - mCurrentY;

    double distanceToMove = sqrt(pow(step.x - mCurrentX, 2) + pow(step.y - mCurrentY, 2));
    double distanceNorm = sqrt(pow(toDestX, 2) + pow(toDestY, 2));
    double xNorm = toDestX/distanceNorm;
    double yNorm = toDestY/distanceNorm;
    double toAngle = radiansToDegrees(acos(xNorm));

    if (toDestY<0){
      toAngle = 360 - toAngle;
    }

    toAngle = fmod(toAngle, 360.0);

    double angleToSpin = toAngle - mCurrentAngle;

    if (angleToSpin<-180){
      angleToSpin+=360;
    }


    //ROS_INFO("distanceNorm: %f",distanceNorm);
    //ROS_INFO("xNorm: %f",xNorm);
    //ROS_INFO("distanceToMove: %f",distanceToMove);
    //ROS_INFO("acos(xNorm/distanceNorm): %f",acos(xNorm/distanceNorm));
    ROS_INFO("mCurrentStep: %d",mCurrentStep);
    ROS_INFO("toAngle: %f",toAngle);
    ROS_INFO("angleToSpin: %f",angleToSpin);

    route_step_t routeStepSpin, routeStepMove;
    routeStepSpin.type = STEP_SPIN;
    routeStepSpin.angle = angleToSpin;
    routeStepSpin.time = fabs((angleToSpin/360.0)*16); //TODO poner el 10 en algun sitio

    routeStepMove.type = STEP_MOVE;
    routeStepMove.time = step.time; 
    routeStepMove.distance = distanceToMove;

    std::vector<route_step_t>::iterator it = mSteps.begin();
    mSteps.insert(it+mCurrentStep+1, {routeStepSpin,routeStepMove});
    //it = mSteps.begin();
    //mSteps.insert(it+mCurrentStep+2, routeStepMove);
    mStepTimer = mNodeHandle.createTimer(ros::Duration(0), boost::bind(&GazeboGuide::currentStepEnded, this), true);
    return; 
  } else if (step.type == STEP_STOP){
    vel.linear.x = 0;
    vel.angular.z=0;
  }

  mLastPublished = vel;
  //Ponemos un timer con la duracion del step
  mStepTimer = mNodeHandle.createTimer(ros::Duration(step.time), boost::bind(&GazeboGuide::currentStepEnded, this), true);
  mInitTime =ros::Time::now().toSec();
  mCanPublish = true;
  

}

void GazeboGuide::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  geometry_msgs::Twist vel;

  if (joy->buttons[mStartButton]){
    if (!mCanStart){
      ROS_INFO("Starting ROUTE -----------");
      mCanStart = true;
      startNextStep();
    }  
  }
}

void GazeboGuide::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
  if (mCanPublish)
  {
    mVelPublisher.publish(mLastPublished);
    mCanPublish=false;

  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_guide");
  GazeboGuide gazebo_guide;

  ros::spin();
}
