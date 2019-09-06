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
#include <std_srvs/Empty.h>
#include <cstdlib>
#include <errno.h>
#include <unistd.h>
#include <sys/wait.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

#define STEP_MOVE 0
#define STEP_SPIN 1
#define STEP_SET 2
#define STEP_MOVE_TO 3
#define STEP_STOP 4
#define STEP_MOVE_TO_AT 5
#define STEP_MOVE_AT 6
#define STEP_SPIN_AT 7

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians *180.0/ M_PI)

class GazeboAutoGuide
{
public:
  GazeboAutoGuide();
  void newPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& aPose);

private:
   void publish();
  void currentStepEnded(void);
  void startNextStep(void);
  void stop(void);
  void startGuide(void);
  void doRestart(void);


  ros::NodeHandle mHandle, mNodeHandle;

  geometry_msgs::PoseWithCovarianceStamped currentPose;

  //int mStartButton;
  double mStartTimerDuration;
  bool mCanPublish, mCanStart, mRestartOnFinish, mRecordLog;
  std::string routeFile;


  ros::Publisher mVelPublisher;
  ros::Subscriber mJoySubscriber;

  geometry_msgs::Twist mLastPublished;
  boost::mutex publish_mutex_;
  ros::Timer mTimer, mStepTimer, mStartTimer, mDoRestart;

  double mLinearFactor, mAngularFactor;

  bool mHasAngleGoal;
  double mAngleGoal;
  int mNumFullSpinsGoal;
  int mCurrentNumSpins;
  bool mAngularVelocityNegative;
  double mCurrentAngleDiff;
  double mCurrentYaw;
  bool mIsSpining = false;

  bool mHasPositionGoal;
  double mPositionGoalX;
  double mPositionGoalY;
  bool mIsMoving = false;
  double mCurrentDistance;
  double mCurrentPoseX;
  double mCurrentPoseY;


  typedef struct
{
    int  type;
    double time;
    double distance;
    double angle;
    double x;
    double y;
    double linearVelocity;
    double angularVelocity;
} route_step_t;

  std::vector<route_step_t> mSteps;
  std::vector<route_step_t> mBaseSteps;

  int mCurrentStep, mMaxReps, mCurrentRep, mSecondsToRecord;

  double mInitTime, mEndTime;
  double mCurrentX, mCurrentY, mCurrentAngle, mAngleThreshold;
  ros::ServiceClient mGazeboRestartService;
  std::string mDistro, mWorkspace;

  double minSpin = 2;

  pid_t rosbagPid;

};

GazeboAutoGuide::GazeboAutoGuide():
  mHandle("~"),
  mStartTimerDuration(5.0),
  mLinearFactor(1.0),
  mAngularFactor(1.0),
  mRestartOnFinish(true),
  mMaxReps(10),
  mAngleThreshold(0.01),
  mDistro("/opt/ros/melodic/bin/rosbag"),
  mWorkspace("/home/valba/catkin_ws")
{
  int mustRestartOnFinish, mustRecord;
  mHandle.param("restart_on_finish", mustRestartOnFinish, 1);
  mHandle.param("max_reps", mMaxReps, 10);
  mHandle.param("start_time", mStartTimerDuration, mStartTimerDuration);
  mHandle.param("record_log", mustRecord, 1);
  mHandle.param("seconds_to_record", mSecondsToRecord, 30);
  mHandle.param("max_reps", mMaxReps, 10);
  mHandle.getParam("route_file", routeFile);
mHandle.param("angle_threshold", mAngleThreshold, 0.01);
  mHandle.getParam("rosbagPath", mDistro);
  mHandle.getParam("worspace", mWorkspace);


  mRestartOnFinish = mustRestartOnFinish==1;
  mRecordLog = mustRecord==1;

  if (mRestartOnFinish){
    mGazeboRestartService = mNodeHandle.serviceClient<std_srvs::Empty>("/gazebo_restart_service/gazebo_restart");
  }

  mCanPublish = false;
  mCanStart = false;
  mCurrentStep = 0;
  mCurrentRep = 0;
  mHasAngleGoal = 0;
  mHasPositionGoal = 0;

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
        mBaseSteps.push_back(routeStep);
      } else if (v.first.compare("spin") == 0) {
        route_step_t routeStep;
        routeStep.type = STEP_SPIN;
        routeStep.angle = v.second.get<double>("<xmlattr>.angle", 1.0);
        routeStep.time = v.second.get<double>("<xmlattr>.time", 1.0);
        mBaseSteps.push_back(routeStep);
      }else if (v.first.compare("set") == 0) {
        route_step_t routeStep;
        routeStep.type = STEP_SET;
        routeStep.angle = v.second.get<double>("<xmlattr>.angle", 1.0);
        routeStep.x = v.second.get<double>("<xmlattr>.x", 1.0);
        routeStep.y = v.second.get<double>("<xmlattr>.y", 1.0);
        mBaseSteps.push_back(routeStep);
      }else if (v.first.compare("moveto") == 0) {
        route_step_t routeStep;
        routeStep.type = STEP_MOVE_TO;
        routeStep.time = v.second.get<double>("<xmlattr>.time", 1.0);
        routeStep.x = v.second.get<double>("<xmlattr>.x", 1.0);
        routeStep.y = v.second.get<double>("<xmlattr>.y", 1.0);
        mBaseSteps.push_back(routeStep);
      }else if (v.first.compare("stop") == 0) {
        route_step_t routeStep;
        routeStep.type = STEP_STOP;
        routeStep.time = v.second.get<double>("<xmlattr>.time", 1.0);
        mBaseSteps.push_back(routeStep);
      }else if (v.first.compare("movetoat") == 0) {
        route_step_t routeStep;
        routeStep.type = STEP_MOVE_TO_AT;
        routeStep.linearVelocity = v.second.get<double>("<xmlattr>.linear_vel", 1.0);
        routeStep.angularVelocity = v.second.get<double>("<xmlattr>.angular_vel", 1.0);
        routeStep.x = v.second.get<double>("<xmlattr>.x", 1.0);
        routeStep.y = v.second.get<double>("<xmlattr>.y", 1.0);
        mBaseSteps.push_back(routeStep);
      } else if (v.first.compare("spinat") == 0) {
        route_step_t routeStep;
        routeStep.type = STEP_SPIN_AT;
        routeStep.angularVelocity = v.second.get<double>("<xmlattr>.angular_vel", 1.0);
        routeStep.angle = v.second.get<double>("<xmlattr>.angle", 1.0);
        mBaseSteps.push_back(routeStep);
      } else if (v.first.compare("moveat") == 0) {
        route_step_t routeStep;
        routeStep.type = STEP_MOVE_AT;
        routeStep.linearVelocity = v.second.get<double>("<xmlattr>.linear_vel", 1.0);
        routeStep.distance = v.second.get<double>("<xmlattr>.distance", 1.0);
        mBaseSteps.push_back(routeStep);
      }
    }

  } catch (boost::exception const &ex) {
    ROS_INFO("Route read error");
  }

  ROS_INFO("ROUTE LOADED");
  ROS_INFO("Num Steps: %d", (int) mBaseSteps.size());


  mVelPublisher = mHandle.advertise<geometry_msgs::Twist>("guide_cmd_vel", 1, true);
  //mJoySubscriber = mNodeHandle.subscribe<sensor_msgs::Joy>("joy", 10, &GazeboAutoGuide::joyCallback, this);

  mStartTimer = mNodeHandle.createTimer(ros::Duration(mStartTimerDuration), boost::bind(&GazeboAutoGuide::startGuide, this), true);
  mTimer = mNodeHandle.createTimer(ros::Duration(0.1), boost::bind(&GazeboAutoGuide::publish, this));

}


void GazeboAutoGuide::currentStepEnded(void){
  mEndTime =ros::Time::now().toSec();
  double seconds = mEndTime-mInitTime;
  ROS_INFO("Current Step Ended: %d", mCurrentStep);
  mCurrentStep+=1;

  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.angular.z=0;
  mLastPublished = vel;

  if (mCurrentStep<mSteps.size()){
      startNextStep();
  } else {
    //Llegamos al final
    stop();
  }
}

void GazeboAutoGuide::doRestart(void){
 
    std_srvs::Empty srvEmpty;
    mGazeboRestartService.call(srvEmpty);

    //Restart Guide

    mCanPublish = false;
    mCanStart = false;
    mCurrentStep = 0;
    mDoRestart = mNodeHandle.createTimer(ros::Duration(1), boost::bind(&GazeboAutoGuide::startGuide, this), true);
 
}

void GazeboAutoGuide::stop(void){
  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.angular.z=0;
  mLastPublished = vel;
  mCanPublish = true;
  mHasAngleGoal = false;
  mHasPositionGoal  = false;

  if (mRecordLog && rosbagPid>0){
    ROS_INFO("Killing rosbag with pid : %d",rosbagPid);

    kill(rosbagPid,SIGINT);

    bool died = false;
    for (int loop; !died && loop < 10; ++loop)
    {
        int status;
        sleep(1);
        if (waitpid(rosbagPid, &status, WNOHANG) == rosbagPid) died = true;
    }

    if (!died) {
      kill(rosbagPid, SIGKILL);
      ROS_INFO("Rosbag killed with SIGKILL");
    } else {
      pid_t  pkillPid = fork();
      if( pkillPid == 0 )
      {
         std::string newDir = mWorkspace;
         chdir(newDir.c_str());
          // we're the child process
          char *argv[] = { (char *) "/usr/bin/pkill",(char *) "-2", (char *) "record", (char*)0 };
          int rc = execv(argv[0], &argv[0]);

          if(rc==-1) {
            ROS_INFO("Pkill Execv error %s \n", strerror(errno));
            }
          exit(1);
          // execve only returns if there was an error
          // check 'errno' and handle it here
      }

      ROS_INFO("Rosbag killed with SIGINT");
    }

  }

  if (mRestartOnFinish && mCurrentRep<mMaxReps){
    mCurrentRep+=1;
    //Wait some time to restart world
    mDoRestart = mNodeHandle.createTimer(ros::Duration(1.5), boost::bind(&GazeboAutoGuide::doRestart, this), true);
  }
}

void GazeboAutoGuide::startNextStep(void){

  route_step_t step = mSteps[mCurrentStep];
  geometry_msgs::Twist vel;

  //Calculamos la velocidad
  vel.linear.x = 0;
  vel.angular.z=0;
  mHasAngleGoal = false;
  mHasPositionGoal = false;
  mCurrentAngleDiff = 0;

 ROS_INFO("STARTING STEP: %d", mCurrentStep);

  if (step.type==STEP_SET){
    //Solo ponemos la posicion inicial para no tener q suscribirnos al topic de posicion
    ROS_INFO("STEP SET");
    mCurrentAngle = step.angle;
    mCurrentX = step.x;
    mCurrentY = step.y;
    mStepTimer = mNodeHandle.createTimer(ros::Duration(0), boost::bind(&GazeboAutoGuide::currentStepEnded, this), true);
    return;   
  } else if (step.type==STEP_MOVE){
    ROS_INFO("STEP MOVE  distance: %f, time %f from (%f, %f)",step.distance, step.time, mCurrentX, mCurrentY);

    vel.angular.z = 0;
    vel.linear.x = mLinearFactor*(step.distance/step.time);
    mCurrentX = mCurrentX + cos(degreesToRadians(mCurrentAngle))*step.distance;
    mCurrentY = mCurrentY + sin(degreesToRadians(mCurrentAngle))*step.distance;

    ROS_INFO("STEP MOVE  vel linear x: %f",vel.linear.x);

  } else if (step.type==STEP_SPIN){
    vel.angular.z = mAngularFactor * degreesToRadians(step.angle)/step.time;
    mCurrentAngle = fmod(mCurrentAngle + step.angle, 360.0);
    ROS_INFO("STEP SPIN  AngleToSpin: %f, time: %f, CurrentAngleAfter: %f", step.angle, step.time,mCurrentAngle);

  } else if (step.type==STEP_SPIN_AT){
    mHasAngleGoal = true;
    

    double finalAngle = step.angle + radiansToDegrees(mCurrentYaw);


    mCurrentNumSpins = 0;
    mNumFullSpinsGoal = (int) floor(fabs(finalAngle)/360.0);
    mAngleGoal = fmod(fabs(finalAngle),360.0);
    mAngularVelocityNegative = false;
    vel.linear.x = 0;
    vel.angular.z = step.angularVelocity;

    if (step.angle<0){
      mAngularVelocityNegative = true;
      //mAngleGoal = 360.0 - mAngleGoal;
      vel.angular.z = -1*step.angularVelocity;
    }

    mAngleGoal= degreesToRadians(mAngleGoal);


    mCurrentAngleDiff = fabs(mCurrentYaw - mAngleGoal);

    if (finalAngle<0){
      mCurrentAngleDiff = fabs(2*M_PI - mAngleGoal) + mCurrentYaw;
    }
    
    mInitTime =ros::Time::now().toSec();
    mCanPublish = true;
    mLastPublished = vel;
    mIsSpining = false;
    ROS_INFO("CurrentYaw: %f CurrentPos: (%f,%f)",mCurrentYaw,mCurrentPoseX,mCurrentPoseY);
    ROS_INFO("STEP SPIN_AT  GoalAngle: %f, angularVelocity: %f, mCurrentAngleDiff: %f, mCurrentYaw: %f", mAngleGoal, vel.angular.z,mCurrentAngleDiff,mCurrentYaw);
    return;
  }else if (step.type==STEP_MOVE_AT){
    vel.angular.z =0;
    mHasPositionGoal = true;

    mPositionGoalX = mCurrentPoseX + cos(mCurrentYaw)*step.distance;
    mPositionGoalY = mCurrentPoseY + sin(mCurrentYaw)*step.distance;
    mCurrentDistance = step.distance;
    vel.linear.x = step.linearVelocity;
    mInitTime =ros::Time::now().toSec();
    mCanPublish = true;
    mLastPublished = vel;
    mIsMoving = false;
    ROS_INFO("CurrentYaw: %f CurrentPos: (%f,%f)",mCurrentYaw,mCurrentPoseX,mCurrentPoseY);
    ROS_INFO("STEP MOVE_AT  Goal: (%f,%f) , linearVelocity: %f", mPositionGoalX, mPositionGoalY, vel.linear.x);
    return;
  }else if (step.type==STEP_MOVE_TO || step.type==STEP_MOVE_TO_AT){
    //Primero hacemos un spin y despues un move

    double toDestX = step.x - mCurrentPoseX;
    double toDestY = step.y - mCurrentPoseY;

    double distanceToMove = sqrt(pow(step.x - mCurrentPoseX, 2) + pow(step.y - mCurrentPoseY, 2));
    double distanceNorm = sqrt(pow(toDestX, 2) + pow(toDestY, 2));
    double xNorm = toDestX/distanceNorm;
    double yNorm = toDestY/distanceNorm;
    double toAngle = radiansToDegrees(acos(xNorm));




    if (toDestY<0){
      toAngle = 360 - toAngle;
    }

    toAngle = fmod(toAngle, 360.0);

    //We find the shortest angle to spin
    double posAng,posNeg;
    if (toAngle>=radiansToDegrees(mCurrentYaw)){
      posAng = toAngle - radiansToDegrees(mCurrentYaw);
      posNeg = radiansToDegrees(mCurrentYaw) + (360.0 - toAngle);
  } else  {
      posAng = toAngle + (360.0 - radiansToDegrees(mCurrentYaw));
      posNeg = radiansToDegrees(mCurrentYaw) - toAngle;
  }




    double angleToSpin = posAng;
    if (posNeg<posAng){
        angleToSpin = -1*posNeg;
    }

    ROS_INFO("mCurrentStep: %d",mCurrentStep);
    ROS_INFO("toAngle: %f",toAngle);
    ROS_INFO("angleToSpin: %f",angleToSpin);
    ROS_INFO("posAng: %f",posAng);
    ROS_INFO("posNeg: %f",posNeg);
    ROS_INFO("GoalX: %f",step.x);
    ROS_INFO("GoalY: %f",step.y);

    route_step_t routeStepSpin, routeStepMove, routeStepStop;

    routeStepStop.type = STEP_STOP;
    routeStepSpin.time = 0.5;
    std::vector<route_step_t>::iterator it = mSteps.begin();

    if (step.type==STEP_MOVE_TO) {
      routeStepSpin.type = STEP_SPIN;
      routeStepSpin.angle = angleToSpin;
      routeStepSpin.time = fabs((angleToSpin/360.0)*16);

      routeStepMove.type = STEP_MOVE;
      routeStepMove.time = step.time; 
      routeStepMove.distance = distanceToMove;

      mSteps.insert(it+mCurrentStep+1, {routeStepSpin,routeStepMove});

    } else if (step.type==STEP_MOVE_TO_AT) {
      bool needSpin = false;
      if (fabs(angleToSpin)>minSpin){

        routeStepSpin.type = STEP_SPIN_AT;
        routeStepSpin.angle = angleToSpin;
        routeStepSpin.angularVelocity = step.angularVelocity;
        needSpin = true;

      }

      routeStepMove.type = STEP_MOVE_AT;
      routeStepMove.distance = distanceToMove;
      routeStepMove.linearVelocity = step.linearVelocity;

      if (needSpin){
        mSteps.insert(it+mCurrentStep+1, {routeStepSpin,routeStepStop,routeStepMove});
      } else {
        mSteps.insert(it+mCurrentStep+1, {routeStepMove});
      }
      
    }
    
    //it = mSteps.begin();
    //mSteps.insert(it+mCurrentStep+2, routeStepMove);
    mStepTimer = mNodeHandle.createTimer(ros::Duration(0), boost::bind(&GazeboAutoGuide::currentStepEnded, this), true);
    return; 
  } else if (step.type == STEP_STOP){
    vel.linear.x = 0;
    vel.angular.z=0;
  }

  mLastPublished = vel;
  //Ponemos un timer con la duracion del step
  mStepTimer = mNodeHandle.createTimer(ros::Duration(step.time), boost::bind(&GazeboAutoGuide::currentStepEnded, this), true);
  mInitTime =ros::Time::now().toSec();
  mCanPublish = true;

}

void GazeboAutoGuide::startGuide(void)
{ 

  if (!mCanStart){
    ROS_INFO("Starting ROUTE -----------");

    
    if (mRecordLog){
      //Start record log
      rosbagPid = fork();

      if( rosbagPid == 0 )
      {
         ROS_INFO("Launching rosbag PID: %d \n", rosbagPid);
         std::string newDir = mWorkspace;
         chdir(newDir.c_str());
          // we're the child process
          char *argv[] = { (char *) mDistro.c_str(), (char *) "record", (char *) "/gtec/gazebo/anchors",(char *) "/gtec/gazebo/pos", (char *) "/gtec/gazebo/erle/maginterfered", (char *) "/gtec/gazebo/erle/imu", (char *) "/gtec/gazebo/px4flow", (char*) "/gtec/toa/ranging", (char*)0 };
          int rc = execv(argv[0], &argv[0]);

          if(rc==-1) {
            ROS_INFO("Execv error %s \n", strerror(errno));
            }
          exit(1);
          // execve only returns if there was an error
          // check 'errno' and handle it here
      } else {
            mCanStart = true;
          mSteps.clear();
          mSteps = mBaseSteps;
          startNextStep();
      }
    } else {
        mCanStart = true;
        mSteps.clear();
        mSteps = mBaseSteps;
        startNextStep();
    } 
  }  
  
}

void GazeboAutoGuide::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);
 // if (mCanPublish)
  //{
    mVelPublisher.publish(mLastPublished);
    //ROS_INFO("Publish vel.linear.x = %f vel.angular.z= %f", mLastPublished.linear.x, mLastPublished.angular.z);
  //  mCanPublish=false;

  //}

}


void GazeboAutoGuide::newPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& aPose) {
    currentPose = *aPose;

      tf::Quaternion q(currentPose.pose.pose.orientation.x, 
        currentPose.pose.pose.orientation.y, 
        currentPose.pose.pose.orientation.z, 
        currentPose.pose.pose.orientation.w);

      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      if (yaw<0){
        yaw = 2*M_PI + yaw;
      }

      mCurrentYaw = yaw;

      mCurrentPoseX = currentPose.pose.pose.position.x;
      mCurrentPoseY= currentPose.pose.pose.position.y;

      //ROS_INFO("CurrentYaw: %f CurrentPos: (%f,%f)",mCurrentYaw,mCurrentPoseX,mCurrentPoseY);
    

    if (mHasAngleGoal){
      //We check if reached the goal angle

      double angleDiff;

      if (mAngularVelocityNegative==false){
        //Angle is growing

      if ((mAngleGoal>2*M_PI-0.2) && (mCurrentYaw<0.2)) {
          currentStepEnded();
        }else if (mCurrentYaw >= mAngleGoal && ((mCurrentYaw - mAngleGoal)<=0.5)){
              currentStepEnded();
        }


/*        if (mCurrentYaw<=mAngleGoal){
          angleDiff = fabs(mCurrentYaw - mAngleGoal);
        } else {
          angleDiff = (2*M_PI - mCurrentYaw) + mAngleGoal; 
        }

        //ROS_INFO("Current AngleDiff= %f", angleDiff);
        //ROS_INFO("Current AngleDiff= %f mCurrentYaw: %f", angleDiff,mCurrentYaw);

        if (mIsSpining) {
          //if (angleDiff<mCurrentAngleDiff && angleDiff>mAngleThreshold){
          //  mCurrentAngleDiff = angleDiff;
          //} else if (fabs(mAngleGoal-mCurrentYaw)<mAngleThreshold){
            //We reach the angle, we check if we need to perform more spins
            if (mNumFullSpinsGoal == mCurrentNumSpins){
              //STOP SPINING
              currentStepEnded();
            } else {
              mCurrentNumSpins +=1;
              mCurrentAngleDiff = angleDiff;
            }
          //}
        } else {
          if (angleDiff<mCurrentAngleDiff) {
            mIsSpining = true;
          }
        }
*/
      } else {
        //Angle is decreasing

        if ((mAngleGoal<0.2) && (mCurrentYaw>2*M_PI-0.2)) {
          currentStepEnded();
        } else if (mCurrentYaw <= mAngleGoal && (mAngleGoal - mCurrentYaw)<=0.5){
              currentStepEnded();
        }


/*        if (mCurrentYaw>=mAngleGoal){
          angleDiff = fabs(mCurrentYaw - mAngleGoal);
        } else {
          angleDiff = (2*M_PI - mAngleGoal) + mCurrentYaw; 
        }

        //ROS_INFO("Current AngleDiff= %f mCurrentYaw: %f", angleDiff,mCurrentYaw);

        if (mIsSpining) {
          if (angleDiff<mCurrentAngleDiff && angleDiff>mAngleThreshold){
            mCurrentAngleDiff = angleDiff;
          } else if (fabs(mAngleGoal-mCurrentYaw)<mAngleThreshold){
            //We reach the angle, we check if we need to perform more spins
            if (mNumFullSpinsGoal == mCurrentNumSpins){
              //STOP SPINING
              currentStepEnded();
            } else {
              mCurrentNumSpins +=1;
              mCurrentAngleDiff = angleDiff;
            }
          }
        }
        else {
          if (angleDiff<mCurrentAngleDiff){
           mIsSpining = true;
          }
        }*/
      }
    } else if (mHasPositionGoal){


      double currentDistance = sqrt(pow(mCurrentPoseX- mPositionGoalX,2) +  pow(mCurrentPoseY- mPositionGoalY,2) );

      if (mIsMoving){
        if (currentDistance<mCurrentDistance){
          mCurrentDistance = currentDistance;
        } else {
          //Goal reached
          currentStepEnded();
        }
      } else {
        if (currentDistance<mCurrentDistance){
          mIsMoving = true;
        }
      }

    }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_auto_guide");
  GazeboAutoGuide gazebo_auto_guide;

  ros::NodeHandle n("~");


  std::string topicPose;
  n.getParam("pose_topic", topicPose);

  ROS_INFO("AutoGuide: Pose Topic: %s", topicPose.c_str());
  ros::Subscriber sub0=n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(topicPose, 20, &GazeboAutoGuide::newPose, &gazebo_auto_guide); 


  ros::spin();
}
