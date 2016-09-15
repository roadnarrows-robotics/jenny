////////////////////////////////////////////////////////////////////////////////
//
// Package:   Jenny Autonmous Grocery Cart ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/jenny
//
// ROS Node:  jenny_controller
//
// File:      jenny_controller.cpp
//
/*! \file
 *
 * \brief The ROS jenny_controller node class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 */
////////////////////////////////////////////////////////////////////////////////

//
// System
//
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <limits>
#include <string>
#include <vector>
#include <map>

//
// Boost libraries
//
#include <boost/bind.hpp>
#include <boost/array.hpp>

//
// ROS
//
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

//
// ROS generated core and industrial messages.
//
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "industrial_msgs/TriState.h"
#include "industrial_msgs/RobotStatus.h"
#include "sensor_msgs/Joy.h"
#include "autorally_msgs/chassisCommand.h"

//
// ROS generated Jenny messages.
//
#include "jenny_control/Velocity.h"

//
// ROS generatated Jenny services.
//
#include "jenny_control/SetRobotMode.h"
#include "jenny_control/SetVelocities.h"
#include "jenny_control/Stop.h"

//
// ROS generated action servers.
//

//
// RoadNarrows
//
#include "rnr/rnrconfig.h"
#include "rnr/log.h"

//
// Jenny embedded jenny library.
//
#include "jenny/jenny.h"
#include "jenny/utils.h"
#include "jenny/robot.h"

//
// Node headers.
//
#include "jenny_controller.h"


using namespace std;
using namespace jenny;
using namespace jenny_control;
using namespace jenny_controller;

/*! zero covariance matrix */
static boost::array<double, 9> ZeroCovariance = {0.0, };


//------------------------------------------------------------------------------
// JennyController Class
//------------------------------------------------------------------------------

JennyController::JennyController(ros::NodeHandle &nh, double hz) :
    m_nh(nh), m_hz(hz), m_maxSpeed(0.25), 
    m_turnDamp(0.5), m_speedDamp(0.5),
    m_horizon(100.0), m_usWeight(0.5)
{
  if(m_nh.getParam("maxSpeed", m_maxSpeed))
  {
    ROS_INFO("Setting Max Speed: %f", m_maxSpeed);
  }
  if(m_nh.getParam("turnDamp", m_turnDamp))
  {
    ROS_INFO("Setting Turn Damp: %f", m_turnDamp);
  }
  if(m_nh.getParam("turnDamp", m_turnDamp))
  {
    ROS_INFO("Setting Speed Damp: %f", m_speedDamp);
  }
  if(m_nh.getParam("horizon", m_horizon))
  {
    ROS_INFO("Setting Horizon: %f", m_horizon);
  }
  if(m_nh.getParam("usWeight", m_usWeight))
  {
    ROS_INFO("Setting USS Weight: %f", m_usWeight);
  }
}

JennyController::~JennyController()
{
  disconnect();
}

int JennyController::connect()
{
  return m_robot.connect();
}

int JennyController::disconnect()
{
  return m_robot.disconnect();
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Services
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void JennyController::advertiseServices()
{
  string  strSvc;

  strSvc = "set_robot_mode";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &JennyController::setRobotMode,
                                          &(*this));

  strSvc = "set_velocities";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &JennyController::setVelocities,
                                          &(*this));

  strSvc = "stop";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &JennyController::stop,
                                          &(*this));
}

bool JennyController::setRobotMode(SetRobotMode::Request  &req,
                                   SetRobotMode::Response &rsp)
{
  const char *svc = "set_robot_mode";

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  m_robot.setRobotMode((JenRobotMode)req.mode.val);

  ROS_INFO("Robot mode set to %d.", req.mode.val);

  return true;
}

bool JennyController::setVelocities(SetVelocities::Request  &req,
                                    SetVelocities::Response &rsp)
{
  const char     *svc = "set_velocities";
  static u32_t    seq = 0;
  int             rc;

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  rc = m_robot.move(req.goal.names, req.goal.velocities);
 
  if( rc == JEN_OK )
  {
    stampHeader(rsp.actual.header, seq++);
    
    // RDK TODO get actual goal, not just copy target goal
    rsp.actual = req.goal;

    ROS_INFO("Robot velocities set.");

    return true;
  }
  else
  {
    ROS_ERROR("Service %s failed: %s(rc=%d).",
          svc, getStrError(rc), rc);
    return false;
  }
}

bool JennyController::stop(Stop::Request  &req,
                          Stop::Response &rsp)
{
  const char *svc = "stop";
  int         rc;

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  rc = m_robot.stop();

  if( rc == JEN_OK )
  {
    ROS_INFO("Stopped.");
    return true;
  }
  else
  {
    ROS_ERROR("Service %s failed: %s(rc=%d).", svc, getStrError(rc), rc);
    return false;
  }
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Topic Publishers
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void JennyController::advertisePublishers(int nQueueDepth)
{
  string  strPub;

  strPub = "robot_status";
  m_publishers[strPub] =
    m_nh.advertise<industrial_msgs::RobotStatus>(strPub, nQueueDepth);
}

void JennyController::publish()
{
  publishRobotStatus();
}

void JennyController::publishRobotStatus()
{
  //JennyRobotStatus status;   // really status 

  // get robot's extended status.
  //m_robot.getRobotStatus(status);

  // update robot status message
  //updateRobotStatusMsg(status, m_msgRobotStatus);

  // publish robot status message
  //m_publishers["robot_status"].publish(m_msgRobotStatus);
}

#if 0 // RDK
void JennyController::updateRobotStatusMsg(JennyRobotStatus &status,
                                          industrial_msgs::RobotStatus &msg)
{
  //
  // Set header.
  //
  stampHeader(msg.header, msg.header.seq+1);

  //
  // Set industrial message compliant robot status values.
  //
  msg.mode.val            = status.m_eRobotMode;
  msg.e_stopped.val       = status.m_eIsEStopped;
  msg.drives_powered.val  = status.m_eAreDrivesPowered;
  msg.motion_possible.val = status.m_eIsMotionPossible;
  msg.in_motion.val       = status.m_eIsInMotion;
  msg.in_error.val        = status.m_eIsInError;
  msg.error_code          = status.m_nErrorCode;

}
#endif // RDK

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Subscribed Topics
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void JennyController::subscribeToTopics(int nQueueDepth)
{
  string  strSub;

  strSub = "Joy";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &JennyController::execJoy,
                                          &(*this));

  strSub = "cmd_velocities";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &JennyController::execSetVelocities,
                                          &(*this));

  strSub = "/wayPointFollower/chassisCommand";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &JennyController::execWayPointChassisCommand,
                                          &(*this));
}

void JennyController::execJoy(const sensor_msgs::Joy &msgJoy)
{
  const char     *topic = "joy";

  double          velLinear, velAngular;
  double          velLeft, velRight;
  double          div;
  vector<string>  names;
  vector<double>  velocities;

  ROS_INFO("%s/%s", m_nh.getNamespace().c_str(), topic);

  velLinear   = msgJoy.axes[1];
  velAngular  = msgJoy.axes[3];

  velLeft   = velLinear + velAngular;
  velRight  = velLinear - velAngular;

  // calculate divider
  if( fabs(velLeft) > 1.0 )
  {
    div = fabs(velLeft);
  }
  else if( fabs(velRight) > 1.0 )
  {
    div = fabs(velRight);
  }
  else
  {
    div = 1.0;
  }

  // normalize speed [-1.0, 1.0]
  velLeft  = fcap(velLeft/div,  -1.0, 1.0);
  velRight = fcap(velRight/div, -1.0, 1.0);

  names.push_back("left");
  velocities.push_back(velLeft);
  names.push_back("right");
  velocities.push_back(velRight);

  m_robot.move(names, velocities);
}

void JennyController::execWayPointChassisCommand(const autorally_msgs::chassisCommand &msgWP)
{
   
  double velAngular;
  double velLeft, velRight;
  double div;
  double dampSpeed;
  double usSteer;
  double finalSpeed;

  vector<string>  names;
  vector<double>  velocities;
  vector<double>  usReadings;
  vector<double>  steerWeights;

  m_nh.getParam("maxSpeed", m_maxSpeed);
  m_nh.getParam("turnDamp", m_turnDamp);
  m_nh.getParam("speedDamp", m_speedDamp);

  velAngular = msgWP.steering;
  //usReadings = m_robot.getUSSReadings();
  for(int i=0; i<5; i++)
  {
    double val = (m_horizon - usReadings[i])/m_horizon;
    if(val <= 0)
    {
      val = 0.0;
    }
    steerWeights[i] = val;
  }

  usSteer = (steerWeights[0] + steerWeights[1] 
                   - steerWeights[3] - steerWeights[4])/4.0;
  usSteer *= m_usWeight;
  velAngular += usSteer;
  dampSpeed = m_maxSpeed - fabs(velAngular)*m_turnDamp;
  finalSpeed = dampSpeed - steerWeights[2]*m_speedDamp;

  velLeft = finalSpeed - velAngular;
  velRight = finalSpeed + velAngular;

  // calculate divider
  if( fabs(velLeft) > 1.0 )
  {
    div = fabs(velLeft);
  }
  else if( fabs(velRight) > 1.0 )
  {
    div = fabs(velRight);
  }
  else
  {
    div = 1.0;
  }

  // normalize speed [-1.0, 1.0]
  velLeft  = fcap(velLeft/div,  -1.0, 1.0);
  velRight = fcap(velRight/div, -1.0, 1.0);

  names.push_back("left");
  velocities.push_back(velLeft);
  names.push_back("right");
  velocities.push_back(velRight);

  m_robot.move(names, velocities);
}

void JennyController::execSetVelocities(const jenny_control::Velocity &msgVel)
{
  const char     *topic = "cmd_velocities";

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), topic);

  m_robot.move(msgVel.names, msgVel.velocities);
}

void JennyController::stampHeader(std_msgs::Header &header,
                                 u32_t             nSeqNum,
                                 const string     &strFrameId)
{
  header.seq      = nSeqNum;
  header.stamp    = ros::Time::now();
  header.frame_id = strFrameId;
}
