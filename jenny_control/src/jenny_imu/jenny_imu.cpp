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
#include "sensor_msgs/Imu.h"

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
using namespace jenny_controller;

/*! zero covariance matrix */
static boost::array<double, 9> ZeroCovariance = {0.0, };


//------------------------------------------------------------------------------
// JennyController Class
//------------------------------------------------------------------------------

JennyController::JennyController(ros::NodeHandle &nh, double hz) :
    m_nh(nh), m_hz(hz)
{
}

JennyController::~JennyController()
{
  disconnect();
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Services
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void JennyController::advertiseServices()
{
  string  strSvc;

  strSvc = "get_imu";
  m_services[strSvc] = m_nh.advertiseService(strSvc,
                                          &JennyController::getImu,
                                          &(*this));

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

bool JennyController::getImu(GetImu::Request  &req,
                            GetImu::Response &rsp)
{
  const char *svc = "get_imu";

  double                    accel[ImuAlt::NUM_AXES];
  double                    gyro[ImuAlt::NUM_AXES];
  double                    rpy[ImuAlt::NUM_AXES];
  sensor::imu::Quaternion   q;
  int                       rc;

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  rc = m_robot.getImu(accel, gyro, rpy, q);

  if( rc == LAE_OK )
  {
    rsp.imu.orientation.x = q.m_x;
    rsp.imu.orientation.y = q.m_y;
    rsp.imu.orientation.z = q.m_z;
    rsp.imu.orientation.w = q.m_w;

    // for now until known
    rsp.imu.orientation_covariance = ZeroCovariance;

    rsp.imu.angular_velocity.x = gyro[sensor::imu::X];
    rsp.imu.angular_velocity.y = gyro[sensor::imu::Y];
    rsp.imu.angular_velocity.z = gyro[sensor::imu::Z];

    // for now until known
    rsp.imu.angular_velocity_covariance = ZeroCovariance;

    rsp.imu.linear_acceleration.x = accel[sensor::imu::X];
    rsp.imu.linear_acceleration.y = accel[sensor::imu::Y];
    rsp.imu.linear_acceleration.z = accel[sensor::imu::Z];

    // for now until known
    rsp.imu.linear_acceleration_covariance = ZeroCovariance;

    stampHeader(rsp.imu.header, 0);

    ROS_INFO("IMU %s data.", req.name.c_str());
    return true;
  }
  else
  {
    ROS_ERROR("Service %s failed on IMU %s: %s(rc=%d).",
          svc, req.name.c_str(), getStrError(rc), rc);
    return false;
  }
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
  LaeMapVelocity  vel;
  int             rc;

  ROS_DEBUG("%s/%s", m_nh.getNamespace().c_str(), svc);

  for(size_t i = 0; i < req.goal.names.size(); ++i)
  {
    vel[req.goal.names[i]] = req.goal.velocities[i];
  }

  rc = m_robot.move(vel);
 
  if( rc == LAE_OK )
  {
    stampHeader(rsp.actual.header, seq++);
    
    // RDK TODO get actual goal, not just copy target goal
    rsp.actual = req.goal;

    ROS_INFO("Robot velocities set.");
    for(size_t i = 0; i < rsp.actual.names.size(); ++i)
    {
      ROS_INFO(" %-12s: vel=%7.2lfdeg/s",
          rsp.actual.names[i].c_str(),
          radToDeg(rsp.actual.velocities[i]));
    }
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

  if( rc == LAE_OK )
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

  // topic conforms to the robot_pose_ekf ROS node
  strPub = "imu_data";
  m_publishers[strPub] =
    m_nh.advertise<sensor_msgs::Imu>(strPub, nQueueDepth);

  strPub = "robot_status";
  m_publishers[strPub] =
    m_nh.advertise<industrial_msgs::RobotStatus>(strPub, nQueueDepth);
}

void JennyController::publish()
{
  publishRobotStatus();
  publishSensorImu();
}

void JennyController::publishRobotStatus()
{
  //JennyRobotStatus status;   // really status 

  // get robot's extended status.
  //m_robot.getRobotStatus(status);

  // update robot status message
  //updateRobotStatusMsg(status, m_msgRobotStatus);

  // publish robot status message
  m_publishers["robot_status"].publish(m_msgRobotStatus);
}

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

void JennyController::publishImu()
{
  double                    accel[ImuAlt::NUM_AXES];
  double                    gyro[ImuAlt::NUM_AXES];
  double                    rpy[ImuAlt::NUM_AXES];
  sensor::imu::Quaternion   q;
  int                       rc;

  //
  // Grab latest IMU data
  //
  if( (rc = m_robot.getImu(accel, gyro, rpy, q)) != LAE_OK )
  {
    return;
  }

  //
  // Convert to ROS standard IMU message
  //
  m_msgImu.orientation.x = q.m_x;
  m_msgImu.orientation.y = q.m_y;
  m_msgImu.orientation.z = q.m_z;
  m_msgImu.orientation.w = q.m_w;

    // for now until known
  m_msgImu.orientation_covariance = ZeroCovariance;

  m_msgImu.angular_velocity.x = gyro[sensor::imu::X];
  m_msgImu.angular_velocity.y = gyro[sensor::imu::Y];
  m_msgImu.angular_velocity.z = gyro[sensor::imu::Z];

    // for now until known
  m_msgImu.angular_velocity_covariance = ZeroCovariance;

  m_msgImu.linear_acceleration.x = accel[sensor::imu::X];
  m_msgImu.linear_acceleration.y = accel[sensor::imu::Y];
  m_msgImu.linear_acceleration.z = accel[sensor::imu::Z];

    // for now until known
  m_msgImu.linear_acceleration_covariance = ZeroCovariance;

  stampHeader(m_msgImu.header, m_msgImu.header.seq+1);

  //
  // Publish messages
  //
  m_publishers["imu_data"].publish(m_msgImu);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Subscribed Topics
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void JennyController::subscribeToTopics(int nQueueDepth)
{
  string  strSub;

  strSub = "cmd_velocities";
  m_subscriptions[strSub] = m_nh.subscribe(strSub, nQueueDepth,
                                          &JennyController::execSetVelocities,
                                          &(*this));
}

void JennyController::execSetVelocities(const jenny_control::Velocity &msgVel)
{
  const char     *topic = "cmd_velocities";
  LaeMapVelocity  vel;

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
