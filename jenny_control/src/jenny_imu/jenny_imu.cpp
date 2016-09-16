////////////////////////////////////////////////////////////////////////////////
//
// Package:   Jenny Autonmous Grocery Cart ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/jenny
//
// ROS Node:  jenny_imu
//
// File:      jenny_imu.cpp
//
/*! \file
 *
 * \brief The ROS jenny_imu node class implementation.
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

//
// ROS generated core and industrial messages.
//
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"

//
// ROS generated Jenny messages.
//

//
// ROS generatated Jenny services.
//

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
#include "jenny/naze32imu.h"

//
// Node headers.
//
#include "jenny_imu.h"


using namespace std;
using namespace jenny;
using namespace jenny_imu;

/*! zero covariance matrix */
static boost::array<double, 9> ZeroCovariance = {0.0, };

static const char *ImuDevName   = "/dev/imu";
static const int   ImuBaudRate  = 115200;

//------------------------------------------------------------------------------
// JennyImu Class
//------------------------------------------------------------------------------

JennyImu::JennyImu(ros::NodeHandle &nh, double hz) :
    m_nh(nh), m_hz(hz)
{
}

JennyImu::~JennyImu()
{
  disconnect();
}

int JennyImu::connect()
{
  string  strIdent;
  int     rc;

  if( (rc = m_imu.open(ImuDevName, ImuBaudRate)) < 0 )
  {
    LOGERROR("%s: Failed to open IMU at %d baud.", ImuDevName, ImuBaudRate);
    return rc;
  }

  else if( (rc = m_imu.readIdentity(strIdent)) < 0 )
  {
    LOGERROR("%s: Failed to read IMU identity.", ImuDevName);
    return rc;
  }

  else
  {
    LOGDIAG2("Connected to IMU %s.", strIdent.c_str());
  }

  return rc;
}

int JennyImu::disconnect()
{
  return m_imu.close();
}

void JennyImu::sense()
{
  m_imu.exec();
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Services
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void JennyImu::advertiseServices()
{
  string  strSvc;

  //strSvc = "get_imu";
  //m_services[strSvc] = m_nh.advertiseService(strSvc,
  //                                        &JennyImu::getImu,
  //                                        &(*this));
}

#if 0 // RDK
bool JennyImu::getImu(GetImu::Request  &req,
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
#endif // RDK


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Topic Publishers
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void JennyImu::advertisePublishers(int nQueueDepth)
{
  string  strPub;

  // topic conforms to the robot_pose_ekf ROS node
  strPub = "/imu/imu";
  m_publishers[strPub] =
    m_nh.advertise<sensor_msgs::Imu>(strPub, nQueueDepth);
}

void JennyImu::publish()
{
  publishImu();
}


void JennyImu::publishImu()
{
  double                    accel[sensor::imu::NumOfAxes];
  double                    gyro[sensor::imu::NumOfAxes];
  double                    rpy[sensor::imu::NumOfAxes];
  double                    mag[sensor::imu::NumOfAxes];    // no magnetometer
  sensor::imu::Quaternion   q;
  int                       rc;

  m_imu.getImuData(accel, gyro, mag, rpy, q);

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

  stampHeader(m_msgImu.header, m_msgImu.header.seq+1, "imu");

  //
  // Publish messages
  //
  m_publishers["/imu/imu"].publish(m_msgImu);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Subscribed Topics
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void JennyImu::subscribeToTopics(int nQueueDepth)
{
}

void JennyImu::stampHeader(std_msgs::Header     &header,
                                 u32_t          nSeqNum,
                                 const string   &strFrameId)
{
  header.seq      = nSeqNum;
  header.stamp    = ros::Time::now();
  header.frame_id = strFrameId;
}
