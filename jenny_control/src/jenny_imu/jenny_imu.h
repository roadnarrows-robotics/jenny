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

#ifndef _JENNY_IMU_H
#define _JENNY_IMU_H

//
// System
//
#include <string>
#include <map>

//
// Boost libraries
//
#include <boost/bind.hpp>

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

namespace jenny_imu
{
  /*!
   * \brief The class embodiment of the jenny_imu ROS node.
   */
  class JennyImu
  {
  public:
    /*! map of ROS server services type */
    typedef std::map<std::string, ros::ServiceServer> MapServices;

    /*! map of ROS client services type */
    typedef std::map<std::string, ros::ServiceClient> MapClientServices;
    
    /*! map of ROS publishers type */
    typedef std::map<std::string, ros::Publisher> MapPublishers;

    /*! map of ROS subscriptions type */
    typedef std::map<std::string, ros::Subscriber> MapSubscriptions;

    /*!
     * \brief Default initialization constructor.
     *
     * \param nh  Bound node handle.
     * \param hz  Application nominal loop rate in Hertz.
     */
    JennyImu(ros::NodeHandle &nh, double hz);

    /*!
     * \brief Destructor.
     */
    virtual ~JennyImu();

    /*!
     * \brief Connect to Jenny imu.
     *
     * \return Returns JEN_OK of success, \h_lt 0 on failure.
     */
    int connect();

    /*!
     * \brief Disconnect from Jenny imu.
     *
     * \return Returns JEN_OK of success, \h_lt 0 on failure.
     */
    int disconnect();

    /*!
     * \brief Execute sense of IMU sensor.
     */
    void sense();

    /*!
     * \brief Advertise all server services.
     */
    virtual void advertiseServices();

    /*!
     * \brief Initialize client services.
     */
    virtual void clientServices()
    {
      // No client services
    }

    /*!
     * \brief Advertise all publishers.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void advertisePublishers(int nQueueDepth=10);

    /*!
     * \brief Subscribe to all topics.
     *
     * \param nQueueDepth   Maximum queue depth.
     */
    virtual void subscribeToTopics(int nQueueDepth=10);

    /*!
     * \brief Publish.
     *
     * Call in main loop.
     */
    virtual void publish();

    /*!
     * \brief Get bound node handle.
     *
     * \return Node handle.
     */
    ros::NodeHandle &getNodeHandle()
    {
      return m_nh;
    }

    /*!
     * \brief Get bound embedded imu instance.
     *
     * \return Robot instance.
     */
    sensor::imu::JennyImuCleanFlight &getImu()
    {
      return m_imu;
    }

  protected:
    ros::NodeHandle      &m_nh;     ///< the node handler bound to this instance
    double                m_hz;     ///< application nominal loop rate
    sensor::imu::JennyImuCleanFlight  m_imu;  ///< IMU

    // ROS services, publishers, subscriptions.
    MapServices       m_services;       ///< server services
    MapClientServices m_clientServices; ///< client services
    MapPublishers     m_publishers;     ///< publishers
    MapSubscriptions  m_subscriptions;  ///< subscriptions

    // Messages for published data.
    sensor_msgs::Imu  m_msgImu; ///< IMU message

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Service callbacks
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get inertia measurement unit's latest sensed data service
     * callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    //bool getImuData(jenny_control::GetImu::Request  &req,
    //               jenny_control::GetImu::Response &rsp);

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Topic Publishers
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Publish IMU sensor topics.
     */
    void publishImu();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Subscribed Topic Callbacks
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Utilities
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Fill in ROS standard message header.
     *
     * \param [out] header    Message header.
     * \param nSeqNum         Sequence number.
     * \param strFrameId      Frame id. No frame = "0", global frame = "1".
     */
    void stampHeader(std_msgs::Header  &header,
                     u32_t             nSeqNum = 0,
                     const std::string &strFrameId = "0");
  };

} // namespace jenny_imu


#endif // _JENNY_IMU_H
