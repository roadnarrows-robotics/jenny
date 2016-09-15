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

#ifndef _JENNY_CONTROLLER_H
#define _JENNY_CONTROLLER_H

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

#define MAX_SPEED 0.5

namespace jenny_controller
{
  /*!
   * \brief The class embodiment of the jenny_controller ROS node.
   */
  class JennyController
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
    JennyController(ros::NodeHandle &nh, double hz);

    /*!
     * \brief Destructor.
     */
    virtual ~JennyController();

    /*!
     * \brief Connect to Jenny robot.
     *
     * \return Returns JEN_OK of success, \h_lt 0 on failure.
     */
    int connect();

    /*!
     * \brief Disconnect from Jenny robot.
     *
     * \return Returns JEN_OK of success, \h_lt 0 on failure.
     */
    int disconnect();

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
     * \brief Get bound embedded robot instance.
     *
     * \return Robot instance.
     */
    jenny::JennyRobot &getRobot()
    {
      return m_robot;
    }

  protected:
    ros::NodeHandle      &m_nh;     ///< the node handler bound to this instance
    double                m_hz;     ///< application nominal loop rate
    jenny::JennyRobot     m_robot;  ///< real-time, Jenny autonmouse robot

    // ROS services, publishers, subscriptions.
    MapServices       m_services;       ///< server services
    MapClientServices m_clientServices; ///< client services
    MapPublishers     m_publishers;     ///< publishers
    MapSubscriptions  m_subscriptions;  ///< subscriptions

    // Messages for published data.
    industrial_msgs::RobotStatus  m_msgRobotStatus; ///< robot status message

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Service callbacks
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Set robot's manual/auto mode service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool setRobotMode(jenny_control::SetRobotMode::Request  &req,
                      jenny_control::SetRobotMode::Response &rsp);

    /*!
     * \brief Set angular velocities service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool setVelocities(jenny_control::SetVelocities::Request  &req,
                       jenny_control::SetVelocities::Response &rsp);

    /*!
     * \brief Stop a set of joints robot service callback.
     *
     * \param req   Service request.
     * \param rsp   Service response.
     *
     * \return Returns true on success, false on failure.
     */
    bool stop(jenny_control::Stop::Request  &req,
              jenny_control::Stop::Response &rsp);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Topic Publishers
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Publish robot status and extended robot status topics.
     */
    void publishRobotStatus();

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Subscribed Topic Callbacks
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Execute joystick (xbox) commands subscibed topic callback.
     *
     * \param msgVel  Velocity message.
     */
    void execJoy(const sensor_msgs::Joy &msgJoy);

    /*!
     * \brief Execute waypoint follower chassis commands subscibed topic callback.
     *
     * \param msgWP  WayPoint message.
     */
    void execWayPointChassisCommand(const autorally_msgs::chassisCommand &msgWP);

    /*!
     * \brief Execute set velocities subscibed topic callback.
     *
     * \param msgVel  Velocity message.
     */
    void execSetVelocities(const jenny_control::Velocity &msgVel);


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

} // namespace jenny_controller


#endif // _JENNY_CONTROLLER_H
