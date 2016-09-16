////////////////////////////////////////////////////////////////////////////////
//
// Package:   jenny
//
// Library:   libjenny
//
// File:      robot.h
//
/*! \file
 *
 * \brief Jenny Robot Class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _ROBOT_H
#define _ROBOT_H

#include <pthread.h>

#include <string>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/units.h"
#include "rnr/i2c.h"
#include "rnr/appkit/Thread.h"

// jenny
#include "jenny/jenny.h"
#include "jenny/utils.h"
#include "jenny/rs160d.h"
#include "jenny/uss.h"
#include "jenny/threadUss.h"

namespace jenny
{
  /*!
   * \brief Laelaps robotic manipulator plus accesories class.
   */
  class JennyRobot
  { 
  public:
    static const double GovernorDft;  ///< speed limit governor start-up default

    /*!
     * \brief Default initialization constructor.
     */
    JennyRobot();

    /*!
     * \brief Destructor.
     */
    virtual ~JennyRobot();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Communication and Robot Initialization Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Connect to jenny.
     *
     * \copydoc doc_return_std
     */
    int connect();

    /*!
     * \brief Disconnect from jenny.
     *
     * \copydoc doc_return_std
     */
    int disconnect();

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Action Methods.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Emergency stop.
     *
     * All motor will stop driving, so arm or accessories may fall.
     *
     * \copydoc doc_return_std
     */
    int estop();

    /*!
     * \brief Reset (clears) emergency stop condition.
     *
     * \note Motors are not re-powered until an move or freeze action is called.
     */
    int resetEStop();

    /*!
     * \brief Freeze robot and accessories at current position.
     *
     * Motors are still being driven. The motors are either dynamically or 
     * regeneratively braked.
     *
     * \copydoc doc_return_std
     */
    int freeze();

    /*!
     * \brief Release robot and accessories.
     *
     * Motors will stop being driven. The robot will coast to a stop (on a flat
     * surface) and is capabable of being externally pushed.
     *
     * \copydoc doc_return_std
     */
    int release();

    /*!
     * \brief Stop robot with full dynamic braking.
     *
     * \copydoc doc_return_std
     */
    int stop();

    /*!
     * \brief Move by setting powertrain angular wheel velocities.
     *
     * The robot is controlled by setting the goal velocities of a [sub]set of 
     * powertrain motors.
     *
     * \param velocity  Map of powertrain velocities.\n
     *                  Keys are left_front, right_front, left_rear, and
     *                  right_rear.\n
     *                  Values are drive shaft/wheel angular velocities
     *                  (radians/second).
     *
     * \copydoc doc_return_std
     */
    int move(const std::vector<std::string> &names,
             const std::vector<double> &velocities);

    /*!
     * \brief Set speed limit governor value.
     *
     * \note Software governor is not implemented as yet.
     *
     * Governor is defined as:\n
     * speed = cap(set_speed, min_speed * governor, max_speed * governor)
     *
     * \param fGovernor   Governor value between [0.0, 1.0].
     *
     * \return Returns new governor value.
     */
    double setGovernor(double fGovernor);

    /*!
     * \brief Increment/decrement speed limit governor value.
     *
     * \note Software governor is not implemented as yet.
     *
     * Governor is defined as:\n
     * speed = set_speed * governor
     *
     * \param fDelta  Governor \h_plusmn delta.
     *
     * \return Returns new governor value.
     */
    double incrementGovernor(double fDelta);

    /*!
     * \brief Get current speed limit governor setting.
     *
     * \note Software governor is not implemented as yet.
     *
     * \return Return value.
     */
    double getGovernor();

    /*!
     * \brief Set robot's operational mode.
     *
     * \param eRobotMode Robot operation mode. See \ref JenRobotMode.
     */
    void setRobotMode(JenRobotMode eRobotMode);

    /*!
     * \brief Attempt to clear all alarms.
     *
     * \note For Laelaps, most alarms are not clearable (e.g. temperature,
     * low battery). Only external intervention is effective.
     *
     * \copydoc doc_return_std
     */
    int clearAlarms();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Reports
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Attribute Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Test if connected to jenny hardware.
     *
     * \return Returns true or false.
     */
    bool isConnected();

    /*!
     * \brief Get the jenny hardware version number.
     *
     * \param [out] nVerMajor     Major version number.
     * \param [out] nVerMinor     Minor version number.
     * \param [out] nVerRevision  Revision version number.
     */
    void getVersion(int &nVerMajor, int &nVerMinor, int &nRevision);

    /*!
     * \brief Get robot's operational mode.
     *
     * \return Robot operation mode. See \ref JenRobotMode.
     */
    JenRobotMode getRobotMode();

    /*!
     * \brief Test if robot is current emergency stopped.
     *
     * \return Returns true or false.
     */
    bool isEStopped();

    /*!
     * \brief Test if robot motor are currently being driven (powered).
     *
     * \return Returns true or false.
     */
    bool areMotorsPowered();

    /*!
     * \brief Test if any joint in any of the kinematic chains is moving.
     *
     * \note RS160D motor controller provide no feedback.
     *
     * \return Returns true or false.
     */
    bool isInMotion();

    /*!
     * \brief Test if robot is alarmed.
     *
     * \return Returns true or false.
     */
    bool isAlarmed();

    /*!
     * \brief Test if robot is safe to operate, given the current robot and
     * alarm state.
     *
     * \return Returns true or false.
     */
    bool canMove();

    /*!
     * \brief Get latest ultrasonic sensor readings.
     *
     * \return Vector of measurements.
     */
    std::vector<double> getUSSReadings()
    {
      return m_uss.getUssData();
    }

  protected:
    // state
    bool            m_bIsConnected;     ///< critical hardware [not] connected
    JenRobotMode    m_eRobotMode;       ///< robot operating mode
    bool            m_bIsEStopped;      ///< robot is [not] emergency stopped
    bool            m_bAlarmState;      ///< robot is [not] alarmed
    double          m_fGovernor;        ///< speed limit governor setting

    // hardware
    int                   m_fdMotorCtlr;  ///< motor controller file descriptor
    sensor::uss::JennyUss m_uss;          ///< ultrasonic sensors

    // threads
    ThreadUss     m_threadUss;

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Hardare Connection and Configuration Methods.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Connect to the jenny built-in sensors.
     *
     * \par Sensors:
     * * IMU
     * * ToF infrared distance sensors
     *
     * \copydoc doc_return_std
     */
    int connSensors();

    /*!
     * \brief Connect to the jenny motor controllers
     *
     * Motors controller serial interface support multi-drop, so one serial
     * device can support up to 8 motor controllers.
     *
     * \param strDevMotorCtlr Motor controllers serial device name.
     *
     * \copydoc doc_return_std
     */
    int connMotorController(const std::string &strDevMotorCtlr);

    /*!
     * \brief Configure jenny for normal operation.
     *
     * The arm, end effector, and accessory effectors are configured for
     * normal operation.
     *
     * \copydoc doc_return_std
     */
    int configForOperation();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Threads 
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Create and start all real-time persistent core threads.
     *
     * \copydoc doc_return_std
     */
    int startCoreThreads();

    /*!
     * \brief Create and start a thread at the given priority and hertz.
     *
     * \param pThread   Pointer the thread object.
     * \param nPriority Thread priority.
     * \param fHz       Thread execution hertz.
     *
     * \copydoc doc_return_std
     */
    int startThread(rnr::Thread *pThread, int nPriority, double fHz);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Friends 
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  }; // class JennyRobot

} // namespace jenny


#endif // _ROBOT_H
