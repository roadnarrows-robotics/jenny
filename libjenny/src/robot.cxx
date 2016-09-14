////////////////////////////////////////////////////////////////////////////////
//
// Package:   jenny
//
// Library:   libjenny
//
// File:      robot.cxx
//
/*! \file
 *
 * \brief Jenny Robot Class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include <string>
#include <utility>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/units.h"
#include "rnr/i2c.h"
#include "rnr/log.h"
#include "rnr/appkit/Thread.h"

// jenny
#include "jenny/jenny.h"
#include "jenny/utils.h"
#include "jenny/rs160d.h"
#include "jenny/robot.h"

using namespace std;
using namespace rnr;
using namespace jenny;

/*!
 * \brief Test for connection.
 *
 * Only works in JennyRobot methods.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define JEN_TRY_CONN() \
  do \
  { \
    if( !isConnected() ) \
    { \
      LOGERROR("Robot is not connected."); \
      return -JEN_ECODE_NO_EXEC; \
    } \
  } while(0)

/*!
 * \brief Test for not estop.
 *
 * Only works in JennyRobot methods.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define JEN_TRY_NOT_ESTOP() \
  do \
  { \
    if( m_bIsEStopped ) \
    { \
      LOGERROR("Robot is emergency stopped."); \
      return -JEN_ECODE_NO_EXEC; \
    } \
  } while(0)


// -----------------------------------------------------------------------------
// Class JennyRobot
// -----------------------------------------------------------------------------

const double JennyRobot::GovernorDft = 1.0;

JennyRobot::JennyRobot()
{
  int   nCtlr;

  // state
  m_bIsConnected      = false;
  m_eRobotMode        = JenRobotModeUnknown;
  m_bIsEStopped       = false;
  m_bAlarmState       = false;

  setGovernor(GovernorDft);

  // hardware
  m_fdMotorCtlr = -1;
}

JennyRobot::~JennyRobot()
{
  disconnect();
}

int JennyRobot::connect()
{
  int     rc = JEN_OK;    // return code

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Pre-connect requirements.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  //
  // Been here, did this.
  //
  if( m_bIsConnected )
  {
    LOGWARN("Jenny already connected to hardware.");
    return JEN_OK;
  }


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Initialize robot status.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  m_bIsConnected      = false;
  m_eRobotMode        = JenRobotModeUnknown;
  m_bIsEStopped       = false;
  m_bAlarmState       = false;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Establish hardware connections.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  //
  // Connect to all standard built-in sensors.
  //
  if( rc == JEN_OK )
  {
    rc = connSensors();
  }

  //
  // Connect to motor controllers.
  //
  if( rc == JEN_OK )
  {
    rc = connMotorController("/dev/ttyS0");
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Initialize and Configure
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
  //
  // Configure hardware for operation.
  //
  if( rc == JEN_OK )
  {
    rc = configForOperation();
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Start real-time, persistent threads.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
  if( rc == JEN_OK )
  {
    rc = startCoreThreads();
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Finale
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
  // failure
  if( rc != JEN_OK )
  {
    LOGERROR("Failed to connect and/or initialize hardware.");
    disconnect();
  }

  // success
  else
  {
    m_bIsConnected      = true;
    m_eRobotMode        = JenRobotModeAuto;
    m_bIsEStopped       = false;
    m_bAlarmState       = false;

    freeze(); // place robot in a safe state

    LOGDIAG1("Connected to Jenny.");
  }

  return rc;
}

int JennyRobot::disconnect()
{
  int   nCtlr;

  if( !isConnected() )
  {
    return JEN_OK;
  }

  //
  // Terminate all threads.
  //

  //
  // Close connections
  RS160DClose(m_fdMotorCtlr);
  m_fdMotorCtlr = -1;

  // reset robot state
  m_bIsConnected      = false;
  m_eRobotMode        = JenRobotModeUnknown;
  m_bIsEStopped       = false;
  m_bAlarmState       = false;

  LOGDIAG1("Disconnected from Jenny.");

  return JEN_OK;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Action Methods.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int JennyRobot::estop()
{
  int   rc;   // return code

  JEN_TRY_CONN();

  RS160DEStop(m_fdMotorCtlr);

  m_bIsEStopped       = true;
  m_bAlarmState       = true;

  rc = JEN_OK;

  LOGDIAG3("Jenny emergency stopped.");

  return rc;
}

int JennyRobot::resetEStop()
{
  int   rc;   // return code

  JEN_TRY_CONN();

  m_bIsEStopped       = false;
  m_bAlarmState       = false;

  rc = JEN_OK;

  LOGDIAG3("Jenny emergency stopped reset.");

  return rc;
}

int JennyRobot::stop()
{
  return freeze();
}

int JennyRobot::freeze()
{
  int   rc;

  JEN_TRY_CONN();

  RS160DUpdateMotorSpeeds(0, m_fdMotorCtlr, RS160D_MOTOR_LEFT_ID);
  RS160DUpdateMotorSpeeds(0, m_fdMotorCtlr, RS160D_MOTOR_RIGHT_ID);

  LOGDIAG3("Jenny frozen at current position.");

  return JEN_OK;
}

int JennyRobot::release()
{
  JEN_TRY_CONN();

  freeze();

  RS160DAlterBraking(RS160D_MOTOR_BRAKE_MIN, m_fdMotorCtlr,
      RS160D_MOTOR_LEFT_ID);
  RS160DAlterBraking(RS160D_MOTOR_BRAKE_MIN, m_fdMotorCtlr,
      RS160D_MOTOR_RIGHT_ID);

  LOGDIAG3("Jenny motor drives released.");

  return JEN_OK;
}

int JennyRobot::move(const vector<string> &names,
                     const vector<double> &velocities)
{
  size_t  i;
  int     motor;
  int     speed;
  int     rc;

  JEN_TRY_CONN();

  for(i=0; i<names.size(); ++i)
  {
    if( names[i] == "left" )
    {
      motor = RS160D_MOTOR_LEFT_ID;
    }
    else if( names[i] == "right" )
    {
      motor = RS160D_MOTOR_RIGHT_ID;
    }
    else
    {
      continue;
    }

    speed = RS160D_MOTOR_SPEED_MAX * velocities[i];

    RS160DUpdateMotorSpeeds(speed, m_fdMotorCtlr, motor);

    LOGDIAG3("Motor %d: speed=%d.", motor, speed);
  }

  return JEN_OK;
}

double JennyRobot::setGovernor(double fGovernor)
{
  m_fGovernor = fcap(fGovernor, 0.0, 1.0);

  return m_fGovernor;
}

double JennyRobot::incrementGovernor(double fDelta)
{
  return setGovernor(m_fGovernor+fDelta);
}

double JennyRobot::getGovernor()
{
  return m_fGovernor;
}

void JennyRobot::setRobotMode(JenRobotMode eRobotMode)
{
  m_eRobotMode = eRobotMode;
}

int JennyRobot::clearAlarms()
{
  JEN_TRY_CONN();

  return JEN_OK;
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Reports
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Attibute Methods.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

bool JennyRobot::isConnected()
{
  return m_bIsConnected;
}

void JennyRobot::getVersion(int &nVerMajor, int &nVerMinor, int &nRevision)
{
  uint_t  uHwVer;

  nVerMajor = 1;
  nVerMinor = 0;
  nRevision = 0;
}

JenRobotMode JennyRobot::getRobotMode()
{
  return m_eRobotMode;
}

bool JennyRobot::isEStopped()
{
  return m_bIsEStopped;
}

bool JennyRobot::areMotorsPowered()
{
  return true;
}

bool JennyRobot::isInMotion()
{
  return false;
}

bool JennyRobot::isAlarmed()
{
  return m_bAlarmState;
}

bool JennyRobot::canMove()
{
  return isConnected();
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Hardare Connection and Configuration Methods.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int JennyRobot::connSensors()
{
  return JEN_OK;
}

int JennyRobot::connMotorController(const std::string &strDevMotorCtlr)
{
  int     rc;

  m_fdMotorCtlr = RS160DOpen(strDevMotorCtlr.c_str());

  if( m_fdMotorCtlr < 0 )
  {
    LOGSYSERROR("%s", strDevMotorCtlr.c_str());
    rc = -JEN_ECODE_NO_DEV; 
  }

  else if( RS160DSetToSerial(m_fdMotorCtlr) < 0 )
  {
    LOGERROR("Failed to configure motor controller.");
    RS160DClose(m_fdMotorCtlr);
    m_fdMotorCtlr = -1;
    rc = -JEN_ECODE_MOT_CTLR;
  }

  else
  {
    LOGDIAG2("RS160D motor controller created on %s.", strDevMotorCtlr.c_str());
    rc = JEN_OK;
  }

  return rc;
}

int JennyRobot::configForOperation()
{
  int     rc;                 // return code

  rc = JEN_OK;

  LOGDIAG2("Configured for operation.");

  return rc;
}

int JennyRobot::startCoreThreads()
{
  int     nPriority;
  double  fHz;
  int     rc;

  //
  // WatchDog thread.
  //
  //nPriority = LaeThreadWd::ThreadWdPrioDft;
  //fHz       = LaeThreadWd::optimizeHz(m_tunes.getWatchDogTimeout());

  //if( (rc = startThread(&m_threadWatchDog, nPriority, fHz)) != JEN_OK )
  //{
  //  return rc;
  //}

  return JEN_OK;
}

int JennyRobot::startThread(Thread *pThread, int nPriority, double fHz)
{
  string  strName;
  int     rc;

  strName = pThread->getThreadName();

  if( (rc = pThread->createThread(nPriority)) < 0 )
  {
    LOGERROR("%s thread: Failed to create.", strName.c_str());
  }
  else if( (rc = pThread->runThread(fHz)) < 0 )
  {
    LOGERROR("%s thread: Failed to start.", strName.c_str());
  }
  else
  {
    LOGDIAG2("%s thread started at %.3lfHz with priority %d.",
        strName.c_str(), fHz, nPriority); 
    rc = JEN_OK;
  }

  return rc;
}

