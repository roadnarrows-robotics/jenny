////////////////////////////////////////////////////////////////////////////////
//
// Package:   jenny
//
// Library:   libjenny
//
// File:      threadUss.h
//
/*! \file
 *
 * \brief Jenny UltraSonic Sensor thread class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _THREAD_USS_H
#define _THREAD_USS_H

#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/appkit/Thread.h"

#include "jenny/jenny.h"
#include "jenny/utils.h"
#include "jenny/uss.h"

/*!
 *  \brief The jenny namespace encapsulates all jenny related
 *  constructs.
 */
namespace jenny
{
  //----------------------------------------------------------------------------
  // ThreadUss Class
  //----------------------------------------------------------------------------
  
  /*!
   * IMU thread class.
   */
  class ThreadUss : public rnr::Thread
  {
  public:
    static const double ThreadImuPrioDft  = 50;     ///< default priority
    static const double ThreadImuHzDft    = 20.0;   ///< default run rate

    /*!
     * \brief Default constructor.
     */
    ThreadUss(sensor::uss::JennyUss &hwif);

    /*!
     * \brief Destructor.
     */
    virtual ~ThreadUss();

  protected:
    sensor::uss::JennyUss &m_hwif;    ///< hardware interface

    /*!
     * \brief Execute watchdog task within scheduled cycle.
     *
     * This function executes under the lock/unlock mutex.
     *
     * \par Context:
     * This thread.
     */
    virtual void exec();
  };
  
} // namespace jenny

#endif // _THREAD_USS_H
