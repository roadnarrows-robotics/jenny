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
 * \brief Jenny UltraSonic Sensor thread class implemenation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 */
////////////////////////////////////////////////////////////////////////////////


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
#include "jenny/threadUss.h"

using namespace std;
using namespace rnr;
using namespace jenny;
using namespace sensor::uss;

//------------------------------------------------------------------------------
// ThreadUss Class
//------------------------------------------------------------------------------
  
ThreadUss::ThreadUss(JennyUss &hwif) :
    Thread("USS"), m_hwif(hwif)
{
}

ThreadUss::~ThreadUss()
{
}

void ThreadUss::exec()
{
  if( m_hwif.isOpen() )
  {
    m_hwif.readSensors();
  }
}
