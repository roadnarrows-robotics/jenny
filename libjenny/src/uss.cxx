////////////////////////////////////////////////////////////////////////////////
//
// Package:   jenny
//
// File:      uss.cxx
//
/*! \file
 *
 * \brief Jenny UltraSonic Sensor array class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 */
////////////////////////////////////////////////////////////////////////////////

#include <pthread.h>
#include <unistd.h>
#include <math.h>

#include <string>
#include <sstream>

#include "rnr/rnrconfig.h"
#include "rnr/serdev.h"
#include "rnr/log.h"

#include "jenny/jenny.h"
#include "jenny/utils.h"
#include "jenny/uss.h"

using namespace std;
using namespace jenny;
using namespace sensor::uss;


//------------------------------------------------------------------------------
// JennyUss Virtual Base Class
//------------------------------------------------------------------------------
 
JennyUss::JennyUss() :
  m_vecMeas(5, 0.0)
{
  m_fd = -1;

  zeroData();

  pthread_mutex_init(&m_mutexIo, NULL);
}

JennyUss::~JennyUss()
{
  close();

  pthread_mutex_destroy(&m_mutexIo);
}

int JennyUss::open(const string &strDevName, int nBaudRate)
{
  string  strDevNameReal;
  int     rc;

  lockIo();

  // Get the real device name, not any symbolic links.
  strDevNameReal  = getRealDeviceName(strDevName);
  //strDevNameReal  = strDevName;

  m_fd = SerDevOpen(strDevNameReal.c_str(), nBaudRate, 8, 'N', 1, false, false);

  if( m_fd < 0 )
  {
    LOGERROR("Failed to open %s@%d.", strDevNameReal.c_str(), nBaudRate);
    rc = -JEN_ECODE_NO_DEV;
  }

  else
  {
    m_strDevName  = strDevNameReal;
    m_nBaudRate   = nBaudRate;

    zeroData();

    LOGDIAG3("Opened interface to USS arduino on %s@%d.",
      strDevNameReal.c_str(), nBaudRate);

    rc = JEN_OK;
  }

  unlockIo();

  return rc;
}

int JennyUss::close()
{
  lockIo();

  if( m_fd >= 0 )
  {
    SerDevClose(m_fd);
    LOGDIAG3("Closed %s interface to USS arduino.", m_strDevName.c_str());
  }

  m_strDevName.clear();
  m_nBaudRate = 0;
  m_fd        = -1;

  unlockIo();

  return JEN_OK;
}

void JennyUss::zeroData()
{
  m_nCursor = 0;

  for(size_t i=0; i<NumOfUss; ++i)
  {
    m_vecMeas[i] = 0.0;
  }
}

void JennyUss::readSensors()
{
  int   c;
  bool  bGetInput = true;

  if( m_fd < 0 )
  {
    return;
  }

  lockIo();

  while( bGetInput )
  {
    c = SerDevGetc(m_fd, 1000); //UssTimeoutUsec);

    switch( c )
    {
      case -1:
        bGetInput = false;
        break;
      case '\n':
        m_bufInput[m_nCursor] = 0;
        //fprintf(stderr, "DBG: %s\n", m_bufInput);
        parseInput();
        m_nCursor = 0;
        bGetInput = false;
        break;
      default:
        if( m_nCursor < UssMaxLineLen-1 )
        {
          m_bufInput[m_nCursor++] = (char)c;
        }
        else
        {
          m_nCursor = 0;    // flush this
        }
        break;
    };
  }

  unlockIo();
}

void JennyUss::parseInput()
{
  int   i;
  int   nStart = UssMaxLineLen;

  for(i=0; i<m_nCursor; ++i)
  {
    switch( m_bufInput[i] )
    {
      case '[':
        nStart = i + 1; 
        break;
      case ']':
        if( i > nStart )
        {
          m_bufInput[i] = 0;
          parseMeasurement(nStart);
        }
        else
        {
          nStart = UssMaxLineLen;
        }
        break;
      case 0:
        return;
      default:
        break;
    }
  }
}

void JennyUss::parseMeasurement(int nStart)
{
  int     ussId;
  double  ussMeas;
  int     n;

  if( strlen(m_bufInput) == 0 )
  {
    return;
  }

  //fprintf(stderr, "DBG: 2-tuple %s\n", &m_bufInput[nStart]);

  n = sscanf(&m_bufInput[nStart], "%d,%lf", &ussId, &ussMeas);

  if( n < 2 )
  {
    return;
  }
  else if( (ussId >= 0) && (ussId < NumOfUss) )
  {
    m_vecMeas[ussId] = ussMeas;
  } 
}

void JennyUss::getUssData(vector<double> &vecMeas)
{
  lockIo();

  vecMeas = m_vecMeas;

  unlockIo();
}
