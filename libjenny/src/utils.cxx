////////////////////////////////////////////////////////////////////////////////
//
// Package:   jenny
//
// Library:   libjenny
//
// File:      utils.cxx
//
//
/*! \file
 *
 * \brief Jenny utilities.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 */
////////////////////////////////////////////////////////////////////////////////


#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <libgen.h>
#include <stdio.h>
#include <errno.h>

#include <string>
#include <sstream>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "jenny/jenny.h"
#include "jenny/utils.h"

using namespace std;


//------------------------------------------------------------------------------
// Private Interface
//------------------------------------------------------------------------------

/*!
 * \ingroup libjenny
 * \brief Jenny Error Code String Table.
 *
 * Table is indexed by jenny error codes (see \ref lae_ecodes). Keep
 * in sync.
 */
static const char *EcodeStrTbl[] =
{
  "Ok",                                     ///< [JEN_OK]

  "Error",                                  ///< [JEN_ECODE_GEN]
  "System error",                           ///< [JEN_ECODE_SYS]
  "Internal error",                         ///< [JEN_ECODE_INTERNAL]
  "Bad value",                              ///< [JEN_ECODE_BAD_VAL]
  "Too big",                                ///< [JEN_ECODE_TOO_BIG]
  "Too small",                              ///< [JEN_ECODE_TOO_SMALL]
  "Value out-of-range",                     ///< [JEN_ECODE_RANGE]
  "Invalid operation",                      ///< [JEN_ECODE_BAD_OP]
  "Operation timed out",                    ///< [JEN_ECODE_TIMEDOUT]
  "Device not found",                       ///< [JEN_ECODE_NO_DEV]
  "No resource available",                  ///< [JEN_ECODE_NO_RSRC]
  "Resource busy",                          ///< [JEN_ECODE_BUSY]
  "Cannot execute",                         ///< [JEN_ECODE_NO_EXEC]
  "Permissions denied",                     ///< [JEN_ECODE_PERM]
  "Dynamixel chain or servo error",         ///< [JEN_ECODE_DYNA]
  "Video error",                            ///< [JEN_ECODE_VIDEO]
  "Bad format",                             ///< [JEN_ECODE_FORMAT]
  "BotSense error",                         ///< [JEN_ECODE_BOTSENSE]
  "File not found",                         ///< [JEN_ECODE_NO_FILE]
  "XML error",                              ///< [JEN_ECODE_XML]
  "Robot is in an alarmed state",           ///< [JEN_ECODE_ALARMED]
  "Operation interrupted",                  ///< [JEN_ECODE_INTR]
  "Robotic link(s) movement obstructed",    ///< [JEN_ECODE_COLLISION]
  "Robot emergency stopped",                ///< [JEN_ECODE_ESTOP]
  "Motor controller or motor error",        ///< [JEN_ECODE_MOT_CTLR]
  "I/O error",                              ///< [JEN_ECODE_IO]

  "Invalid error code"                      ///< [JEN_ECODE_BADEC]
};


//------------------------------------------------------------------------------
// Public Interface
//------------------------------------------------------------------------------

const char *jenny::getStrError(const int ecode)
{
  int ec = ecode >= 0 ? ecode : -ecode;

  if( ec >= arraysize(EcodeStrTbl) )
  {
    ec = JEN_ECODE_BADEC;
  }

  return EcodeStrTbl[ec];
}

uint_t jenny::strToVersion(const string &str)
{
  int   nMajor    = 0;
  int   nMinor    = 0;
  int   nRevision = 0;

  sscanf(str.c_str(), "%d.%d.%d", &nMajor, &nMinor, &nRevision);

  return JEN_VERSION(nMajor, nMinor, nRevision);
}

bool jenny::operator<(const struct timeval& lhs, const struct timeval& rhs)
{
  if( lhs.tv_sec < rhs.tv_sec )
  {
    return true;
  }
  else if( (lhs.tv_sec == rhs.tv_sec) && (lhs.tv_usec < rhs.tv_usec) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool jenny::operator==(const struct timeval& lhs, const struct timeval& rhs)
{
  return (lhs.tv_sec == rhs.tv_sec) && (lhs.tv_usec == rhs.tv_usec)?
            true: false;
}

bool jenny::operator>(const struct timeval& lhs, const struct timeval& rhs)
{
  return !((lhs < rhs) || (lhs == rhs));
}

struct timeval jenny::operator+(const struct timeval& op1,
                                  const struct timeval& op2)
{
  struct timeval sum = op1;

  sum.tv_sec  += op2.tv_sec;
  sum.tv_usec += op2.tv_usec;

  if( sum.tv_usec > MILLION )
  {
    ++sum.tv_sec;
    sum.tv_usec -= MILLION;
  }

  return sum;
}

struct timeval jenny::operator-(const struct timeval& op1,
                                  const struct timeval& op2)
{
  struct timeval diff;

  diff.tv_sec = op1.tv_sec - op2.tv_sec;

  if( op1.tv_usec >= op2.tv_usec)
  {
    diff.tv_usec = op1.tv_usec - op2.tv_usec;
  }
  else
  {
    --diff.tv_sec;
    diff.tv_usec = MILLION + op1.tv_usec - op2.tv_usec;
  }

  return diff;
}

long jenny::dt_usec(struct timeval& t1, struct timeval& t0)
{
  struct timeval  dt = t1 - t0;

  return (long)dt.tv_sec * MILLION + (long)dt.tv_usec;
}

double jenny::dt(struct timeval& t1, struct timeval& t0)
{
  if( t0 < t1 )
  {
    return (double)dt_usec(t1, t0) / (double)MILLION;
  }
  else
  {
    return -(double)dt_usec(t0, t1) / (double)MILLION;
  }
}

bool jenny::operator<(const struct timespec& lhs,
                        const struct timespec& rhs)
{
  if( lhs.tv_sec < rhs.tv_sec )
  {
    return true;
  }
  else if( (lhs.tv_sec == rhs.tv_sec) && (lhs.tv_nsec < rhs.tv_nsec) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool jenny::operator==(const struct timespec& lhs,
                         const struct timespec& rhs)
{
  return (lhs.tv_sec == rhs.tv_sec) && (lhs.tv_nsec == rhs.tv_nsec)?
            true: false;
}

bool jenny::operator>(const struct timespec& lhs,
                        const struct timespec& rhs)
{
  return !((lhs < rhs) || (lhs == rhs));
}

struct timespec jenny::operator+(const struct timespec& op1,
                                   const struct timespec& op2)
{
  struct timespec sum = op1;

  sum.tv_sec  += op2.tv_sec;
  sum.tv_nsec += op2.tv_nsec;

  if( sum.tv_nsec > BILLION )
  {
    ++sum.tv_sec;
    sum.tv_nsec -= BILLION;
  }

  return sum;
}

struct timespec jenny::operator-(const struct timespec& op1,
                                   const struct timespec& op2)
{
  struct timespec diff;

  diff.tv_sec = op1.tv_sec - op2.tv_sec;

  if( op1.tv_nsec >= op2.tv_nsec)
  {
    diff.tv_nsec = op1.tv_nsec - op2.tv_nsec;
  }
  else
  {
    --diff.tv_sec;
    diff.tv_nsec = BILLION + op1.tv_nsec - op2.tv_nsec;
  }

  return diff;
}

long long jenny::dt_nsec(struct timespec& t1, struct timespec& t0)
{
  struct timespec dt = t1 - t0;

  return (long long)dt.tv_sec * BILLION + (long long)dt.tv_nsec;
}

double jenny::dt(struct timespec& t1, struct timespec& t0)
{
  if( t0 < t1 )
  {
    return (double)dt_nsec(t1, t0) / (double)BILLION;
  }
  else
  {
    return -(double)dt_nsec(t0, t1) / (double)BILLION;
  }
}
 
string jenny::getRealDeviceName(const string &strDevName)
{
  char    buf[MAX_PATH+1];
  ssize_t len;

  //
  // Symbolic link.
  //
  if( (len = readlink(strDevName.c_str(), buf, MAX_PATH)) > 0 )
  {
    buf[len] = 0;

    // absollute path
    if( buf[0] == '/' )
    {
      string strRealDevName(buf);
      return strRealDevName;
    }

    // relative path
    else
    {
      char          s[strDevName.size()+1];
      stringstream  ss;

      strcpy(s, strDevName.c_str());

      char *sDirName = dirname(s);

      ss << sDirName << "/" << buf;

      return ss.str();
    }
  }

  //
  // Real device.
  //
  else
  {
    return strDevName;
  }
}
  
vector<string> &jenny::split(const string   &s,
                                 char           delim,
                                 vector<string> &elems)
{
  stringstream ss(s);
  string item;

  while( getline(ss, item, delim) )
  {
    elems.push_back(item);
  }
  return elems;
}

vector<string> jenny::split(const string &s, char delim)
{
  vector<string> elems;

  split(s, delim, elems);

  return elems;
}
