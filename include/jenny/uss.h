////////////////////////////////////////////////////////////////////////////////
//
// Package:   jenny
//
// File:      uss.h
//
/*! \file
 *
 * \brief Jenny UltraSonic Sensor array class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _USS_H
#define _USS_H

#include <pthread.h>

#include <string>

#include "rnr/rnrconfig.h"

#include "jenny/jenny.h"

/*!
 *  \brief The sensor namespace.
 */
namespace sensor
{
  // the uss namespace
  namespace uss
  {
    //--------------------------------------------------------------------------
    // USS Data
    //--------------------------------------------------------------------------

    /*!
     * \brief Axes indices.
     */
    enum UssId
    {
      Left        = 0,    ///< left uss
      FrontLeft   = 1,    ///< front left
      Front       = 2,    ///< front
      FrontRight  = 3,    ///< front right
      Right       = 4,    ///< right

      NumOfUss    = 5     ///< number of sensors
    };
  
    static const char  *UssDevNameDft   = "/dev/ttyACM0";
    static const int    UssBaudRateDft  = 115200;
  
    static const size_t UssMaxLineLen   = 80;   ///< include NL or NULL
    static const uint_t UssTimeoutUsec  = 5000; ///< 5 msec


    //--------------------------------------------------------------------------
    // JennyUss Class
    //--------------------------------------------------------------------------
    
    /*!
     * UltraSonic Sensor array class with serial interface.
     */
    class JennyUss
    {
    public:
      /*!
       * \brief Default constructor.
       */
      JennyUss();
  
      /*!
       * \brief Destructor.
       */
      virtual ~JennyUss();
  
      /*!
       * \bried Open connection to USS Arduino
       *
       * \param strDevName  Serial device name.
       * \param nBaudRate   Serial device baud rate.
       *
       * \copydoc doc_return_std
       */
      virtual int open(const std::string &strDevName = UssDevNameDft,
                       const int nBaudRate = UssBaudRateDft);
  
      /*!
       * \brief Close connection to motor controller.
       *
       * \copydoc doc_return_std
       */
      virtual int close();
  
      /*!
       * \brief Read USS measurements from Arduino.
       * 
       * \copydoc doc_return_std
       */
      virtual void readSensors();
  
      /*!
       * \brief Check if USS serial interface is open.
       *
       * \return Returns true or false.
       */
      virtual bool isOpen()
      {
        return m_fd >= 0? true: false;
      }

      /*!
       * \brief Get the total USS degress of freedom.
       *
       * \return DoF
       */
      virtual int numOfUSS()
      {
        return NumOfUss;
      }

      /*!
       * \brief Get USS device name.
       *
       * \return String.
       */
      std::string getDevName()
      {
        return m_strDevName;
      }

      /*!
       * \brief Get the last sensed USS data.
       *
       * \param [out] vecMeas Distance measurements.
       */
      virtual void getUssData(std::vector<double> &vecMeas);

    protected:
      //
      // Hardware interface.
      //
      std::string   m_strDevName;   ///< serial device name
      int           m_nBaudRate;    ///< device baudrate
      int           m_fd;           ///< opened device file descriptor

      //
      // Parse data
      //
      char    m_bufInput[UssMaxLineLen];
      int     m_nCursor;

      //
      // Sensor data
      //
      std::vector<double> m_vecMeas;    ///< uss measurements

      // mutual exclusions
      pthread_mutex_t m_mutexIo;    ///< low-level I/O mutex
  
      /*!
       * \brief Lock the shared I/O resource.
       *
       * The lock()/unlock() primitives provide for safe multi-threading.
       *
       * \par Context:
       * Any.
       */
      void lockIo()
      {
        pthread_mutex_lock(&m_mutexIo);
      }
    
      /*!
       * \brief Unlock the shared I/O resource.
       *
       * \par Context:
       * Any.
       */
      void unlockIo()
      {
        pthread_mutex_unlock(&m_mutexIo);
      }

      /*!
       * \brief Zero USS data.
       */
      void zeroData();

      /*!
       * \brief Parse input line.
       *
       * \par Input line example:
       *  [0, 157.48][1, 157.48][2, 157.48][3, 0.00][4, 157.48]<NULL>
       */
      void parseInput();

      /*!
       * \brief Parse measurement 2-tuple.
       *
       * \par Input example:
       *  1, 157.4<NULL>
       *
       * \param nStart  Starting point in received input buffer.
       */
      void parseMeasurement(int nStart);
    };

  } // namespace uss

} // namespace sensor


#endif // _USS_H
