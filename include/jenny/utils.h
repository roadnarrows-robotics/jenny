////////////////////////////////////////////////////////////////////////////////
//
// Package:   jenny
//
// Library:   libjenny
//
// File:      utils.h
//
//
/*! \file
 *
 * \brief Common utilities.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _UTILS_H
#define _UTILS_H

#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "jenny/jenny.h"

#define M_TAU (2.0 * M_PI)    ///< tau = 2 * pi

namespace jenny
{
  const long      MILLION = 1000000;
  const long long BILLION = 1000000000;


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // String functions.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Convert version dotted string to integer equivalent.
   *
   * \param str   Dotted version string "M[.m[.R]]".
   *
   * \return Version number.
   */
  extern uint_t strToVersion(const std::string &str);

  /*!
   * \brief Get the error string describing the \h_laelaps error code.
   *
   * The absolute value of the error code is taken prior retrieving the string.
   * An unknown or out-of-range error code will be mapped to
   * \ref LAE_ECODE_BADEC.
   *
   * \param  ecode  Instance of \ref lae_ecodes.
   *
   * \return Returns the appropriate error code string.
   */
  extern const char *getStrError(const int ecode);

  /*!
   * \brief Boolean to string.
   *
   * \param b   Boolean value.
   *
   * \return Pointer to null-terminated constant character string.
   */
  inline const char *boolstr(bool b)
  {
    return b? "true": "false";
  }


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Math functions.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Convert degrees to radians
   *
   * \param d   Degrees.
   *
   * \return Radians.
   */
  inline double degToRad(double d)
  {
    return d / 360.0 * M_TAU;
  }

  /*!
   * \brief Convert radians to degrees
   *
   * \param r   Radians.
   *
   * \return Degrees.
   */
  inline double radToDeg(double r)
  {
    return r / M_TAU * 360.0;
  }

  /*!
   * \brief Integer absolute value.
   *
   * \param a   Integer value.
   *
   * \return |a|
   */
  inline int iabs(int a)
  {
    return a>=0? a: -a;
  }

  /*!
   * \brief Cap value within limits [min, max].
   *
   * \param a     Value.
   * \param min   Minimum.
   * \param max   Maximum.
   *
   * \return a: min \h_le a \h_le max
   */
  inline double fcap(double a, double min, double max)
  {
    return a<min? min: a>max? max: a;
  }

  /*!
   * \brief Cap value within limits [min, max].
   *
   * \param a     Value.
   * \param min   Minimum.
   * \param max   Maximum.
   *
   * \return a: min \h_le a \h_le max
   */
  inline int cap(int a, int min, int max)
  {
    return a<min? min: a>max? max: a;
  }

  /*!
   * \brief Cap value within limits [min, max].
   *
   * \param a     Value.
   * \param min   Minimum.
   * \param max   Maximum.
   *
   * \return a: min \h_le a \h_le max
   */
  inline uint_t cap(uint_t a, uint_t min, uint_t max)
  {
    return a<min? min: a>max? max: a;
  }

  /*!
   * \brief Sign of a.
   *
   * \return Return 1.0 or -1.0.
   */
  inline double signof(double a)
  {
    return a<0? -1.0: 1.0;
  }

  /*!
   * \brief Sign of a.
   *
   * \return Return 1 or -1.
   */
  inline int signof(int a)
  {
    return a<0? -1: 1;
  }


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // timeval functions.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  typedef struct timeval timeval_t;     ///< typedef'ed timeval structure
  typedef struct timespec timespec_t;   ///< typedef'ed timespec structure

  /*!
   * \brief Compare operator to test if left hand side time is earlier than
   * the right hand side time.
   *
   * lhs \h_lt rhs
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  bool operator<(const struct timeval& lhs, const struct timeval& rhs);

  /*!
   * \brief Compare operator to test if left hand side time equals
   * the right hand side time.
   *
   * lhs == rhs
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  bool operator==(const struct timeval& lhs, const struct timeval& rhs);

  /*!
   * \brief Compare operator to test if left hand side time is later than
   * the right hand side time.
   *
   * lhs \h_gt rhs
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  bool operator>(const struct timeval& lhs, const struct timeval& rhs);

  /*!
   * \brief Addition operator.
   *
   * op1 + op2
   *
   * \param op1   Left hand side time.
   * \param op2   Right hand side time.
   *
   * \return Sum of times.
   */
  struct timeval operator+(const struct timeval& op1,
                           const struct timeval& op2);

  /*!
   * \brief Subtraction operator.
   *
   * op1 - op2, op1 \h_ge op2.
   *
   * \param op1   Left hand side time.
   * \param op2   Right hand side time.
   *
   * \return Difference of times.
   */
  struct timeval operator-(const struct timeval& op1,
                           const struct timeval& op2);

  /*!
   * \brief Calculate delta time in microseconds.
   *
   * t1 - t0, t1 \h_ge t0
   *
   * \param t1  Later time.
   * \param t0  Earlier time.
   *
   * \return Returns dt in microseconds.
   */
  long dt_usec(struct timeval& t1, struct timeval& t0);

  /*!
   * \brief Calculate delta time.
   *
   * t1 - t0
   *
   * \param t1  Time 1.
   * \param t0  Time 0.
   *
   * \return Returns dt as a double. May be negative.
   */
  double dt(struct timeval& t1, struct timeval& t0);


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // timespec functions.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Compare operator to test if left hand side time is earlier than
   * the right hand side time.
   *
   * lhs \h_lt rhs
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  bool operator<(const struct timespec& lhs, const struct timespec& rhs);

  /*!
   * \brief Compare operator to test if left hand side time equals
   * the right hand side time.
   *
   * lhs == rhs
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  bool operator==(const struct timespec& lhs, const struct timespec& rhs);

  /*!
   * \brief Compare operator to test if left hand side time is later than
   * the right hand side time.
   *
   * lhs \h_gt rhs
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  bool operator>(const struct timespec& lhs, const struct timespec& rhs);

  /*!
   * \brief Addition operator.
   *
   * op1 + op2
   *
   * \param op1   Left hand side time.
   * \param op2   Right hand side time.
   *
   * \return Sum of times.
   */
  struct timespec operator+(const struct timespec& op1,
                            const struct timespec& op2);

  /*!
   * \brief Subtraction operator.
   *
   * op1 - op2, op1 \h_ge op2.
   *
   * \param op1   Left hand side time.
   * \param op2   Right hand side time.
   *
   * \return Difference of times.
   */
  struct timespec operator-(const struct timespec& op1,
                            const struct timespec& op2);

  /*!
   * \brief Calculate delta time in nanoseconds.
   *
   * t1 - t0, t1 \h_ge t0
   *
   * \param t1  Later time.
   * \param t0  Earlier time.
   *
   * \return Returns dt in nanoseconds.
   */
  long long dt_nsec(struct timespec& t1, struct timespec& t0);
  
  /*!
   * \brief Calculate delta time.
   *
   * t1 - t0
   *
   * \param t1  Time 1.
   * \param t0  Time 0.
   *
   * \return Returns dt as a double. May be negative.
   */
  double dt(struct timespec& t1, struct timespec& t0);
  

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Miscellanea.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
  /*!
   * \brief Get real device name.
   *
   * If the given device name is a symbolic link, then the real device the link
   * references is returned. Otherwise the given device name is returned.
   *
   * \param strDevName  Given device name.
   *
   * \return String.
   */
  std::string getRealDeviceName(const std::string &strDevName);

    /*!
   * \brief Split string at the delimiter character.
   *
   * \param s     String to split.
   * \param delem Delimiter character.
   * \param [out] elems   Vector of split strings.
   *
   * \return Reference to vector of split strings.
   */
  std::vector<std::string> &split(const std::string &s,
                                  char delim,
                                  std::vector<std::string> &elems);

  /*!
   * \brief Split string at the delimiter character.
   *
   * \param s     String to split.
   * \param delem Delimiter character.
   *
   * \return Vector of split strings.
   */
  std::vector<std::string> split(const std::string &s, char delim);

  // ---------------------------------------------------------------------------
  // Class Dim
  // ---------------------------------------------------------------------------

  /*!
   * \brief Object width x height x length dimensions class.
   */
  class Dim
  {
  public:
    double  m_width;    ///< object width (meters)
    double  m_height;   ///< object height (meters)
    double  m_length;   ///< object length (meters)
    
    /*!
     * \brief Default constructor.
     */
    Dim()
    {
      clear();
    }

    /*!
     * \brief Initialization constructor.
     *
     * \param width     Object width (meters)
     * \param height    Object height (meters)
     * \param length    Object length (meters)
     */
    Dim(double width, double height, double length)
    {
      m_width   = width;
      m_height  = height;
      m_length  = length;
    }

    /*!
     * \brief Copy constructor.
     *
     * \param src   Source object.
     */
    Dim(const Dim &src)
    {
      m_width   = src.m_width;
      m_height  = src.m_height;
      m_length  = src.m_length;
    }

    /*!
     * \brief Destructor.
     */
    virtual ~Dim()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    Dim operator=(const Dim &rhs)
    {
      m_width   = rhs.m_width;
      m_height  = rhs.m_height;
      m_length  = rhs.m_length;

      return *this;
    }

    void clear()
    {
      m_width   = 0.0;
      m_length  = 0.0;
      m_height  = 0.0;
    }

  }; // class Dim


} // namespace jenny


#endif // _UTILS_H
