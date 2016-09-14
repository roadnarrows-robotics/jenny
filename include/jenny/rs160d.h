////////////////////////////////////////////////////////////////////////////////
//
// Package:   jenny
//
// library:   libjenny
//
// File:      rs160d.h
//
/*! \file
 *
 * \brief  Funtions for serial control of RS160D motor controller
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 */
////////////////////////////////////////////////////////////////////////////////


#ifndef _RS160D_H
#define _RS160D_H

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <errno.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/serdev.h"

#include "jenny/jenny.h"

//
// Limits and Ids
//
#define RS160D_MOTOR_LEFT_ID    0     ///< left motor id
#define RS160D_MOTOR_RIGHT_ID   1     ///< right motor id
#define RS160D_MOTOR_AUX_ID     2     ///< auxilliary id

#define RS160D_MOTOR_SPEED_MIN  (-249) ///< min speed value = max reverse speed
#define RS160D_MOTOR_SPEED_MAX  249   ///< max speed value = max forward speed
#define RS160D_MOTOR_BRAKE_MIN  0     ///< min brake value = coasting
#define RS160D_MOTOR_BRAKE_MAX  31    ///< max brake value = max braking
#define RS160D_MOTOR_SLEW_MIN   0     ///< min pwr slew value = quickest ramp up
#define RS160D_MOTOR_SLEW_MAX   90    ///< max pwr slew value = slowest ramp up

/*
  Strings that set RS160D motor controllers to PWM mode
*/
#define SETLEFTPWM      "@0sm1\r"
#define SETRIGHTPWM     "@1sm1\r"

/*
  Strings that set RS160D motor controllers to bo controlled over
  serial. Need to be set for ANY computer control.
*/
#define SETLEFTSERIAL   "@0sj0\r"
#define SETRIGHTSERIAL  "@1sj0\r"

/*
  Default BAUD rate for RS160D motor controllers.
  This rate can be changed in RS160D registers.
*/
#define BAUDRATE        38400

struct RS160Ds
{
  int m_fdFront;
  int m_fdRear;
};

extern int RS160DOpen(const char *sDevice);

/*
  Function opens serial comunication with set port.
  (No actual communication with RS160D controllers in this function,
  USB->Serial adapters must be attached for success)
  Needs to be passed: Dev address (/dev/ttyUSB0 ....)
                      and a descriptor to modify/fill
  Returns: 0 on success and -1 on failure (0 or -1) 
*/
int RS160DOpenConnection(const char *Dev, int *Descriptor); 

/*
  Function sets RS160D controllers to be controlled via serial
  Needs to be passed: File descriptors 1 at a time.
  Returns: 0 on Success, -1 on Failure
*/
int RS160DSetToSerial(int Descriptor);

/*
  Function updates the speed of a single wheel.
  Needs to be passed: A speed (-250 < speed < 250), a file descriptor,
                      and a motor (0 = left, 1 = right)
  Returns: 0 on Success, -1 on incorrect speed, -2 on serial write fail.
*/
int RS160DUpdateMotorSpeeds(int Speed, int Descriptor, int Side);


/*
  Function changes how agressively each individual motor brakes.
  Needs to be passed: A brake rate (0 < Braking < 31: 31 = full brake),
                      a file descriptor, and a motor)
  Returns: 0 on Success, -1 on incorrect speed, -2 on serial write fail.
*/
int RS160DAlterBraking(int Braking, int Descriptor, int Side); 


/*
  Function changes how agressively each individual motor changes dirrection.
  Needs to be passed: A slew rate (0 < slew < 90: 0 = immediate change),
                      a file descriptor, and a motor)
  Returns: 0 on Success, -1 on incorrect speed, -2 on serial write fail.
*/
int RS160DAlterSlew(int Slew, int Descriptor, int Side);


/*
  E-Stop Function to lock up all wheels ICE
  Needs to be passed: Both descriptors.
*/
void RS160DEStop(int Descriptor);

/*
  Closing function to release a connection to a device.
*/
int RS160DClose(int Descriptor);

#endif // _RS160D_H
