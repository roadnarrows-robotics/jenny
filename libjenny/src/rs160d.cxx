////////////////////////////////////////////////////////////////////////////////
//
// Package:   jenny
//
// library:   libjenny
//
// File:      rs160d.cxx
//
/*! \file
 *
 * \brief  Funtions for serial control of RS160D motor controller
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 */
////////////////////////////////////////////////////////////////////////////////

//
//  TODO: get speed, 
//  TODO: encoder value
//  TODO: ...
//

#include <stdlib.h>
#include <string.h>
#include <dirent.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "jenny/jenny.h"
#include "jenny/rs160d.h"

static int   KMotDevCnt = 0;
static char *KMotDev[2] = {NULL, NULL};

static int RS160GlobFilter(const struct dirent *name)
{
  //printf("%s\n", name->d_name);
  
  if( !strncmp(name->d_name, "kmot", strlen("kmot")) )
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

static void RS160Glob()
{
  struct  dirent **namelist;
  int     n;
  int     i;

  n = scandir("/dev", &namelist, RS160GlobFilter, alphasort);

  for(i=0; i<n && i<2; ++i)
  {
    KMotDev[i] = (char *)malloc(5+strlen(namelist[i]->d_name)+1);
    sprintf(KMotDev[i], "/dev/%s", namelist[i]->d_name);
    //fprintf(stderr, "DBG: [%d] %s\n", i, KMotDev[i]);
  }

  free(namelist);

  KMotDevCnt = i;
}

int RS160DOpen(const char *sDevice)
{
  int   fd;

  fd = SerDevOpen(sDevice, BAUDRATE, 8, 'N', 1, false, false);

  return fd;
}

int RS160DOpenConnection( const char *Dev, 
                      int *Descriptor) {
  if( KMotDevCnt == 0 )
  {
    RS160Glob();
  }

  if( !strcmp(Dev, "0") && (KMotDevCnt >= 1) )
  {
    Dev = KMotDev[0];
  }
  else if( !strcmp(Dev, "1") && (KMotDevCnt >= 2) )
  {
    Dev = KMotDev[1];
  }
  *Descriptor = SerDevOpen(Dev, BAUDRATE, 8, 'N', 1, false, false);
  if(*Descriptor < 0) {
    return -1;
  }
  return 0;
}

int WriteToSerial(const char *ControlString, int Descriptor) {
  size_t Length;
  ssize_t err;
  Length = strlen(ControlString);
  err = SerDevWrite(Descriptor, (byte_t *)ControlString, Length, 0);
  if(err < 0) {
    return -1;
  }
  if(err == 0) {
    return -2;
  }
  if(err != Length) {
    return -3;
  }
  return 0;
}

void SerWriteErrCheck( int err)
{
  if(err == -1) {
    LOGERROR("Complete serial write failure.");
  }
  if(err == -2) {
    LOGERROR("Nothing Written.");
  }
  if(err == -3) {
    LOGERROR("Incorrect number of bytes written.");
  }
}

int RS160DSetToSerial(int Descriptor)
{
  int     err;
  byte_t  buf[1028];
  ssize_t n;

  // get out cal
  WriteToSerial("\033", Descriptor);

  err = WriteToSerial( SETLEFTSERIAL, Descriptor);
  if(err < 0) {
    LOGERROR("Failed to set left motor to serial control.");
    SerWriteErrCheck(err);
    return -1;
  }
  err = WriteToSerial( SETRIGHTSERIAL, Descriptor);
  if(err < 0) {
    LOGERROR("Failed to set right motor to serial control.");
    SerWriteErrCheck(err);
    return -1;
  }
  err = WriteToSerial( SETLEFTPWM, Descriptor);
  if(err < 0) {
    LOGERROR("Failed to set left motor to PWM mode.");
    SerWriteErrCheck(err);
    return -1;
  }
  err = WriteToSerial( SETRIGHTPWM, Descriptor);
  if(err < 0) {
    LOGERROR("Failed to set right motor to PWM mode.");
    SerWriteErrCheck(err);
    return -1;
  }

  //WriteToSerial("p\r", Descriptor);

  //n = SerDevRead(Descriptor, buf, sizeof(buf)-1, 10000);

  //if( n > 0 )
  //{
  //  buf[n] = 0;
  //  LOGDIAG2("Motor ctlr short summary: %s\n", buf);
  //}

  return 0;
}

int RS160DUpdateMotorSpeeds(int Speed, int Descriptor, int Side)
{
  char BuffToMotor[20];
  int err;

  if(Speed > RS160D_MOTOR_SPEED_MAX)
  {
    LOGWARN("Speed (%i) too high.", Speed);
    Speed = RS160D_MOTOR_SPEED_MAX;
  }

  if(Speed < RS160D_MOTOR_SPEED_MIN)
  {
    LOGWARN("Speed (%i) too low.", Speed);
    Speed = RS160D_MOTOR_SPEED_MIN;
  }

  if( Side == RS160D_MOTOR_RIGHT_ID )
  {
    Speed = -Speed;
  }

  sprintf(BuffToMotor, "@%dst%d\r", Side, Speed);

  err = WriteToSerial(BuffToMotor, Descriptor);

  if(err < 0)
  {
    LOGERROR("Failed to update motor speed.");
    SerWriteErrCheck(err);
    return -2;
  }
  return 0;
}

int RS160DAlterBraking(int Braking, int Descriptor, int Side)
{
  char BuffToMotor[20];
  int err;

  if(Braking > RS160D_MOTOR_BRAKE_MAX)
  {
    LOGWARN("Brake rate (%i) too high.", Braking);
    Braking = RS160D_MOTOR_BRAKE_MAX;
  }

  if(Braking < RS160D_MOTOR_BRAKE_MIN)
  {
    LOGWARN("Brake rate (%i) too low.", Braking);
    Braking = RS160D_MOTOR_BRAKE_MIN;
  }

  sprintf(BuffToMotor, "@%dsB%d\r", Side, Braking);

  err = WriteToSerial(BuffToMotor, Descriptor);

  if(err < 0) {
    LOGERROR("Failed to update brake rate.");
    SerWriteErrCheck(err);
    return -2;
  }
  return 0;
}

int RS160DAlterSlew(int Slew, int Descriptor, int Side) {
  char BuffToMotor[20];
  int err;

  if(Slew > RS160D_MOTOR_SLEW_MAX)
  {
    LOGWARN("Slew rate (%i) too high.", Slew);
    Slew = RS160D_MOTOR_SLEW_MAX;
  }

  if(Slew < RS160D_MOTOR_SLEW_MIN)
  {
    LOGWARN("Slew rate (%i) too low.", Slew);
    Slew = RS160D_MOTOR_SLEW_MIN;
  }

  sprintf(BuffToMotor, "@%dsS%d\r", Side, Slew);
  err = WriteToSerial(BuffToMotor, Descriptor);

  if(err < 0) {
    LOGERROR("Failed to update slew rate.");
    SerWriteErrCheck(err);
    return -2;
  }
  return 0;
}

void RS160DEStop(int Descriptor)
{
  WriteToSerial( "@0sS0\r", Descriptor);
  WriteToSerial( "@1sS0\r", Descriptor);
  WriteToSerial( "@0sB31\r", Descriptor);
  WriteToSerial( "@1sB31\r", Descriptor);
  WriteToSerial( "@0st0\r", Descriptor);
  WriteToSerial( "@1st0\r", Descriptor);
}

int RS160DClose(int Descriptor) {
  int error;
  SerDevFIFOOutputFlush(Descriptor);
  error = SerDevClose(Descriptor);
  if(error == -1) {
    LOGERROR("Failed to close device.");
    return -1;
  }
  return 0;
}
