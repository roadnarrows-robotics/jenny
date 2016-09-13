////////////////////////////////////////////////////////////////////////////////
//
// Package:   jenny
//
// File:      naze32imu.cxx
//
/*! \file
 *
 * \brief Naze32 Inertial Measurement Unit class implementation.
 *
 * The current Jenny uses the open-source CleanFlight firmware loaded
 * on a Naze32 controller. The interface is serial USB.
 *
 * \sa https://github.com/cleanflight
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
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
#include "jenny/naze32imu.h"

using namespace std;
using namespace jenny;
using namespace sensor::imu;
using namespace sensor::imu::msp;


//------------------------------------------------------------------------------
// Quaternion Class
//------------------------------------------------------------------------------

Quaternion Quaternion::operator=(const Quaternion &rhs)
{
  m_x = rhs.m_x;
  m_y = rhs.m_y;
  m_z = rhs.m_z;
  m_w = rhs.m_w;

  return *this;
}

void Quaternion::clear()
{
  m_x = 0.0;
  m_y = 0.0;
  m_z = 0.0;
  m_w = 1.0;
}

void Quaternion::convert(double phi, double theta, double psi)
{
  double half_phi, half_theta, half_psi;
  double cos_phi, sin_phi;
  double cos_theta, sin_theta;
  double cos_psi, sin_psi;

  half_phi   = phi / 2.0;
  half_theta = theta / 2.0;
  half_psi   = psi / 2.0;

  cos_phi   = cos(half_phi);
  sin_phi   = sin(half_phi);
  cos_theta = cos(half_theta);
  sin_theta = sin(half_theta);
  cos_psi   = cos(half_psi);
  sin_psi   = sin(half_psi);

  m_x = cos_phi*cos_theta*cos_psi + sin_phi*sin_theta*sin_psi;
  m_y = sin_phi*cos_theta*cos_psi - cos_phi*sin_theta*sin_psi;
  m_z = cos_phi*sin_theta*cos_psi + sin_phi*cos_theta*sin_psi;
  m_w = cos_phi*cos_theta*sin_psi - sin_phi*sin_theta*cos_psi;
}


//------------------------------------------------------------------------------
// JennyImu Virtual Base Class
//------------------------------------------------------------------------------
 
JennyImu::JennyImu(string strIdent) : m_strIdent(strIdent)
{
  m_fd = -1;

  zeroData();

  pthread_mutex_init(&m_mutexIo, NULL);
  pthread_mutex_init(&m_mutexOp, NULL);
}

JennyImu::~JennyImu()
{
  close();

  pthread_mutex_destroy(&m_mutexOp);
  pthread_mutex_destroy(&m_mutexIo);
}

int JennyImu::open(const string &strDevName, int nBaudRate)
{
  string  strDevNameReal;
  int     rc;

  lockIo();

  // Get the real device name, not any symbolic links.
  strDevNameReal  = getRealDeviceName(strDevName);

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

    LOGDIAG3("Opened interface to IMU on %s@%d.",
      strDevNameReal.c_str(), nBaudRate);

    rc = JEN_OK;
  }

  unlockIo();

  return rc;
}

int JennyImu::close()
{
  lockIo();

  if( m_fd >= 0 )
  {
    SerDevClose(m_fd);
    LOGDIAG3("Closed %s interface to IMU.", 
        m_strDevName.c_str());
  }

  m_strDevName.clear();
  m_nBaudRate = 0;
  m_fd        = -1;

  unlockIo();

  return JEN_OK;
}

void JennyImu::zeroData()
{
  for(int i = 0; i < NumOfAxes; ++i)
  {
    m_accelRaw[i] = 0;
    m_gyroRaw[i]  = 0;
    m_magRaw[i]   = 0;
    m_rpyRaw[i]   = 0;

    m_accel[i]    = 0.0;
    m_gyro[i]     = 0.0;
    m_mag[i]      = 0.0;
    m_rpy[i]      = 0.0;
  }

  m_quaternion.clear();
}

void JennyImu::compute()
{
  computeQuaternion();
  computeDynamics();
}

void JennyImu::computeQuaternion()
{
  m_quaternion.convert(m_rpy[ROLL], m_rpy[PITCH], m_rpy[YAW]);
}

void JennyImu::computeDynamics()
{
}

void JennyImu::exec()
{
  lockOp();

  if( readRawImu() == JEN_OK )
  {
    convertRawToSI();
    compute();
  }

  unlockOp();
}

void JennyImu::getRawInertiaData(int accel[], int gyro[])
{
  for(int i = 0; i < NumOfAxes; ++i)
  {
    accel[i] = m_accelRaw[i];
    gyro[i]  = m_gyroRaw[i];
  }
}

void JennyImu::getInertiaData(double accel[], double gyro[])
{
  for(int i = 0; i < NumOfAxes; ++i)
  {
    accel[i] = m_accel[i];
    gyro[i]  = m_gyro[i];
  }
}

void JennyImu::getMagnetometerData(double mag[])
{
  for(int i = 0; i < NumOfAxes; ++i)
  {
    mag[i] = m_mag[i];
  }
}

void JennyImu::getAttitude(double rpy[])
{
  for(int i = 0; i < NumOfAxes; ++i)
  {
    rpy[i] = m_rpy[i];
  }
}

void JennyImu::getAttitude(double &roll, double &pitch, double &yaw)
{
  roll  = m_rpy[ROLL];
  pitch = m_rpy[PITCH];
  yaw   = m_rpy[YAW];
}

void JennyImu::getQuaternion(Quaternion &q)
{
  q = m_quaternion;
}

void JennyImu::getImuData(double accel[], double gyro[], double mag[],
                        double rpy[], Quaternion &q)
{
  lockOp();

  getInertiaData(accel, gyro);
  getMagnetometerData(mag);
  getAttitude(rpy);
  getQuaternion(q);

  unlockOp();
}


//------------------------------------------------------------------------------
// JennyImuCleanFlight IMU class with Cleanflight firmware on a supported board.
//------------------------------------------------------------------------------

JennyImuCleanFlight::JennyImuCleanFlight() :
  JennyImu("CleanFlight Naze32 with MPU-6050 IMU sensor")
{
}

JennyImuCleanFlight::~JennyImuCleanFlight()
{
}

int JennyImuCleanFlight::readIdentity(string &strIdent)
{
  MspIdent  ident;
  int       rc;

  if( (rc = mspReadIdent(ident)) == JEN_OK )
  {
    stringstream  ss;

    ss << "CleanFlight v" << ident.m_uFwVersion
      << " Naze32 with MPU-6050 IMU sensor";

    m_strIdent  = ss.str();
    strIdent    = m_strIdent;
  }

  return rc;
}

int JennyImuCleanFlight::readRawImu()
{
  int   rc;

  if( (rc = mspReadRawImu()) == JEN_OK )
  {
    rc = mspReadAttitude();
  }

  return rc;
}

int JennyImuCleanFlight::readRawInertia()
{
  return mspReadRawImu();
}

int JennyImuCleanFlight::readRawRollPitchYaw()
{
  return mspReadAttitude();
}

int JennyImuCleanFlight::convertRawToSI()
{
  for(int i = 0; i < NumOfAxes; ++i)
  {
    m_accel[i] = m_accelRaw[i] * MspMpu6050RawToG * MspGToMPerSec2;
    m_gyro[i]  = degToRad(m_gyroRaw[i] * MspMpu6050RawToDegPerSec);

    if( i != YAW )
    {
      m_rpy[i]  = degToRad(m_rpyRaw[i] * MspAttitudeRawToDeg);
    }
  }
  m_rpy[YAW] = degToRad(m_rpyRaw[YAW]);

  return JEN_OK;
}

int JennyImuCleanFlight::mspReadIdent(MspIdent &ident)
{
  static const uint_t CmdId       = MspCmdIdIdent;
  static const size_t RspDataLen = 7;

  byte_t  rspData[RspDataLen];
  int     rc;

  lockIo();

  if( (rc = sendCmd(CmdId, NULL, 0)) == JEN_OK )
  {
    rc = receiveRsp(CmdId, rspData, RspDataLen);
  }

  if( rc == JEN_OK )
  {
    ident.m_uFwVersion  = (uint_t)rspData[0];
    ident.m_uMultiType  = (uint_t)rspData[1];
    ident.m_uMspVersion = (uint_t)rspData[2];
    unpack32(&rspData[3], ident.m_uCaps);
  }

  unlockIo();

  return rc;
}

int JennyImuCleanFlight::mspReadRawImu()
{
  static const uint_t CmdId       = MspCmdIdRawImu;
  static const size_t RspDataLen  = 18;

  byte_t  rspData[RspDataLen];
  int     i, n;
  int     rc;

  lockIo();

  if( (rc = sendCmd(CmdId, NULL, 0)) == JEN_OK )
  {
    rc = receiveRsp(CmdId, rspData, RspDataLen);
  }

  if( rc == JEN_OK )
  {
    for(i = 0, n = 0; i < NumOfAxes; ++i, n += 2)
    {
      unpack16(&rspData[n], m_accelRaw[i]);
    }
    for(i = 0; i < NumOfAxes; ++i, n += 2)
    {
      unpack16(&rspData[n], m_gyroRaw[i]);
    }
    for(i = 0; i < NumOfAxes; ++i, n += 2)
    {
      unpack16(&rspData[n], m_magRaw[i]);
    }
  }

  unlockIo();

  return rc;
}

int JennyImuCleanFlight::mspReadAttitude()
{
  static const uint_t CmdId       = MspCmdIdAttitude;
  static const size_t RspDataLen = 6;

  byte_t  rspData[RspDataLen];
  int     i, n;
  int     rc;

  lockIo();

  if( (rc = sendCmd(CmdId, NULL, 0)) == JEN_OK )
  {
    rc = receiveRsp(CmdId, rspData, RspDataLen);
  }

  if( rc == JEN_OK )
  {
    for(i = 0, n = 0; i < NumOfAxes; ++i, n += 2)
    {
      unpack16(&rspData[n], m_rpyRaw[i]);
    }
  }

  unlockIo();

  return rc;
}


int JennyImuCleanFlight::sendCmd(uint_t cmdId, byte_t cmdData[], size_t lenData)
{
  byte_t  cmd[MspCmdMaxLen];
  uint_t  chksum;
  size_t  n;

  for(n = 0; n<strlen(MspCmdPreamble); ++n)
  {
    cmd[n] = MspCmdPreamble[n];
  }

  chksum = 0;

  cmd[n++] = (byte_t)lenData;
  chksum  ^= (byte_t)lenData;

  cmd[n++] = (byte_t)cmdId;
  chksum  ^= (byte_t)cmdId;
  
  for(size_t i = 0; i < lenData; ++i)
  {
    cmd[n++] = (byte_t)cmdData[i];
    chksum  ^= (byte_t)cmdData[i];
  }

  cmd[n++] = (byte_t)chksum;

  if( SerDevWrite(m_fd, cmd, n, TCmdTimeout) == n )
  {
    return JEN_OK;
  }
  else
  {
    flush(TFlushDelay);
    LOGERROR("IMU: Cmd %d: Send failed.", cmdId);
    return -JEN_ECODE_IO;
  }
}

int JennyImuCleanFlight::receiveRsp(uint_t cmdId,
                                  byte_t rspData[],
                                  size_t lenData)
{
  byte_t  rsp[MspRspMaxLen];
  uint_t  chksum;
  size_t  lenRsp;
  size_t  n;
  size_t  fldSize;
  uint_t  fldCmdId;
  byte_t  fldChkSum;
  size_t  i;

  lenRsp = MspRspMinLen + lenData;

  if( (n = SerDevRead(m_fd, rsp, lenRsp, TRspTimeout)) < 0 )
  {
    LOGERROR("IMU: Cmd %d response: Receive failed.", cmdId);
    flush(TFlushDelay);
    return -JEN_ECODE_IO;
  }
  else if( n < lenRsp )
  {
    LOGERROR("IMU: Cmd %d response: Receive partial response: "
        "rcv'd %zu bytes, expected %zu bytes.", cmdId, n, lenRsp);
    flush(TFlushDelay);
    return -JEN_ECODE_IO;
  }

  fldSize = (size_t)rsp[MspFieldPosSize];

  if( fldSize != lenData )
  {
    LOGERROR("IMU: Cmd %d response: Data length mismatch: " \
          "Received %zu bytes, expected %zu bytes.", cmdId, fldSize, lenData);
    resyncComm();
    return -JEN_ECODE_IO;
  }

  fldCmdId = (uint_t)rsp[MspFieldPosCmdId];

  if( fldCmdId != cmdId )
  {
    LOGERROR("IMU: Cmd %d response: Command Id mismatch: Received %d.",
        cmdId, fldCmdId);
    resyncComm();
    return -JEN_ECODE_IO;
  }

  chksum  = rsp[MspFieldPosSize];
  chksum ^= rsp[MspFieldPosCmdId];

  for(i = 0, n = MspFieldPosDataStart; i < lenData; ++i, ++n)
  {
    chksum ^= rsp[n];
    rspData[i] = rsp[n];
  }

  fldChkSum = rsp[lenRsp-1];

  if( chksum != fldChkSum )
  {
    LOGERROR("IMU: Cmd %d response: Checksum mismatch: " \
          "Received 0x%02x, calculated 0x%02x.", cmdId, fldChkSum, chksum);
    resyncComm();
    return -JEN_ECODE_IO;
  }

  return JEN_OK;
}

void JennyImuCleanFlight::resyncComm()
{
  MspIdent  ident;
  int       nMaxTries = 3;
  int       nTries;
  int       rc;

  LOGDIAG3("IMU: Resynchronizing serial communication.");


  for(nTries = 0; nTries < nMaxTries; ++nTries)
  {
    flush(TFlushDelay);
    if( (rc = mspReadIdent(ident)) == JEN_OK )
    {
      return;
    }
  }

  LOGWARN("IMU: Failed to resynchronize communication in %d tries.", nMaxTries);
}

void JennyImuCleanFlight::flush(uint_t t)
{
  if( t > 0 )
  {
    usleep(t);
  }

  SerDevFIFOOutputFlush(m_fd);
  SerDevFIFOInputFlush(m_fd);
}

int JennyImuCleanFlight::pack16(uint_t val, byte_t buf[])
{
  buf[0] = (byte_t)(val & 0xff);
  buf[1] = (byte_t)((val >> 8) & 0xff);
  return 2;
}

int JennyImuCleanFlight::unpack16(byte_t buf[], uint_t &val)
{
  val = ((uint_t)(buf[1]) << 8) | (uint_t)buf[0];
  return 2;
}

int JennyImuCleanFlight::unpack16(byte_t buf[], int &val)
{
  s16_t v;
  v = ((s16_t)(buf[1]) << 8) | (s16_t)buf[0];
  val = (int)v;
  return 2;
}

int JennyImuCleanFlight::pack32(uint_t val, byte_t buf[])
{
  buf[0] = (byte_t)(val & 0xff);
  buf[1] = (byte_t)((val >>  8) & 0xff);
  buf[2] = (byte_t)((val >> 16) & 0xff);
  buf[3] = (byte_t)((val >> 24) & 0xff);
  return 4;
}

int JennyImuCleanFlight::unpack32(byte_t buf[], uint_t &val)
{
  val = ((uint_t)(buf[3]) << 24) |
        ((uint_t)(buf[2]) << 16) |
        ((uint_t)(buf[1]) <<  8) |
        (uint_t)buf[0];
  return 4;
}

int JennyImuCleanFlight::unpack32(byte_t buf[], int &val)
{
  s32_t v;

  v = ((s32_t)(buf[3]) << 24) |
      ((s32_t)(buf[2]) << 16) |
      ((s32_t)(buf[1]) <<  8) |
      (s32_t)buf[0];
  val = (int)v;
  return 4;
}
