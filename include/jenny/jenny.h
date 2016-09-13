////////////////////////////////////////////////////////////////////////////////
//
// Package:   jenny
//
// File:      jenny.h
//
/*! \file
 *
 * \brief Top-level package include file.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _JENNY_H
#define _JENNY_H

/*!
 *  \brief The jenny namespace encapsulates all jenny related constructs.
 */
namespace jenny
{

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \ingroup jenny_h
   * \defgroup lae_ecodes  Jenny Error Codes
   *
   * Jenny package-wide error codes.
   *
   * \{
   */
  static const int JEN_OK               =  0; ///< not an error, success

  static const int JEN_ECODE_GEN        =  1; ///< general, unspecified error
  static const int JEN_ECODE_SYS        =  2; ///< system (errno) error
  static const int JEN_ECODE_INTERNAL   =  3; ///< internal error (bug)
  static const int JEN_ECODE_BAD_VAL    =  4; ///< bad value general error
  static const int JEN_ECODE_TOO_BIG    =  5; ///< value/list/size too big
  static const int JEN_ECODE_TOO_SMALL  =  6; ///< value/list/size too small
  static const int JEN_ECODE_RANGE      =  7; ///< value out-of-range
  static const int JEN_ECODE_BAD_OP     =  8; ///< invalid operation error
  static const int JEN_ECODE_TIMEDOUT   =  9; ///< operation timed out error
  static const int JEN_ECODE_NO_DEV     = 10; ///< device not found error
  static const int JEN_ECODE_NO_RSRC    = 11; ///< no resource available error
  static const int JEN_ECODE_BUSY       = 12; ///< resource busy error
  static const int JEN_ECODE_NO_EXEC    = 13; ///< cannot execute error
  static const int JEN_ECODE_PERM       = 14; ///< no permissions error
  static const int JEN_ECODE_DYNA       = 15; ///< dynamixel error
  static const int JEN_ECODE_VIDEO      = 16; ///< video error
  static const int JEN_ECODE_FORMAT     = 17; ///< bad format
  static const int JEN_ECODE_BOTSENSE   = 18; ///< botsense error
  static const int JEN_ECODE_NO_FILE    = 19; ///< file not found
  static const int JEN_ECODE_XML        = 20; ///< XML error
  static const int JEN_ECODE_ALARMED    = 21; ///< robot is alarmed
  static const int JEN_ECODE_INTR       = 22; ///< operation interrupted
  static const int JEN_ECODE_COLLISION  = 23; ///< robot link(s) in collision
  static const int JEN_ECODE_ESTOP      = 24; ///< robot emergency stopped
  static const int JEN_ECODE_MOT_CTLR   = 25; ///< motor controller error
  static const int JEN_ECODE_IO         = 26; ///< I/O error

  static const int JEN_ECODE_BADEC      = 27; ///< bad error code

  static const int JEN_ECODE_NUMOF      = 28; ///< number of error codes
  /*! \} */

  /*! \} */

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   *
   * \ingroup jenny_h
   * \defgroup lae_prod  Jenny Product Identifiers
   *
   * jenny product ids, names, and descriptions.
   *
   *  Products ids are classified by class, family, size, dof, and special 
   *  fields.
   *
   * \{
   */

  //
  // Product release status
  //
  #define JEN_PROD_GA       0x000   ///< product general availability
  #define JEN_PROD_BETA     0x001   ///< product beta version
  #define JEN_PROD_ALPHA    0x002   ///< product alpha version
  #define JEN_PROD_PROTO    0x003   ///< product prototype version

  //
  // Product classes
  //
  #define JEN_CLASS_MOBILE_BASE 0xBA000000    ///< base

  //
  // Product Ids and Names
  //
  // product family
  const char* const JenProdFamilyUnknown  = "?"; ///< unknown product family
  const char* const JenProdFamily         = "Jenny"; ///< product family name

  // product ids
  static const int  JenProdIdUnknown  = 0;  ///< unknown/undefined product id
  static const int  JenProdIdStd      = 1;  ///< standard Jenny product id
  static const int  JenProdIdLarge    = 2;  ///< large Jenny product id

  // product sizes
  const char* const JenProdModelStd   = "Standard"; ///< standard model
  const char* const JenProdModelLarge = "Large";    ///< future large model

  /*!
   * \brief Convert version triplet to integer equivalent.
   *
   * \param major     Major version number.
   * \param minor     Minor version number.
   * \param revision  Revision number.
   */
  #define JEN_VERSION(major, minor, revision) \
    ((((major)&0xff)<<24) | (((minor)&0xff)<<16) | ((revision)&0xffff))

  /*!
   * \brief Get version major number from version.
   *
   * \param ver       Version number.
   *
   * \return Major number.
   */
  #define JEN_VER_MAJOR(ver)    (((ver)>>24) & 0xff)

  /*!
   * \brief Get version minor number from version.
   *
   * \param ver       Version number.
   *
   * \return Minor number.
   */
  #define JEN_VER_MINOR(ver)    (((ver)>>16) & 0xff)

  /*!
   * \brief Get revision number from version.
   *
   * \param ver       Version number.
   *
   * \return Revision number.
   */
  #define JEN_VER_REV(ver)    ((ver) & 0xffff)
  /*! \} */

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   *
   * \ingroup jenny_h
   * \defgroup lae_env  Jenny Environment
   *
   * Jenny directories and files.
   *
   * \{
   */

  //
  // Images directory.
  //
  #ifdef JEN_IMAGE_DIR
  const char* const JenImageDir = JEN_IMAGE_DIR;  ///< image directory
  #else
  const char* const JenImageDir = "/usr/local/share/jenny/images";
                                                  ///< image directory
  #endif

  //
  // Icons directory.
  //
  #ifdef JEN_ICON_DIR
  const char* const JenIconDir = JEN_ICON_DIR;    ///< icon directory
  #else
  const char* const JenIconDir = "/usr/local/share/jenny/images/icons";
                                                  ///< icon directory
  #endif

  //
  // Paths
  //
 
    /*! \brief System configuration search path. */
  const char* const JenSysCfgPath  = "/etc/jenny:/etc";

  /*! \brief User configuration search path and inheritance order. */
  const char* const JenUserCfgPath = "/etc/jenny:~/.roadnarrows";

  //
  // Top-level configuration XML file basename.
  //
  #ifdef JEN_ETC_CFG
  const char* const JenEtcCfg = JEN_ETC_CFG;      ///< xml configuration file
  #else
  const char* const JenEtcCfg = "jenny.conf";
                                                  ///< xml configuration file
  #endif

  //
  // Tuning XML file basename.
  //
  #ifdef JEN_ETC_TUNE
  const char* const JenEtcTune = JEN_ETC_TUNE;    ///< xml tune file
  #else
  const char* const JenEtcTune = "jenny_tune.conf";
                                                  ///< xml tune file
  #endif

  //
  // Specific Jenny application configuration file basenames.
  //
  const char* const JenPanelXml = "jenny_panel.xml";    ///< control panel cfg
  const char* const JenFrontCamXml = "jenny_fcam.xml";  ///< front camera cfg
  const char* const JenXboxXml  = "jenny_xbox.xml";     ///< xbox teleop cfg

  //
  // XML stylesheet URL
  //
  #ifdef JEN_XSL_URL
  const char* const JenXslUrl = JEN_XSL_URL;      ///< xml stylesheet url
  #else
  const char* const JenXslUrl =
    "http://roadnarrows.com/xml/jenny/1.0/jenny.xsl";
                                                  ///< xml stylesheet url
  #endif

  //
  // XML schema instance URL
  //
  #ifdef JEN_XSI_URL
  const char* const JenXsiUrl = JEN_XSI_URL;      ///< xml schema instance url
  #else
  const char* const JenXsiUrl =
    "http://roadnarrows.com/xml/jenny/1.0/jenny.xsd";
                                                  ///< xml schema instance url
  #endif

  /*! \} */

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   *
   * \ingroup jenny_h
   * \defgroup lae_types  Jenny Common Simple Types
   *
   *  Common types used to control and report robot information.
   *  fields.
   *
   * \{
   */

  /*!
   * \brief jenny tri-state type.
   *
   * Basically, a tri-state value is either unknown, 0, or 1.
   */
  enum JenTriState
  {
    // unknown
    JenTriStateUnknown  = -1,     ///< unknown state

    // low state synonyms
    JenTriStateFalse    = 0,      ///< false
    JenTriStateOff      = 0,      ///< off
    JenTriStateDisabled = 0,      ///< disabled
    JenTriStateLow      = 0,      ///< low
    JenTriStateOpen     = 0,      ///< open
    JenTriStateDark     = 0,      ///< dark

    // high state synonyms
    JenTriStateTrue     = 1,      ///< true
    JenTriStateOn       = 1,      ///< on
    JenTriStateEnabled  = 1,      ///< enabled
    JenTriStateHigh     = 1,      ///< high
    JenTriStateClosed   = 1,      ///< closed
    JenTriStateLight    = 1       ///< light
  };

  /*!
   * \brief jenny mode of operation.
   *
   * Robot mode of operation.
   */
  enum JenRobotMode
  {
    JenRobotModeUnknown = -1,   ///< unknown mode state
    JenRobotModeManual  =  1,   ///< can only be operated locally, not remotely
    JenRobotModeAuto    =  2    ///< fully available
  };

  /*!
   * \brief Robot or joint operational states.
   */
  enum JenOpState
  {
    JenOpStateUncalibrated  = 0,    ///< uncalibrated
    JenOpStateCalibrating   = 1,    ///< calibrating
    JenOpStateCalibrated    = 2     ///< calibrated
  };

  /*!
   * \brief Asynchronous task state.
   */
  enum JenAsyncTaskState
  {
    JenAsyncTaskStateIdle     = 0,    ///< idle, no async task running
    JenAsyncTaskStateWorking  = 1     ///< async task running
  };

  /*!
   * \brief Length/Distance Norm
   */
  enum JenNorm
  {
    JenNormL1   = 1,    ///< L1 norm (taxicab or manhattan norm)
    JenNormL2   = 2,    ///< L2 norm (Euclidean norm)
    JenNormLinf = 3     ///< Linf norm (maximum, infinity, or supremum norm)
  };

  /*! \} */

} // namespace jenny


#endif // _JENNY_H
