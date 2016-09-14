////////////////////////////////////////////////////////////////////////////////
//
// Package:   Jenny Autonmous Grocery Cart ROS Package
//
// Link:      https://github.com/roadnarrows-robotics/jenny
//
// ROS Node:  jenny_controller
//
// File:      jenny_controller_main.cpp
//
/*! \file
 *
 * \brief The ROS jenny_controller node class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 */
////////////////////////////////////////////////////////////////////////////////

//
// System
//
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <sstream>

//
// ROS 
//
#include "ros/ros.h"

//
// RoadNarrows
//
#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"

//
// Jenny embedded jenny library.
//
#include "jenny/jenny.h"

//
// Node headers.
//
#include "jenny_controller.h"

using namespace ::std;
using namespace jenny;
using namespace jenny_controller;


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Node Specific Defines and Data
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

//
// Application exit codes
//
#define APP_EC_OK   0   ///< success
#define APP_EC_INIT 2   ///< initialization fatal error
#define APP_EC_EXEC 4   ///< execution fatal error

#define NO_SIGNAL   0   ///< no signal receieved value

//
// Data
//
const char *NodeName  = "jenny_controller"; ///< this ROS node's name
static int  RcvSignal = NO_SIGNAL;          ///< received 'gracefull' signal



//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// RoadNarrows Specific Defines and Data
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

//
// Options
//

/*!
 * \brief The package information.
 *
 * For ROS nodes, RN package information is equivalent to this ROS application
 * information.
 * */
static const PkgInfo_T PkgInfo =
{
  NodeName,                       ///< package name
  "1.0.0",                        ///< package version
  "2016.09.14",                   ///< date (and time)
  "2016",                         ///< year
  NodeName,                       ///< package full name
  "Robin Knight",                 ///< authors
  "Divide By Zero",               ///< owner
  "(C) 2016 Dived By Zero"        ///< disclaimer
};

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T AppPgmInfo =
{
  // usage_args
  "[ROSOPTIONS]",

  // synopsis
  "The %P ROS node provides ROS control interfaces to the Jenny autonomous "
  "grocery cart",
  
  // long_desc 
  "",
 
  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T AppOptsInfo[] =
{
  {NULL, }
};

/*!
 * \brief Signal handler to allow graceful shutdown of ROS node.
 *
 * \note This handler overrides the roscpp SIGINT handler.
 *
 * \param sig   Signal number.
 */
static void sigHandler(int sig)
{
  RcvSignal = sig;

  // All the default sigint handler does is call shutdown()
  //ros::shutdown();
}

/*!
 *  \brief ROS Jenny controller node main.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \return Returns exit code.
 */
int main(int argc, char *argv[])
{
  string  strNodeName;    // ROS-given node name
  double  hz = 30.0;      // ROS loop rate
  int     nMaxTries = 5;  // maximum tries
  int     rc;             // return code

  // 
  // Initialize the node. Parse the command line arguments and environment to
  // determine ROS options such as node name, namespace and remappings.
  // This call does not contact the master. This lets you use
  // ros::master::check() and other ROS functions after calling ros::init()
  // to check on the status of the master.
  //
  ros::init(argc, argv, NodeName);

  //
  // Parse node-specific options and arguments (from librnr).
  //
  OptsGet(NodeName, &PkgInfo, &AppPgmInfo, AppOptsInfo, true, &argc, argv);
 
  //
  //
  // A ctrl-c interrupt will stop attempts to connect to the ROS core.
  //
  ros::NodeHandle nh(NodeName);

  // actual ROS-given node name
  strNodeName = ros::this_node::getName();

  //
  // Failed to connect.
  //
  if( !ros::master::check() )
  {
    // add optional non-ROS unit tests here, then simply exit.
    return APP_EC_OK;
  }

  ROS_INFO("%s: Node started.", strNodeName.c_str());
  
  //
  // Create a jenny node object.
  //
  JennyController  jenny(nh, hz);

  //
  // Connect to the Jenny.
  //
  for(int i = 0; i < nMaxTries; ++i)
  {
    if( (rc = jenny.connect()) == JEN_OK )
    {
      stringstream ss;
      ss  << strNodeName << ": Connected to Jenny.";  
      ROS_INFO("%s", ss.str().c_str());
      LOGDIAG1("%s", ss.str().c_str());
      break;
    }
    else
    {
      stringstream ss;
      ss  << strNodeName
          << ": Failed to connect to Jenny. Error code="
          << rc;
      ROS_ERROR("%s", ss.str().c_str());
      LOGERROR("%s", ss.str().c_str());
      sleep(5);
    }
  }

  if( rc != JEN_OK )
  {
    stringstream ss;
    ss  << strNodeName.c_str()
        << "Failed to connect to Jenny after "
        << nMaxTries
        << " tries - giving up.";
    ROS_FATAL("%s", ss.str().c_str());
    LOGERROR("%s", ss.str().c_str());
    return APP_EC_INIT;
  }

  //
  // Signals
  //

  // Override the default ros sigint handler. This must be set after the first
  // NodeHandle is created.
  signal(SIGINT, sigHandler);

  // try to end safely with this signal
  signal(SIGTERM, sigHandler);

  //
  // Advertise services.
  //
  jenny.advertiseServices();

  ROS_INFO("%s: Services registered.", strNodeName.c_str());

  //
  // Advertise publishers.
  //
  jenny.advertisePublishers();
  
  ROS_INFO("%s: Publishers registered.", strNodeName.c_str());
  
  //
  // Subscribed to topics.
  //
  jenny.subscribeToTopics();
  
  ROS_INFO("%s: Subscribed topics registered.", strNodeName.c_str());

  //
  // Create Action Servers
  //

  //ROS_INFO("%s: Action servers created.", strNodeName.c_str());

  // set loop rate in Hertz
  ros::Rate loop_rate(hz);

  ROS_INFO("%s: Ready.", strNodeName.c_str());

  //
  // Main loop.
  //
  while( (RcvSignal == NO_SIGNAL) && ros::ok() )
  {
    // make any callbacks on pending ROS events
    ros::spinOnce(); 

    // publish all advertized topics
    jenny.publish();

    // sleep to keep at loop rate
    loop_rate.sleep();
  }

  return APP_EC_OK;
}
