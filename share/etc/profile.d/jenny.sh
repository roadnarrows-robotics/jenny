###############################################################################
#
# /etc/profile.d/kuon.sh
#
# Odroid version
#
# RoadNarrows Laelaps global environment.
# 2016.06.22
#
###############################################################################

# ..............................................................................
# ROS environment
# ..............................................................................
if [ -f /opt/rnr_ros/indigo/devel/setup.sh ]
then
  . /opt/rnr_ros/indigo/devel/setup.sh
elif [ -f /prj/ros/indigo/devel/setup.sh ]
then
  . /prj/ros/indigo/devel/setup.sh
fi


# ..............................................................................
# Tune Parameter: ROS_MASTER_URI
#
# This environment variable determines where the ROS core services are located
# on the network, and whether roscore should be automatically started on-target
# during boot up.
#
# Syntax:
#   export ROS_MASTER_URI=uri
#
#   uri  ::= http://host[:port]
#   host ::= domain_name | ip_addr
#   port ::= NUMBER
# 
# Default:
#   ROS_MASTER_URI is unset (== ROS_MASTER_URI=http://localhost:11311)
# ..............................................................................
#export ROS_MASTER_URI=http://localhost:11311

# ..............................................................................
# Tune Parameter: LAELAPS_REMOTE_TELEOP
#
# This environment variable determines whether to automatically start the
# teleoperation related ROS nodes on-target during boot up.
#
# Syntax:
#   export LAELAPS_REMOTE_TELEOP=state
#
#   state ::= 0 | 1 | false | true
#
# Default:
#   LAELAPS_REMOTE_TELEOP is unset (== LAELAPS_REMOTE_TELEOP=false)
# ..............................................................................
#export LAELAPS_REMOTE_TELEOP=false

# ..............................................................................
# Tune Parameter: GSCAM_CONFIG
#
# ROS gstreamer configuration
# ..............................................................................
export GSCAM_CONFIG="v4l2src device=/dev/eecam ! video/x-raw-rgb,width=320,height=240,framerate=10/1 ! ffmpegcolorspace"


# ..............................................................................
# RoadNarrows rnmake environment.
# ..............................................................................
export RNMAKE_ARCH_DFT=odroid

# paths
export PYTHONPATH=$PYTHONPATH:/prj/lib/python2.7/site-packages:/usr/local/lib/python2.7/site-packages
export LD_LIBRARY_PATH=/usr/local/lib/rnr:$LD_LIBRARY_PATH
export PATH=/prj/bin:/usr/local/bin:$PATH


# ..............................................................................
# Useful functions.
# ..............................................................................

#
# Find ROS node executable library.
#
# Usage: find_ros_node <path> <package> <node>
#
find_ros_node()
{
  local IFS=":"
  for p in ${1}
  do
    node="${p}/${2}/${3}"
    if [ -n "${node}" -a -x ${node} ]
    then
      echo ${node}
      return
    fi
  done
  echo ""
}

#
# Check if ROS is running.
#
# Usage: is_ros_running
#
is_ros_running()
{
  rostopic list >/dev/null 2>&1
  return $?
}

#
# Check if ROS node is running.
#
# Usage: is_ros_node_running <node>
#
is_ros_node_running()
{
  local node
  node="$1"
  rosnode list 2>/dev/null | grep --quiet "${node}"
  return $?
}

#
# Check if roscore should be started locally on-target.
# 
do_start_roscore()
{
  if [ -z "${ROS_MASTER_URI}" ]
  then
    return 0
  fi
  rosmaster="${ROS_MASTER_URI##http://}"
  rosmaster="${rosmaster%%:*}"
  laename=$(/bin/hostname)
  laename=${laename:-kuon}
  laeaddr=$(/bin/hostname -I)
  laeaddr=${laeaddr:-127.0.1.1}
  case "${rosmaster}"
  in
    "")           return 0;;
    "${laename}") return 0;;
    "localhost")  return 0;;
    "${laeaddr}") return 0;;
    "127.0.1.1")  return 0;;
    "127.0.0.1")  return 0;;
    *)            return 1;;
  esac
}

#
# Check if teleoperation should be started locally on-target.
# 
do_start_teleop()
{
  if [ -z "${LAELAPS_REMOTE_TELEOP}" ]
  then
    return 0
  fi
  case "${LAELAPS_REMOTE_TELEOP}"
  in
    "true") return 1;;
    "1")    return 1;;
    *)      return 0;;
  esac
}

#
# Get the ROS master port number.
# 
get_ros_master_port()
{
  rosmaster="${ROS_MASTER_URI##http://}"
  rosport="${rosmaster##*:}"
  echo ${rosport:-11311}
}
