### Laelaps laelaps_control/Velocity
#
# Powertrain velocities.
#
# Set powertrain target velocities. Any drives not specified remain at their
# respective current velocities. Angular velocity is measured at the fixed
# rotational drive_shaft - wheel - tire powertrain components.
#

# powertrain velocities
Header    header      # Standard header.
string[]  names       # Powertrain names: [left, right]
float64[] velocities  # Normalized angular drive shaft/wheel velocities [-1, 1]
