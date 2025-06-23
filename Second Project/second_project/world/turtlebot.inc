define kinect ranger
(
  sensor
  (
    pose [0.0 0.0 0.10 0.0]
    size [0.1 0.1 0.1]
    range [0 6.5]
    fov 180.0
    samples 180
  )
  color "black"
  size [0.06 0.15 0.03]
)

define turtlebot position
(
  pose [0.0 0.0 0.0 0.0]
  localization "odom"
  odom_error [0.05 0.05 0.0 0.1]
  size [0.54 0.40 0.40]
  origin [0.0 0.0 0.0 0.0]
  gui_nose 1
  drive "diff"
  color "grey"
  kinect()
)
