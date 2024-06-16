define kinect ranger
(
  sensor
  (
    pose [ 0.0 0.0 0.0 0.0 ]
    size [0.1 0.1 0.2]
    range  [0 6.5]
    fov 360
    samples 1440
  )
  # generic model properties
  color "black"
  size [ 0.0 0.0 0.0 ]
)

define scout position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  drive "omni"

  odom_error [0.1 0.1 0.1 0.1]
  
  size [0.612 0.580 0.245 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  color "gray"
  
  kinect(pose [ 0.0 0.0 -0.11 0.0 ])
)
