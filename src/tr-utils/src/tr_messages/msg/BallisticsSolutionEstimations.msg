std_msgs/Header header
uint8 robot_type
float64 pitch
float64 yaw
float64[36] covariance   # from odometry propagation

# robot_type constants — mirror Panel.msg values
uint8 HERO = 1
uint8 INFANTRY = 3
uint8 SENTRY = 7
