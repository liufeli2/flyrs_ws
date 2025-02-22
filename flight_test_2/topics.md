jetson@nano:~$ ros2 topic info -v "/mavros/imu/data"
Type: sensor_msgs/msg/Imu

Publisher count: 1

Node name: imu
Node namespace: /mavros
Topic type: sensor_msgs/msg/Imu
Endpoint type: PUBLISHER
GID: 01.0f.d7.4b.81.50.ad.86.01.00.00.00.00.00.8d.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Subscription count: 0

--------------------------

jetson@nano:~$ ros2 topic info -v "/mavros/vision_pose/pose"
Type: geometry_msgs/msg/PoseStamped

Publisher count: 0

Subscription count: 1

Node name: vision_pose
Node namespace: /mavros
Topic type: geometry_msgs/msg/PoseStamped
Endpoint type: SUBSCRIPTION
GID: 01.0f.d7.4b.81.50.ad.86.01.00.00.00.00.01.1c.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

---------------------------
jetson@nano:~$ ros2 topic info -v "/mavros/setpoint_position/local"
Type: geometry_msgs/msg/PoseStamped

Publisher count: 0

Subscription count: 1

Node name: setpoint_position
Node namespace: /mavros
Topic type: geometry_msgs/msg/PoseStamped
Endpoint type: SUBSCRIPTION
GID: 01.0f.d7.4b.81.50.ad.86.01.00.00.00.00.00.c9.04.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

----------------------------

jetson@nano:~$ ros2 topic info -v "/vicon/ROB498_Drone/ROB498_Drone"
Type: geometry_msgs/msg/PoseStamped

Publisher count: 1

Node name: vicon_client
Node namespace: /
Topic type: geometry_msgs/msg/PoseStamped
Endpoint type: PUBLISHER
GID: 01.0f.9f.e0.8b.3a.12.2a.01.00.00.00.00.00.1a.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Subscription count: 0

----------------------------

jetson@nano:~$ ros2 topic info -v "/camera/pose/sample"
Type: nav_msgs/msg/Odometry

Publisher count: 1

Node name: camera
Node namespace: /camera
Topic type: nav_msgs/msg/Odometry
Endpoint type: PUBLISHER
GID: 01.0f.8e.f2.1c.25.39.24.01.00.00.00.00.00.1c.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
  Durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Subscription count: 0

