
## Be Able to Read VICON
- Connect the Jetson to the correct Wifi (shouldn't be a problem now that we have a working dongle). The SSID is `TP_LINK_ROB498` and the password is `rob498drones`.
- Verify the following: 1) Wifi tab, 2) **IPv4** tab, 3) Method is `Manual`, 4) **Address** box is `10.42.0.101`, 5) toggle to make sure setting are applied, 6) Run `ifconfig` command to confirm these details.
- New Terminal Window: `ros2 topic list`, in this you should see a VICON thing.
- If not, ask the TA (they also need to have it on on their end)
- `ros2 topic echo /vicon/ROB498_Drone/ROB498_Drone` should now to read vicon sensor info. 

## Be able to interface with MAVROS

- In a terminal, launch MAVROS by running `ros2 launch mavros.launch.py`, which used to be `ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:921600`.


### Check These
- If the quadrotor is manually moved forward, the pose reported by motion capture system should change accordingly (translation.x field of /vicon/ROB498_Drone/ROB498_Drone should increase). Similarly roll, pitch, and yaw angle changes need to be verified by manually moving the quadrotor.



