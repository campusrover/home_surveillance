echo "********************************************"
echo "Copy this file to ~/.bashrc and edit it according to the instuctions below"
echo "********************************************"

echo "[running ~/.bashrc]"
source ~/rosutils/common_alias.bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Support for new bru mode setter
export BRU_MY_IP=$(myip)
export BRU_VPN_IP=$(myvpnip)

# Setting for simulation mode
$(bru mode onboard)
$(bru name vnc -m $(myvpnip))

# Setting for rafael mode
# $(bru mode real)
# $(bru name rafael -m 100.74.60.34)

# Setting for roba mode
# $(bru mode real)
# $(bru name roba -m 100.86.78.102) 

# Setting for robb mode
# $(bru mode real)
# $(bru name robb -m 100.99.186.125)
 
# Setting for robc mode
# $(bru mode real)
# $(bru name robc -m 100.117.252.97)

# Support for Linorobot
export LINOLIDAR="2wd"
export LINOBASE="ydlidar"
