# install install-helper package from clearpath
sudo apt-get install ros-kinetic-robot-upstart

# install YOUR launch file [within the src folder]
#rosrun robot_upstart install eurobot_bringup/launch/minimal.launch
rosrun robot_upstart install eurobot_bringup/launch/minimal.launch --job eurobot_bringup --user root --rosdistro kinetic --logdir eurobot_bringup/logs

# to complete the installation
sudo systemctl daemon-reload && sudo systemctl start eurobot


# to reload use
sudo service eurobot start
sudo service eurobot stop


# to view output to terminal
sudo tail /var/log/upstart/eurobot.log -n 30
