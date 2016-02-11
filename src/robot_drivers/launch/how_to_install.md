<!-- rosrun robot_upstart install robot_drivers/launch/eurobot_task.launch --job eurobot_task --user root --rosdistro kinetic --logdir robot_drivers/logs -->
<!-- sudo systemctl daemon-reload && sudo systemctl start eurobot_task -->


rosrun robot_upstart install eurobot_bringup/launch/eurobot_final.launch --job eurobot_final --user root --rosdistro kinetic --logdir eurobot_bringup/logs