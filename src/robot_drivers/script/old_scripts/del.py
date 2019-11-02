 def move_linear(self, displacement, target_speed=0.2, precision=2, should_avoid_obstacles=False, collision_distance=0.55, move_timeout=20, sensor_timeout=1000):
        rospy.sleep(0.5); self.set_motion(0.0, 0.0)

        target_speed = abs(target_speed)
        status = False

        start_position = copy.deepcopy(self.robot_position)
        end_position = copy.deepcopy(self.robot_position)

        end_position["x"] += math.sin(end_position["yaw"])  * displacement
        end_position["y"] += math.cos(end_position["yaw"])  * displacement
        
        total_distance_to_travel = self.get_distance(start_position, end_position)
        distance_pid_controller = PID(-1.0, 1.0, 10.0, 0, 1.5)

        def _f(x):
            return  round(x, 3)

        start_time = time.time()
        while not rospy.is_shutdown():
            
            if move_timeout > 0:
                if (time.time() - start_time) >= move_timeout:
                    break
            
            current_distance = self.get_distance(self.robot_position, start_position)
            scale = distance_pid_controller.calculate(total_distance_to_travel, current_distance)

            if (round(abs(distance_pid_controller.pid_impl.pre_error), precision) == 0):
                break

            if should_avoid_obstacles:
                access_key = ""
                if self.get_sign(displacement) >= 0:
                    access_key = "front"
                else:
                    access_key = "back"

                distance_remaining = distance_pid_controller.pid_impl.pre_error * displacement
                if not self.ensure_clear_path(access_key=access_key, collision_range=min(collision_distance, abs(distance_remaining)), timeout=sensor_timeout):
                    status = False

                    rospy.loginfo("Timed out while waiting for obstacle")
                    break

            linear_velocity  = scale * target_speed * self.get_sign(displacement)
            angular_velocity = 0.0 #((x_scale * target_speed) / self.base_width) * self.get_sign(y_scale)

            self.set_motion(linear_velocity, -angular_velocity)
            rospy.sleep(0.1)

        self.set_motion(0.0, 0.0); rospy.sleep(0.5)
        return status


     # if should_avoid_obstacles:
            #     access_key = ""
            #     if self.get_sign(yaw_scale) >= 0:
            #         access_key = "right"
            #     else:
            #         access_key = "left"

            #     distance_remaining = yaw_pid_controller.pid_impl.pre_error * angular_displacement
            #     if not self.ensure_clear_path(access_key=access_key, collision_range=min(collision_distance, abs(distance_remaining)), timeout=sensor_timeout):
            #         status = False

            #         rospy.loginfo("Timed out while waiting for obstacle")
            #         break