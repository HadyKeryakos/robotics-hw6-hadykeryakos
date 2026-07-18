# robotics-hw6-hadykeryakos
to build: colcon build
dont forget to : source install/setup.bash
to run: ros2 launch perception_manager launch.py
to run action: ros2 action send_goal /start_detection perception_interfaces/action/StartDetection "{target_class: 'bottle'}" --feedback
to run service: ros2 service call /set_confidence perception_interfaces/srv/SetConfidenceThreshold "{threshold: 0.9}"