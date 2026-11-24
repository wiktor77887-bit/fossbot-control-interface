# Image with the FOSSBot control interface (click teleop + dashboard) and usb_cam.
# Built by compose.yaml; can also be built alone from the repository root:
#   docker build -f docker/Dockerfile -t fossbot_control_interface:jazzy .
FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

# usb_cam is optional for the package (launch argument start_camera), so it is
# not in package.xml; the container always has it for the laptop camera.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-usb-cam \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# Dependencies first, so editing the code does not reinstall them.
COPY fossbot_control_interface/package.xml src/fossbot_control_interface/package.xml
RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*

COPY fossbot_control_interface src/fossbot_control_interface
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-select fossbot_control_interface

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "fossbot_control_interface", "interface.launch.py"]
