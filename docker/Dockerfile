# Image with the project's own nodes: click teleop, gestures, dashboard. The robot
# and camera drivers run in a separate container (docker/Dockerfile.drivers).
# Built by compose.yaml; can also be built alone from the repository root:
#   docker build -f docker/Dockerfile -t fossbot_control_interface:jazzy .
FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

WORKDIR /ws

# Dependencies first, so editing the code does not reinstall them.
COPY fossbot_control_interface/package.xml src/fossbot_control_interface/package.xml
RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*

# MediaPipe for gesture control. Kept on the system NumPy (matplotlib and the
# system OpenCV are compiled against it) and without the OpenCV wheel it pulls
# in: a second OpenCV with its own Qt next to PyQt5 is asking for trouble.
# libgles2/libegl1 are needed by the MediaPipe Tasks runtime even on CPU.
ARG MEDIAPIPE_VERSION=1.2.3
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip libgles2 libegl1 curl \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --break-system-packages --no-cache-dir \
      "mediapipe==${MEDIAPIPE_VERSION}" \
      "numpy==$(python3 -c 'import numpy; print(numpy.__version__)')" \
    && pip uninstall --break-system-packages -y opencv-contrib-python \
    && python3 -c "import cv2, mediapipe; assert cv2.__file__.startswith('/usr/lib/')"

# Hand landmarker model, at the default path of the gesture_control node
RUN curl -fsSL -o /root/hand_landmarker.task \
    https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task

COPY fossbot_control_interface src/fossbot_control_interface
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-select fossbot_control_interface

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "fossbot_control_interface", "interface.launch.py"]
