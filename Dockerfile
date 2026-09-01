###############################################################################
# rosbag2_dynamic_recorder — development container
#
# Target: rosbag2 on ROS 2 Rolling.
#
# This is a DEV image: it includes rosbag2 source, its build toolchain, the
# test dependencies, and a non-root "rosbag" user so you can iterate on code
# and tests without polluting the core install.
###############################################################################

# ROS 2 Rolling base with the ros-core toolchain (no GUI)
FROM ros:rolling-ros-core

# --- Basic runtime deps for building/testing ROS 2 packages ----------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-rosinstall-generator \
        python3-vcstool \
        git \
        curl \
        ca-certificates \
        locales \
        sudo \
        build-essential \
        cmake \
        doxygen \
        gdb \
        # rosbag2 test/runtime deps that aren't in ros-core on some distros
        python3-pytest \
        python3-pytest-cov
# NOTE: deliberately NOT running `rm -rf /var/lib/apt/lists/*` here.
# This is a dev image, and without the package lists any later `apt-get install` --
# including the ones `rosdep install` shells out to -- fails to locate every package.
# Worse, rosdep still exits 0 in that case, so the failure is silent and only shows up
# much later as a CMake "could not find yaml-cpp" error. Keeping the lists costs ~50MB.

# --- rosbag2 workspace dependencies -----------------------------------------
# The workspace source is mounted at runtime, so `rosdep install --from-paths src` cannot
# run at image build time. This is the set rosdep resolves for the rosbag2 tree; keeping it
# in the image means a recreated container is immediately usable instead of silently broken.
# Re-run `rosdep install -r -y --from-paths src --ignore-src` inside the container if the
# upstream dependency set changes.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libyaml-cpp-dev \
        libzstd-dev \
        liblz4-dev \
        libsqlite3-dev \
        libeigen3-dev \
        pybind11-dev \
        python3-pytest-timeout \
        ros-rolling-yaml-cpp-vendor \
        ros-rolling-keyboard-handler \
        ros-rolling-test-msgs \
        ros-rolling-example-interfaces \
        ros-rolling-ros-testing \
        ros-rolling-ament-cmake-clang-format

# --- mcap CLI (optional dev tool) -------------------------------------------
# Used by the spike's acceptance checks (`mcap info` / `mcap doctor`). Tolerated as a
# best-effort step so a release-URL change cannot break the whole image build.
ARG MCAP_VERSION=v0.0.50
RUN curl -fsSL -o /usr/local/bin/mcap \
      "https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2F${MCAP_VERSION}/mcap-linux-amd64" \
    && chmod +x /usr/local/bin/mcap \
    || echo "WARNING: mcap CLI download failed; install it manually if needed."

# --- Create a non-root developer user --------------------------------------
# NOTE: Ubuntu-based ROS images already ship an 'ubuntu' user at UID/GID 1000,
# so we use 1001 to avoid a collision with the base image's user.
ARG USERNAME=rosbag
ARG USER_UID=1001
ARG USER_GID=1001
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Source ROS 2 in every interactive shell (non-interactive shells must source manually)
RUN echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc

# --- Placeholder mount points -------------------------------------------------
# The rosbag2 source tree and a build/output dir are mounted at runtime so the
# image stays thin and the host edits are reflected live:
#   - ./src        -> /home/rosbag/ros2_ws/src   (rosbag2 + deps source)
#   - ./build      -> /home/rosbag/ros2_ws/build
#   - ./install    -> /home/rosbag/ros2_ws/install
#   - ./log        -> /home/rosbag/ros2_ws/log
# NOTE: no brace expansion here -- Docker RUN uses /bin/sh (dash on Ubuntu), which does not
# expand `{src,build,install,log}` and would create one directory with that literal name.
RUN mkdir -p /home/${USERNAME}/ros2_ws/src \
             /home/${USERNAME}/ros2_ws/build \
             /home/${USERNAME}/ros2_ws/install \
             /home/${USERNAME}/ros2_ws/log

# Default command: keep the container alive for interactive `docker exec` work
CMD ["bash", "-l"]
