###############################################################################
# rosbag2_dynamic_recorder -- development container
#
# The same environment .github/workflows/test.yml builds and tests in, plus a shell to work from:
# an official ros-base image, the four apt packages CI adds, and the dependencies rosdep resolves
# from our own package.xml files. Nothing here is needed to *use* the project -- that is an apt
# install and a colcon build, see docs/install.md -- which is why this lives in .devcontainer/.
#
# ROS_DISTRO picks the target: jazzy, kilted or rolling. From the repository root:
#   docker compose -f .devcontainer/docker-compose.yml build
#   ROS_DISTRO=jazzy docker compose -f .devcontainer/docker-compose.yml build
###############################################################################

ARG ROS_DISTRO=rolling
FROM ros:${ROS_DISTRO}-ros-base
# A FROM line ends the scope of the ARGs before it; redeclare to use it below.
ARG ROS_DISTRO

# --- What CI installs, and a few things a person at a shell wants ------------
# colcon and pytest are not in ros-base. node is not a ROS dependency at all: the UI tests execute
# the page's own JavaScript and skip themselves when node is absent.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-pytest \
        nodejs \
        git \
        gdb \
        sudo \
        curl \
        ca-certificates
# NOTE: deliberately NOT running `rm -rf /var/lib/apt/lists/*` here. This is a dev image, and
# without the package lists any later `apt-get install` -- including the ones `rosdep install`
# shells out to -- fails to locate every package. Worse, rosdep still exits 0 in that case, so the
# failure is silent and only shows up much later as a CMake "could not find ..." error.

# --- Package dependencies, from the manifests -------------------------------
# Only the package.xml files are copied in, so this layer is rebuilt when a <depend> changes and
# not when code does. The source itself is bind-mounted at runtime. rosdep wants one package per
# directory, so each manifest is placed under a directory named after it.
COPY packages/*/package.xml /tmp/manifests/
RUN set -e; \
    for f in /tmp/manifests/*.xml; do \
      name=$(sed -n 's:.*<name>\(.*\)</name>.*:\1:p' "$f"); \
      mkdir -p "/tmp/rosdep/$name" && cp "$f" "/tmp/rosdep/$name/package.xml"; \
    done; \
    apt-get update; \
    for attempt in 1 2 3 4 5; do \
      rosdep update && break; \
      [ "$attempt" -eq 5 ] && exit 1; \
      sleep $((attempt * 10)); \
    done; \
    rosdep install --from-paths /tmp/rosdep --ignore-src -r -y; \
    rm -rf /tmp/manifests /tmp/rosdep

# --- mcap CLI (optional dev tool) -------------------------------------------
# For inspecting recorded bags by hand (`mcap info` / `mcap doctor`). Tolerated as a best-effort
# step so a release-URL change cannot break the whole image build.
ARG MCAP_VERSION=v0.0.50
RUN curl -fsSL -o /usr/local/bin/mcap \
      "https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2F${MCAP_VERSION}/mcap-linux-amd64" \
    && chmod +x /usr/local/bin/mcap \
    || echo "WARNING: mcap CLI download failed; install it manually if needed."

# --- Non-root developer user ------------------------------------------------
# Ubuntu-based ROS images already ship an 'ubuntu' user at UID/GID 1000, so 1001 avoids a
# collision with the base image's user.
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
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# --- Mount points -------------------------------------------------------------
# The compose file bind-mounts the repository's packages/ and the build output here, so host
# edits are reflected live and the image stays thin. No brace expansion: Docker RUN uses /bin/sh
# (dash on Ubuntu), which would create one directory with the literal name.
RUN mkdir -p /home/${USERNAME}/ros2_ws/src \
             /home/${USERNAME}/ros2_ws/build \
             /home/${USERNAME}/ros2_ws/install \
             /home/${USERNAME}/ros2_ws/log

# Default command: keep the container alive for interactive `docker exec` work
CMD ["bash", "-l"]
