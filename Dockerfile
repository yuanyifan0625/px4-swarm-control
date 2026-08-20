FROM ubuntu:24.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=jazzy
ENV RUNS_IN_DOCKER=true

ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USER_NAME=ncrl
ARG PROJECT_DIR=/home/ncrl/docker_ubuntu24
ARG ROS_WS=/home/ncrl/docker_ubuntu24/px4_ws
ARG MICRO_XRCE_DDS_AGENT_VERSION=v2.4.3

ENV USER=${USER_NAME}
ENV HOME=/home/${USER_NAME}
ENV PROJECT_DIR=${PROJECT_DIR}
ENV ROS_WS=${ROS_WS}
ENV MICRO_XRCE_DDS_AGENT_VERSION=${MICRO_XRCE_DDS_AGENT_VERSION}

# Install base tools and register the ROS 2 Jazzy apt repository for Ubuntu 24.04 (noble).
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg2 \
        locales \
        lsb-release \
        software-properties-common && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
        > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Jazzy and common development tools.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        build-essential \
        cmake \
        git \
        nano \
        python3-colcon-common-extensions \
        python3-dev \
        python3-pip \
        python3-rosdep \
        python3-setuptools \
        python3-venv \
        python3-wheel \
        ros-dev-tools \
        ros-jazzy-desktop \
        sudo \
        vim \
        wget && \
    rm -rf /var/lib/apt/lists/*

# Create or reuse a non-root user before PX4 setup so full ubuntu.sh can update dialout/.bashrc.
RUN if getent group "${GROUP_ID}" >/dev/null; then \
        EXISTING_GROUP="$(getent group "${GROUP_ID}" | cut -d: -f1)" && \
        groupmod --new-name "${USER_NAME}" "${EXISTING_GROUP}" || true; \
    else \
        groupadd --gid "${GROUP_ID}" "${USER_NAME}"; \
    fi && \
    if getent passwd "${USER_ID}" >/dev/null; then \
        EXISTING_USER="$(getent passwd "${USER_ID}" | cut -d: -f1)" && \
        usermod --login "${USER_NAME}" \
            --home "/home/${USER_NAME}" \
            --move-home \
            --gid "${GROUP_ID}" \
            --groups sudo,dialout \
            --shell /bin/bash \
            "${EXISTING_USER}"; \
    else \
        useradd \
            --create-home \
            --no-log-init \
            --uid "${USER_ID}" \
            --gid "${GROUP_ID}" \
            --groups sudo,dialout \
            --shell /bin/bash \
            "${USER_NAME}"; \
    fi && \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER_NAME}" && \
    chmod 0440 "/etc/sudoers.d/${USER_NAME}"

# Install PX4 SITL/Gazebo dependencies at image build time so recreated containers remain usable.
COPY PX4-Autopilot/Tools/setup/ubuntu.sh /tmp/px4_setup/ubuntu.sh
COPY PX4-Autopilot/Tools/setup/requirements.txt /tmp/px4_setup/requirements.txt
RUN chmod +x /tmp/px4_setup/ubuntu.sh && \
    /tmp/px4_setup/ubuntu.sh && \
    rm -rf /tmp/px4_setup /var/lib/apt/lists/*

# Install the Micro XRCE-DDS Agent version used by the pinned PX4 v1.17 contract.
RUN git clone --depth 1 --branch "${MICRO_XRCE_DDS_AGENT_VERSION}" \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
        /tmp/Micro-XRCE-DDS-Agent && \
    cmake -S /tmp/Micro-XRCE-DDS-Agent \
        -B /tmp/Micro-XRCE-DDS-Agent/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DUAGENT_SUPERBUILD=ON && \
    cmake --build /tmp/Micro-XRCE-DDS-Agent/build --target help \
        > /tmp/micro_xrce_agent_targets.txt && \
    sed -n '1,200p' /tmp/micro_xrce_agent_targets.txt && \
    cmake --build /tmp/Micro-XRCE-DDS-Agent/build --parallel "$(nproc)" && \
    UAGENT_BIN="$(find /tmp/Micro-XRCE-DDS-Agent/build -type f -name MicroXRCEAgent -perm /111 | head -n 1)" && \
    test -n "${UAGENT_BIN}" && \
    echo "MicroXRCEAgent binary: ${UAGENT_BIN}" && \
    install -d /usr/local/lib && \
    find /tmp/Micro-XRCE-DDS-Agent/build -type f -name "*.so*" \
        -exec install -m 0644 {} /usr/local/lib/ \; && \
    install -m 0755 "${UAGENT_BIN}" /usr/local/bin/MicroXRCEAgent && \
    ldconfig && \
    rm -rf /tmp/Micro-XRCE-DDS-Agent /tmp/micro_xrce_agent_targets.txt && \
    ldconfig && \
    ldd /usr/local/bin/MicroXRCEAgent && \
    ! ldd /usr/local/bin/MicroXRCEAgent | grep "not found" && \
    ( \
        UAGENT_HELP_STATUS=0; \
        MicroXRCEAgent --help >/tmp/micro_xrce_agent_help.txt 2>&1 || UAGENT_HELP_STATUS="$?"; \
        test "${UAGENT_HELP_STATUS}" -le 1; \
    ) && \
    sed -n '1,20p' /tmp/micro_xrce_agent_help.txt && \
    rm -f /tmp/micro_xrce_agent_help.txt

RUN rosdep init || true

# Create the default project and ROS workspace paths inside the image.
# The compose bind mount overlays PROJECT_DIR with the host project directory.
RUN mkdir -p "${ROS_WS}/src" && \
    chown -R "${USER_ID}:${GROUP_ID}" "${PROJECT_DIR}"

# Load ROS 2 Jazzy and the workspace overlay for interactive container shells.
RUN echo "" >> "/home/${USER_NAME}/.bashrc" && \
    echo "# ROS 2 Jazzy default environment" >> "/home/${USER_NAME}/.bashrc" && \
    echo "source /opt/ros/jazzy/setup.bash" >> "/home/${USER_NAME}/.bashrc" && \
    echo "if [ -f ${ROS_WS}/install/setup.bash ]; then" >> "/home/${USER_NAME}/.bashrc" && \
    echo "    source ${ROS_WS}/install/setup.bash" >> "/home/${USER_NAME}/.bashrc" && \
    echo "fi" >> "/home/${USER_NAME}/.bashrc" && \
    chown "${USER_ID}:${GROUP_ID}" "/home/${USER_NAME}/.bashrc"

USER ${USER_NAME}
WORKDIR ${PROJECT_DIR}

CMD ["/bin/bash"]
