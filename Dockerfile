FROM ubuntu:24.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=jazzy

ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USER_NAME=ncrl
ARG PROJECT_DIR=/home/ncrl/docker_ubuntu24
ARG ROS_WS=/home/ncrl/docker_ubuntu24/px4_ws

ENV USER=${USER_NAME}
ENV HOME=/home/${USER_NAME}
ENV PROJECT_DIR=${PROJECT_DIR}
ENV ROS_WS=${ROS_WS}

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

RUN rosdep init || true

# Create or reuse a non-root user whose UID/GID match the host user.
# This keeps bind-mounted files editable from both the host and container.
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
            --groups sudo \
            --shell /bin/bash \
            "${EXISTING_USER}"; \
    else \
        useradd \
            --create-home \
            --no-log-init \
            --uid "${USER_ID}" \
            --gid "${GROUP_ID}" \
            --groups sudo \
            --shell /bin/bash \
            "${USER_NAME}"; \
    fi && \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER_NAME}" && \
    chmod 0440 "/etc/sudoers.d/${USER_NAME}"

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
