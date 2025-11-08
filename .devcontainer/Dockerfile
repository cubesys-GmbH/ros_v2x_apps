# CUBE devcontainer
ARG WORKSPACE_VERSION="jazzy-develop"
FROM ghcr.io/cubesys-gmbh/workspace:${WORKSPACE_VERSION}
SHELL ["/bin/bash", "-c"]

# Change user
USER root

# Ensure valid ROS2 public key is installed (might have expired since creation of workspace image)
RUN wget -O /usr/share/keyrings/ros-archive-keyring.gpg \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key

# Install system packages
RUN export DEBIAN_FRONTEND=noninteractive && \
    apt-get update && \
    apt-get -y install --no-install-recommends git

ARG USERNAME="cube"
ARG USER_HOME_DIR=/home/${USERNAME}

# Change user
USER ${USERNAME}

CMD ["bash"]
