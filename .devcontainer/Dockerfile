# CUBE devcontainer 
ARG ROS_DISTRO="humble"
FROM ghcr.io/cubesys-gmbh/workspace:${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Change user
USER root

# Install system packages
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && apt-get -y install --no-install-recommends \
    git

ARG USERNAME="cube"
ARG USER_HOME_DIR=/home/${USERNAME}

# Change user
USER ${USERNAME}

CMD ["bash"]
