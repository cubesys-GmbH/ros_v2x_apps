# CUBE devcontainer
FROM ghcr.io/cubesys-gmbh/workspace:jazzy
SHELL ["/bin/bash", "-c"]

# Change user
USER root

# Install system packages
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && apt-get -y install --no-install-recommends \
    pip \
    git

ARG USERNAME="cube"
ARG USER_HOME_DIR=/home/${USERNAME}

# Change user
USER ${USERNAME}

# Setup entrypoint
COPY ./entrypoint.bash /

ENTRYPOINT ["/entrypoint.bash"]
CMD ["bash"]
