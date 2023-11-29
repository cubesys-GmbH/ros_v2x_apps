# CUBE devcontainer
FROM ghcr.io/cubesys-gmbh/workspace:humble
SHELL ["/bin/bash", "-c"]

# Change user
USER root

# Install system packages
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && apt-get -y install --no-install-recommends \
    pip \
    git

ARG USERNAME="cube"
ARG USER_HOME_DIR=/home/${USERNAME}

# Workaround for Python nodes 
COPY requirements.txt ./
RUN pip install -U pip
RUN pip install --no-cache-dir -r requirements.txt

# Change user
USER ${USERNAME}

# Setup entrypoint
COPY ./entrypoint.bash /

ENTRYPOINT ["/entrypoint.bash"]
CMD ["bash"]
