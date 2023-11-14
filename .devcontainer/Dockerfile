# CUBE devcontainer
FROM ghcr.io/cubesys-gmbh/workspace:humble
SHELL ["/bin/bash", "-c"]
USER root

# install system packages
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive && apt-get -y install --no-install-recommends \
    pip \
    git

COPY requirements.txt ./
RUN pip install -U pip
RUN pip install --no-cache-dir -r requirements.txt

ARG USERNAME="cube"
ARG USER_HOME_DIR=/home/${USERNAME}
USER ${USERNAME}
#RUN mkdir ${USER_HOME_DIR}/apps

# setup entrypoint
COPY ./entrypoint.bash /

ENTRYPOINT ["/entrypoint.bash"]
CMD ["bash"]
