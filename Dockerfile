ARG WORKSPACE_VERSION="jazzy-develop"
FROM ghcr.io/cubesys-gmbh/workspace:${WORKSPACE_VERSION}
USER cube
COPY --chown=cube dev_ws/src /home/cube/dev_ws/src
WORKDIR /home/cube/dev_ws
RUN source /opt/cube/*/setup.bash && colcon build
COPY --chmod=755 <<EOF entrypoint.bash
#!/bin/bash
source /home/cube/dev_ws/install/setup.bash
exec "\$@"
EOF
ENTRYPOINT ["/home/cube/dev_ws/entrypoint.bash"]
CMD ["ros2", "run", "v2x_apps", "cam_listener"]