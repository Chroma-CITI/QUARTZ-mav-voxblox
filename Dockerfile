# Start from the ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Set the working directory
WORKDIR /opt/catkin_ws/

# install ssh client and git
RUN apt-get update && apt-get install -y git openssh-client
# download public key for github.com
RUN mkdir -p -m 0600 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts
#RUN mkdir -p -m 700 /root/.ssh  && echo "Host gitlab.inria.fr\n StrictHostKeyChecking accept-new\n  UserKnownHostsFile /root/.ssh/known_hosts\n" > /root/.ssh/config

# Install necessary packages
COPY requirements.txt /tmp/requirements.txt
RUN apt-get update && \
    apt-get install -y $(cat /tmp/requirements.txt | cut -d'=' -f1) && \
    rm -rf /var/lib/apt/lists/*

RUN catkin init \
    && catkin config --extend /opt/ros/noetic \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin config --merge-devel

# Set the working directory
WORKDIR /opt/catkin_ws/src/

# Clone the repository and get the commit to be checkout at from the docker-compose.yaml file
ARG BRANCH='master'
ARG BRANCH_COMMIT=$BRANCH # Checkout the last commit per default
RUN --mount=type=ssh git clone --single-branch --branch $BRANCH git@github.com:Chroma-CITI/QUARTZ-mav-voxblox.git \
    && cd /opt/catkin_ws/src/mav_voxblox_planning/ \
    && echo "The commit to be check out is: $BRANCH_COMMIT" \
    && git checkout $BRANCH_COMMIT

# Install the package's dependencies
RUN --mount=type=ssh wstool init . /opt/catkin_ws/src/mav_voxblox_planning/install/install_https.rosinstall \
    && wstool update

# # Go back to the workspace root
WORKDIR /opt/catkin_ws/

# HACK: ugly way to ensure that the voxblox_ros repo used is the same as in map_frontiers
ARG VOXBLOX_BRANCH='main'
ARG VOXBLOX_BRANCH_COMMIT=$VOXBLOX_BRANCH # Checkout the last commit per default
RUN --mount=type=ssh git clone --single-branch --branch $VOXBLOX_BRANCH git@github.com:Chroma-CITI/QUARTZ-map-frontiers.git --no-checkout \
    && cd map-frontiers/ \
    && git sparse-checkout init \
    && git sparse-checkout set voxblox voxblox_ros voxblox_msgs voxblox_rviz_plugin \
    && git checkout $VOXBLOX_BRANCH_COMMIT \
    && rm -rf /opt/catkin_ws/src/voxblox/voxblox_ros/ \
    && mv voxblox* /opt/catkin_ws/src/ \
    && cd /opt/catkin_ws/ && rm -rf map-frontiers/

# Initialize and build the Catkin workspace
# TODO: need to remove the catkin_ignore in mav_voxblox_planning/voxblox_rrt_planner before this build
RUN catkin build protobuf_catkin \
    && catkin build mav_voxblox_planning voxblox_rrt_planner -DCMAKE_CXX_STANDARD=14

# # Source the setup.bash so that the package is available in the environment
RUN echo "source /opt/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# # Set the entrypoint
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /opt/catkin_ws/devel/setup.bash && exec bash"]
