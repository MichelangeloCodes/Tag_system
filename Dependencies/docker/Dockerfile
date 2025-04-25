FROM ros:humble-ros-base

# Arguments to create a user
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Update system and install packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    # Networking and debugging tools
    net-tools iputils-ping curl wget \
    # Development tools
    build-essential cmake g++ git \
    # Python + ROS tools
    python3-pip python3-dev python3-colcon-common-extensions \
    # Handy utilities
    bash-completion vim nano htop less tree unzip zip \
    # Debugging and system monitoring
    lsof strace gdb tmux iproute2 rsync \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

# Set locale to prevent warnings
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Create a non-root user with sudo access
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to the new user
USER $USERNAME
WORKDIR /home/$USERNAME

# Set up ROS + colcon autocompletion
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> ~/.bashrc && \
    echo 'source /usr/share/bash-completion/bash_completion' >> ~/.bashrc

# Default to bash
CMD ["bash"]

