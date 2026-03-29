# Use the official MoveIt 2 Humble release image
FROM moveit/moveit2:humble-release

# Set non-interactive mode for apt-get
ENV DEBIAN_FRONTEND=noninteractive

# Install additional useful tools (including gedit)
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    wget \
    zsh \
    tmux \
    vim \
    gedit \
    g++ \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Ensure proper locale settings
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    && locale-gen en_US.UTF-8 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Set the default shell to bash
ENV SHELL=/bin/bash

# Automatically source ROS 2 Humble setup in new shells
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Default command
CMD ["/bin/bash"]
