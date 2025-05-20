FROM osrf/ros:humble-desktop

USER root

ENV USE_SDL2=1
ENV USE_GSTREAMER=1

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp \
    python3-pip git build-essential sudo \
    python3-dev libmtdev-dev libgles2-mesa libsdl2-dev \
    libsdl2-image-dev libsdl2-mixer-dev libsdl2-ttf-dev libportmidi-dev ffmpeg \
    libsm6 libxext6 libxrender1 libgstreamer1.0 gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa \
    gstreamer1.0-pulseaudio xclip xsel && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --upgrade pip wheel Cython && \
    python3 -m pip install --no-cache-dir setuptools==58.2.0 \
    docutils pygments kivy[base] kivy[pygame] kivy[angle_sdl2] \
    https://github.com/kivymd/KivyMD/archive/master.zip gitpython

ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

RUN sed -i 's/^#force_color_prompt=yes/force_color_prompt=yes/' ~/.bashrc && \
    echo 'HISTFILE=~/.bash_history' >> ~/.bashrc && \
    echo 'HISTSIZE=1000' >> ~/.bashrc && \
    echo 'HISTFILESIZE=2000' >> ~/.bashrc && \
    echo 'shopt -s histappend' >> ~/.bashrc && \
    echo 'PROMPT_COMMAND="history -a; $PROMPT_COMMAND"' >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc


# Set up ROS2 and Cyclone DDS environment
COPY config/cyclonedds.xml /etc/cyclonedds.xml

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/etc/cyclonedds.xml