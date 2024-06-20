FROM ghcr.io/nasa/astrobee:latest-rolling-ubuntu20.04
ENV DEBIAN_FRONTEND=dialog

RUN apt-get update \
  && apt-get install -y \
  libgoogle-glog-dev libgflags-dev libgtest-dev \
  libluajit-5.1-dev \
  ros-rolling-xacro \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir /src && mkdir /src/astrobee


# installing opencv4
RUN apt-get purge -y libopencv*

RUN cd /home
RUN mkdir opencv && cd opencv

RUN wget https://github.com/opencv/opencv/archive/refs/tags/4.2.0.zip
RUN unzip 4.2.0.zip
RUN rm 4.2.0.zip
RUN wget https://github.com/opencv/opencv_contrib/archive/refs/tags/4.2.0.zip
RUN unzip 4.2.0.zip
RUN rm 4.2.0.zip

RUN sudo apt update
RUN sudo apt install build-essential cmake git pkg-config libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev

RUN apt-get install opencl-headers

RUN cd /home/opencv/opencv-4.2.0
RUN mkdir -p build && cd build

RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-4.2.0/modules \
    -D BUILD_EXAMPLES=ON ..

RUN make -j4
RUN sudo make install

RUN export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/opencv/opencv-4.2.0/build/lib

# zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.5/zsh-in-docker.sh)" -- \
    -t robbyrussell
RUN echo source /opt/ros/rolling/setup.zsh >> /root/.zshrc
RUN echo source /src/astrobee/install/setup.zsh >> /root/.zshrc
CMD [ "zsh" ]
ENV MAKEFLAGS="-j4"
