FROM ubuntu 

LABEL description="Container for GTSAM Python Development" 

ARG DEBIAN_FRONTEND=noninteractive

## Install packages
RUN apt-get update -q && \
    apt-get install -y \
    # Python
    software-properties-common \
    # Basic Requirments
    build-essential cmake git python3-pip \
    # GTSAM Dependancies
    libboost-all-dev libtbb-dev

## Python Installation
RUN add-apt-repository ppa:deadsnakes/ppa && \
    apt-get install python3.7 -y

## Install GTSAM
WORKDIR /tmp
RUN git clone https://github.com/borglab/gtsam.git && \
    pip3 install -r gtsam/cython/requirements.txt && \
    cd gtsam && \
    mkdir build && \
    cd build && \
    cmake -DGTSAM_INSTALL_CYTHON_TOOLBOX=ON \
          -DGTSAM_PYTHON_VERSION=3 \
          .. && \
    make check && \
    make install && \
    cd cython && \
    python3.7 setup.py install
