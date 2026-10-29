FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    python3-colcon-common-extensions \
    python3-lxml \
    python3-pyqt5 \
    fonts-noto-cjk \
    liburdfdom-tools \
    libceres-dev \
    libminizip-dev \
    ros-humble-gtsam \
    python3-venv \
    python3-pip \
    python3-scipy \
    python3-yaml \
    libxcb-cursor0 \
    mesa-utils \
    x11-utils \
    && rm -rf /var/lib/apt/lists/*

# HMI 后端运行于同一 ROS Python 环境。
RUN python3 -m pip install --no-cache-dir \
    "fastapi>=0.95,<1" "uvicorn>=0.22,<1" "websockets>=11,<16" \
    "psutil>=5.9,<7" "PySide6==6.8.3"

# 隔离 PCT 的 GPU 地图依赖，不覆盖 ROS 系统 Python。
RUN python3 -m venv --system-site-packages /opt/pct \
    && /opt/pct/bin/pip install --no-cache-dir \
        numpy==1.24.4 cupy-cuda12x==13.6.0 \
        nvidia-cuda-runtime-cu12==12.6.77 \
        nvidia-cuda-nvrtc-cu12==12.6.85 \
        nvidia-cuda-cccl-cu12==12.6.77 \
    && printf '%s\n' \
        /opt/pct/lib/python3.10/site-packages/nvidia/cuda_runtime/lib \
        /opt/pct/lib/python3.10/site-packages/nvidia/cuda_nvrtc/lib \
        > /etc/ld.so.conf.d/pct.conf \
    && ldconfig
ENV LD_LIBRARY_PATH=/opt/pct/lib/python3.10/site-packages/nvidia/cuda_runtime/lib:/opt/pct/lib/python3.10/site-packages/nvidia/cuda_nvrtc/lib:${LD_LIBRARY_PATH}

WORKDIR /workspace

CMD ["sleep", "infinity"]
