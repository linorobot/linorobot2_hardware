FROM python:3.11-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential udev curl pkg-config \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
    platformio \
    "cmake<4.0.0" \
    catkin_pkg \
    "empy==3.3.4" \
    colcon-common-extensions \
    lark

WORKDIR /workspace
