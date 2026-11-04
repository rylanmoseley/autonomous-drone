# os/Dockerfile
# Containerized build environment for Autonomous Drone OS images
FROM debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    debootstrap \
    qemu-user-static \
    binfmt-support \
    parted \
    dosfstools \
    e2fsprogs \
    xz-utils \
    ca-certificates \
    curl \
    git \
    build-essential \
    cmake \
    sudo \
    util-linux \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

ENTRYPOINT ["/bin/bash"]
