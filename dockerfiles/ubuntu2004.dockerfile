FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /build
COPY ./scripts .

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        sudo \
        gnupg2 \
        lsb-release \
        build-essential \
        software-properties-common \
        cmake \
        git \
        tmux && \
    bash install_dependencies.bash && \
    rm -rf /build && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

ENTRYPOINT ["/bin/bash"]
