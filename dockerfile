# Use the latest version of Ubuntu as the base image
FROM ubuntu:latest

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Update package lists, install basic tools, toolchains, stlink-tools, and clean up
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
    gcc-arm-none-eabi \
    libc6-dev \
    build-essential \
    make \
    software-properties-common \
    cmake \
    ninja-build \
    libnewlib-arm-none-eabi 

# Set the working directory
WORKDIR /workspace

# Copy your project files into the container
COPY . /workspace/


