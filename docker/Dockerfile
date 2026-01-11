# Phantom Omni Haptic Device - ROS2 Container (FireWire Version)
#
# Uses OpenHaptics AE 3.0 + JUJU FireWire Driver
# Based on: https://github.com/jhu-cisst-external/phantom-omni-1394-drivers
#
# Prerequisites:
#   - Thunderbolt to FireWire adapter connected
#   - Phantom Omni powered on (use SECOND FireWire port!)
#   - Host: sudo modprobe firewire_ohci
#   - Host: sudo chmod 666 /dev/fw*
#
# Build:
#   docker compose build
#
# Run:
#   docker compose up

FROM ros:humble-ros-base

LABEL maintainer="Dongjune Chang <dongjune.chang@gmail.com>"
LABEL description="Phantom Omni FireWire with ROS2 and OpenHaptics AE 3.0"

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Set up Arizona mirror for APT (faster downloads)
RUN echo "deb http://mirror.arizona.edu/ubuntu/ jammy main restricted universe multiverse" > /etc/apt/sources.list && \
    echo "deb http://mirror.arizona.edu/ubuntu/ jammy-updates main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirror.arizona.edu/ubuntu/ jammy-backports main restricted universe multiverse" >> /etc/apt/sources.list && \
    echo "deb http://mirror.arizona.edu/ubuntu/ jammy-security main restricted universe multiverse" >> /etc/apt/sources.list

# Install base dependencies
RUN apt-get update && apt-get install -y \
    # FireWire support (REQUIRED for Phantom Omni)
    libraw1394-11 \
    libraw1394-dev \
    libraw1394-tools \
    # OpenGL dependencies
    libglw1-mesa \
    freeglut3 \
    freeglut3-dev \
    libglew-dev \
    libqt5opengl5-dev \
    # ncurses (required by OpenHaptics)
    libncurses5 \
    libncurses5-dev \
    # Build tools
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    # ROS2 packages
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-tf2-ros \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    # Utilities
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# ============================================
# OpenHaptics AE 3.0 + JUJU FireWire Driver
# ============================================
# IMPORTANT: OpenHaptics 3.4 uses libPhantomIOLib42.so (USB only!)
# OpenHaptics AE 3.0 uses libPHANToMIO.so which works with JUJU driver
#
# SDK files are provided via build context from OPENHAPTICS_SDK_DIR
# Required files:
#   - OpenHapticsAE_Linux_v3_0.zip (from JHU or 3DS)
#   - Linux_JUJU_PDD_64-bit.tgz (from JHU github)

ARG OPENHAPTICS_SDK_DIR=openhaptics
WORKDIR /tmp/openhaptics_install
COPY ${OPENHAPTICS_SDK_DIR}/ /tmp/openhaptics_install/

# Install OpenHaptics AE 3.0 from .deb package
RUN if [ -f /tmp/openhaptics_install/OpenHapticsAE_Linux_v3_0.zip ]; then \
        echo "=== Installing OpenHaptics AE 3.0 ===" && \
        unzip /tmp/openhaptics_install/OpenHapticsAE_Linux_v3_0.zip -d /tmp/openhaptics_install/ && \
        # Install PHANTOM Device Drivers (base libraries)
        dpkg --force-all -i "/tmp/openhaptics_install/OpenHapticsAE_Linux_v3_0/PHANTOM Device Drivers/64-bit/phantomdevicedrivers_4.3-3_amd64.deb" || true && \
        # Install OpenHaptics AE SDK
        dpkg --force-all -i "/tmp/openhaptics_install/OpenHapticsAE_Linux_v3_0/OpenHaptics-AE 3.0/64-bit/openhaptics-ae_3.0-2_amd64.deb" || true && \
        # Create symlinks for library compatibility
        mkdir -p /usr/lib && \
        ln -sf /usr/lib64/libHD.so.3.0.0 /usr/lib/libHD.so.3.0.0 && \
        ln -sf /usr/lib/libHD.so.3.0.0 /usr/lib/libHD.so.3 && \
        ln -sf /usr/lib/libHD.so.3 /usr/lib/libHD.so && \
        ln -sf /usr/lib64/libHL.so.3.0.0 /usr/lib/libHL.so.3.0.0 && \
        ln -sf /usr/lib/libHL.so.3.0.0 /usr/lib/libHL.so.3 && \
        ln -sf /usr/lib/libHL.so.3 /usr/lib/libHL.so && \
        ln -sf /usr/lib64/libHDU.a /usr/lib/libHDU.a && \
        ln -sf /usr/lib64/libHLU.a /usr/lib/libHLU.a && \
        echo "OpenHaptics AE 3.0 installed successfully"; \
    else \
        echo "ERROR: OpenHapticsAE_Linux_v3_0.zip not found!"; \
        exit 1; \
    fi

# Install JUJU FireWire Driver (replaces vendor's libPHANToMIO)
RUN if [ -f /tmp/openhaptics_install/Linux_JUJU_PDD_64-bit.tgz ]; then \
        echo "=== Installing JUJU FireWire Driver ===" && \
        tar -xzf /tmp/openhaptics_install/Linux_JUJU_PDD_64-bit.tgz -C /tmp/openhaptics_install/ && \
        # Remove vendor's libPHANToMIO (not compatible with firewire-core)
        rm -f /usr/lib64/libPHANToMIO.so* 2>/dev/null || true && \
        rm -f /usr/sbin/PHANToMConfiguration 2>/dev/null || true && \
        # Install JUJU version (uses libraw1394.so.11)
        strip /tmp/openhaptics_install/Linux_JUJU_PDD_64-bit/libPHANToMIO.so.4.3 && \
        cp /tmp/openhaptics_install/Linux_JUJU_PDD_64-bit/libPHANToMIO.so.4.3 /usr/lib64/ && \
        ln -sf /usr/lib64/libPHANToMIO.so.4.3 /usr/lib64/libPHANToMIO.so.4 && \
        ln -sf /usr/lib64/libPHANToMIO.so.4.3 /usr/lib64/libPHANToMIO.so && \
        # Symlinks in /usr/lib for compatibility
        ln -sf /usr/lib64/libPHANToMIO.so.4.3 /usr/lib/libPHANToMIO.so.4.3 && \
        ln -sf /usr/lib/libPHANToMIO.so.4.3 /usr/lib/libPHANToMIO.so.4 && \
        ln -sf /usr/lib/libPHANToMIO.so.4.3 /usr/lib/libPHANToMIO.so && \
        # Install configuration utility
        strip /tmp/openhaptics_install/Linux_JUJU_PDD_64-bit/PHANToMConfiguration && \
        cp /tmp/openhaptics_install/Linux_JUJU_PDD_64-bit/PHANToMConfiguration /usr/sbin/ && \
        echo "JUJU FireWire Driver installed successfully"; \
    else \
        echo "ERROR: Linux_JUJU_PDD_64-bit.tgz not found!"; \
        echo "Download from: https://github.com/jhu-cisst-external/phantom-omni-1394-drivers"; \
        exit 1; \
    fi

# Create default Phantom Omni configuration
# This allows hdInitDevice(HD_DEFAULT_DEVICE) to find the device
RUN mkdir -p /etc/SensAble/PHANToMDeviceDrivers && \
    echo -e "PHANToM Default,omni,FireWire,Default PHANToM" > /etc/SensAble/PHANToMDeviceDrivers/PHANToM.ini

# Create symlinks for CMakeLists.txt compatibility
# Some ROS packages expect headers at /opt/OpenHaptics/Developer/3.4-0/include
RUN mkdir -p /opt/OpenHaptics/Developer/3.4-0/include && \
    ln -sf /usr/include/HD /opt/OpenHaptics/Developer/3.4-0/include/HD && \
    ln -sf /usr/include/HDU /opt/OpenHaptics/Developer/3.4-0/include/HDU && \
    ln -sf /usr/include/HL /opt/OpenHaptics/Developer/3.4-0/include/HL && \
    ln -sf /usr/include/HLU /opt/OpenHaptics/Developer/3.4-0/include/HLU && \
    mkdir -p /opt/OpenHaptics/Developer/3.4-0/lib/lin-x86-64 && \
    ln -sf /usr/lib/libHD.so /opt/OpenHaptics/Developer/3.4-0/lib/lin-x86-64/ && \
    ln -sf /usr/lib/libHDU.a /opt/OpenHaptics/Developer/3.4-0/lib/lin-x86-64/ && \
    ln -sf /usr/lib/libHL.so /opt/OpenHaptics/Developer/3.4-0/lib/lin-x86-64/ && \
    ln -sf /usr/lib/libHLU.a /opt/OpenHaptics/Developer/3.4-0/lib/lin-x86-64/

# Configure library paths
RUN echo "/usr/lib64" > /etc/ld.so.conf.d/openhaptics.conf && \
    ldconfig

# Create symlink for legacy binaries that require libraw1394.so.8
# The JUJU driver uses libraw1394.so.11, but PHANToMTest/Configuration need .so.8
RUN ln -sf /lib/x86_64-linux-gnu/libraw1394.so.11 /lib/x86_64-linux-gnu/libraw1394.so.8

# Cleanup
RUN rm -rf /tmp/openhaptics_install

# Set OpenHaptics environment variables
ENV OH_SDK_BASE=/opt/OpenHaptics/Developer/3.4-0
ENV LD_LIBRARY_PATH=/usr/lib64:/usr/lib:${OH_SDK_BASE}/lib/lin-x86-64:${LD_LIBRARY_PATH}

# ============================================
# ROS2 Workspace Setup
# ============================================
WORKDIR /workspace
RUN mkdir -p /workspace/src

# Setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash 2>/dev/null || true" >> ~/.bashrc && \
    echo "export OH_SDK_BASE=/opt/OpenHaptics/Developer/3.4-0" >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/usr/lib64:/usr/lib:${OH_SDK_BASE}/lib/lin-x86-64:${LD_LIBRARY_PATH}' >> ~/.bashrc

# Setup entrypoint
COPY ros2_entrypoint.sh /ros2_entrypoint.sh
RUN chmod +x /ros2_entrypoint.sh

ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["bash"]
