FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu

RUN apt update -y
RUN apt install sudo -y

# Install needed packages
RUN sudo apt install -y python3 python3-pip
RUN sudo apt install -y curl wget
RUN sudo apt install -y zip unzip

# Install sdkman
RUN curl -s "https://get.sdkman.io?rcupdate=true" | bash

# Install java
RUN bash -c "source /root/.sdkman/bin/sdkman-init.sh && sdk install java 11.0.8-open"

# Install gradle
RUN bash -c "source /root/.sdkman/bin/sdkman-init.sh && sdk install gradle"

RUN echo "source /root/.sdkman/bin/sdkman-init.sh" >> /root/.bashrc

# Install WPILib
RUN wget https://github.com/wpilibsuite/allwpilib/releases/download/v2020.3.2/WPILib_Linux-2020.3.2.tar.gz -O wpilib.tar.gz
RUN mkdir -p /root/wpilib/2020
RUN tar -zxvf wpilib.tar.gz -C /root/wpilib/2020
