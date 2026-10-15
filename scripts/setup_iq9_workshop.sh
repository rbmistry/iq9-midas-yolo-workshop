#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

sudo apt update
sudo apt install -y curl gnupg2 lsb-release ca-certificates software-properties-common locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null


sudo add-apt-repository -y ppa:ubuntu-qcom-iot/qcom-ppa
sudo add-apt-repository -y ppa:ubuntu-qcom-iot/qirp
sudo apt update

SDK_PACKAGE=qirp-sdk
desktop_status=$(dpkg-query -W -f='${Status}' ubuntu-desktop 2>/dev/null || true)
if [ "$desktop_status" = "install ok installed" ]; then
  SDK_PACKAGE=qirp-sdk-desktop
fi

sudo apt install -y \
  "$SDK_PACKAGE" \
  ros-jazzy-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-argcomplete \
  ros-jazzy-qrb-ros-camera \
  ros-jazzy-qrb-ros-nn-inference \
  ros-jazzy-qrb-ros-tensor-list-msgs \
  ros-jazzy-image-publisher \
  ros-jazzy-cv-bridge \
  ros-jazzy-usb-cam \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-web-video-server \
  python3-numpy \
  libopencv-dev \
  libqnn-dev \
  libqnn1 \
  libtensorflow-lite-c-qcom1 \
  libtensorflow-lite-qcom-dev \
  v4l-utils \
  git

if [ -f /usr/share/qirp-setup.sh ]; then
  # The setup script is idempotent enough for workshop use; source it so this shell
  # gets the environment and so participants see any detected camera path.
  # shellcheck disable=SC1091
  set +u
  source /usr/share/qirp-setup.sh || true
  set -u
fi
echo "IQ9 workshop dependencies installed."
echo "Browser output viewer dependency installed: ros-jazzy-web-video-server."
