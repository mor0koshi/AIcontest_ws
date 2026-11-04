#!/bin/bash
# Raspberry Pi 5(Ubuntu 24.04 64bit)をローバー用にセットアップする。
#   bash setup_pi5.sh            # 全部(Nucleo とは UART = USB でつなぐ。CAN は設定しない)
#   bash setup_pi5.sh ros        # ROS 2 Jazzy だけ
#   bash setup_pi5.sh ws nucleo lidar net   # 指定した項目だけ
#   bash setup_pi5.sh can        # Pi も CAN バスにつなぐとき(CAN モジュール MCP2518FD。real.launch.xml は nucleo_link:=can)
# 何度実行しても同じ結果になるように書いてある。sudo のパスワードを聞かれる。
# 手順の説明は docs/pi5_setup.md。
set -euo pipefail

WS=${WS:-$HOME/AIcontest_ws}
CAN_OSC=${CAN_OSC:-20000000}        # CAN モジュールの水晶(20.000MHz)
CAN_INT_GPIO=${CAN_INT_GPIO:-25}    # MCP2518FD の INT をつないだ GPIO
ROS_DOMAIN=${ROS_DOMAIN:-42}        # 会場でほかのチームの ROS と混ざらないように(0〜101 で他と違う番号)

step() { echo; echo "=== $* ==="; }

do_ros() {
  step "ROS 2 Jazzy"
  sudo apt-get update
  sudo apt-get install -y software-properties-common curl
  sudo add-apt-repository -y universe
  if [ ! -f /etc/apt/sources.list.d/ros2.list ] && ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
    sudo apt-get update
  fi
  # Ubuntu Desktop が入っている Pi なら RViz なども入れる(画面をつないで Pi の上でも見られる)。Server なら ros-base だけ
  ROS_PKG=ros-jazzy-ros-base
  if dpkg -s ubuntu-desktop >/dev/null 2>&1 || dpkg -s ubuntu-desktop-minimal >/dev/null 2>&1; then ROS_PKG=ros-jazzy-desktop; fi
  echo "→ $ROS_PKG を入れる"
  sudo apt-get install -y $ROS_PKG ros-dev-tools \
    ros-jazzy-xacro ros-jazzy-rplidar-ros ros-jazzy-joy ros-jazzy-robot-state-publisher \
    can-utils python3-yaml
  sudo rosdep init 2>/dev/null || true
  rosdep update
}

do_ws() {
  step "ワークスペース($WS)"
  mkdir -p "$WS/src"
  if [ ! -d "$WS/src/natto_library" ]; then
    git clone https://github.com/NITIC-Robot-Club/natto_library.git "$WS/src/natto_library"
  fi
  if [ ! -d "$WS/src/aicontest_rover" ] || [ ! -d "$WS/src/aicontest_can_bridge" ]; then
    echo "!! $WS/src に aicontest_rover と aicontest_can_bridge がない。GitHub から取ってくる:"
    echo "   git clone https://github.com/mor0koshi/AIcontest_ws.git $WS   (ネットがなければ開発PCから tools/deploy_pi.sh)"
    return
  fi
  source /opt/ros/jazzy/setup.bash
  cd "$WS"
  rosdep install -y --from-paths src --ignore-src
  colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
}

do_can() {
  step "CAN(MCP2518FD、水晶 ${CAN_OSC}Hz、INT=GPIO${CAN_INT_GPIO}、1Mbps)"
  CFG=/boot/firmware/config.txt
  LINE="dtoverlay=mcp251xfd,spi0-0,oscillator=${CAN_OSC},interrupt=${CAN_INT_GPIO}"
  grep -q '^dtparam=spi=on' "$CFG" || echo 'dtparam=spi=on' | sudo tee -a "$CFG" >/dev/null
  if grep -q '^dtoverlay=mcp251xfd' "$CFG"; then
    sudo sed -i "s|^dtoverlay=mcp251xfd.*|$LINE|" "$CFG"
  else
    echo "$LINE" | sudo tee -a "$CFG" >/dev/null
  fi
  # 起動時に can0 を 1Mbps で立ち上げる(systemd-networkd)
  sudo tee /etc/systemd/network/80-can0.network >/dev/null <<EOF
[Match]
Name=can0

[CAN]
BitRate=1M
RestartSec=100ms
EOF
  sudo systemctl enable systemd-networkd
  echo "→ 再起動後に確認: dmesg | grep -i mcp251xfd ; ip -details link show can0"
}

do_lidar() {
  step "RPLIDAR A1 を /dev/rplidar に固定"
  # CP2102(10c4:ea60)。ほかに同じチップの USB シリアルを挿すなら serial で区別する
  sudo tee /etc/udev/rules.d/99-rplidar.rules >/dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="rplidar"
EOF
  # Ubuntu Desktop の brltty(点字ディスプレイ用)は 22.04 まで CP2102 を掴んで /dev/ttyUSB0 を消した。24.04 では外されたが、使わないので念のため止める
  if dpkg -s brltty >/dev/null 2>&1; then
    sudo systemctl disable --now brltty-udev.service brltty.service 2>/dev/null || true
    sudo systemctl mask brltty-udev.service brltty.service 2>/dev/null || true
  fi
  sudo udevadm control --reload-rules && sudo udevadm trigger
  sudo usermod -aG dialout "$USER"
  echo "→ 起動: ros2 launch aicontest_rover real.launch.xml"
}

do_nucleo() {
  step "Nucleo の USB(ST-LINK の仮想シリアル)を /dev/nucleo に固定"
  # ST-LINK/V2-1(0483:374b)と STLINK-V3(0483:374e / 374f)。ほかに ST-LINK を挿すなら serial で区別する
  sudo tee /etc/udev/rules.d/99-nucleo.rules >/dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="0666", SYMLINK+="nucleo"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", MODE="0666", SYMLINK+="nucleo"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374f", MODE="0666", SYMLINK+="nucleo"
EOF
  sudo udevadm control --reload-rules && sudo udevadm trigger
  sudo usermod -aG dialout "$USER"
  # ModemManager が /dev/ttyACM* をモデムだと思って勝手に開くことがあるので止める(入っていれば)
  if systemctl list-unit-files | grep -q '^ModemManager'; then sudo systemctl disable --now ModemManager || true; fi
  echo "→ Nucleo を挿して確認: ls -l /dev/nucleo"
}

do_net() {
  step "ROS のネットワーク設定(~/.bashrc)"
  add() { grep -qxF "$1" ~/.bashrc || echo "$1" >> ~/.bashrc; }
  add "source /opt/ros/jazzy/setup.bash"
  add "[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash"
  # 会場の Wi-Fi でほかのチームと混ざらないように。モニター用の PC も同じ番号にする
  add "export ROS_DOMAIN_ID=$ROS_DOMAIN"
  echo "→ 新しいターミナルで有効。モニター用PCでも export ROS_DOMAIN_ID=$ROS_DOMAIN"
}

ITEMS=${*:-ros ws nucleo lidar net}
for it in $ITEMS; do
  case $it in
    ros) do_ros ;; ws) do_ws ;; nucleo) do_nucleo ;; can) do_can ;; lidar) do_lidar ;; net) do_net ;;
    *) echo "unknown item: $it (ros ws nucleo lidar net can)"; exit 1 ;;
  esac
done
echo; echo "done: $ITEMS(dialout グループは再ログイン後に有効。CAN を設定したら再起動)"
