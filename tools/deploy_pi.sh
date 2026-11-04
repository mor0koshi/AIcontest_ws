#!/bin/bash
# 開発PCで編集したものを Pi に直接送って、Pi の上でビルドする(ふだんは GitHub 経由の tools/update_pi.sh。これはネットがない会場用の予備)。
#   tools/deploy_pi.sh <ユーザー>@<PiのIP>            # 自作パッケージを送ってビルド
#   tools/deploy_pi.sh <ユーザー>@<PiのIP> --no-build # 送るだけ(course.yaml を直しただけのとき など)
#   PI=fukui@192.168.1.50 tools/deploy_pi.sh          # 環境変数でも指定できる
# 初めての Pi では先に setup_pi5.sh を実行する(このスクリプトが setup_pi5.sh も送る。docs/pi5_setup.md)。
# natto_library は送らない(Pi では setup_pi5.sh が GitHub から clone する)。
set -euo pipefail

PI=${PI:-${1:-}}
[ -z "$PI" ] && { echo "使い方: $0 <ユーザー>@<PiのIP> [--no-build]"; exit 1; }
BUILD=1
for a in "$@"; do [ "$a" = "--no-build" ] && BUILD=0; done
WS=$HOME/AIcontest_ws
PI_WS='~/AIcontest_ws'

echo "===== 送る → $PI:$PI_WS"
ssh "$PI" "mkdir -p $PI_WS/src $PI_WS/tools"
# --delete: 開発PCで消したファイルは Pi でも消す。__pycache__ は送らない
rsync -av --delete --exclude __pycache__ \
  "$WS/src/aicontest_rover" "$WS/src/aicontest_can_bridge" "$PI:$PI_WS/src/"
rsync -av --exclude __pycache__ "$WS/tools/" "$PI:$PI_WS/tools/"

if [ $BUILD = 1 ]; then
  echo "===== Pi でビルド"
  ssh "$PI" "bash -lc 'source /opt/ros/jazzy/setup.bash && cd $PI_WS && \
    colcon build --symlink-install --packages-select aicontest_can_bridge aicontest_rover \
      --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release'"
fi
echo
echo "===== 完了。Pi で起動: ros2 launch aicontest_rover real.launch.xml"
echo "      開発PCで見る: ros2 launch aicontest_rover monitor.launch.xml(同じ ROS_DOMAIN_ID で)"
