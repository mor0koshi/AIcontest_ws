#!/bin/bash
# Pi の上で: GitHub から最新を取ってきて、自作パッケージだけビルドする。
#   bash ~/AIcontest_ws/tools/update_pi.sh
# 開発PCで直したら git push しておく。ネットがない会場では開発PCから tools/deploy_pi.sh で直接送る。
set -euo pipefail
WS=$(cd "$(dirname "$0")/.." && pwd)
cd "$WS"
echo "===== git pull ($WS)"
# Pi の上で直したファイルがあると止まる(上書きしない)。git status で確認し、要らなければ git checkout -- <ファイル>
git pull --ff-only
echo "===== ビルド"
set +u; source /opt/ros/jazzy/setup.bash; set -u
colcon build --symlink-install --packages-select aicontest_can_bridge aicontest_rover \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
echo; echo "===== 完了: $(git log --oneline -1)"
echo "起動: source install/setup.bash && ros2 launch aicontest_rover real.launch.xml"
