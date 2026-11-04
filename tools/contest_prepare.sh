#!/bin/bash
# 本番(または練習)で配置を course.yaml に書いたあと、走らせる前にやることを1つのコマンドで行う。
#
#   tools/contest_prepare.sh                          # パッケージの config/map/course.yaml
#   tools/contest_prepare.sh ~/course_today.yaml      # 別のファイル
#   tools/contest_prepare.sh ~/course_today.yaml --sim   # 確認用のシミュレーションも起動する(RViz に出る。約3分)
#
# やること:
#   1. 入力の確認(椅子が重なっていないか、コースの外にないか、角度を度で書いているか など)
#   2. 地図とルートを作る(make_map.py)。ルートの検査で問題が出たら、椅子の通り抜ける向き・回る順番を自動で直す
#   3. 結果の表示。problem / warning が残っていたら、その区間を RViz で特に注意して見る(contest_day.md 3)
#   4. --sim のとき: 同じ地図・ルートで、現実に近いシミュレーション(sim_realistic.launch.xml)を起動する
# 実機で走らせるときは、最後に表示されるコマンドで real.launch.xml を起動する(スタート位置と椅子は course.yaml から読む)
set -e
WS=$HOME/AIcontest_ws
COURSE=${1:-$WS/src/aicontest_rover/config/map/course.yaml}
[ "$1" = "--sim" ] && COURSE=$WS/src/aicontest_rover/config/map/course.yaml
SIM=0
for a in "$@"; do [ "$a" = "--sim" ] && SIM=1; done
COURSE=$(realpath "$COURSE")
source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

echo "===== 配置: $COURSE"
# 実機用: パッケージの地図・ルート(real.launch.xml が読む)を作り直す
ros2 run aicontest_rover make_map.py "$COURSE"
# シミュレーション用: 同じものを1つのフォルダにまとめる(course.yaml の写し + 地図 + ルート)
OUT=$WS/course_today
ros2 run aicontest_rover make_map.py "$COURSE" --out "$OUT" > /dev/null 2>&1
echo
echo "===== 実機で走らせるとき(Pi で)"
echo "  ros2 launch aicontest_rover real.launch.xml course_path:=$COURSE lidar_port:=/dev/rplidar"
if [ $SIM = 1 ]; then
  echo "===== 確認用のシミュレーションを起動(止めるときは Ctrl+C)"
  ros2 launch aicontest_rover sim_realistic.launch.xml course_dir:=$OUT
else
  echo "===== 確認用のシミュレーション(約3分)"
  echo "  $0 $COURSE --sim"
fi
