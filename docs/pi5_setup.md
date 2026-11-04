# Raspberry Pi 5 のセットアップ

ローバーに載せる Raspberry Pi 5(16GB)を、開発PCと同じ環境(Ubuntu 24.04 + ROS 2 Jazzy + natto_library)にする手順。
ほとんどは [tools/setup_pi5.sh](../tools/setup_pi5.sh) で自動化してある。

## 1. OS を入れる
**Ubuntu Desktop がもう入っている Pi**(いまの Pi はこれ)は、OS の書き込みはいらない。Pi の画面で端末を開いて次をする:
```bash
lsb_release -a; uname -m        # 24.04 と aarch64 であること(22.04 だと ROS 2 Jazzy は入らない)
sudo apt install -y openssh-server git   # Desktop には SSH サーバーが入っていない(開発PCから入る・deploy_pi.sh で使う)
hostname -I                     # Pi の IP。開発PCから ssh <ユーザー>@<このIP> で入れるか確認
```
- Desktop 版なら `setup_pi5.sh` は ROS を `ros-jazzy-desktop`(RViz 入り)で入れる。Pi に画面をつないで `real.launch.xml launch_visualization:=true` でも見られる(本番は CPU を食うので、既定どおり RViz なし + 開発PCの monitor.launch.xml がよい)
- Desktop 版の brltty(点字ディスプレイ用)は 22.04 まで RPLIDAR の USB(CP2102)を掴むことがあった。24.04 では起きないはずだが、`setup_pi5.sh lidar` が念のため止める

**まっさらな Pi のとき**:
1. 開発PCで Raspberry Pi Imager を使い、**Ubuntu Server 24.04 LTS (64-bit)** を microSD(32GB以上)に書く
   - Raspberry Pi OS ではなく Ubuntu にする(ROS 2 Jazzy を apt で入れるため)
   - Imager の設定で、ユーザー名・Wi-Fi・SSH を有効にしておく
2. Pi 5 を起動し、開発PCから `ssh <ユーザー名>@<PiのIP>` で入る

## 2. スクリプトを実行する
コードは GitHub(https://github.com/mor0koshi/AIcontest_ws 、公開)を通して Pi に入れる。
```bash
# Pi の上で(初回だけ)
sudo apt install -y git
git clone https://github.com/mor0koshi/AIcontest_ws.git ~/AIcontest_ws
bash ~/AIcontest_ws/tools/setup_pi5.sh   # ROS・natto_library・ビルド・Nucleo の USB・LiDAR・ネットワークを全部(CAN は設定しない)
sudo reboot                              # dialout グループ・udev を確実に有効にする
```
**2回目から**: 開発PCで直して `git push` → Pi で
```bash
bash ~/AIcontest_ws/tools/update_pi.sh   # git pull + 自作パッケージだけビルド
```
- 本番の配置(course.yaml)は、開発PCで書いて push しても、Pi で直接書いてもよい。Pi で直したまま `update_pi.sh` を実行すると、pull が止まる(上書きはしない)。`git status` で確認して、要らなければ `git checkout -- <ファイル>`、残すなら Pi で commit する
- **会場にネットがないとき**: 開発PCと Pi を同じ Wi-Fi(スマホのテザリングなど)にして、開発PCから `tools/deploy_pi.sh <ユーザー>@<PiのIP>` で直接送る。送ったあとの Pi は git から見ると「変更あり」になるので、ネットが戻ったら Pi で `git checkout -- . && bash tools/update_pi.sh` でそろえる
- natto_library と `contest_rules.md`・`docs/natto_issues.md`(チーム内メモ)は GitHub に上げていない。natto_library は `setup_pi5.sh ws` が本家から clone する

| 項目 | やること |
|---|---|
| `ros` | ROS 2 Jazzy(Desktop 版の Pi なら desktop = RViz 入り、Server なら ros-base)、rplidar_ros、xacro、can-utils など |
| `ws` | natto_library を clone、rosdep、ビルド(Pi 5 16GB なら並列でも足りる) |
| `nucleo` | Nucleo の USB(ST-LINK の仮想シリアル)を `/dev/nucleo` に固定、dialout グループ、ModemManager を止める(`/dev/ttyACM*` を勝手に開くため) |
| `can` | (CAN でつなぐときだけ)`/boot/firmware/config.txt` に MCP2518FD の設定(水晶 20MHz、INT=GPIO25)、起動時に can0 を 1Mbps で立ち上げ |
| `lidar` | RPLIDAR A1(CP2102)を `/dev/rplidar` に固定、dialout グループ、brltty を念のため止める(Desktop 版) |
| `net` | `~/.bashrc` に ROS の読み込みと `ROS_DOMAIN_ID` |

## 3. 配線
**既定(UART)**: Nucleo の USB(ST-LINK 側の micro USB)を Pi の USB につなぐだけ。CAN は Nucleo と C610 だけでつなぐ(終端抵抗はバスの両端)。
USB ケーブルはモーターの振動で抜けないように固定する。

**CAN でつなぐとき(CAN モジュール ⇔ Pi 5 の 40ピン)**:
| モジュール | Pi のピン番号 | 機能 |
|---|---|---|
| VCC(3.3V) | 1 | 3.3V |
| 5V(要確認) | 2 | 5V(MCP2562FD のバス側電源) |
| GND | 6 | GND |
| SI / MOSI | 19 | GPIO10 |
| SO / MISO | 21 | GPIO9 |
| SCK | 23 | GPIO11 |
| CS | 24 | GPIO8(CE0) |
| INT | 22 | GPIO25 |

- モジュールの電圧ジャンパは**必ず 3.3V 側**(Pi の GPIO は 5V に耐えられない)
- 終端抵抗のジャンパは、Pi がバスの端なら ON、途中なら OFF

## 4. 確認
```bash
ls -l /dev/nucleo /dev/rplidar         # Nucleo と LiDAR
ros2 launch aicontest_can_bridge serial_bridge.launch.xml   # Nucleo とだけつなぐ
#   → ログに "Opened /dev/nucleo"、"[nucleo] AIcontest_432 low layer start ..."、"[nucleo] IMU found"、10秒ごとに "rx ... telemetry"
ros2 topic echo /joint_states --qos-reliability best_effort   # 計測輪を手で回すと値が変わるか
ros2 launch aicontest_rover real.launch.xml          # LiDAR は既定で /dev/rplidar
```
画面は開発PCで見る(real.launch.xml は既定で RViz を起動しない。Pi に画面をつないでいるなら `launch_visualization:=true` でも見られる):
```bash
# 開発PCで(Pi と同じ Wi-Fi、同じ ROS_DOMAIN_ID)
export ROS_DOMAIN_ID=42
ros2 launch aicontest_rover monitor.launch.xml       # natto の表示用ノード + RViz だけ
```
- RViz に何も出ない: `ros2 topic list` で Pi のトピックが見えるか。見えなければ ROS_DOMAIN_ID、同じネットワークか、ファイアウォール(`sudo ufw status`)を確認
- `open(/dev/nucleo) failed`: USB がつながっているか、`bash setup_pi5.sh nucleo` をしたか、再ログインしたか(dialout)。`/dev/ttyACM0` なら `nucleo_port:=/dev/ttyACM0`
- `rx 0 telemetry` / `bad packets` が多い: ファームウェアが UART 版(`app.c` の `PI_LINK_UART`)か、速さ(460800)が合っているか

CAN でつなぐときの確認:
```bash
dmesg | grep -i mcp251xfd              # MCP2518FD が見つかったか
ip -details link show can0             # state ERROR-ACTIVE、bitrate 1000000
candump can0                           # Nucleo / C610 のフレームが流れるか
ros2 launch aicontest_rover real.launch.xml nucleo_link:=can lidar_port:=/dev/rplidar
```
CAN が見つからないときは、水晶の周波数(`CAN_OSC`)と INT のピン(`CAN_INT_GPIO`)を確認して `bash setup_pi5.sh can` をやり直す。

## 5. 会場での注意: ROS_DOMAIN_ID
ROS 2 は同じネットワークにいる機械どうしで自動的につながる。会場の Wi-Fi にほかのチームの ROS があると、トピックが混ざって**よそのロボットの指令を受け取る**おそれがある。
- `setup_pi5.sh` は `ROS_DOMAIN_ID=42` を設定する(`ROS_DOMAIN=17 bash setup_pi5.sh net` のように変えられる)
- 別室のモニター用PCも**同じ番号**にする(`export ROS_DOMAIN_ID=42`)
- 当日、ほかのチームと番号がかぶっていないか確認する

## メモ
- 開発PCと違い、Pi の `python3` は Ubuntu 標準の 3.12 なので、ビルドの `-DPython3_EXECUTABLE` は本来いらない(付けても害はない)
- ビルドに時間がかかる場合は、開発PCでソースを編集して `rsync` で送り、Pi では `colcon build --packages-select aicontest_rover` のように必要なものだけビルドする
