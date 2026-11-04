# Pi ⇔ Nucleo 通信仕様 v0.4(UART が既定、CAN も選べる)

Raspberry Pi 5 / Nucleo-L432KC / C610(M2006)×4 の取り決め。
**v0.4(2026-11-04)から、Pi ⇔ Nucleo は UART(Nucleo の USB)が既定**。CAN は Nucleo と C610 だけのバスになる。
これまでどおり Pi も CAN バスにつなぐ形(下の「CAN」)も、ファームウェアの `app.c` の `PI_LINK` と `real.launch.xml` の `nucleo_link:=can` で選べる。
**送るデータの中身はどちらも同じ**(UART では CAN の 0x310〜0x314 の中身をつなげて 1 パケットにする)。

## 役割分担
- **Pi (ROS 2) = 上位**: 経路計画、自己位置推定、メカナムの逆運動学(`/cmd_vel` → 各車輪の角速度)、計測輪からのオドメトリ
- **Nucleo (STM32) = 下位**: 各車輪の速度PID(1 kHz)、計測輪のカウント、IMU(**BMX055**、I2C)の読み取り、ウォッチドッグ

## UART(v0.4、既定)
### なぜ UART にしたか(2026-11-04)
- **モーター(C610)の CAN バスを Pi から切り離せる**: Pi 側の CAN の設定ミス(水晶の周波数・ビットレートの間違い)でエラーフレームが出ると、
  C610 の通信まで止まる。UART なら Pi に何があってもモーターのバスには影響しない
- 部品と配線が少ない(USB ケーブル1本。CAN モジュール・SPI の配線・Pi の CAN の設定がいらない)
- CAN を共有すると、Pi は C610 のフィードバック(1秒に 4000フレーム)も全部受け取って捨てることになる。UART で送るのは必要な分だけ(1秒に約 6KB)

### つなぎ方
- Nucleo-L432KC の USB(ST-LINK の仮想シリアル)= USART2(PA2 = TX、PA15 = RX)。Pi では `/dev/ttyACM0`(`tools/setup_pi5.sh nucleo` で `/dev/nucleo` に固定)
- **460800 bps**、8N1、フロー制御なし(L432 は 32MHz なので誤差 0.6%)。ファームウェアの `PI_LINK_BAUD` と Pi の `serial_bridge.yaml` の `baud` を合わせる
- USB ケーブルはモーターの振動で抜けないように固定する。Nucleo の電源も Pi の USB から取れる

### パケット
```
COBS( [種類 1バイト][データ][CRC16 2バイト] ) + 0x00(区切り)
```
- **COBS**(Consistent Overhead Byte Stuffing): データに 0x00 が出ないように符号化し、0x00 を区切り専用にする。途中から読み始めても、次の 0x00 で合わせ直せる
- **CRC16**: CCITT-FALSE(初期値 0xFFFF、多項式 0x1021、反転なし)を「種類 + データ」にかけ、**リトルエンディアン**で付ける。合わなければ捨てる
- 実装: ファームウェア `Core/Src/pi_link.c`、Pi `aicontest_can_bridge/include/aicontest_can_bridge/serial_framing.hpp`(単体テスト `test/test_serial_framing.cpp`)

| 種類 | 向き | 周期 | データ |
|---|---|---|---|
| 0x01 COMMAND | Pi → Nucleo | 100 Hz | 8 バイト: 駆動輪の目標速度 = 下の CAN 0x300 と同じ |
| 0x10 TELEMETRY | Nucleo → Pi | 100 Hz | 34 バイト: 0x310(8)+ 0x311(8)+ 0x312(8)+ 0x313(8)+ 0x314(2)の中身をこの順につなげたもの |
| 0x20 TEXT | Nucleo → Pi | 1 行ごと | Nucleo の printf の文字(1 行、改行なし)。Pi の変換ノードが `[nucleo] ...` としてログに出す |

- 長さ: COMMAND は符号化後 約 13 バイト、TELEMETRY は約 39 バイト。460800bps(1秒に約 46KB)の 1割以下
- ウォッチドッグ: Nucleo は COMMAND が **100 ms** 届かなければ全モーター停止(CAN 版と同じ)。Pi の変換ノードは `/command_joint_states` が 0.5秒来なければ 0 を送る
- **USB は Pi との通信に使うので、PC のシリアルモニターで Nucleo の printf を見ることはできない**(Pi のログに出る)。
  Pi なしで printf を見たいときは、ファームウェアを `PI_LINK_CAN` にしてビルドする(USB に printf がそのまま出る)

### Pi 側
- 変換ノード: `aicontest_can_bridge` の `serial_bridge_node`(`launch/serial_bridge.launch.xml`、`config/serial_bridge.yaml`)。トピックは CAN 版と同じ
- 実機なしの試験: `python3 fake_nucleo_serial.py`(`/tmp/fake_nucleo` に仮想シリアルを作る)→ `ros2 launch aicontest_can_bridge serial_bridge.launch.xml port:=/tmp/fake_nucleo`

---

# CAN(v0.3、Pi も CAN バスにつなぐとき)

## v0.2 からの変更(2026-11-04)
- 計測輪を3つ → **2つ(0x311 = 前後向き、0x312 = 横向き)**にし、**0x313 を IMU(BMX055、I2C)用**にした(シミュレーションで、計測輪2つ+IMU のほうが向きの精度が段違いに良かったため)
- 0x314 を 2バイトにし、IMU の校正状態を足した。bit5 = IMU の異常

## v0.1 からの変更
- 計測輪のフレームを、dinosaur_library の `encoder_can_send` の形式(1輪1フレーム、[累積カウント, rpm])に合わせた
- 状態フラグを別フレーム 0x314 に分けた。Nucleo のタイムスタンプとシーケンス番号はなくした

## バス
- Classic CAN、1 Mbps(C610の仕様に合わせる)、標準ID(11bit)
- 終端抵抗120Ωはバスの**両端の2か所だけ**
- マルチバイト値はすべて **リトルエンディアン**(STM32・Raspberry Piともにネイティブ)。float は IEEE 754 単精度

## ID一覧
CANはIDが小さいほど優先される。モーター制御(0x200番台)がPi↔Nucleoの通信より常に優先されるように、Pi↔Nucleoは0x300番台を使う。

| ID | 送信元 → 宛先 | 周期 | 内容 |
|---|---|---|---|
| 0x200 | Nucleo → C610 (ID1〜4) | 1 kHz | 電流指令(DJI仕様) |
| 0x201〜0x204 | C610 → Nucleo | 1 kHz | モーターのフィードバック(DJI仕様) |
| 0x300 | Pi → Nucleo | 100 Hz | 駆動輪の目標速度 |
| 0x310 | Nucleo → Pi | 100 Hz | 駆動輪の実速度 |
| 0x311 | Nucleo → Pi | 100 Hz | 計測輪0 = 前後向き(`encoder_can` 形式) |
| 0x312 | Nucleo → Pi | 100 Hz | 計測輪1 = 横向き(`encoder_can` 形式) |
| 0x313 | Nucleo → Pi | 100 Hz | IMU(BMX055)の旋回の速さと向き |
| 0x314 | Nucleo → Pi | 100 Hz | 状態フラグ・IMU の校正状態 |

使わないID: 0x1FF、0x200〜0x20B、0x2FF(ロボマス系ESCが使う)

## Pi → Nucleo
### 0x300 駆動輪目標速度(DLC 8)
| byte | 型 | 内容 |
|---|---|---|
| 0-1 | int16 | 車輪0 目標角速度 [0.01 rad/s] |
| 2-3 | int16 | 車輪1 目標角速度 [0.01 rad/s] |
| 4-5 | int16 | 車輪2 目標角速度 [0.01 rad/s] |
| 6-7 | int16 | 車輪3 目標角速度 [0.01 rad/s] |

- 角速度は**車輪軸**の値(ギア比36:1の換算はNucleoが行う)。`/command_joint_states` の velocity をそのまま100倍して送る
- 範囲 ±327.67 rad/s(M2006の出力軸の最大は約52 rad/s なので十分)
- **ウォッチドッグ**: Nucleoは0x300を100 ms受信しなかったら全モーターを電流0にし、PIDの積分項をリセットする

## Nucleo → Pi
10 ms ごとに 0x310 → 0x311 → 0x312 → 0x313 → 0x314 の順で続けて送る。

### 0x310 駆動輪実速度(DLC 8)
| byte | 型 | 内容 |
|---|---|---|
| 0-1 | int16 | 車輪0 実角速度 [0.01 rad/s] |
| 2-3 | int16 | 車輪1 実角速度 [0.01 rad/s] |
| 4-5 | int16 | 車輪2 実角速度 [0.01 rad/s] |
| 6-7 | int16 | 車輪3 実角速度 [0.01 rad/s] |

### 0x311・0x312 計測輪0(前後向き)・1(横向き)(DLC 8、dinosaur_library `encoder_can` 形式)
| byte | 型 | 内容 |
|---|---|---|
| 0-3 | int32 | 累積カウント(起動からの合計。4逓倍込み) |
| 4-7 | float | 計測輪の回転数 [rpm](符号付き) |

- 計測輪0・1(TIM1・TIM2)は `encoder_can_send(&can1, 0x311, &enc0)` などで送る
- **`encoder_update` は送る直前に、10 ms ごとに1回だけ呼ぶ**。rpm は「前回の update からの回転数」なので、1 ms ごとに update すると最後の 1 ms 分だけの粗い値になる
- 1回転あたりのカウント数(`encoder_init` の `counts_per_rev`)は**計測輪1回転あたり**で設定する。エンコーダーと計測輪の間にギアがあればその比も含める

### 0x313 IMU(DLC 8)
| byte | 型 | 内容 |
|---|---|---|
| 0-3 | float | 旋回の速さ(z軸)[rad/s]。**ロボットを上から見て反時計回りが正** |
| 4-7 | float | 向き [rad](ジャイロを積分した向き、−π〜π、**反時計回りが正**、電源を入れたときが 0。Pi 側は旋回の速さだけを使う) |

- **使うのは BMX055**(2026-11-04 決定)。ジャイロ ±1000°/s・100Hz。**電源を入れてから約1秒、ジャイロのずれを測るので動かさない**。地磁気は使わない(屋内は乱れるため)
- ジャイロの z は、取り付けの向きによって符号を合わせる(実機で左に回して正になることを確認。逆なら imu.c の `IMU_YAW_SIGN`)
- (BNO055 にする場合: imu.c の `IMU_TYPE` を `IMU_BNO055` に。IMUPLUS モード、heading は Nucleo で反時計回りを正に直す)
- 10 ms ごとに I2C で読む(I2C3、標準モード 100 kHz。6 バイトで約 0.6 ms)
- **IMU が応答しないときは両方とも NaN を送る**(古い値を送り続けると Pi 側が回り続けていると思い込むため)。Pi の変換ノードは NaN なら /imu を出さず、オドメトリは計測輪だけで続ける
- BMX055 を使う場合も同じ形式で送る(向きはジャイロの積算)
- Pi 側は旋回の速さを使い、ずれ(バイアス)は Pi 側でも止まっているあいだに測り直す。向き(heading)は予備

### 0x314 状態フラグ(DLC 2)
| byte | 型 | 内容 |
|---|---|---|
| 0 | uint8 | 状態フラグ(下表) |
| 1 | uint8 | IMU の校正状態(BNO055 の CALIB_STAT と同じ形: bit5-4 がジャイロ 0〜3。**BMX055 は起動時にずれを測れていれば 0x30**、IMU がなければ 0) |

| bit | 意味 |
|---|---|
| 0 | ウォッチドッグで停止中(Piからの指令が途絶えた) |
| 1〜4 | C610 ID1〜4のフィードバックが途絶えている |
| 5 | IMU が応答しない |
| 6〜7 | 予約 |

## Pi側の扱い
- 計測輪の rpm を `rpm × 2π / 60` で rad/s に直し、`/joint_states` の **velocity** に入れる。`header.stamp` は受信した時刻
  - natto_wheel_odometry(omni)が使うのは velocity だけ。前回の stamp との差 × 速度 で位置を足していく
  - 累積カウントは `/joint_states` の position(`count × 2π / counts_per_rev`)に入れておく。デバッグ用
- 駆動輪4つ + 計測輪2つの JointState は、0x313 を受け取ったときにまとめて1つ出す(同じ周期の値になる)。同時に 0x313 の IMU を `/imu`(sensor_msgs/Imu)に出す
- 0x311〜0x313 が 100 ms 届かなければ `/joint_states` を出すのをやめる。natto_wheel_odometry は stamp の間隔が `timeout_sec`(0.1 s)を超えると自分で再同期する

### 受信フィルタ
- バスに流れるフレームの大半(C610のフィードバック)はPiでは不要
- 変換ノードは SocketCAN の `CAN_RAW_FILTER` で **0x310〜0x314だけ** を受信する(カーネル内で絞り込む)。`can_id = 0x310, can_mask = 0x7F8` で 0x310〜0x317 が通る
- natto_canable には ID フィルタ機能がないため使わない

## バス負荷の見積もり
8バイトフレーム1つ≒111〜135 bit、1バイトフレーム1つ≒55〜65 bit(ビットスタッフィングの有無で変わる)

| 内容 | フレーム/秒 | 負荷 |
|---|---|---|
| C610フィードバック ×4 | 4000 | 44〜54 % |
| Nucleo → C610 電流指令 | 1000 | 11〜14 % |
| Pi ↔ Nucleo(8バイト ×5種) | 500 | 6〜7 % |
| 状態フラグ 0x314 | 100 | 1 % 未満 |
| 合計 | 5600 | 約 62〜75 % |
