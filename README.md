# teleop_udp

ROS2パッケージ：UDPを介してコントローラーデータを送受信します。

## 概要

このパッケージは、`sensor_msgs/Joy`メッセージをUDPパケットとして送受信するための2つのノードを提供します。

- **udp_sender_node**: Joyトピックをsubscribeし、UDPパケットとして送信
- **udp_receiver_node**: UDPパケットを受信し、Joyトピックとしてpublish

## パケット形式

8バイトのバイナリデータ（リトルエンディアン）:

| オフセット | サイズ | 型 | 内容 |
|-----------|--------|------|------|
| 0 | 4 | uint32_t | ボタンビットマスク |
| 4 | 1 | uint8_t | 左スティックX (0-255, 中央128) |
| 5 | 1 | uint8_t | 左スティックY (0-255, 中央128) |
| 6 | 1 | uint8_t | 右スティックX (0-255, 中央128) |
| 7 | 1 | uint8_t | 右スティックY (0-255, 中央128) |

### ボタンビットマスク

| ビット | ボタン |
|--------|--------|
| 0x00000001 | SELECT |
| 0x00000002 | L3 |
| 0x00000004 | R3 |
| 0x00000008 | START |
| 0x00000010 | UP |
| 0x00000020 | RIGHT |
| 0x00000040 | DOWN |
| 0x00000080 | LEFT |
| 0x00000100 | L |
| 0x00000200 | R |
| 0x00000400 | L2 |
| 0x00000800 | R2 |
| 0x00001000 | TRIANGLE |
| 0x00002000 | CIRCLE |
| 0x00004000 | CROSS |
| 0x00008000 | SQUARE |

## インストール

```bash
cd ~/ros2_ws/src
git clone <repository_url> teleop_udp
cd ~/ros2_ws
colcon build --packages-select teleop_udp
source install/setup.bash
```

## 使用方法

### udp_receiver_node

UDPパケットを受信してJoyメッセージをpublishします。

```bash
ros2 run teleop_udp udp_receiver_node
```

#### パラメータ

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `host` | `0.0.0.0` | バインドするホストアドレス |
| `port` | `12345` | 受信ポート番号 |
| `frame_id` | `joy` | Joyメッセージのframe_id |

#### 例

```bash
ros2 run teleop_udp udp_receiver_node --ros-args -p port:=5000
```

### udp_sender_node

Joyメッセージをsubscribeし、UDPパケットとして送信します。

```bash
ros2 run teleop_udp udp_sender_node
```

#### パラメータ

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `target_host` | `127.0.0.1` | 送信先ホストアドレス |
| `target_port` | `12345` | 送信先ポート番号 |

#### 例

```bash
ros2 run teleop_udp udp_sender_node --ros-args -p target_host:=192.168.1.100 -p target_port:=5000
```

## トピック

### udp_receiver_node

| トピック | 型 | 方向 |
|---------|------|------|
| `joy` | sensor_msgs/Joy | Publish |

### udp_sender_node

| トピック | 型 | 方向 |
|---------|------|------|
| `joy` | sensor_msgs/Joy | Subscribe |

## ライセンスおよびコピーライト

© 2025 nop

このプロジェクトはMITライセンスの下で公開されています。詳細は[LICENSE](LICENSE)ファイルをご覧ください。
