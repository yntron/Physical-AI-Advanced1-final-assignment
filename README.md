# AI自動ブレーキ付き 差動二輪ロボット（ROS 2 + Gazebo + YOLO）

Physical AI講座 応用編 最終課題。
Gazebo上の差動二輪ロボットをROS 2からキーボードで操作し、
**前方カメラの映像を学習済みの物体検出AI（YOLOv8n）で解析して、人が近いと自動でブレーキをかける**。

```
キーボード (teleop_twist_keyboard) ── /cmd_vel_teleop ─┐
                                                        ├─> safety_stop ノード ── /cmd_vel ──> ros_gz_bridge ──> Gazebo のロボット
Gazebo の前方カメラ ── ros_gz_bridge ── /camera/image ──> YOLOv8n（推論のみ）┘
```

- **ベースにしたもの**：講座サンプル「ROS2とGazeboの連携によるロボット操作」（初級）
  https://qiita.com/N622/items/9d89f77d85d9da0af29e
  （SDFに動力を付ける → teleopでキー操作 → ros_gz_bridgeでGazeboへ橋渡し、という構成）
- **追加したもの**
  1. ロボットに前方カメラを搭載し、映像を ROS 2 に流す
  2. 学習済み YOLOv8n（COCO、推論のみ・追加学習なし）で人を検出
  3. 人の映り込みの大きさ（バウンディングボックスの高さ / 画像の高さ）で距離を推定し、近ければ前進を禁止
  4. フェイルセーフ設計：一度ブレーキがかかると、「人が十分遠い状態が続く」かつ「操作者が前進以外の操作をした」ときだけ解除する
     （近づきすぎると人が画面からはみ出して YOLO が見失うため、「見えない = 安全」とは判断しない）

## 動作環境

| 項目 | 内容 |
|---|---|
| OS | Ubuntu 24.04（Windows 11 の WSL2 上の Ubuntu 24.04 でも可） |
| ROS 2 | Jazzy |
| シミュレータ | Gazebo Harmonic（`ros-jazzy-ros-gz` で入る） |
| AI | Ultralytics YOLOv8n（CPU で動作、GPU 不要） |

※ macOS に ROS 2 / Gazebo を直接入れることはできない。
Docker で Ubuntu 24.04 のデスクトップを動かし、ブラウザから操作する。手順は「1-B」を参照。

---

## 1-A. ROS 2 Jazzy と Gazebo のインストール（Ubuntu / WSL2）

公式手順（https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html ）に沿っています。

```bash
# ロケール（UTF-8）
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS 2 のリポジトリを追加
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# ROS 2 本体 + Gazebo連携 + キーボード操作 + ビルドツール
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard \
                    ros-dev-tools python3-pip git

# 新しいターミナルで毎回 ROS 2 を読み込むようにする
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**確認**：`gz sim shapes.sdf` で Gazebo の画面が出れば OK（確認後は閉じる）。


## 1-B. macOS（Apple Silicon）で Docker を使う場合

macOS に ROS 2 / Gazebo を直接入れることはできないため、
Docker で Ubuntu 24.04 + ROS 2 Jazzy のデスクトップを動かし、ブラウザ（noVNC）から操作する。
この構成で実際に動作を確認している（「動作確認済みの内容」を参照）。

イメージは `tiryoh/ros2-desktop-vnc:jazzy`。`ros-jazzy-ros-gz` と
`ros-jazzy-teleop-twist-keyboard` は最初から入っているので、手順 1-A は不要。

```bash
# Docker Desktop を起動し、Settings > Resources でメモリを 8GB 以上にしておく
mkdir -p ~/pai_ws/src

docker run -d --name pai \
  -p 6080:80 --security-opt seccomp=unconfined --shm-size=2g \
  -v ~/pai_ws:/home/ubuntu/pai_ws \
  tiryoh/ros2-desktop-vnc:jazzy
```

ブラウザで http://127.0.0.1:6080 を開くとデスクトップが出る。
以降はデスクトップ内のターミナルで、手順 2（pip）と手順 3（ビルド）を実行する。

- `-v` でマウントしているので、`~/pai_ws` は macOS 側とコンテナ内で同じファイルが見える。
  録画した MP4 もここに保存すれば macOS 側から取り出せる。
- Gazebo の描画はソフトウェアレンダリングになるため重い。起動前に
  `export LIBGL_ALWAYS_SOFTWARE=1` を設定する。
  それでも重いときは `gui:=false` で起動し、検出結果ウィンドウと `record_video` で撮影する
  （AI の判断は検出結果ウィンドウだけで確認できる）。
- コンテナを作り直すと pip で入れたパッケージは消える。マウント先を変えるなど作り直しが必要なときは、
  先に `docker commit pai pai-ready:local` で保存しておくと再インストールを避けられる。

## 2. AI（YOLO）のインストール

```bash
# 先に CPU 版の PyTorch を入れる（GPU 版より大幅に小さい）
pip install --break-system-packages torch torchvision --index-url https://download.pytorch.org/whl/cpu

# ROS 2 と衝突しないよう numpy 1 系・OpenCV 4.11 系に固定して入れる
pip install --break-system-packages "numpy<2" "opencv-python<4.12" ultralytics
```

> `--break-system-packages` は、Ubuntu 24.04 で pip をユーザー領域に入れるために必要なオプションです。
> numpy を 2 系にすると ROS 2 や Ubuntu 付属の Python ライブラリが壊れることがあるため、`numpy<2` に固定しています。

## 3. ビルド

```bash
mkdir -p ~/pai_ws/src
cd ~/pai_ws/src
git clone <このリポジトリのURL> pai_safety_stop   # または zip を展開してここに置く
cd ~/pai_ws
colcon build
source install/setup.bash
```

## 4. 実行

**ターミナル1：シミュレーション + AI**

```bash
cd ~/pai_ws && source install/setup.bash
ros2 launch pai_safety_stop safety_stop.launch.py
```

- Gazebo の画面と、検出結果のウィンドウ「AI safety stop (YOLO)」が開きます。
- **初回は人のモデル（Gazebo Fuel）と YOLO の重み（yolov8n.pt）を自動ダウンロードする**ため、1〜数分かかります（要インターネット）。
- Gazebo の左下の再生ボタンが止まっている場合は押してください（通常は自動で開始）。

**ターミナル2：キーボード操作**

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop
```

このターミナルを選択した状態でキーを押します。

| キー | 動き |
|---|---|
| `i` | 前進 |
| `,` | 後退 |
| `j` / `l` | 左旋回 / 右旋回 |
| `k` | 停止 |
| `z` | 速度を下げる（最初に5回ほど押して約0.3 m/s にすると見やすい） |

**キー操作なしで試す（自動前進モード）**

```bash
ros2 launch pai_safety_stop safety_stop.launch.py auto_forward_speed:=0.3
```

## 5. 動画の撮影

Gazebo の画面と検出結果ウィンドウを左右に並べ、画面録画します。

- Ubuntu 標準：`Print Screen` キー → 右下の録画（ビデオカメラ）アイコン → 範囲を選んで録画開始
- または OBS Studio

**画面録画がうまくいかない場合の保険**（検出ウィンドウの映像を MP4 に直接保存）：

```bash
# ターミナル3
cd ~/pai_ws && source install/setup.bash
ros2 run pai_safety_stop record_video --ros-args -p output:=demo.mp4
# 撮り終わったら Ctrl+C で保存
```

**撮影の台本（例：約90秒）**

| 時間 | 操作 | 画面で見せること |
|---|---|---|
| 0:00〜0:10 | （冒頭にタイトルを表示） | 「AIが人を見つけて自動でブレーキ」 |
| 0:10〜0:30 | `i` で前進 | 人に近づくにつれ、検出枠が大きくなる |
| 0:30〜0:40 | （そのまま） | **AI BRAKE: STOP** で停止。枠が赤くなる |
| 0:40〜0:50 | もう一度 `i` | 前進キーを押しても止まったまま |
| 0:50〜1:05 | `j` で左旋回 → `k` | 人が画面から外れ、**AI BRAKE: GO** に戻る |
| 1:05〜1:30 | `i` で前進、`l` で人の方へ向き直して再接近 | もう一度自動で止まる |

## 6. パラメータ調整

| パラメータ | 既定値 | 意味 |
|---|---|---|
| `stop_ratio` | 0.65 | 人の高さ / 画像の高さ がこれ以上でブレーキ（大きいほど近くで止まる。検証では約3m手前で停止） |
| `release_ratio` | 0.3 | これ未満を「十分遠い」とみなす |
| `release_frames` | 5 | 「十分遠い」が何回連続したら解除可能か（推論5Hz → 約1秒） |
| `auto_forward_speed` | 0.0 | 0より大きいと、キー入力がない間その速度で自動前進 |
| `show_window` | true | 検出結果ウィンドウを表示するか |

例：`ros2 launch pai_safety_stop safety_stop.launch.py stop_ratio:=0.75`

## 7. うまく動かないとき

| 症状 | 対処 |
|---|---|
| 人が出てこない | 初回はダウンロード中。ターミナル1のログを待つ。ネット接続を確認 |
| `numpy` / `cv2` の import エラー | 手順2の pip コマンドをもう一度実行（numpy が 2 系になっていないか `python3 -c "import numpy;print(numpy.__version__)"` で確認） |
| 検出ウィンドウが出ない | `show_window:=false` で起動し、`ros2 run rqt_image_view rqt_image_view /detection/image` で表示 |
| WSL2 で Gazebo が真っ黒・落ちる | `export LIBGL_ALWAYS_SOFTWARE=1` を設定してから起動 |
| ロボットが動かない | teleop のターミナルを選択してからキーを押しているか確認 |
| 旋回の角度が大きすぎる | teleop の既定の角速度は 1.0 rad/s。`c` キーで下げるか、`-p turn:=0.3 -p speed:=0.3` を付けて起動する |

### 実際に詰まった点（Docker / Apple Silicon）

| 症状 | 原因と対処 |
|---|---|
| `docker pull` が `connection timed out` で中断する | ネットワークの切断。取得済みのレイヤーは再利用されるので、同じ `docker pull` をもう一度実行すれば続きから進む |
| `colcon build` が setuptools のバージョンで警告・失敗する | `pip install ultralytics` が setuptools を 80 以上に上げてしまい、`colcon-core`（setuptools<80 を要求）と衝突する。`pip install --break-system-packages "setuptools<80"` で戻す |
| `pip install torch` が NVIDIA 関連を大量に落としてくる | aarch64 では PyPI の標準ホイールが CPU 版相当なので、そのまま使って問題ない。README 手順2の CPU 版インデックス指定は x86_64 向け |
| `python3 -I` を付けると numpy 2 系が読まれて import エラーになる | `-I` が `~/.local` を無視して OS 側の numpy を拾うため。通常の `python3` で実行する |

## 動作確認済みの内容

macOS（Apple Silicon）+ Docker（手順 1-B）で確認した。環境は次のとおり。

| 項目 | 内容 |
|---|---|
| コンテナ | `tiryoh/ros2-desktop-vnc:jazzy`（Ubuntu 24.04.5 LTS、aarch64） |
| ROS 2 | Jazzy |
| シミュレータ | Gazebo Sim 8.15.0 |
| AI | ultralytics 8.4.197（YOLOv8n）、torch 2.15.0、CPU 推論 |
| 依存 | numpy 1.26.4、opencv-python 4.11.0 |

確認できた動作：

1. `auto_forward_speed:=0.3` で自動前進 → 高さ比が `stop_ratio`（0.65）を超えた時点で
   `AI BRAKE ON`（記録された値は 0.67〜0.70）
2. ブレーキ後は `/safety/stop` が `true`、`/cmd_vel` の `linear.x` が 0.0 になり、前進指令が入っても停止を維持

`/detection/image` を `record_video` で MP4 に保存できることも確認した（640×480、15fps）。

未確認：「旋回して人が視界から外れると約1秒後に解除される」という解除側の動作は、
このコードのロジックとしては実装しているが、上記環境ではまだ測っていない。

GUI を表示した場合、この環境では実時間比（RTF）が約 0.5 だった。
撮影時は倍速にするか、`gui:=false` で軽くする。

## クレジット

- ベース：N622「ROS2とGazeboの連携によるロボット操作」 https://qiita.com/N622/items/9d89f77d85d9da0af29e
- 人物モデル：Gazebo 公式サンプル `actor.sdf` と同じ Fuel モデル（Mingfei/actor）
- 物体検出：Ultralytics YOLOv8（AGPL-3.0） https://github.com/ultralytics/ultralytics
- 本リポジトリのライセンス：AGPL-3.0（Ultralytics に合わせる）
