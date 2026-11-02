"""AI安全停止ノード（YOLOによる人物検出 → 自動ブレーキ）

データの流れ:
    キーボード操作 (/cmd_vel_teleop) ─┐
                                       ├─> [このノード] ─> /cmd_vel ─> Gazebo のロボット
    前方カメラ (/camera/image) ──> YOLO ┘

- YOLO（COCOで学習済み、推論のみ）でカメラ画像から「person」を検出する。
- 人のバウンディングボックスの高さが画像の stop_ratio 以上（= 近い）なら、ブレーキをかける。
- ブレーキ中は「前進」だけを禁止する（旋回と後退はできるので、人を避けられる）。
- フェイルセーフ: ブレーキは一度かかると保持される。解除の条件は次の2つが両方そろうこと。
    1. 人が十分遠い／見えない状態が release_frames 回連続した
    2. 操作者が前進以外の指令（停止・旋回・後退）を出した
  近づきすぎて人が画面からはみ出すと YOLO が見失うことがあるため、
  「見えなくなった = 安全」とは判断しない。
"""

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from ultralytics import YOLO

PERSON_CLASS_ID = 0  # COCO の "person"


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    """sensor_msgs/Image を OpenCV の BGR 画像へ変換する。

    cv_bridge を使わないのは、pip で入れた numpy / OpenCV と
    ROS 付属のバイナリの組み合わせで衝突しやすいため。
    """
    channels = 4 if msg.encoding in ('rgba8', 'bgra8') else 3
    frame = np.frombuffer(msg.data, dtype=np.uint8)
    frame = frame.reshape(msg.height, msg.step // channels, channels)[:, : msg.width, :]
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if msg.encoding == 'rgba8':
        return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
    if msg.encoding == 'bgra8':
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame.copy()


def bgr_to_image_msg(frame: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = msg.width * 3
    msg.data = frame.tobytes()
    return msg


class SafetyStopNode(Node):
    def __init__(self):
        super().__init__('safety_stop')

        # ---- パラメータ ----
        self.declare_parameter('model', 'yolov8n.pt')   # 学習済みモデル（初回に自動ダウンロード）
        self.declare_parameter('confidence', 0.4)       # 検出の信頼度しきい値
        self.declare_parameter('stop_ratio', 0.65)      # 人の高さ / 画像の高さ がこれ以上で停止
        self.declare_parameter('release_ratio', 0.3)    # これ未満を「十分遠い」とみなす
        self.declare_parameter('release_frames', 5)     # 「十分遠い」が何回連続したら解除可能か
        self.declare_parameter('inference_hz', 5.0)     # 推論の頻度
        self.declare_parameter('auto_forward_speed', 0.0)  # >0 ならキー入力がない間、自動で前進
        self.declare_parameter('show_window', True)     # 検出結果ウィンドウを表示するか

        p = self.get_parameter
        self.model = YOLO(p('model').value)
        self.confidence = float(p('confidence').value)
        self.stop_ratio = float(p('stop_ratio').value)
        self.release_ratio = float(p('release_ratio').value)
        self.release_frames = int(p('release_frames').value)
        self.auto_speed = float(p('auto_forward_speed').value)
        self.show_window = bool(p('show_window').value)

        # ---- 状態 ----
        self.latest_image = None
        self.teleop_cmd = None          # 最後に受け取ったキーボード指令
        self.brake = False              # ブレーキ中か
        self.clear_count = 0            # 「十分遠い」が連続した回数
        self.operator_ack = False       # ブレーキ後に前進以外の操作があったか
        self.person_ratio = 0.0

        # ---- 通信 ----
        self.create_subscription(Twist, '/cmd_vel_teleop', self.on_teleop, 10)
        self.create_subscription(Image, '/camera/image', self.on_image, 1)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.det_pub = self.create_publisher(Image, '/detection/image', 1)
        self.stop_pub = self.create_publisher(Bool, '/safety/stop', 10)
        self.ratio_pub = self.create_publisher(Float32, '/safety/person_ratio', 10)

        self.create_timer(1.0 / float(p('inference_hz').value), self.run_inference)
        self.create_timer(0.05, self.publish_cmd)  # 20Hz で速度指令を出す

        self.get_logger().info(
            f'AI safety stop ready: model={p("model").value}, stop_ratio={self.stop_ratio}'
        )

    # キーボード操作の指令を保存
    def on_teleop(self, msg: Twist):
        self.teleop_cmd = msg
        if self.brake and msg.linear.x <= 0.0:
            self.operator_ack = True  # 停止・旋回・後退 = 操作者が状況を把握した

    def on_image(self, msg: Image):
        self.latest_image = msg

    # ---- AI（推論のみ）----
    def run_inference(self):
        if self.latest_image is None:
            return
        msg = self.latest_image
        frame = image_msg_to_bgr(msg)

        result = self.model.predict(
            frame, classes=[PERSON_CLASS_ID], conf=self.confidence, verbose=False
        )[0]

        # 最も大きく映っている人（= 最も近い人）の高さ比を求める
        ratio = 0.0
        for x1, y1, x2, y2 in result.boxes.xyxy.cpu().numpy():
            ratio = max(ratio, (y2 - y1) / frame.shape[0])
        self.person_ratio = float(ratio)
        self.update_brake(ratio)

        self.stop_pub.publish(Bool(data=self.brake))
        self.ratio_pub.publish(Float32(data=self.person_ratio))
        self.publish_debug_image(result.plot(), msg.header)

    def update_brake(self, ratio: float):
        if ratio >= self.stop_ratio:
            if not self.brake:
                self.get_logger().warn(f'Person close (ratio={ratio:.2f}) -> AI BRAKE ON')
            self.brake = True
            self.clear_count = 0
            self.operator_ack = False
            return
        if not self.brake:
            return
        self.clear_count = self.clear_count + 1 if ratio < self.release_ratio else 0
        if self.clear_count >= self.release_frames and self.operator_ack:
            self.brake = False
            self.get_logger().info('Path clear and operator responded -> AI BRAKE OFF')

    def publish_debug_image(self, annotated: np.ndarray, header):
        """動画用: 検出枠 + 状態表示を描いた画像を出す"""
        h, w = annotated.shape[:2]
        if self.brake:
            color, label = (0, 0, 255), 'AI BRAKE: STOP'
            cv2.rectangle(annotated, (0, 0), (w - 1, h - 1), color, 8)
        else:
            color, label = (0, 160, 0), 'AI BRAKE: GO'
        cv2.rectangle(annotated, (0, 0), (w, 70), (255, 255, 255), -1)
        cv2.putText(annotated, label, (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.putText(
            annotated,
            f'person height ratio: {self.person_ratio:.2f} (stop >= {self.stop_ratio:.2f})',
            (12, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1,
        )
        self.det_pub.publish(bgr_to_image_msg(annotated, header))
        if self.show_window:
            cv2.imshow('AI safety stop (YOLO)', annotated)
            cv2.waitKey(1)

    # ---- 速度指令 ----
    def publish_cmd(self):
        cmd = Twist()
        if self.teleop_cmd is not None:
            # teleop_twist_keyboard はキーを押したときに一度だけ送るので、最後の指令を保持する
            cmd.linear.x = self.teleop_cmd.linear.x
            cmd.angular.z = self.teleop_cmd.angular.z
        else:
            # キー入力がまだない間は、自動前進（auto_forward_speed）
            cmd.linear.x = self.auto_speed

        # AI ブレーキ: 前進だけを禁止する
        if self.brake and cmd.linear.x > 0.0:
            cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = SafetyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        if node.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
