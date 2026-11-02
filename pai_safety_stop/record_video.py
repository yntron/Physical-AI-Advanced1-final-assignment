"""検出結果の映像 (/detection/image) を MP4 に保存するノード（動画提出用の保険）

使い方:
    ros2 run pai_safety_stop record_video --ros-args -p output:=demo.mp4
Ctrl+C で保存して終了する。
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self):
        super().__init__('record_video')
        self.declare_parameter('output', 'demo.mp4')
        self.declare_parameter('fps', 15.0)
        self.output = self.get_parameter('output').value
        fps = float(self.get_parameter('fps').value)

        self.latest = None
        self.writer = None
        self.frames = 0
        self.fps = fps
        self.create_subscription(Image, '/detection/image', self.on_image, 1)
        # 推論は 5Hz 程度なので、一定の fps で最新フレームを書き込んで滑らかな時間軸にする
        self.create_timer(1.0 / fps, self.write_frame)
        self.get_logger().info(f'Recording /detection/image -> {self.output} (Ctrl+C to stop)')

    def on_image(self, msg: Image):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self.latest = frame.copy()

    def write_frame(self):
        if self.latest is None:
            return
        if self.writer is None:
            h, w = self.latest.shape[:2]
            self.writer = cv2.VideoWriter(
                self.output, cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (w, h)
            )
        self.writer.write(self.latest)
        self.frames += 1

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(
                f'Saved {self.output} ({self.frames / self.fps:.1f} s)'
            )


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
