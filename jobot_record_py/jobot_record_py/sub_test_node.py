#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import wave
import numpy as np
import os
import time

from jobot_audio.msg import AudioFrame


class AudioSaver(Node):
    """
    ROS2 音频保存节点
    - 订阅 /audio/raw
    - 将接收的音频帧写入 wav 文件
    """

    def __init__(self):
        super().__init__('audio_saver')

        # ---- 参数配置 ----
        self.declare_parameter("output_dir", "/tmp")
        self.declare_parameter("file_prefix", "audio_record")

        self.output_dir = self.get_parameter("output_dir").value
        self.file_prefix = self.get_parameter("file_prefix").value

        os.makedirs(self.output_dir, exist_ok=True)
        filename = os.path.join(
            self.output_dir, f"{self.file_prefix}_{int(time.time())}.wav"
        )
        self.get_logger().info(f"保存音频到 {filename}")

        # wav 文件句柄（收到第一帧时才创建，保证参数正确）
        self.wav_file = None
        self.filename = filename

        # ---- 订阅者 ----
        self.subscription = self.create_subscription(
            AudioFrame,
            "/audio/raw",
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: AudioFrame):
        """收到音频帧时写入文件"""
        if self.wav_file is None:
            # 创建 wav 文件
            self.wav_file = wave.open(self.filename, 'wb')
            self.wav_file.setnchannels(msg.channels)
            self.wav_file.setsampwidth(2)  # int16 = 2 bytes
            self.wav_file.setframerate(msg.sample_rate)

        # msg.data 是 int16 列表，需要转为 bytes
        data_np = np.array(msg.data, dtype=np.int16)
        self.wav_file.writeframes(data_np.tobytes())

    def destroy_node(self):
        if self.wav_file is not None:
            self.wav_file.close()
            self.get_logger().info(f"音频文件已保存: {self.filename}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AudioSaver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
