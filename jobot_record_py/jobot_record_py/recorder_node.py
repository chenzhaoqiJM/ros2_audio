#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import pyaudio
import numpy as np
import threading
import time

from std_msgs.msg import Header
from jobot_audio.msg import AudioFrame


class AudioRecorder(Node):
    """
    ROS2 录音节点
    - 采集音频数据
    - 发布到 /audio/raw
    """

    def __init__(self):
        super().__init__('audio_recorder')

        # ---- 参数配置 ----
        self.declare_parameter("device_index", 0)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 1)
        self.declare_parameter("frame_size", 1024)  # 每次采样点数
        self.declare_parameter("format", "pcm16")   # 支持 pcm16 / float32

        self.device_index = self.get_parameter("device_index").value
        self.sample_rate = self.get_parameter("sample_rate").value
        self.channels = self.get_parameter("channels").value
        self.frame_size = self.get_parameter("frame_size").value
        self.format = self.get_parameter("format").value

        # 格式映射
        self.FORMAT_MAP = {
            "pcm16": pyaudio.paInt16,
            "float32": pyaudio.paFloat32
        }
        if self.format not in self.FORMAT_MAP:
            self.get_logger().error(f"不支持的音频格式: {self.format}")
            raise ValueError("Invalid audio format")

        self.pa = pyaudio.PyAudio()

        # ---- 打开音频流 ----
        try:
            self.stream = self.pa.open(
                format=self.FORMAT_MAP[self.format],
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.frame_size,
                input_device_index=self.device_index
            )
        except Exception as e:
            self.get_logger().error(f"打开音频设备失败: {e}")
            raise e

        # ---- 发布者 ----
        self.publisher_ = self.create_publisher(AudioFrame, "/audio/raw", 10)

        # ---- 录音线程 ----
        self.is_recording = True
        self.thread = threading.Thread(target=self._record_loop, daemon=True)
        self.thread.start()

        self.get_logger().info("Audio recorder node started.")

    def _record_loop(self):
        """后台采集音频并发布"""
        while rclpy.ok() and self.is_recording:
            try:
                frame = self.stream.read(self.frame_size, exception_on_overflow=False)
                # 转换为 numpy 数组
                if self.format == "pcm16":
                    audio_np = np.frombuffer(frame, dtype=np.int16)
                elif self.format == "float32":
                    audio_np = np.frombuffer(frame, dtype=np.float32)
                    # 为了消息兼容，转 int16
                    audio_np = (audio_np * 32767).astype(np.int16)

                # ---- 构建消息 ----
                # self.get_logger().info(f"audio_np Len: {len(audio_np)}")
                msg = AudioFrame()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "mic_" + str(self.device_index)

                msg.sample_rate = self.sample_rate
                msg.channels = self.channels
                msg.sample_format = 0 if self.format == "pcm16" else 1
                msg.data = audio_np.tolist()

                self.publisher_.publish(msg)

            except Exception as e:
                self.get_logger().warn(f"录音失败: {e}")
                time.sleep(0.1)

    def destroy_node(self):
        """节点关闭时释放资源"""
        self.is_recording = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.stream.stop_stream()
        self.stream.close()
        self.pa.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AudioRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
