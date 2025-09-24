#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
try:
    package_share_directory = get_package_share_directory('jobot_vad_py')
except:
    package_share_directory = ''
    print('NO INSTALL MODE')

import numpy as np
import os
from collections import deque
from scipy.signal import resample, resample_poly

import onnxruntime as ort
from std_msgs.msg import Header
from jobot_audio.msg import AudioFrame
from jobot_audio.msg import VADResult  # 你需要新建 msg/VADResult.msg

WIN_SAMPLES = 512   # 32 ms @ 16k
CTX_SAMPLES = 64
RATE_VAD    = 16000


# -------- Silero VAD 封装 -------- #
class SileroVAD:
    def __init__(self, model_path="silero_vad.onnx"):
        self.sess  = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
        self.state = np.zeros((2, 1, 128), dtype=np.float32)
        self.ctx   = np.zeros((1, CTX_SAMPLES), dtype=np.float32)
        self.sr    = np.array(RATE_VAD, dtype=np.int64)

    def reset(self):
        self.state.fill(0)
        self.ctx.fill(0)

    def __call__(self, wav: np.ndarray) -> float:
        """输入 (512,) float32[-1,1]"""
        wav = wav[np.newaxis, :]   # (1,512)
        x = np.concatenate((self.ctx, wav), axis=1)  # (1,576)
        self.ctx = x[:, -CTX_SAMPLES:]

        prob, self.state = self.sess.run(
            None,
            {"input": x.astype(np.float32),
             "state": self.state,
             "sr":    self.sr}
        )
        return float(prob)


# -------- VAD ROS2 节点 -------- #
class VADNode(Node):
    def __init__(self):
        super().__init__("vad_node")

        self.declare_parameter("model_path", os.path.join(package_share_directory, "model/silero_vad.onnx"))
        self.declare_parameter("trig_on", 0.23)
        self.declare_parameter("trig_off", 0.2)

        model_path = self.get_parameter("model_path").value
        self.vad = SileroVAD(model_path)

        self.trig_on  = self.get_parameter("trig_on").value
        self.trig_off = self.get_parameter("trig_off").value
        self.hist     = deque(maxlen=10)

        self.sub_audio = self.create_subscription(
            AudioFrame, "/audio/raw", self.audio_cb, 10)

        self.pub_vad = self.create_publisher(VADResult, "vad_out", 10)

        self.speech_detected = False

        self.get_logger().info("✅ VAD 节点已启动，等待 audio_in 话题...")

    def audio_cb(self, msg: AudioFrame):
        # (1) 转 numpy int16
        wav_src = np.array(msg.data, dtype=np.int16)
        # self.get_logger().info(f"receve wav_src Len: {len(wav_src)}")

        # (2) 重采样到 16k 单声道
        if msg.sample_rate != RATE_VAD:
            n_samples = int(len(wav_src) * RATE_VAD / msg.sample_rate)
            wav = resample(wav_src, n_samples).astype(np.int16)
            # wav = resample_poly(wav_src, RATE_VAD, msg.sample_rate).astype(np.int16)

            # self.get_logger().info(
            #     f"🔄 重采样: {msg.sample_rate}Hz -> {RATE_VAD}Hz, {len(wav_src)} → {n_samples}"
            # )

        # 如果多声道，取第一个通道
        if msg.channels > 1:
            self.get_logger().info(f"🎚 多声道 {msg.channels} → 取第1通道")
            wav = wav[::msg.channels]

        # (3) 分帧，每帧 512 点

        frame = wav / 32768.0
        if frame.max() != frame.min():  # 避免除零
    # 线性拉伸到 [-0.8, 0.8]
            frame = 1.6 * (frame - frame.min()) / (frame.max() - frame.min()) - 0.8
        else:
            frame = np.zeros_like(frame)


        if len(frame) < WIN_SAMPLES:
            frame = np.pad(frame, (0, WIN_SAMPLES - len(frame)), mode='constant')

        # SileroVAD 推理
        p = self.vad(frame)
        # self.hist.append(p)
        # prob_avg = np.mean(self.hist)

        # 双阈值 VAD 状态更新
        if p > self.trig_on:
            self.speech_detected = True
        else:
            self.speech_detected = False


        # --- 打印每帧信息 ---
        self.get_logger().info(f"prob={p:.3f}, avg={p:.3f}, speech={self.speech_detected}")

        

        # 发布结果
        out = VADResult()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.prob = float(p)
        out.is_speech = bool(self.speech_detected)
        self.pub_vad.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VADNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
