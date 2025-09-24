import pyaudio
import wave
import numpy as np
import matplotlib.pyplot as plt

def record_audio(filename="output.wav", record_seconds=3, rate=44100, channels=1, chunk=1024):
    audio = pyaudio.PyAudio()

    # 打开输入流
    stream = audio.open(format=pyaudio.paInt16,
                        channels=channels,
                        rate=rate,
                        input=True,
                        frames_per_buffer=chunk)

    print("开始录音...")
    frames = []

    for _ in range(0, int(rate / chunk * record_seconds)):
        data = stream.read(chunk)
        frames.append(data)

    print("录音结束。")

    # 停止并关闭流
    stream.stop_stream()
    stream.close()
    audio.terminate()

    # 保存为 wav 文件
    wf = wave.open(filename, 'wb')
    wf.setnchannels(channels)
    wf.setsampwidth(audio.get_sample_size(pyaudio.paInt16))
    wf.setframerate(rate)
    wf.writeframes(b''.join(frames))
    wf.close()

    return b''.join(frames), rate, channels

# 录音并获取原始字节数据
raw_bytes, rate, channels = record_audio("test.wav", record_seconds=3)

# 解析字节为 int16 数组
samples = np.frombuffer(raw_bytes, dtype=np.int16)

# 生成时间轴
time_axis = np.linspace(0, len(samples) / rate, num=len(samples))

# 可视化
plt.figure(figsize=(10, 4))
plt.plot(time_axis, samples, linewidth=0.5)
plt.title("Audio Waveform")
plt.xlabel("Time (s)")
plt.ylabel("Amplitude")
plt.show()
