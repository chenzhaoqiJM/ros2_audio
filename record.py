import pyaudio
import wave

def record_audio(filename="output.wav", record_seconds=5, rate=48000, channels=1, chunk=1024):
    audio = pyaudio.PyAudio()

    # 打开输入流（录音设备）
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

    print(type(data))
    print(data)
    print(len(data))

    print("录音结束。")

    # 停止并关闭流
    stream.stop_stream()
    stream.close()
    audio.terminate()

    # 保存到 wav 文件
    wf = wave.open(filename, 'wb')
    wf.setnchannels(channels)
    wf.setsampwidth(audio.get_sample_size(pyaudio.paInt16))
    wf.setframerate(rate)
    wf.writeframes(b''.join(frames))
    wf.close()

if __name__ == "__main__":
    record_audio("test.wav", record_seconds=5)
