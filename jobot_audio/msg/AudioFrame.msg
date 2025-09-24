# 头部，包含时间戳和frame_id
std_msgs/Header header

# 采样率
int32 sample_rate

# 声道数
int32 channels

# 格式（0=PCM16, 1=float32）
int32 sample_format

# 每一帧音频数据（int16 PCM）
int16[] data
