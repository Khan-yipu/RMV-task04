# HK Camera ROS2 Driver

这个包提供了一个用于Hikvision相机的ROS2驱动程序，支持MVS SDK 4.6。

## 功能特性

- 自动发现和连接Hikvision相机（GigE/USB）
- 相机断开后自动重连
- 高帧率稳定图像采集
- 图像格式转换（sensor_msgs/msg/Image）
- 发布到可配置的话题（默认：/image_raw）
- 动态参数配置（曝光时间、增益、帧率、像素格式）
- **读取当前实际帧率，发布实际帧率到~/actual_frame_rate话题**

### 实际帧率读取

驱动程序现在可以读取相机的实际帧率，这与设定的帧率不同。实际帧率是相机当前真正达到的帧率，可能会受到曝光时间、图像大小等因素的影响。

#### 相关API

- `getActualFrameRate()`: 获取相机的实际帧率
- `getCurrentFrameRate()`: 获取当前设定的帧率

#### 话题

- `~/actual_frame_rate` (std_msgs/Float64): 发布实际帧率，频率1Hz

## 参数配置

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| device_ip | string | "" | 指定相机IP地址（GigE相机） |
| device_serial | string | "" | 指定相机序列号（USB相机） |
| camera_name | string | "hk_camera" | 相机名称 |
| frame_id | string | "camera_link" | 图像帧ID |
| topic_name | string | "image_raw" | 图像话题名称 |
| exposure_time | double | 8000.0 | 曝光时间（微秒） |
| gain | double | 1.0 | 增益 |
| frame_rate | double | 30.0 | 设定帧率（fps） |
| pixel_format | string | "BGR8" | 像素格式（Mono8/BGR8/RGB8） |
| image_width | int | 1920 | 图像宽度 |
| image_height | int | 1080 | 图像高度 |
| auto_reconnect | bool | true | 自动重连 |

## 编译
首先将仓库克隆到自己工作区的`src/`目录下，例如`path/to/camera_ws/src/`

```bash
cd /path/to/camera_ws
colcon build --packages-select hk_camera
source install/setup.bash
```


## 使用方法

### 1. 启动相机节点

```bash
# 使用默认配置
ros2 launch hk_camera camera.launch.py

# 指定相机IP
ros2 launch hk_camera camera.launch.py device_ip:=192.168.1.100

# 指定相机序列号（USB相机）
ros2 launch hk_camera camera.launch.py device_serial:=XXXXXXXXXX

# 自定义参数
ros2 launch hk_camera camera.launch.py frame_rate:=60.0 exposure_time:=5000.0
```

### 2. 查看图像

```bash
ros2 run rqt_image_view rqt_image_view

# 或者使用rviz2
rviz2
```

使用rviz2 在界面中应该能看到类似于这样的输出
![rviz2](./rviz2_output.jpg)

### 3. 监控实际帧率

```bash
ros2 topic echo /hk_camera_node/actual_frame_rate
```

### 4. 动态调整参数

```bash
# 调整曝光时间
ros2 param set /hk_camera_node exposure_time 10000.0

# 调整增益
ros2 param set /hk_camera_node gain 2.0

# 调整帧率
ros2 param set /hk_camera_node frame_rate 60.0
```

## 话题

| 话题名 | 类型 | 描述 |
|--------|------|------|
| /image_raw | sensor_msgs/Image | 图像数据 |
| ~/actual_frame_rate | std_msgs/Float64 | 实际帧率（新增） |


## 注意事项

1. 确保已安装Hikvision MVS SDK 4.6并正确配置环境变量
2. 相机实际帧率可能会受到以下因素影响：
   - 曝光时间（曝光时间过长会降低帧率）
   - 图像大小（分辨率越高，帧率可能越低）
   - 网络带宽（GigE相机）
   - USB带宽（USB相机）

