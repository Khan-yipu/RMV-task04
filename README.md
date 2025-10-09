# Hikvision 相机 ROS 2 驱动

这是一个用于海康威视工业相机的 ROS 2 软件包，提供图像采集和参数控制功能。

## 功能特性

- 支持海康威视 GigE 和 USB 相机
- 实时图像采集和发布
- 可配置的相机参数（曝光、增益、帧率等）
- 支持触发模式
- 自动重连机制
- RViz 可视化支持

## 安装依赖

确保已安装 ROS 2 Humble 以及海康相机的 SDK（通常安装在`opt/MVS/`），然后安装依赖：

```bash
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
```

或：
如果想方便地安装 ROS 2, 一个比较方便的办法是借助于fishros 的一键安装命令：
```bash
wget http://fishros.com/install -O fishros && . fishros
```

或者使用`rosdep`:
```bash
rosdep install --from-paths src --ignore-src -y
```

## 构建

把该项目克隆到工作空间的`src`目录下面，如`example_path/example_ws/src/`。然后在`example_ws`目录下运行：

```bash
colcon build --packages-select hik_camera
source install/setup.bash
```

## 使用方法

### 启动相机节点：

```bash
ros2 launch hik_camera hik_camera_launch.py
```

我运行保存的一些图片放在了`img/`。

使用自定义配置文件：

```bash
ros2 launch hik_camera hik_camera_launch.py config_path:=/path/to/your/config.yaml
```

## 参数配置

### 配置文件方式
编辑 `config/hik_camera_params.yaml` 文件：

```yaml
camera_driver:
  ros__parameters:
    exposure: 4000.0        # 曝光时间（微秒）
    image_gain: 16.9807     # 图像增益
    use_trigger: false      # 是否启用触发模式
    fps: 165.0              # 帧率设置
    pixel_format_code: 17301513  # 像素格式代码
    camera_frame: "camera_optical_frame"  # 坐标系名称
    serial_number: ""       # 可选：通过序列号指定相机
```

### 命令行参数覆盖
启动时动态设置参数：
```bash
# 设置曝光时间
ros2 launch hik_camera hik_camera_launch.py exposure:=8000.0

# 设置帧率和增益
ros2 launch hik_camera hik_camera_launch.py fps:=30.0 image_gain:=10.0

# 使用自定义配置文件
ros2 launch hik_camera hik_camera_launch.py config_path:=/path/to/custom_params.yaml
```

### 运行时动态调整
```bash
# 查看当前参数
ros2 param list /camera_driver

# 获取参数值
ros2 param get /camera_driver exposure

# 设置参数
ros2 param set /camera_driver exposure 6000.0
ros2 param set /camera_driver fps 120.0
```

## 节点信息

- **节点名称**: `camera_driver`
- **发布话题**: `/image_raw` (sensor_msgs/Image)
- **参数服务**: 支持动态参数配置






