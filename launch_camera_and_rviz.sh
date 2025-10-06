#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
CAMERA_WS_DIR="$(dirname "$SCRIPT_DIR")"

# 设置ROS2环境
source /opt/ros/humble/setup.bash
source "$CAMERA_WS_DIR/install/setup.bash"

# 启动相机节点和RViz2
echo "启动海康威视相机节点..."
ros2 launch hk_camera camera.launch.py &
CAMERA_PID=$!

# 等待相机节点启动并确认图像话题存在
echo "等待相机节点启动..."
sleep 5

# 检查相机节点是否存在
echo "检查相机节点..."
for i in {1..10}; do
    local camera_nodes=$(ros2 node list 2>/dev/null | grep hk_camera)
    if [ -n "$camera_nodes" ]; then
        echo "找到相机节点："
        echo "$camera_nodes"
        # 使用第一个节点显示参数列表
        local first_node=$(echo "$camera_nodes" | head -1)
        echo "节点 $first_node 的可用参数列表："
        ros2 param list $first_node 2>/dev/null | head -10
        break
    fi
    if [ $i -eq 10 ]; then
        echo "警告：未找到相机节点，继续启动RViz2..."
    fi
    sleep 1
done

# 检查图像话题是否存在
echo "检查图像话题..."
for i in {1..10}; do
    if ros2 topic info /image_raw >/dev/null 2>&1; then
        echo "图像话题 /image_raw 已找到"
        break
    fi
    if [ $i -eq 10 ]; then
        echo "警告：未找到图像话题 /image_raw，继续启动RViz2..."
    fi
    sleep 1
done

# 创建rviz配置目录
RVIZ_CONFIG_DIR="$SCRIPT_DIR/rviz"
mkdir -p "$RVIZ_CONFIG_DIR"

# 生成带时间戳的配置文件名
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
RVIZ_CONFIG_FILE="$RVIZ_CONFIG_DIR/camera_view_$TIMESTAMP.rviz"

# 复制工作配置文件到新的位置
if [ -f "$SCRIPT_DIR/config/work_camera_view.rviz" ]; then
    cp "$SCRIPT_DIR/config/work_camera_view.rviz" "$RVIZ_CONFIG_FILE"
    echo "启动RViz2，配置文件已保存到: $RVIZ_CONFIG_FILE"
    rviz2 -d "$RVIZ_CONFIG_FILE" &
    RVIZ_PID=$!
else
    echo "警告：工作配置文件不存在，使用默认配置启动RViz2"
    rviz2 &
    RVIZ_PID=$!
fi

# 如果RViz2配置文件不存在，创建一个简单的配置
if [ ! -f "$SCRIPT_DIR/camera_view.rviz" ]; then
    echo "创建RViz2配置文件..."
    cat > "$SCRIPT_DIR/camera_view.rviz" << EOF
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /Image1
        - /Image1/Topic1
      Splitter Ratio: 0.5
    Tree Height: 565
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.588679016
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz_common/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: Image
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Image Topic: /image_raw
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Image
      Normalize Range: true
      Queue Size: 2
      Transport Hint: raw
      Unreliable: false
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: camera_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.785398006439209
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785398006439209
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002c4fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002c4000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500670065007400730100000041000000e80000000000000000fb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb0000001e0044006900730070006c00610079007300200061006e006400200043006f006e00740072006f006c0100000000000002c40000000000000000000000010000010f000002c4fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002c4000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004b00000003efc0100000002fb0000000800540069006d00650100000000000004b0000002eb00fffffffb0000000800540069006d006501000000000000045000000000000000000000023f000002c400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1200
  X: 65
  Y: 24
EOF
fi

# 等待RViz2启动
echo "等待RViz2启动..."
sleep 3

# 检查RViz2是否仍在运行
if ! kill -0 $RVIZ_PID 2>/dev/null; then
    echo "错误：RViz2启动失败"
    kill $CAMERA_PID 2>/dev/null
    exit 1
fi

echo "============================================="
echo "相机节点和RViz2已启动"
echo "正在初始化相机系统..."
echo ""
echo "监控信息将每5秒更新一次"
echo "============================================="
echo "按Ctrl+C退出"

# 获取相机参数的函数
get_camera_param() {
    local node_name=$1
    local param_name=$2
    # 直接获取参数值，过滤掉描述信息
    local value=$(ros2 param get /$node_name $param_name 2>/dev/null | tail -1 | awk '{print $NF}')
    # 如果值是字符串类型（带引号），去掉引号
    if [[ "$value" == \"*\" ]]; then
        value="${value%\"}"
        value="${value#\"}"
    fi
    # 如果值为空，返回N/A
    if [ -z "$value" ]; then
        echo "N/A"
    else
        echo "$value"
    fi
}

# 获取实际帧率的函数
get_actual_framerate() {
    local value=$(timeout 2 ros2 topic echo /hk_camera/actual_frame_rate --once 2>/dev/null | grep -o 'data: [0-9.]*' | awk -F': ' '{print $2}')
    if [ -z "$value" ]; then
        echo "N/A"
    else
        echo "$value"
    fi
}

# 监控进程状态并显示相机信息
monitor_processes() {
    local count=0
    while true; do
        # 检查相机节点是否还在运行
        if ! kill -0 $CAMERA_PID 2>/dev/null; then
            echo -e "\n相机节点已停止，正在关闭RViz2..."
            kill $RVIZ_PID 2>/dev/null
            break
        fi
        
        # 检查RViz2是否还在运行
        if ! kill -0 $RVIZ_PID 2>/dev/null; then
            echo -e "\nRViz2已停止，正在关闭相机节点..."
            kill $CAMERA_PID 2>/dev/null
            break
        fi
        
        # 每5秒显示一次状态信息
        if [ $((count % 5)) -eq 0 ]; then
            # 先检查节点是否存在
            # 获取所有相机节点（可能有多个）
            local camera_nodes=$(ros2 node list 2>/dev/null | grep hk_camera)
            if [ -z "$camera_nodes" ]; then
                echo -e "\n============================================="
                echo "当前相机工作状态：✗ 节点未找到"
                echo "============================================="
                sleep 5
                continue
            fi
            
            # 使用第一个找到的相机节点
            local camera_node=$(echo "$camera_nodes" | head -1)
            # 去掉开头的斜杠
            camera_node=${camera_node#/}
            
            # 获取各种参数
            local device_ip=$(get_camera_param "$camera_node" "device_ip")
            local image_width=$(get_camera_param "$camera_node" "image_width")
            local image_height=$(get_camera_param "$camera_node" "image_height")
            local pixel_format=$(get_camera_param "$camera_node" "pixel_format")
            local frame_rate=$(get_camera_param "$camera_node" "frame_rate")
            local exposure_time=$(get_camera_param "$camera_node" "exposure_time")
            local gain=$(get_camera_param "$camera_node" "gain")
            local actual_fps=$(get_actual_framerate)
            
            # 检查图像话题是否活跃
            local image_topic_hz=$(timeout 3 ros2 topic hz /image_raw --once 2>/dev/null | grep -o 'average rate: [0-9.]*' | awk -F': ' '{print $2}' || echo "N/A")
            
            # 如果分辨率获取失败，尝试从话题信息获取
            if [ "$image_width" = "N/A" ] || [ "$image_height" = "N/A" ]; then
                local topic_info=$(timeout 2 ros2 topic info /image_raw -v 2>/dev/null | grep -A 5 "Publisher count:" | grep -o "[0-9]*x[0-9]*" | head -1)
                if [ -n "$topic_info" ]; then
                    image_width=$(echo "$topic_info" | cut -d'x' -f1)
                    image_height=$(echo "$topic_info" | cut -d'x' -f2)
                fi
            fi
            
            # 显示状态信息
            echo -e "\n============================================="
            echo "当前相机工作状态：✓ 正常运行"
            echo "设备IP：$device_ip"
            echo "图像分辨率：${image_width}x${image_height}"
            echo "像素格式：$pixel_format"
            echo "设定帧率：$frame_rate fps"
            echo "实际帧率：$actual_fps fps"
            echo "图像发布频率：$image_topic_hz Hz"
            echo "曝光时间：$exposure_time μs"
            echo "增益：$gain"
            echo "============================================="
        fi
        
        sleep 1
        count=$((count + 1))
    done
}

# 启动监控进程并在后台运行
monitor_processes &
MONITOR_PID=$!

# 等待用户中断
trap 'echo -e "\n正在关闭所有进程..."; kill $CAMERA_PID $RVIZ_PID $MONITOR_PID 2>/dev/null; echo "所有进程已关闭"; exit' INT
wait