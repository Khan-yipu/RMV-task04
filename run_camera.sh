#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
# camera_ws是src目录的父目录
CAMERA_WS_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# 设置ROS2环境
source /opt/ros/humble/setup.bash
# 尝试多个可能的install路径
if [ -f "$CAMERA_WS_DIR/install/setup.bash" ]; then
    source "$CAMERA_WS_DIR/install/setup.bash"
elif [ -f "$CAMERA_WS_DIR/install/local_setup.bash" ]; then
    source "$CAMERA_WS_DIR/install/local_setup.bash"
else
    echo "警告：未找到install/setup.bash，尝试使用已安装的包"
fi

# 清理函数
cleanup() {
    echo -e "\n正在关闭所有进程..."
    if [ ! -z "$CAMERA_PID" ]; then
        kill $CAMERA_PID 2>/dev/null
    fi
    if [ ! -z "$RVIZ_PID" ]; then
        kill $RVIZ_PID 2>/dev/null
    fi
    if [ ! -z "$MONITOR_PID" ]; then
        kill $MONITOR_PID 2>/dev/null
    fi
    echo "所有进程已关闭"
    exit 0
}

# 信号处理
trap cleanup SIGINT SIGTERM

# 检查相机节点是否存在
check_camera_node() {
    ros2 node list 2>/dev/null | grep -q "hk_camera"
}

# 检查相机是否真正连接并工作
check_camera_connected() {
    # 检查节点是否存在
    if ! check_camera_node; then
        return 1
    fi
    
    # 检查实际帧率是否为0（表示未连接）
    local actual_fps=$(get_actual_framerate)
    if [[ "$actual_fps" == "0" ]] || [[ "$actual_fps" == "N/A" ]]; then
        return 1
    fi
    
    # 检查是否有图像话题在发布
    local topic_hz=$(get_image_topic_hz)
    if [[ "$topic_hz" == "N/A" ]] || [[ -z "$topic_hz" ]]; then
        return 1
    fi
    
    return 0
}

# 获取相机参数
get_camera_param() {
    local param_name=$1
    local value=$(ros2 param get /hk_camera $param_name 2>/dev/null | tail -1 | awk '{print $NF}')
    # 去掉引号
    if [[ "$value" == \"*\" ]]; then
        value="${value%\"}"
        value="${value#\"}"
    fi
    echo "$value"
}

# 获取实际帧率
get_actual_framerate() {
    local value=$(timeout 2 ros2 topic echo /hk_camera/actual_frame_rate --once 2>/dev/null | grep -o 'data: [0-9.]*' | awk -F': ' '{print $2}')
    echo "${value:-N/A}"
}

# 获取图像话题频率
get_image_topic_hz() {
    local value=$(timeout 3 ros2 topic hz /image_raw --once 2>/dev/null | grep -o 'average rate: [0-9.]*' | awk -F': ' '{print $2}')
    echo "${value:-N/A}"
}

# 监控函数
monitor_camera() {
    local count=0
    while [ $count -lt 1000 ]; do  # 防止无限循环
        if check_camera_node; then
            if check_camera_connected; then
                # 每5秒显示一次状态
                if [ $((count % 3)) -eq 0 ]; then
                    # 获取各种参数
                    device_ip=$(get_camera_param "device_ip")
                    image_width=$(get_camera_param "image_width")
                    image_height=$(get_camera_param "image_height")
                    pixel_format=$(get_camera_param "pixel_format")
                    frame_rate=$(get_camera_param "frame_rate")
                    exposure_time=$(get_camera_param "exposure_time")
                    gain=$(get_camera_param "gain")
                    actual_fps=$(get_actual_framerate)
                    image_topic_hz=$(get_image_topic_hz)
                    
                    # 显示状态信息
                    echo -e "\n============================================="
                    echo "当前相机工作状态：✓ 正常运行 ($(date '+%H:%M:%S'))"
                    echo "设备IP：${device_ip:-N/A}"
                    echo "图像分辨率：${image_width:-N/A}x${image_height:-N/A}"
                    echo "像素格式：${pixel_format:-N/A}"
                    echo "设定帧率：${frame_rate:-N/A} fps"
                    echo "实际帧率：${actual_fps} fps"
                    echo "图像发布频率：${image_topic_hz} Hz"
                    echo "曝光时间：${exposure_time:-N/A} μs"
                    echo "增益：${gain:-N/A}"
                    echo "============================================="
                    
                    # 提示动态调整参数的方法
                    echo -e "\n提示：动态调整参数的方法："
                    echo "  ros2 param set /hk_camera exposure_time 10000.0  # 调整曝光时间"
                    echo "  ros2 param set /hk_camera gain 2.0              # 调整增益"
                    echo "  ros2 param set /hk_camera pixel_format \"RGB8\"   # 修复颜色问题(人脸蓝色)"
                    echo "  ros2 param set /hk_camera frame_rate 60.0       # 调整帧率"
                    echo "  ros2 run rqt_reconfigure rqt_reconfigure       # GUI调整参数"
                fi
            else
                # 相机节点存在但未真正连接
                if [ $((count % 3)) -eq 0 ]; then
                    echo -e "\n============================================="
                    echo "当前相机工作状态：✗ 连接失败 ($(date '+%H:%M:%S'))"
                    echo "相机节点存在但未成功连接到设备"
                    echo "请检查相机设备是否连接或已被其他程序占用"
                    echo "============================================="
                fi
            fi
        else
            echo -e "\n警告：相机节点未运行"
        fi
        
        sleep 1
        count=$((count + 1))
    done
}

# 主程序
echo "============================================="
echo "海康威视相机启动脚本"
echo "============================================="

# 启动相机节点
echo "启动相机节点..."
ros2 launch hk_camera camera.launch.py start_rviz:=false &
CAMERA_PID=$!

# 等待相机节点启动
echo "等待相机节点启动..."
sleep 5

# 检查相机节点
if ! check_camera_node; then
    echo "错误：相机节点启动失败"
    cleanup
    exit 1
fi

echo "相机节点启动成功"

# 颜色问题已在代码中修复（Bayer格式转换为RGB8）
# echo "颜色问题已在代码层面修复"

# 启动RViz2
echo "启动RViz2..."
# 查找配置文件
CONFIG_FILE=""
if [ -f "$SCRIPT_DIR/config/work_camera_view.rviz" ]; then
    CONFIG_FILE="$SCRIPT_DIR/config/work_camera_view.rviz"
elif [ -f "$CAMERA_WS_DIR/src/hk_camera/config/work_camera_view.rviz" ]; then
    CONFIG_FILE="$CAMERA_WS_DIR/src/hk_camera/config/work_camera_view.rviz"
fi

if [ -n "$CONFIG_FILE" ]; then
    echo "使用配置文件: $CONFIG_FILE"
    rviz2 -d "$CONFIG_FILE" &
else
    echo "使用默认配置启动RViz2"
    rviz2 &
fi
RVIZ_PID=$!

# 等待RViz2启动
sleep 3

# 启动监控
echo "启动监控系统..."
monitor_camera &
MONITOR_PID=$!

echo -e "\n============================================="
echo "所有服务已启动"
echo "相机节点 PID: $CAMERA_PID"
echo "RViz2 PID: $RVIZ_PID"
echo "监控 PID: $MONITOR_PID"
echo "============================================="
echo "按 Ctrl+C 退出"

# 等待监控进程
wait $MONITOR_PID