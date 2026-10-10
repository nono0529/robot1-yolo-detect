# 仿真物体识别：YOLOv8 OBB 使用

本教程使用 OBB 模型


## 第一步：上传文件——终端 C

下载以下文件：

- `best_obb.pt`：旧版 OBB 权重。
- `isaac_yolov8_obb.py`：配套视觉脚本。

打开一个终端，记为 **终端 C（视觉）**，执行：

```bash
mkdir -p /root/jaka/model_delivery
```

然后通过云端文件上传界面，把两个文件放到以下位置（这是路径，不是命令）：

```text
/root/jaka/model_delivery/best_obb.pt
/root/jaka/isaac_yolov8_obb.py
```

## 第二步：检查权重——继续在终端 C

```bash
conda activate yolov8
source /opt/ros/noetic/setup.bash
source /root/jaka/devel/setup.bash
sha256sum /root/jaka/model_delivery/best_obb.pt
python -c "from ultralytics import YOLO; m=YOLO('/root/jaka/model_delivery/best_obb.pt'); print('task:', m.task); print('classes:', m.names)"
```

输出应包含 `task: obb`，校验码应为：

```text
749e5d8334ace5ce78c587c6bcad3abc2088f605a62ceb965bf197f601e40e3e
```

类别对应：0 手电筒（flashlight）、1 烟雾弹（smoke_grenade）、2 手雷（hand_grenade）、3 弹匣（magazine）、4 口粮（ration）。

## 第三步：启动 ROS 和仿真——终端 A、B

1. **终端 A（ROS）：** 启动 ROS 主节点，保持运行。
2. **终端 B（仿真）：** 启动 Isaac Sim 场景，让仿真开始运行，保持运行。


## 第四步：启动 OBB 视觉——回到终端 C

在 **终端 C** 输入：

```bash
python /root/jaka/isaac_yolov8_obb.py _model:=/root/jaka/model_delivery/best_obb.pt _imgsz:=1024 _conf:=0.35 _depth_aligned:=true _publish_official:=true
```

这条命令启用 OBB 检测、深度位置计算和官方视觉话题

查看终端的 `OBB model loaded` 日志，确认加载的是 `/root/jaka/model_delivery/best_obb.pt`。画面中应显示物体名称、旋转框和 `(u,v,深度m,角度)`。


## 第五步：查看当前识别结果——新开终端 D

新开 **终端 D（检查）**，执行：

```bash
source /opt/ros/noetic/setup.bash
source /root/jaka/devel/setup.bash
rostopic echo -n 1 /robot1_vision/detections
```

收到一帧后，命令自动结束。核对：

- `name` 和画面里的物体一致。
- `center_pixel` 是目标的像素中心。
- `depth_m` 是合理的正数。
- `position_camera_m` 是有效的相机坐标，不是 `null`。
- `yaw_image_deg` 是图像内长轴角度，允许负数。

接着在 **同一个终端 D** 执行：

```bash
rostopic info /yolo_annotated_image
rostopic echo -n 1 /mycaryolo
```

确认标注图像由当前 OBB 节点发布，位置消息正常输出。



## 更新日志

- 提供 YOLOv8 OBB 权重和对应视觉脚本。
