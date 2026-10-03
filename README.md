# 仿真视觉模型

这份交付包含：

- `best.pt`：训练好的普通 YOLO 检测模型，任务类型为 `detect`。
- `isaac_yolov8_named.py`：加入类别名称、置信度显示的完整视觉脚本
- `README.md`：本说明。

需要使用比赛官方云端环境，保留其 ROS 工作空间、消息包及依赖。

## 一、上传文件

打开云端 JupyterLab，将两个文件上传到以下位置：

```text
/root/jaka/model_delivery/best.pt
/root/jaka/isaac_yolov8_named.py
```

`model_delivery` 文件夹没有就新建。

## 二、检查模型

打开终端，逐条执行：

```bash
conda activate yolov8
```

```bash
python -c "import ultralytics; from ultralytics import YOLO; m=YOLO('/root/jaka/model_delivery/best.pt'); print('version:',ultralytics.__version__); print('task:',m.task); print('classes:',m.names)"
```

应该显示 `task: detect`，类别如下：

| 编号 | 类别名称 | 中文 |
| --- | --- | --- |
| 0 | flashlight | 手电筒 |
| 1 | smoke_grenade | 烟雾弹 |
| 2 | hand_grenade | 手雷 |
| 3 | magazine | 弹匣 |
| 4 | ration | 干粮 |

可再执行校验：

```bash
sha256sum /root/jaka/model_delivery/best.pt
```

预期结果：

```text
1b5858b8b76362d5f23dc766aa91743e8696dbbf38722837d15ad9fcac07019b
```

校验一致表示上传的文件与交付模型相同。

## 三、启动视觉

先按照官方教程启动 ROS 和仿真，让仿真运行并发布相机图像。以下命令仅替换视觉程序，不启动大模型或抓取控制。

如果旧视觉程序已经运行，先到它的终端按 `Ctrl+C` 停止。不要同时启动两个视觉节点；如果视觉由 launch 启动，应先停止对应启动任务，再按官方教程分别启动所需节点。测试时先不要发送抓取命令。

新开视觉终端，逐条执行：

```bash
conda activate yolov8
```

```bash
source /root/jaka/devel/setup.bash
```

```bash
python /root/jaka/isaac_yolov8_named.py
```


## 四、

检测框标签示例：

```text
flashlight 72%
(568,232,2.11m,-90.00)
```

- 第一行：YOLO 输出的类别名称和置信度，不经过大模型。
- 第二行：像素坐标 u、v，深度（米），原脚本估计的平面角度（度）。

对照当前画面检查：手电筒是否被框住、框上是否写着 `flashlight`，其他物体也按上表逐一核对。同时观察漏检、重复框和遮挡情况。


也可另外打开终端查看视觉发布的数据：

```bash
source /root/jaka/devel/setup.bash
rostopic echo /mycaryolo
```

其中 `name` 是类别，`conf` 是置信度，`pose.position` 是相机坐标计算结果。
