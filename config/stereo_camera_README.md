# 双目相机配置说明

## 概述

OceanSim 现已支持双目水下相机系统。双目相机由两个并排的 UW_Camera 组成，通过配置文件可以灵活调整基线（baseline）等参数。

## 配置文件

配置文件位置：`config/stereo_camera.yaml`

### 配置参数说明

```yaml
stereo_camera:
  # 基线长度（单位：米）
  # 左相机和右相机中心之间的距离
  # 典型值：
  #   - 窄基线 (0.05-0.1m): 近距离检测
  #   - 中等基线 (0.1-0.2m): 通用水下机器人应用
  #   - 宽基线 (0.2-0.3m): 远距离深度估计
  baseline: 0.12

  # 焦距（单位：毫米）
  focal_length: 2.1

  # 分辨率（宽 x 高，单位：像素）
  resolution: [1920, 1080]

  # 更新频率（Hz，null 表示自动）
  frequency: 30

  # 裁剪范围（近裁剪面, 远裁剪面），单位：米
  clipping_range: [0.1, 100.0]

  # 相机在局部坐标系中的平移偏移（x, y, z），单位：米
  # 注意：Y 分量会根据基线自动调整为 +/- baseline/2
  translation: [0.3, 0.0, 0.1]

  # 相机朝向，四元数格式（w, x, y, z）
  orientation: [1.0, 0.0, 0.0, 0.0]

  # ROS2 配置
  ros2:
    enable: true
    base_topic: "/oceansim/robot/stereo"
    publish_frequency: 5
    jpeg_quality: 50

  # 视口可视化
  viewport:
    enable: true
    show_both: true

  # 数据录制
  recording:
    directory: null
    format: "png"
```

## 使用方法

### 1. 启用双目相机

在 `ui_builder.py` 中，`_on_init()` 方法内设置：

```python
self._use_stereo_camera = True  # 启用双目相机
self._stereo_config_path = "config/stereo_camera.yaml"  # 配置文件路径
```

### 2. 修改基线参数

编辑 `config/stereo_camera.yaml` 文件：

```yaml
stereo_camera:
  baseline: 0.15  # 修改为你需要的基线长度（单位：米）
```

### 3. 运行示例

1. 打开 Isaac Sim 并加载 OceanSim 扩展
2. 在 UI 中勾选 "Underwater Camera" 选项
3. 点击 "LOAD" 按钮加载场景
4. 点击 "RUN" 运行仿真

### 4. ROS2 话题

双目相机会发布以下 ROS2 话题：

- `/oceansim/robot/stereo/left` - 左相机图像（CompressedImage）
- `/oceansim/robot/stereo/right` - 右相机图像（CompressedImage）

## 基线选择建议

| 应用场景 | 推荐基线 | 说明 |
|---------|---------|------|
| 近距离检测（< 2m） | 0.05-0.1m | 更好的近距离精度 |
| 中距离操作（2-10m） | 0.1-0.15m | 平衡精度和视差范围 |
| 远距离导航（> 10m） | 0.15-0.3m | 更大的可测量深度范围 |

## 代码架构

### 文件结构

```
isaacsim/oceansim/sensors/
├── UW_Camera.py          # 单目水下相机（基础类）
└── StereoUWCamera.py     # 双目水下相机（新增）

config/
├── stereo_camera.yaml    # 双目相机配置文件
└── stereo_camera_README.md  # 本说明文件
```

### 类关系

```
UW_Camera (单目相机基类)
    ↑
StereoUWCamera (双目相机)
    ├── left_cam: UW_Camera (左相机)
    └── right_cam: UW_Camera (右相机)
```

## 注意事项

1. **坐标系**：双目相机的两个相机沿 Y 轴排列（水平排列）
   - 左相机位置：translation[1] - baseline/2
   - 右相机位置：translation[1] + baseline/2

2. **视差与深度**：基线越长，可测量的最大深度越大，但近距离盲区也会增加

3. **性能**：双目相机同时渲染两个视图，GPU 负载约为单目相机的两倍

4. **修改生效**：修改 YAML 配置文件后，需要重新加载场景才能生效
