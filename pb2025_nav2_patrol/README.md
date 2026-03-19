# pb2025_nav2_patrol 巡逻跑点包使用指南

`pb2025_nav2_patrol` 是一个用于在 ROS 2 + Nav2 环境下执行自动化巡逻任务的功能包。它通过读取预设的路点（Waypoints），或者接收动态下发的路点，依次调用 Nav2 的 `NavigateToPose` 动作来实现连续的巡逻导航。

## 1. 启动节点

通过 launch 文件启动巡逻节点，它会自动加载默认的配置文件：

```bash
ros2 launch pb2025_nav2_patrol patrol_launch.py
```

## 2. 基础配置 (config/waypoints.yaml)

默认情况下，`patrol_launch.py` 会加载配置文件 `config/waypoints.yaml`。你可以直接编辑这个文件来设置初始路点和其他参数：

```yaml
waypoints:
  - x: 1.0
    y: 0.0
    yaw: 0.0
  - x: 1.0
    y: 1.0
    yaw: 1.57
  - x: 0.0
    y: 1.0
    yaw: 3.14
loop: true              # 是否循环巡逻
wait_at_point: 2.0      # 到达每个点后的停留等待时间（秒）
retry_on_fail: false    # 导航失败时是否原地重试当前点（设为 false 则失败后直接跳过，前往下一个点）
frame_id: map           # 所在坐标系
auto_start: false       # 启动时是否自动开始巡逻（建议在 RViz 手动打点时设为 false，确保打完所有点后再通过指令开始）
action_name: navigate_to_pose # 调用的 Nav2 Action 名字
```

> **注意**：如果你修改了 `patrol_node.py` 的底层 Python 源码，请务必返回到工作空间根目录下重新编译包：
> ```bash
> colcon build --packages-select pb2025_nav2_patrol
> source install/setup.bash
> ```

## 3. RViz2 可视化打点 (设置多个途经点)

支持在 RViz2 中直接可视化标点，并在后台记录为多个途径点。

### 第 1 步：配置 RViz2 的打点话题

通常，RViz2 顶部的 **2D Goal Pose** 工具默认会将坐标发送到 `/goal_pose` 给 Nav2。我们需要将它的发送目标改成巡逻节点识别的追加目标点话题 `/goal_plan_pose`。

1. 打开 **RViz2**。
2. 在顶部菜单栏点击 **Panels** -> 勾选 **Tool Properties**（打开工具属性面板）。
3. 在弹出的 **Tool Properties** 面板中，找到并展开 **2D Goal Pose** 选项。
4. 将里面的 **Topic**（话题）从 `/goal_pose` 修改为 `/goal_plan_pose`。

### 第 2 步：在地图上依次打点

1. 确保 `patrol_node` 已经在后台运行并且能获取到地图（TF 树正常）。
2. 在 RViz 中，点击顶部的 **2D Goal Pose** 按钮。
3. 在地图上**点击并拖拽**（用于指定该点的朝向），此时一个新的点位置就会发送给 `/goal_plan_pose`。
4. `patrol_node` 接收到后，会控制台输出提示，并将该点**追加**到巡逻列表中。由于 `auto_start` 设置为 `false`，此时**机器人不会立即运动**。

### 第 3 步：一键执行巡逻
当你按照路线在地图上点完所需的所有途径点后，发布以下指令来告诉机器人正式开始巡逻运动：
```bash
ros2 topic pub /start_patrol std_msgs/msg/Bool "data: true" -1
```
*(提示：途中如果想让机器人停下，可以发布相同的指令但将参数改为 `data: false`。)*

### 第 4 步：RViz 轨迹可视化配置（可选）

为了方便在 RViz 中观察巡逻路线，在 RViz 的 Displays 面板点击 **Add**：
- 添加 **Path** 插件，Topic 设置为 `patrol/waypoints` （显示整条巡逻线）
- 添加 **MarkerArray** 插件，Topic 设置为 `patrol/markers` （显示打点的球体和连线高亮，当前前往的点为绿色，其余点为蓝色）

## 4. 动态交互进阶 (Topics & Services)

### 增加/替换巡逻点
- **替换全部巡逻点 (`/goal_plan`)**: 发送 `nav_msgs/Path` 消息。节点会清空旧的巡逻点，将 Path 中的每个 Pose 转换为新的巡逻点，并重启巡逻。
- **追加单个巡逻点 (`/goal_plan_pose`)**: 发送 `geometry_msgs/PoseStamped` 消息。节点会将该点追加到当前的巡逻点列表末尾（即 RViz 打点使用的接口）。

### 保存巡逻点和地图
当你打完所有点觉得满意了，希望将其固化成以后的默认配置文件时：
```bash
ros2 topic pub /save_point std_msgs/msg/Bool "data: true" -1
```
节点会自动抓取所有你刚刚打下的路点，在功能包安装路径的 `share/pb2025_nav2_patrol/save/<时间戳>/` 目录下创建一个新的 `waypoints.yaml` 保存当前所有路点，同时自动调用 `nav2_map_server` 将当前地图 (`map.yaml` 和 `map.pgm`) 一并保存到该目录下。

### 清除巡逻点（撤销错误打点）
如果打点打乱了想全部清空重新来，无需重启节点：
```bash
# 通过话题清空
ros2 topic pub /clear_patrol std_msgs/msg/Bool "data: true" -1
```
或者
```bash
# 通过服务清空
ros2 service call /clear_waypoints std_srvs/srv/Empty
```
清空后即可在 RViz 中重新进行打点配置。
